//! read wav and wave64 audio segments, silence, and seek locations
//! riff.zig shares parsing between seekable and streaming input
const std = @import("std");
const types = @import("../../types.zig");
const input_helpers = @import("../../input.zig");
const Source = @import("../../Source.zig");
const PacketSource = @import("../../PacketSource.zig");
const riff = @import("riff.zig");
const Stream = riff.Reader(true);
const SeekIndex = @import("../seek_index.zig").Index(SegmentLocation);
/// cached wavl segment with resume cursor and held silence history
const SegmentLocation = struct {
    segment: types.Segment,
    /// next chunk position after this segment
    next_offset: u64,
    /// last data frame, or zero before any data; silence holds this value
    previous_frame: u64 = 0,
};
const Self = @This();

pub const Error = types.Error;
pub const wav = riff.wav;
pub const wave64 = riff.wave64;

/// parse metadata and borrowed audio ranges; keep input unchanged
/// errors may leave input at any position
pub const parse = riff.parse;
/// parse with the same metadata limits used by sequential playback
pub const parseOptions = riff.parseOptions;

allocator: std.mem.Allocator,
source: Source,
options: types.Options,
parsed: ?types.ParsedAudio = null,
stream: ?Stream = null,
segments: ?types.AudioData.Iterator = null,
/// cached wavl segments; frame positions are set once frame size is known
index: SeekIndex = .{},
metadata: types.Info = undefined,
scheme: ?types.InputScheme = null,
/// fixed input storage borrowed by packets
buffer: []u8 = &.{},
packet_capacity: usize = 0,
/// retain final audio beyond the packet buffer while resolving wave64 padding
padding_tail: [7]u8 = undefined,
tail_length: usize = 0,
tail_position: usize = 0,
current: ?types.Segment = null,
segment_bytes: u64 = 0,
frame: u64 = 0,

/// recognize a four byte prefix; validate full headers when opening
pub fn probe(prefix: []const u8) bool {
    return wav.probe(prefix) or wave64.probe(prefix);
}

/// open borrowed input after detection consumed its four byte prefix
/// allocate buffers and any wavl index; parse ranges for seekable input
/// or walk chunks incrementally for sequential input
pub fn init(
    allocator: std.mem.Allocator,
    source: Source,
    prefix: [4]u8,
    options: types.Options,
) Error!PacketSource {
    const seekable = source.seek_to != null and try source.getSize() != null;
    const state = try allocator.create(Self);
    state.* = .{ .allocator = allocator, .source = source, .options = options };
    errdefer state.deinit();
    state.buffer = try allocator.alloc(u8, options.bufferCapacity());
    // seekable inputs scan metadata and cache segment layout before playback
    if (seekable) {
        state.parsed = try parseOptions(source, options);
        state.segments = state.parsed.?.data.iterate(source);
        state.metadata = state.parsed.?.info;
        try state.initIndex();
        return state.interface(state.parsed.?.scheme);
    }
    // sequential inputs keep chunk walker at first audio segment
    state.stream = try Stream.initPrefix(allocator, source, options, prefix);
    state.current = try state.stream.?.next();
    const stream = &state.stream.?;
    const fmt = stream.format orelse return error.InvalidData;
    state.metadata = .{
        .format = stream.kind,
        .sample_rate = fmt.sample_rate,
        .channels = fmt.channels,
        .channel_mask = fmt.channelMask(),
        .total_frames = if (stream.finished) stream.frames else null,
    };
    return state.interface(fmt.decodingScheme(stream.byte_order));
}

/// accept fixed units once, calculate packet capacity and seek frame positions
pub fn configure(self: *Self, scheme: types.InputScheme) Error!void {
    if (self.scheme != null) return error.InvalidState;
    const unit = scheme.unitBytes() orelse return error.UnsupportedEncoding;
    if (unit == 0) return error.InvalidData;
    if (unit > self.buffer.len) return error.LimitExceeded;
    // round buffer down to whole encoded units
    self.packet_capacity = self.buffer.len / unit * unit;
    if (self.parsed) |parsed| {
        if (scheme == .samples) {
            var frames: u64 = 0;
            if (parsed.data.layout == .wavl) {
                var previous: u64 = 0;
                for (self.index.entries.items) |*entry| {
                    entry.frame = frames;
                    // remember preceding data frame so seeks into silence can restore held value
                    entry.location.previous_frame = previous;
                    frames = std.math.add(
                        u64,
                        frames,
                        try entry.location.segment.frameCount(unit),
                    )
                        catch return error.InvalidData;
                    if (entry.location.segment == .data) previous = frames - 1;
                }
            } else {
                frames = try parsed.data.frameCount(self.source, unit);
            }
            if (parsed.info.total_frames) |declared| {
                if (frames != declared) return error.InvalidData;
            }
            self.metadata.total_frames = frames;
        }
        self.segments = parsed.data.iterate(self.source);
    }
    self.scheme = scheme;
}

/// borrow aligned audio packet, emit held silence or end, bytes expire on next read
pub fn next(self: *Self) Error!PacketSource.Event {
    const scheme = self.scheme orelse return error.InvalidState;
    const unit = scheme.unitBytes().?;
    while (true) {
        if (self.tail_position < self.tail_length) {
            const count = @min(
                self.packet_capacity,
                self.tail_length - self.tail_position,
            );
            @memcpy(
                self.buffer[0..count],
                self.padding_tail[self.tail_position..][0..count],
            );
            self.tail_position += count;
            return self.audioPacket(count, scheme);
        }
        // fetch next segment only after current data or silence exhausted
        if (self.current == null) {
            self.current = if (self.stream) |*stream|
                try stream.next()
            else
                try self.segments.?.next();
            self.segment_bytes = 0;
            if (self.current == null) {
                if (self.stream) |stream| {
                    self.metadata.total_frames = stream.frames;
                }
                return .end;
            }
        }
        switch (self.current.?) {
            .silence => |frames| {
                // held silence advances pcm timeline
                self.current = null;
                if (frames == 0) continue;
                self.frame = std.math.add(u64, self.frame, frames)
                    catch return error.InvalidData;
                return .{ .silence = frames };
            },
            .data => |range| {
                const remaining = range.length - self.segment_bytes;
                if (remaining == 0) {
                    self.current = null;
                    continue;
                }
                var count: usize = @intCast(
                    @min(remaining, self.packet_capacity),
                );
                if (self.stream) |*stream| {
                    if (stream.kind == .wave64) {
                        // retain final frames plus up to seven padding bytes
                        const tail = range.length
                            - (range.length - @min(range.length, 7))
                                / unit
                                * unit;
                        if (remaining <= tail) {
                            // only the padding-sized suffix needs extra storage
                            const buffered: usize = @intCast(
                                @min(remaining, self.packet_capacity),
                            );
                            const extra: usize = @intCast(remaining - buffered);
                            try stream.read(self.buffer[0..buffered]);
                            try stream.read(self.padding_tail[0..extra]);
                            if (try stream.next() != null) {
                                return error.UnsupportedChunkLayout;
                            }
                            // metadata now complete, separate actual audio bytes from trailing alignment
                            const length = try stream.wave64DataLength();
                            if (length < self.segment_bytes) {
                                return error.InvalidData;
                            }
                            const audio_bytes: usize = @intCast(
                                length - self.segment_bytes,
                            );
                            count = @min(audio_bytes, buffered);
                            self.tail_length = audio_bytes - count;
                            self.tail_position = 0;
                            self.metadata.total_frames = stream.frames;
                            self.current = null;
                        } else {
                            count = @intCast(@min(count, remaining - tail));
                            try stream.read(self.buffer[0..count]);
                        }
                    } else try stream.read(self.buffer[0..count]);
                } else {
                    // borrowed segment ranges may be separated by metadata chunks
                    try self.source.seekTo(range.offset + self.segment_bytes);
                    try input_helpers.readRequired(
                        self.source.reader,
                        self.buffer[0..count],
                    );
                }
                if (count % unit != 0) return error.InvalidData;
                self.segment_bytes += count;
                if (self.segment_bytes == range.length) self.current = null;
                if (count == 0) continue;
                return self.audioPacket(count, scheme);
            },
        }
    }
}

/// publish whole audio units and advance their untrimmed timeline
fn audioPacket(
    self: *Self,
    count: usize,
    scheme: types.InputScheme,
) Error!PacketSource.Event {
    const unit = scheme.unitBytes().?;
    if (count % unit != 0) return error.InvalidData;
    var packet = types.Packet{ .bytes = self.buffer[0..count] };
    // sample units map directly to pcm frames, block durations left unknown
    if (scheme == .samples) {
        if (self.frame > std.math.maxInt(i64)) {
            return error.InvalidData;
        }
        packet.timestamp = .{
            .value = @intCast(self.frame),
            .time_base = .{
                .numerator = 1,
                .denominator = self.metadata.sample_rate,
            },
        };
        packet.duration = count / unit;
        self.frame = std.math.add(u64, self.frame, count / unit)
            catch return error.InvalidData;
    }
    return .{ .packet = packet };
}

/// return known stream metadata and untrimmed frame count
pub fn info(self: *const Self) types.Info {
    return self.metadata;
}

/// report frame seeking only for indexed, configured sample units
pub fn canSeek(self: *const Self) bool {
    return self.parsed != null
        and self.scheme != null
        and self.scheme.? == .samples;
}

/// seek at or before requested frame, replay prior data when silence needs history
pub fn seek(self: *Self, frame: u64) Error!PacketSource.SeekResult {
    try PacketSource.validateSeek(
        self.scheme != null,
        self.canSeek(),
        frame,
        0,
        self.metadata.total_frames,
    );
    const parsed = self.parsed.?;
    const scheme = self.scheme.?;
    self.tail_length = 0;
    self.tail_position = 0;
    self.segments = parsed.data.iterate(self.source);
    self.current = null;
    self.segment_bytes = 0;
    // seeking exactly to end leaves iterator exhausted
    if (frame == self.metadata.total_frames.?) {
        switch (self.segments.?.state) {
            .single => |*segment| segment.* = null,
            .wavl => |*segments| {
                segments.offset = try segments.list.children.end();
            },
        }
        self.frame = frame;
        return .{
            .frame = std.math.cast(i64, frame) orelse return error.InvalidData,
        };
    }
    var anchor = frame;
    var segment_frame: u64 = 0;
    switch (parsed.data.layout) {
        .single => |single| {
            self.segments.?.state.single = null;
            if (single == .silence) anchor = 0;
            self.current = single;
        },
        .wavl => {
            var entry = self.index.atOrBefore(frame)
                orelse return error.InvalidState;
            if (entry.location.segment == .silence) {
                // replay prior data to restore held silence; leading silence
                // starts at zero
                anchor = entry.location.previous_frame;
                entry = self.index.atOrBefore(anchor)
                    orelse return error.InvalidState;
            }
            self.segments.?.state.wavl.offset = entry.location.next_offset;
            segment_frame = entry.frame;
            self.current = entry.location.segment;
        },
    }
    // translate pcm anchor into byte cursor or remaining silence count
    const remaining = anchor - segment_frame;
    switch (self.current.?) {
        .data => |range| {
            self.segment_bytes = remaining * scheme.samples;
            try self.source.seekTo(range.offset + self.segment_bytes);
        },
        .silence => |count| self.current = .{ .silence = count - remaining },
    }
    self.frame = anchor;
    return .{
        .frame = std.math.cast(i64, anchor) orelse return error.InvalidData,
    };
}

/// free reader buffers and index, leave borrowed source open
pub fn deinit(self: *Self) void {
    if (self.stream) |*stream| stream.deinit();
    self.index.deinit(self.allocator);
    self.allocator.free(self.buffer);
    const allocator = self.allocator;
    allocator.destroy(self);
}

/// open an unchanged parsed layout with seek and size support
/// allocate packet storage and any wavl index
/// trust supplied metadata; wire limits belong to the preceding parseOptions call
pub fn initParsed(
    allocator: std.mem.Allocator,
    source: Source,
    parsed: types.ParsedAudio,
    options: types.Options,
) Error!PacketSource {
    if (source.seek_to == null or source.get_size == null) {
        return error.Unseekable;
    }
    const state = try allocator.create(Self);
    state.* = .{
        .allocator = allocator,
        .source = source,
        .options = options,
        .parsed = parsed,
        .metadata = parsed.info,
    };
    errdefer state.deinit();
    state.buffer = try allocator.alloc(u8, options.bufferCapacity());
    state.segments = parsed.data.iterate(state.source);
    try state.initIndex();
    return state.interface(parsed.scheme);
}

/// cache nonempty wavl segments within index budget, frame positions assigned later
fn initIndex(self: *Self) Error!void {
    const layout = self.parsed.?.data.layout;
    if (layout != .wavl) return;
    const source = self.source;
    const size = try source.getSize() orelse return error.Unseekable;
    var segments = layout.wavl.iterate(source);
    while (try segments.next()) |segment| {
        if (segment == .data and try segment.data.end() > size) {
            return error.TruncatedData;
        }
        // skip empty segments
        if (
            switch (segment) {
                .data => |range| range.length == 0,
                .silence => |frames| frames == 0,
            }
        )
            continue;
        try self.index.append(
            self.allocator,
            self.options.max_index_bytes,
            .{
                // assigned during configuration, before seeking
                .frame = 0,
                .location = .{
                    .segment = segment,
                    .next_offset = segments.offset,
                },
            },
        );
    }
}

/// transfer reader ownership into packet source with parsed audio format
fn interface(self: *Self, encoding: types.DecodingScheme) PacketSource {
    const format = types.AudioFormat{
        .channels = self.metadata.channels,
        .sample_rate = self.metadata.sample_rate,
        // extensible format may supply explicit speaker mask in either input mode
        .channel_mask = self.metadata.channel_mask,
    };
    return .{
        .backend = .{ .wave = self },
        .encoding = encoding,
        .format = format,
        .seekable = self.parsed != null and encoding == .pcm,
        .max_payload_bytes = self.options.max_payload_bytes,
    };
}
