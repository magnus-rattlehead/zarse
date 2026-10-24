//! read wav and wave64 audio segments, silence, and seek locations
//! riff.zig shares parsing between seekable and streaming input
const std = @import("std");
const types = @import("../../types.zig");
const input_helpers = @import("../../input.zig");
const Source = @import("../../Source.zig");
const PacketSource = @import("../../PacketSource.zig");
const riff = @import("riff.zig");
const Stream = riff.Reader;
const SeekIndex = @import("../seek_index.zig").Index(SegmentLocation);
/// cached wavl segment and held silence history
const SegmentLocation = struct {
    segment: types.Segment,
    /// last data frame, or zero before any data; silence holds this value
    previous_frame: u64 = 0,
};
const Self = @This();

pub const Error = types.Error;
allocator: std.mem.Allocator,
source: Source,
seekable: bool = false,
stream: ?Stream = null,
segment_cursor: usize = 0,
/// audio segments and their untrimmed frame positions
index: SeekIndex = .{},
metadata: types.Info = undefined,
scheme: types.InputScheme = .packets,
buffer: []u8 = &.{},
packet_capacity: usize = 0,
/// retain final audio beyond the packet buffer while resolving wave64 padding
padding_tail: [7]u8 = undefined,
tail_length: usize = 0,
tail_position: usize = 0,
current: ?types.Segment = null,
segment_bytes: u64 = 0,

/// recognize a four byte prefix; validate full headers when opening
pub fn probe(prefix: []const u8) bool {
    return riff.probe(prefix);
}

/// open input after detection consumed its four byte prefix
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
    state.* = .{ .allocator = allocator, .source = source };
    errdefer state.deinit();
    state.buffer = try allocator.alloc(u8, options.bufferCapacity());
    var walker = try Stream.initPrefix(
        allocator,
        source,
        options,
        prefix,
        seekable,
    );
    defer walker.deinit();
    state.current = try walker.next();
    const fmt = walker.format orelse return error.InvalidData;
    const encoding = fmt.decodingScheme(walker.byte_order);
    state.scheme = if (fmt.fixedWidth()) .{ .samples = fmt.block_align } else .{
        .blocks = fmt.block_align,
    };
    const unit = state.scheme.unitBytes().?;
    if (unit > state.buffer.len) return error.LimitExceeded;
    state.packet_capacity = state.buffer.len / unit * unit;
    state.seekable = seekable and fmt.fixedWidth();
    if (seekable) {
        var frames: u64 = 0;
        var previous: u64 = 0;
        while (state.current) |segment| {
            const count = if (!fmt.fixedWidth())
                0
            else if (walker.kind == .wave64 and segment == .data)
                segment.data.length / unit
            else
                try segment.frameCount(unit);
            if (count != 0 or (segment == .data and segment.data.length != 0)) {
                try state.index.append(
                    allocator,
                    options.max_index_bytes,
                    .{
                        .frame = frames,
                        .location = .{
                            .segment = segment,
                            .previous_frame = previous,
                        },
                    },
                );
                frames = std.math.add(u64, frames, count)
                    catch return error.InvalidData;
                if (segment == .data and count != 0) previous = frames - 1;
            }
            state.current = try walker.next();
        }
        // wave64 data may include up to seven alignment bytes
        if (
            walker.kind == .wave64
                and fmt.fixedWidth()
                and state.index.entries.items.len != 0
        ) {
            state.index.entries.items[0]
                .location
                .segment
                .data
                .length = try walker.wave64DataLength();
        }
    } else {
        state.stream = walker;
        walker.table = &.{};
    }
    state.metadata = if (state.stream) |*stream|
        stream.info()
    else
        walker.info();
    return .{
        .backend = .{ .wave = state },
        .encoding = encoding,
        .seekable = state.seekable,
    };
}

/// return aligned audio, held silence, or end; packet bytes expire on the next read
pub fn next(self: *Self) Error!PacketSource.Event {
    const scheme = self.scheme;
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
            self.current = try self.nextSegment();
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
                    // audio segments may be separated by metadata chunks
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

fn nextSegment(self: *Self) Error!?types.Segment {
    if (self.stream) |*stream| return stream.next();
    if (self.segment_cursor == self.index.entries.items.len) return null;
    const segment = self.index.entries.items[self.segment_cursor]
        .location
        .segment;
    self.segment_cursor += 1;
    return segment;
}

/// publish whole audio units and advance their untrimmed timeline
fn audioPacket(
    self: *Self,
    count: usize,
    scheme: types.InputScheme,
) Error!PacketSource.Event {
    const unit = scheme.unitBytes().?;
    if (count % unit != 0) return error.InvalidData;
    return .{ .packet = .{ .bytes = self.buffer[0..count] } };
}

/// return known stream metadata and untrimmed frame count
pub fn info(self: *const Self) types.Info {
    return self.metadata;
}

/// seek at or before requested frame, replay prior data when silence needs history
pub fn seek(self: *Self, frame: u64) Error!PacketSource.SeekResult {
    try PacketSource.validateSeek(
        self.seekable,
        frame,
        0,
        self.metadata.total_frames,
    );
    const scheme = self.scheme;
    self.tail_length = 0;
    self.tail_position = 0;
    self.current = null;
    self.segment_bytes = 0;
    // seeking exactly to end leaves iterator exhausted
    if (frame == self.metadata.total_frames.?) {
        self.segment_cursor = self.index.entries.items.len;
        return .{ .frame = frame };
    }
    var anchor = frame;
    var selected = self.index.indexAtOrBefore(frame)
        orelse return error.InvalidState;
    const entries = self.index.entries.items;
    if (entries[selected].location.segment == .silence) {
        anchor = entries[selected].location.previous_frame;
        selected = self.index.indexAtOrBefore(anchor)
            orelse return error.InvalidState;
    }
    self.segment_cursor = selected + 1;
    const segment_frame = entries[selected].frame;
    self.current = entries[selected].location.segment;
    // translate pcm anchor into byte cursor or remaining silence count
    const remaining = anchor - segment_frame;
    switch (self.current.?) {
        .data => |range| {
            self.segment_bytes = remaining * scheme.samples;
            try self.source.seekTo(range.offset + self.segment_bytes);
        },
        .silence => |count| self.current = .{ .silence = count - remaining },
    }
    return .{ .frame = anchor };
}

/// leave input open
pub fn deinit(self: *Self) void {
    if (self.stream) |*stream| stream.deinit();
    self.index.deinit(self.allocator);
    self.allocator.free(self.buffer);
    const allocator = self.allocator;
    allocator.destroy(self);
}
