//! coordinate packets, decoding, trimming, and seeking
//! return interleaved f32 samples in source channel order
//! integer pcm is normalized to negative one through one; floats retain their scale
const std = @import("std");
const types = @import("types.zig");
const Source = @import("Source.zig");
const PacketSource = @import("PacketSource.zig");
const Codec = @import("Codec.zig");
const Decoder = @This();

pub const Error = types.DecoderError;
pub const Options = types.Options;

allocator: std.mem.Allocator,
packets: PacketSource,
codec: Codec,
metadata: types.Info,
format: types.AudioFormat,
presentation: types.Presentation,
inline_frame: [8]f32 = @splat(0),
last_frame: []f32 = &.{},
history_channels: u16 = 0,
/// unread borrowed codec output; drain before advancing/resetting codec
pending: []const f32 = &.{},
// borrows codec output; don't advance codec until the format change is accepted
deferred_block: ?types.AudioBlock = null,
silence: u64 = 0,
// untrimmed pcm position, signed for origins before zero
decoded_frame: i128 = 0,
// temporary untrimmed target while decoding seek preroll
discard_until: ?u64 = null,
frame_offset: u64 = 0,
draining: bool = false,
finished: bool = false,
format_notice: bool = false,
pending_format: ?types.AudioFormat = null,
// sticky read failure, successful seek clears it
failure: ?Error = null,

/// open audio and allocate state; borrow exclusive input access until cleanup
/// keep reader and buffer in place, and input open and unchanged
pub fn init(allocator: std.mem.Allocator, source: Source) Error!Decoder {
    return initOptions(allocator, source, .{});
}

/// open audio with buffer and index limits
pub fn initOptions(
    allocator: std.mem.Allocator,
    source: Source,
    options: Options,
) Error!Decoder {
    var packets = try PacketSource.init(allocator, source, options);
    defer packets.deinit();
    return initTakePackets(allocator, &packets, options);
}

/// open parsed wav and wave64 ranges from the same unchanged, seekable source
pub fn initParsed(
    allocator: std.mem.Allocator,
    source: Source,
    parsed: types.ParsedAudio,
) Error!Decoder {
    var packets = try PacketSource.initParsed(allocator, source, parsed, .{});
    defer packets.deinit();
    return initTakePackets(allocator, &packets, .{});
}

/// take ownership of the packet source, even on failure
/// source is emptied before any fallible work and remains safe to deinit
/// initialize the codec and reserve last frame history for the channel limit
pub fn initTakePackets(
    allocator: std.mem.Allocator,
    source: *PacketSource,
    options: Options,
) Error!Decoder {
    var packets = source.take();
    defer packets.deinit();
    var codec = try Codec.initSource(allocator, &packets, options);
    defer codec.deinit();
    // container framing must match codec units before reading packets
    try packets.configure(codec.scheme);
    var result = Decoder{
        .allocator = allocator,
        .metadata = packets.info(),
        .format = packets.format,
        .presentation = packets.presentation,
        .decoded_frame = packets.decoded_origin,
        .last_frame = &.{},
        .packets = packets.take(),
        .codec = codec.take(),
    };
    errdefer result.deinit();
    // reserve held silence frame for initial or allowed future channel count
    result.history_channels = @max(
        result.format.channels,
        options.reserve_channels,
    );
    if (result.history_channels > result.inline_frame.len) {
        result.last_frame = try allocator.alloc(f32, result.history_channels);
    }
    @memset(result.history(), 0);
    // some sources discover output format while decoding initial packets
    while (result.format.channels == 0 or result.format.sample_rate == 0) {
        switch (try result.advance()) {
            .finished => return error.InvalidData,
            .format_changed => try result.applyFormat(),
            .ready => {},
        }
    }
    // report playable frame count after trimming
    if (result.presentation.end) |end| {
        if (end < result.presentation.start) return error.InvalidData;
        result.metadata.total_frames = end - result.presentation.start;
    } else if (result.packets.info().total_frames) |count| {
        const end = @as(i128, result.packets.decoded_origin) + count;
        if (end < result.presentation.start) return error.InvalidData;
        result.metadata.total_frames = std.math.cast(
            u64,
            end - result.presentation.start,
        )
            orelse return error.InvalidData;
    }
    return result;
}

/// transfer the pipeline, pending output and history; leave self safe to deinit
/// borrowed input must still outlive the returned decoder
pub fn take(self: *Decoder) Decoder {
    const owned = self.*;
    self.* = .{
        .allocator = self.allocator,
        .packets = .{},
        .codec = .{},
        .metadata = self.metadata,
        .format = self.format,
        .presentation = .{},
        .finished = true,
    };
    return owned;
}

/// free buffers and codec state; leave input open
pub fn deinit(self: *Decoder) void {
    self.codec.deinit();
    self.packets.deinit();
    self.allocator.free(self.last_frame);
    self.last_frame = &.{};
    self.pending = &.{};
    self.deferred_block = null;
    self.silence = 0;
    self.finished = true;
    self.format_notice = false;
}

fn history(self: *Decoder) []f32 {
    return if (self.last_frame.len != 0)
        self.last_frame
    else
        self.inline_frame[0..self.history_channels];
}

/// return output format and known frame count after trimming
pub fn info(self: *const Decoder) types.Info {
    return self.metadata;
}

/// return frames written; leave unused output untouched; zero means end
/// borrow caller-owned output only during this call; retain no reference to it
/// output must be nonempty and channel aligned; after a format change
/// check metadata and retry; after an input error, discard output and
/// seek successfully or reopen before reading
pub fn read(self: *Decoder, out: []f32) Error!usize {
    if (self.failure) |err| return err;
    // deliver deferred format change before checking old output alignment
    if (self.format_notice) {
        self.format_notice = false;
        self.applyFormat() catch |err| {
            self.failure = err;
            return err;
        };
        return error.FormatChanged;
    }
    if (out.len == 0 or out.len % self.format.channels != 0) {
        return error.InvalidBufferSize;
    }
    const count = self.readFrames(out) catch |err| {
        // input failures stay sticky, format notice can be retried
        if (err != error.FormatChanged) self.failure = err;
        return err;
    };
    // position tracks presented frames returned to caller
    self.frame_offset = std.math.add(u64, self.frame_offset, count) catch {
        self.failure = error.InvalidData;
        return error.InvalidData;
    };
    return count;
}

/// fill output from pending samples, held silence and new codec blocks
fn readFrames(self: *Decoder, out: []f32) Error!usize {
    const channels = self.format.channels;
    var written: usize = 0;
    while (written < out.len) {
        if (self.pending.len != 0) {
            // consume existing output before advancing codec and invalidating its view
            const count = @min(out.len - written, self.pending.len);
            @memcpy(out[written..][0..count], self.pending[0..count]);
            self.pending = self.pending[count..];
            written += count;
        } else if (self.silence != 0) {
            const frames: usize = @intCast(
                @min((out.len - written) / channels, self.silence),
            );
            // wav silence holds previous frame, initial history contains zeros
            for (0..frames) |frame_index| {
                @memcpy(
                    out[written + frame_index * channels..][0..channels],
                    self.history()[0..channels],
                );
            }
            written += frames * channels;
            self.silence -= frames;
        } else {
            switch (try self.advance()) {
                .ready => {},
                .finished => break,
                .format_changed => {
                    // return old format samples first, announce change on next read
                    if (written != 0) {
                        self.format_notice = true;
                        break;
                    }
                    try self.applyFormat();
                    return error.FormatChanged;
                },
            }
        }
    }
    return written / channels;
}

/// internal reason pipeline stopped advancing
const Advance = enum { ready, finished, format_changed };

/// drive pipeline until output, format change or fully drained end
fn advance(self: *Decoder) Error!Advance {
    if (self.finished) return .finished;
    // format accepted, process saved block before requesting more codec output
    if (self.deferred_block) |block| {
        self.deferred_block = null;
        try self.acceptBlock(block);
        return .ready;
    }
    // reject repeated empty codec output between input packets
    var empty_events: usize = 0;
    // mp3 packet duration counts pcm frames consumed during seek warmup
    var warmup_frames: u64 = 0;
    while (true) {
        const result = try self.codec.receive();
        switch (result) {
            .audio => |block| {
                // pause before exposing samples with different channel layout or rate
                if (!self.format.eql(block.format)) {
                    self.pending_format = block.format;
                    self.deferred_block = block;
                    return .format_changed;
                }
                if (block.samples.len == 0) {
                    empty_events += 1;
                    if (empty_events > 1) return error.InvalidState;
                    // missing reservoir bytes still use up part of the seek discard distance
                    self.decoded_frame = std.math.add(
                        i128,
                        self.decoded_frame,
                        warmup_frames,
                    )
                        catch return error.InvalidData;
                    warmup_frames = 0;
                    continue;
                }
                try self.acceptBlock(block);
                if (self.pending.len != 0) return .ready;
            },
            .format_changed => |format| {
                if (self.format.eql(format)) return error.InvalidState;
                self.pending_format = format;
                return .format_changed;
            },
            .finished => {
                if (!self.draining) return error.InvalidState;
                self.finished = true;
                const total = self.decoded_frame;
                // mp3 follows minimp3, declared counts cap playback
                // clean eof may end playback earlier
                if (self.metadata.format != .mp3) {
                    if (self.packets.info().total_frames) |expected| {
                        const count = total - self.packets.decoded_origin;
                        if (count < expected) return error.TruncatedData;
                        if (count > expected) return error.InvalidData;
                    }
                    if (total < self.presentation.start)
                        return error.TruncatedData;
                    if (self.presentation.end) |end| {
                        if (total < end) return error.TruncatedData;
                    }
                }
                // final playable count uses actual output capped by presentation end
                const end: u64 = @intCast(
                    @min(
                        total,
                        self.presentation.end orelse std.math.maxInt(u64),
                    ),
                );
                self.metadata.total_frames = end -| self.presentation.start;
                return .finished;
            },
            .need_input => {
                if (self.draining) return error.InvalidState;
                const event = try self.packets.next();
                switch (event) {
                    .packet => |packet| {
                        // refusing requested input would deadlock the loop
                        if (try self.codec.submit(packet) != .accepted) {
                            return error.InvalidState;
                        }
                        warmup_frames = if (
                            self.metadata.format == .mp3
                                and self.discard_until != null
                        )
                            packet.duration orelse return error.InvalidData
                        else
                            0;
                        empty_events = 0;
                    },
                    .presentation => |bounds| {
                        // leading trim fixed at opening, later events may only establish final end
                        if (bounds.start != self.presentation.start) {
                            return error.InvalidData;
                        }
                        if (self.presentation.end) |end| {
                            if (bounds.end != end) {
                                return error.InvalidData;
                            }
                        }
                        if (bounds.end) |end| {
                            if (
                                end < bounds.start
                                    or end < self.decoded_frame
                            ) {
                                return error.InvalidData;
                            }
                            self.metadata.total_frames = end - bounds.start;
                        }
                        self.presentation = bounds;
                    },
                    .silence => |frames| {
                        const start = self.decoded_frame;
                        self.decoded_frame = std.math.add(i128, start, frames)
                            catch return error.InvalidData;
                        // intersect decoded interval with presentation bounds and seek target
                        const first = @max(
                            start,
                            @max(
                                self.presentation.start,
                                self.discard_until orelse 0,
                            ),
                        );
                        const last = @min(
                            self.decoded_frame,
                            self.presentation.end orelse std.math.maxInt(u64),
                        );
                        self.silence = @intCast(@max(0, last - first));
                        if (self.silence != 0) return .ready;
                    },
                    .end => {
                        // input ended, continue receiving delayed output before reporting eof
                        try self.codec.finish();
                        self.draining = true;
                    },
                }
            },
        }
    }
}

/// accept pending format within reserved channel capacity, clear held frame
fn applyFormat(self: *Decoder) Error!void {
    const format = self.pending_format orelse return error.InvalidState;
    if (format.channels == 0 or format.sample_rate == 0) {
        return error.InvalidData;
    }
    // format changes cannot allocate beyond reserved history
    if (format.channels > self.history_channels) return error.LimitExceeded;
    @memset(self.history(), 0);
    if (
        self.format.sample_rate != 0
            and self.format.sample_rate != format.sample_rate
    ) {
        // single sample rate can no longer describe duration of whole stream
        self.metadata.variable_sample_rate = true;
    }
    self.format = format;
    self.pending_format = null;
    self.metadata.channel_mask = format.channel_mask;
    self.metadata.channels = format.channels;
    self.metadata.sample_rate = format.sample_rate;
    self.metadata.total_frames = null;
}

/// advance decode timeline and borrow playable part of codec output
fn acceptBlock(self: *Decoder, block: types.AudioBlock) Error!void {
    const channels = block.format.channels;
    if (channels == 0 or block.samples.len % channels != 0) {
        return error.InvalidState;
    }
    // timeline advances by decoded samples, timestamp checked for valid time base
    const start = self.decoded_frame;
    if (block.timestamp) |timestamp| {
        if (
            timestamp.time_base.denominator == 0
                or timestamp.time_base.numerator == 0
        ) {
            return error.InvalidData;
        }
    }
    const frames = block.frameCount();
    self.decoded_frame = std.math.add(i128, start, frames)
        catch return error.InvalidData;
    // save full decoded tail before trimming, future silence repeats it
    if (frames != 0) {
        @memcpy(
            self.history()[0..channels],
            block.samples[block.samples.len - channels..],
        );
    }
    // intersect decoded interval with presentation bounds and seek target
    const first = @max(
        start,
        @max(self.presentation.start, self.discard_until orelse 0),
    );
    const last = @min(
        self.decoded_frame,
        self.presentation.end orelse std.math.maxInt(u64),
    );
    if (last > first) {
        const offset: usize = @intCast(first - start);
        const count: usize = @intCast(last - first);
        self.pending = block.samples[offset * channels..][0..count * channels];
    }
}

/// seek to a presented frame; success clears prior input errors
/// unsupported seeking and known range rejections leave decoder state unchanged
pub fn seekFrame(self: *Decoder, frame: u64) Error!void {
    if (!self.packets.seekable) return error.Unseekable;
    if (self.metadata.total_frames) |total| {
        if (frame > total) return error.OutOfRange;
    }
    // translate presented frame to untrimmed decode timeline
    const target = std.math.add(u64, self.presentation.start, frame)
        catch return error.OutOfRange;
    if (target > std.math.maxInt(i64)) return error.OutOfRange;
    self.seekInternal(target) catch |err| {
        self.failure = err;
        return err;
    };
    self.frame_offset = frame;
    self.failure = null;
}

/// reset at indexed anchor, decode preroll and retain output at target
fn seekInternal(self: *Decoder, target: u64) Error!void {
    // start early enough to rebuild codec history, saturate before frame zero
    const origin: u64 = @intCast(@max(0, self.packets.decoded_origin));
    const anchor = try self.packets.seek(
        @max(origin, target -| self.codec.preroll_frames),
    );
    if (@as(i128, anchor.frame) > target) return error.InvalidState;
    try self.codec.reset();
    self.pending = &.{};
    self.deferred_block = null;
    self.silence = 0;
    self.decoded_frame = anchor.frame;
    self.draining = false;
    self.finished = false;
    self.format_notice = false;
    self.pending_format = null;
    @memset(self.history(), 0);
    // discard preroll samples while preserving any block portion past target
    self.discard_until = target;
    defer self.discard_until = null;
    while (self.decoded_frame < target) {
        switch (try self.advance()) {
            .ready => {},
            .finished => {
                if (self.decoded_frame < target) return error.OutOfRange;
            },
            .format_changed => try self.applyFormat(),
        }
    }
}

/// return the frame position after the last successful read or seek
pub fn tellFrame(self: *const Decoder) u64 {
    return self.frame_offset;
}
