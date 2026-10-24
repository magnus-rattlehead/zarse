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
presentation: types.Presentation,
inline_frame: [8]f32 = @splat(0),
last_frame: []f32 = &.{},
/// drain pending output before advancing or resetting the codec
pending: []const f32 = &.{},
silence: u64 = 0,
// untrimmed pcm position
decoded_frame: u64 = 0,
// temporary untrimmed target while decoding seek preroll
discard_until: ?u64 = null,
frame_offset: u64 = 0,
finished: bool = false,
// sticky read failure, successful seek clears it
failure: ?Error = null,

/// give the decoder exclusive input access until cleanup
/// keep reader and buffer in place, and input open and unchanged
/// insufficient capacity returns LimitExceeded during indexing or a later read
pub fn init(
    allocator: std.mem.Allocator,
    source: Source,
    options: Options,
) Error!Decoder {
    var packets = try PacketSource.init(allocator, source, options);
    defer packets.deinit();
    var codec = try Codec.initSource(allocator, &packets, options);
    defer codec.deinit();
    var result = Decoder{
        .allocator = allocator,
        .metadata = packets.info(),
        .presentation = packets.presentation,
        .decoded_frame = packets.decoded_origin,
        .last_frame = &.{},
        .packets = packets.take(),
        .codec = codec.take(),
    };
    errdefer result.deinit();
    if (result.metadata.channels > result.inline_frame.len) {
        result.last_frame = try allocator.alloc(f32, result.metadata.channels);
    }
    @memset(result.history(), 0);
    // report playable frame count after trimming
    if (result.presentation.end) |end| {
        if (end < result.presentation.start) return error.InvalidData;
        result.metadata.total_frames = end - result.presentation.start;
    } else if (result.packets.info().total_frames) |count| {
        const end = std.math.add(u64, result.packets.decoded_origin, count)
            catch return error.InvalidData;
        if (end < result.presentation.start) return error.InvalidData;
        result.metadata.total_frames = end - result.presentation.start;
    }
    return result;
}

/// leave self empty and safe to deinit
pub fn take(self: *Decoder) Decoder {
    const owned = self.*;
    self.* = .{
        .allocator = self.allocator,
        .packets = .{},
        .codec = .{},
        .metadata = self.metadata,
        .presentation = .{},
        .finished = true,
    };
    return owned;
}

/// leave input open
pub fn deinit(self: *Decoder) void {
    self.codec.deinit();
    self.packets.deinit();
    self.allocator.free(self.last_frame);
    self.last_frame = &.{};
    self.pending = &.{};
    self.silence = 0;
    self.finished = true;
}

fn history(self: *Decoder) []f32 {
    return if (self.last_frame.len != 0)
        self.last_frame
    else
        self.inline_frame[0..self.metadata.channels];
}

/// return output format and known frame count after trimming
pub fn info(self: *const Decoder) types.Info {
    return self.metadata;
}

/// return frames written; leave unused output untouched; zero means end
/// output must be nonempty and channel aligned
/// after an input error, discard output and seek successfully or reopen
pub fn read(self: *Decoder, out: []f32) Error!usize {
    if (self.failure) |err| return err;
    if (out.len == 0 or out.len % self.metadata.channels != 0) {
        return error.InvalidBufferSize;
    }
    const count = self.readFrames(out) catch |err| {
        self.failure = err;
        return err;
    };
    self.frame_offset = std.math.add(u64, self.frame_offset, count) catch {
        self.failure = error.InvalidData;
        return error.InvalidData;
    };
    return count;
}

/// fill output from pending samples, held silence and new codec blocks
fn readFrames(self: *Decoder, out: []f32) Error!usize {
    const channels = self.metadata.channels;
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
        } else if (!try self.advance()) break;
    }
    return written / channels;
}

/// read packets until playable samples, silence, or end
fn advance(self: *Decoder) Error!bool {
    if (self.finished) return false;
    while (true) {
        switch (try self.packets.next()) {
            .packet => |packet| {
                const samples = try self.codec.decode(packet.bytes);
                if (samples.len == 0) {
                    // missing mp3 reservoir bytes still count toward seek preroll
                    if (
                        self.metadata.format == .mp3
                            and self.discard_until != null
                    ) {
                        self.decoded_frame = std.math.add(
                            u64,
                            self.decoded_frame,
                            packet.duration orelse return error.InvalidData,
                        )
                            catch return error.InvalidData;
                    }
                    continue;
                }
                try self.acceptSamples(samples);
                if (self.pending.len != 0) return true;
            },
            .presentation => |bounds| {
                // leading trim is fixed at opening; later events may establish the end
                if (bounds.start != self.presentation.start)
                    return error.InvalidData;
                if (self.presentation.end) |end| {
                    if (bounds.end != end) return error.InvalidData;
                }
                if (bounds.end) |end| {
                    if (end < bounds.start or end < self.decoded_frame)
                        return error.InvalidData;
                    self.metadata.total_frames = end - bounds.start;
                }
                self.presentation = bounds;
            },
            .silence => |frames| {
                const start = self.decoded_frame;
                self.decoded_frame = std.math.add(u64, start, frames)
                    catch return error.InvalidData;
                const first = @max(
                    start,
                    @max(self.presentation.start, self.discard_until orelse 0),
                );
                const last = @min(
                    self.decoded_frame,
                    self.presentation.end orelse std.math.maxInt(u64),
                );
                self.silence = last -| first;
                if (self.silence != 0) return true;
            },
            .end => {
                self.finished = true;
                const total = self.decoded_frame;
                // mp3 declared counts cap playback; clean eof may end playback earlier
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
                const end: u64 = @intCast(
                    @min(
                        total,
                        self.presentation.end orelse std.math.maxInt(u64),
                    ),
                );
                self.metadata.total_frames = end -| self.presentation.start;
                return false;
            },
        }
    }
}

/// advance the decode timeline and select playable samples
fn acceptSamples(self: *Decoder, samples: []const f32) Error!void {
    const channels = self.metadata.channels;
    if (samples.len % channels != 0) return error.InvalidState;
    const start = self.decoded_frame;
    const frames = samples.len / channels;
    self.decoded_frame = std.math.add(u64, start, frames)
        catch return error.InvalidData;
    // save full decoded tail before trimming, future silence repeats it
    if (frames != 0) {
        @memcpy(self.history()[0..channels], samples[samples.len - channels..]);
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
        self.pending = samples[offset * channels..][0..count * channels];
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
    const origin = self.packets.decoded_origin;
    const anchor = try self.packets.seek(
        @max(origin, target -| self.codec.preroll_frames),
    );
    if (anchor.frame > target) return error.InvalidState;
    try self.codec.reset();
    self.pending = &.{};
    self.silence = 0;
    self.decoded_frame = anchor.frame;
    self.finished = false;
    @memset(self.history(), 0);
    // discard preroll samples while preserving any block portion past target
    self.discard_until = target;
    defer self.discard_until = null;
    while (self.decoded_frame < target) {
        if (!try self.advance() and self.decoded_frame < target)
            return error.OutOfRange;
    }
}

/// return the frame position after the last successful read or seek
pub fn tellFrame(self: *const Decoder) u64 {
    return self.frame_offset;
}
