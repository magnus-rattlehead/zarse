//! Zig Opus multistream decoder. Upstream attribution and licenses: opus/README.md
const std = @import("std");
const types = @import("../types.zig");
const Codec = @import("../Codec.zig");
const MultistreamDecoder = @import("opus/MultistreamDecoder.zig");
const Self = @This();

pub const Error = types.Error;
pub const Config = types.OpusConfig;

// use extra seek history to settle energy prediction beyond the 80 ms
// min recommended by rfc 7845
pub const preroll_frames: u64 = 24000; // 500 ms at 48 khz

allocator: std.mem.Allocator,
decoder: MultistreamDecoder,
channels: u16,
gain: f32,

/// allocate a 48 khz decoder and output buffer in source channel order
pub fn init(
    allocator: std.mem.Allocator,
    config: Config,
    format: types.AudioFormat,
    options: types.Options,
) Error!Codec {
    try config.validate(format);
    const unit = Self{
        .allocator = allocator,
        .decoder = MultistreamDecoder.init(
            allocator,
            format.channels,
            config.streams,
            config.coupled_streams,
            &config.mapping,
        )
            catch |err| return switch (err) {
                error.OutOfMemory => error.OutOfMemory,
                error.BadArg => error.InvalidData,
            },
        .channels = format.channels,
        // convert fixed point db to amplitude, divide gain by 256 then db by 20
        .gain = std.math.pow(
            f32,
            10,
            @as(f32, @floatFromInt(config.gain_q8)) / (20 * 256),
        ),
    };
    // 5760 samples per channel cover the max 120 ms packet
    return Codec.UnitAdapter(Self)
        .init(allocator, unit, .opus, null, 5760, format, options);
}

/// decode one packet with header gain and return the frame count
/// output must hold 5760 samples per channel. The pipeline handles trimming
/// input failures require reset before reuse
pub fn decodeUnit(self: *Self, bytes: []const u8, output: []f32) Error!usize {
    if (bytes.len == 0) return error.InvalidData;
    const sample_count = self.decoder.decode(bytes, output)
        catch |err| return switch (err) {
            error.BufferTooSmall => error.InvalidBufferSize,
            error.Truncated,
            error.FrameTooLong,
            error.UnevenFrames,
            error.BadFrameLength,
            error.BadFrameCount,
            error.UnevenCbr,
            error.InvalidPacket => error.InvalidData,
        };
    // apply header gain only to samples produced by this packet
    for (output[0..sample_count * self.channels]) |*sample| {
        sample.* *= self.gain;
    }
    return sample_count;
}

/// reset history with the same channels and mapping
pub fn reset(self: *Self) Error!void {
    self.decoder.reset();
}

pub fn deinit(self: *Self) void {
    self.decoder.deinit(self.allocator);
    self.* = undefined;
}
