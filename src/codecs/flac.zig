//! decode complete flac frames using the adapted audiophile/flac core
//! core source attribution and mit notice: flac/license
const std = @import("std");
const types = @import("../types.zig");
const Codec = @import("../Codec.zig");
pub const Frame = @import("flac/Frame.zig");
const BitReader = @import("flac/bit_reader.zig");
const Self = @This();

pub const preroll_frames: u64 = 0;
allocator: std.mem.Allocator,
frame: Frame,

pub fn mapError(err: (Frame.ReadError || types.FlacConfig.Error)) types.Error {
    return switch (err) {
        error.EndOfStream => error.TruncatedData,
        error.ReadFailed => error.ReadFailed,
        error.InvalidSampleRate,
        error.InvalidBlockSize,
        error.InvalidFrameSize,
        error.InvalidBitDepth,
        error.InvalidChannelCount,
        error.InvalidChannelLayout,
        error.ChecksumMismatch,
        error.OutOfSync,
        error.InvalidSubframeType,
        error.InvalidPredictor,
        error.UnknownResidualCodingMethod,
        error.InvalidShift,
        error.Utf8OverlongEncoding,
        error.Utf8ExpectedContinuation,
        error.InvalidBitCount,
        error.UnaryOverflow => error.InvalidData,
    };
}

pub fn init(
    allocator: std.mem.Allocator,
    config: types.FlacConfig,
    format: types.AudioFormat,
    options: types.Options,
) types.Error!Codec {
    try validateConfig(config);
    if (
        format.channels != config.channels
            or format.sample_rate != config.sample_rate
    )
        return error.InvalidData;
    var unit: Self = .{
        .allocator = allocator,
        .frame = try Frame.init(allocator, config),
    };
    defer unit.deinit();
    return Codec.UnitAdapter(Self)
        .init(
            allocator,
            unit.take(),
            .flac,
            .packets,
            config.max_block_size,
            format,
            options,
        );
}

/// leave self empty and safe to deinit
pub fn take(self: *Self) Self {
    return .{ .allocator = self.allocator, .frame = self.frame.take() };
}

pub fn validateConfig(info: types.FlacConfig) types.Error!void {
    info.validate() catch |err| return mapError(err);
}

/// output must be channel aligned and hold the configured maximum block; input failures require reset
pub fn decodeUnit(
    self: *Self,
    bytes: []const u8,
    out: []f32,
) types.Error!usize {
    if (bytes.len == 0) return error.InvalidData;
    var bits: BitReader = .init(bytes);
    const count = self.frame.read(&bits, true) catch |err| return mapError(err);
    if (bits.byteCount() != bytes.len) return error.InvalidData;
    const channels = self.frame.info.channels;
    const scale = 1.0
        / @as(
            f32,
            @floatFromInt(
                @as(u32, 1) << @as(u5, @intCast(self.frame.sample_bits - 1)),
            ),
        );
    for (0..count) |i| {
        for (self.frame.subframes[0..channels], 0..) |sub, channel| {
            out[i * channels + channel] = @as(f32, @floatFromInt(sub.buffer[i]))
                * scale;
        }
    }
    return count;
}

pub fn reset(self: *Self) types.Error!void {
    self.frame.block_size = 0;
}

pub fn deinit(self: *Self) void {
    self.frame.deinit(self.allocator);
}
