//! wave ima/dvi adpcm block decoder (not aiff ima4)
const std = @import("std");
const types = @import("../../types.zig");
const Codec = @import("../../Codec.zig");
const Self = @This();

pub const Config = types.ImaAdpcmConfig;
pub const Error = types.Error;
pub const preroll_frames: u64 = 0;

config: Config,
channels: u16,

pub fn init(
    allocator: std.mem.Allocator,
    config: Config,
    format: types.AudioFormat,
    options: types.Options,
) Error!Codec {
    if (format.sample_rate == 0 or format.channels == 0 or format.channels > 2)
        return error.InvalidData;
    if (
        config.block_align < 4 * format.channels
            or config.samples_per_block == 0
    )
        return error.InvalidData;
    return Codec.UnitAdapter(Self)
        .init(
            allocator,
            .{ .config = config, .channels = format.channels },
            .ima_adpcm,
            .{ .blocks = config.block_align },
            config.samples_per_block,
            format,
            options,
        );
}

/// todo: implementation
pub fn decodeUnit(self: *Self, bytes: []const u8, output: []f32) Error!usize {
    _ = self;
    _ = bytes;
    _ = output;
    return error.UnsupportedEncoding;
}

pub fn reset(_: *Self) Error!void {}
pub fn deinit(_: *Self) void {}
