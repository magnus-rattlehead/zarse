//! decode mu-law into interleaved f32 samples
const std = @import("std");
const types = @import("../../types.zig");
const Codec = @import("../../Codec.zig");
const Self = @This();

pub const Error = types.Error;
pub const preroll_frames: u64 = 0;

channels: u16,

pub fn init(
    allocator: std.mem.Allocator,
    format: types.AudioFormat,
    options: types.Options,
) Error!Codec {
    if (format.channels == 0 or format.sample_rate == 0)
        return error.InvalidData;
    return Codec.UnitAdapter(Self)
        .init(
            allocator,
            .{ .channels = format.channels },
            .mulaw,
            try scheme(format.channels),
            1,
            format,
            options,
        );
}

pub fn scheme(channels: u16) Error!types.InputScheme {
    if (channels == 0) return error.InvalidData;
    return .{ .samples = channels };
}

pub fn decodeUnit(self: *Self, bytes: []const u8, _: []f32) Error!usize {
    if (self.channels == 0) return error.InvalidState;
    if (bytes.len == 0 or bytes.len % self.channels != 0)
        return error.InvalidData;
    // todo: expand each mu-law codeword, normalize to f32, return frames
    return error.UnsupportedEncoding;
}

pub fn reset(_: *Self) Error!void {}
pub fn deinit(_: *Self) void {}
