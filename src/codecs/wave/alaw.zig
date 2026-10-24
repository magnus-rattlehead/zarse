//! decode a-law into interleaved f32 samples
const std = @import("std");
const types = @import("../../types.zig");
const Codec = @import("../../Codec.zig");
const Self = @This();

pub const Error = types.Error;
pub const preroll_frames: u64 = 0;

const decode_table: [256]f32 = table: {
    @setEvalBranchQuota(10000);
    var values: [256]f32 = undefined;
    for (&values, 0..) |*value, byte| {
        value.* = decodeSample(@intCast(byte));
    }
    break :table values;
};

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
            .alaw,
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

pub fn decodeUnit(self: *Self, bytes: []const u8, output: []f32) Error!usize {
    if (self.channels == 0) return error.InvalidState;
    if (bytes.len == 0 or bytes.len % self.channels != 0)
        return error.InvalidData;
    for (bytes, 0..) |byte, index| {
        output[index] = decode_table[byte];
    }
    return bytes.len / self.channels;
}

fn decodeSample(byte: u8) f32 {
    const input = byte ^ 0x55; // a-law inverts alternating bits
    const sign = input & 0x80; // extract sign bit
    const exponent = (input & 0x70) >> 4; // extract three segment bits
    const mantissa = input & 0x0f; // extract the four magnitude bits
    // base steps are 16 pcm units wide; reconstruct their midpoint with +8
    var sample: i32 = (@as(i32, mantissa) << 4) + 8;
    // nonzero segments start at 256; segments above one double at each step
    if (exponent != 0) sample += 256;
    if (exponent > 1) sample <<= @intCast(exponent - 1);

    // set sign bit denotes a positive sample
    sample = if (sign == 0) -sample else sample;
    return @as(f32, @floatFromInt(sample)) / 32768.0;
}

pub fn reset(_: *Self) Error!void {}
pub fn deinit(_: *Self) void {}
