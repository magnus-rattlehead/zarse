//! convert pcm into interleaved f32 samples
const std = @import("std");
const types = @import("../types.zig");
const Codec = @import("../Codec.zig");
const Self = @This();

pub const Error = types.Error;
pub const Config = types.PcmFormat;
pub const preroll_frames: u64 = 0;

format: Config,
channels: u16,

/// validate the format and allocate the adapter and sample buffer
pub fn init(
    allocator: std.mem.Allocator,
    config: Config,
    format: types.AudioFormat,
    options: types.Options,
) Error!Codec {
    if (format.sample_rate == 0) return error.InvalidData;
    // pcm unit contains exactly one sample per channel
    const input_scheme = try scheme(config, format.channels);
    const unit = Self{ .format = config, .channels = format.channels };
    const adapter = try Codec.UnitAdapter(Self)
        .init(allocator, unit, input_scheme, 1, format, options);
    return adapter.interface(.pcm);
}

/// convert complete frames; output must fit all samples
/// output must be channel aligned; buffer errors leave state and output unchanged
pub fn decodeUnit(self: *Self, bytes: []const u8, output: []f32) Error!usize {
    const size = self.format.sample_type.byteSize();
    try Codec.validateOutput(
        output,
        self.channels,
        bytes.len / size / @max(1, self.channels),
    );
    if (bytes.len == 0) return error.InvalidData;
    // reject partial channel frames before converting samples
    if (bytes.len % (size * self.channels) != 0) return error.InvalidData;
    const samples = bytes.len / size;
    if (output.len < samples) return error.InvalidBufferSize;
    for (output[0..samples], 0..) |*sample, sample_index| {
        sample.* = decodeSample(
            bytes[sample_index * size..][0..size],
            self.format,
        );
    }
    return samples / self.channels;
}

/// reset stateless pcm conversion
pub fn reset(_: *Self) Error!void {}

/// clean up stateless pcm conversion
pub fn deinit(_: *Self) void {}

/// validate the format and return its frame size
pub fn scheme(format: Config, channels: u16) Error!types.InputScheme {
    if (channels == 0) return error.InvalidData;
    try validateFormat(format);
    return .{
        .samples = @as(u32, channels)
            * @as(u32, @intCast(format.sample_type.byteSize())),
    };
}

/// reject zero valid_bits or precision beyond the storage width
pub fn validateFormat(format: types.PcmFormat) types.ValidationError!void {
    if (format.valid_bits) |valid_bits| {
        if (valid_bits == 0 or valid_bits > format.sample_type.byteSize() * 8) {
            return error.InvalidData;
        }
    }
}

/// convert one complete encoded sample to f32
fn decodeSample(bytes: []const u8, format: types.PcmFormat) f32 {
    const endian = format.byte_order;
    // unsigned pcm centered at 128, signed pcm divided by sign bit magnitude
    // float formats preserve encoded scale, valid bits already left aligned
    return switch (format.sample_type) {
        .unsigned8 => (@as(f32, @floatFromInt(bytes[0])) - 128.0) / 128.0,
        .signed8 => @as(f32, @floatFromInt(@as(i8, @bitCast(bytes[0]))))
            / 0x1p7,
        .signed16 => @as(
            f32,
            @floatFromInt(std.mem.readInt(i16, bytes[0..2], endian)),
        )
            / 0x1p15,
        .signed24 => @as(
            f32,
            @floatFromInt(std.mem.readInt(i24, bytes[0..3], endian)),
        )
            / 0x1p23,
        .signed32 => @as(
            f32,
            @floatFromInt(std.mem.readInt(i32, bytes[0..4], endian)),
        )
            / 0x1p31,
        .signed64 => @as(
            f32,
            @floatFromInt(std.mem.readInt(i64, bytes[0..8], endian)),
        )
            / 0x1p63,
        .float32 => @bitCast(std.mem.readInt(u32, bytes[0..4], endian)),
        .float64 => @floatCast(
            @as(f64, @bitCast(std.mem.readInt(u64, bytes[0..8], endian))),
        ),
    };
}
