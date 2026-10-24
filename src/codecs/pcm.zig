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
    return Codec.UnitAdapter(Self)
        .init(allocator, unit, .pcm, input_scheme, 1, format, options);
}

/// convert complete frames; output must fit all samples
pub fn decodeUnit(self: *Self, bytes: []const u8, output: []f32) Error!usize {
    const size = self.format.sample_type.byteSize();
    if (bytes.len == 0) return error.InvalidData;
    if (bytes.len % (size * self.channels) != 0) return error.InvalidData;
    const samples = bytes.len / size;
    // specialize once per buffer so sample conversion has no format dispatch
    switch (self.format.sample_type) {
        inline else => |sample_type| switch (self.format.byte_order) {
                inline else => |endian| convertSamples(
                        sample_type,
                        endian,
                        bytes,
                        output[0..samples],
                    ),
            },
    }
    return samples / self.channels;
}

fn convertSamples(
    comptime sample_type: types.PcmSampleType,
    comptime endian: std.builtin.Endian,
    bytes: []const u8,
    output: []f32,
) void {
    const format: Config = .{
        .sample_type = sample_type,
        .byte_order = endian,
    };
    const size = comptime sample_type.byteSize();
    var index: usize = 0;
    // packed 24-bit samples use the scalar reader; the other storage widths
    // map directly to integer or floating point vector lanes
    if (sample_type != .signed24) {
        const lanes = std.simd.suggestVectorLength(f32) orelse 4;
        const Int = switch (sample_type) {
            .unsigned8 => u8,
            .signed8 => i8,
            .signed16 => i16,
            .signed32 => i32,
            .signed64 => i64,
            .float32 => u32,
            .float64 => u64,
            .signed24 => unreachable,
        };
        const Vec = @Vector(lanes, f32);
        while (output.len - index >= lanes) : (index += lanes) {
            var values: @Vector(lanes, Int) = @bitCast(
                bytes[index * size..][0..lanes * size].*,
            );
            if (endian != @import("builtin").cpu.arch.endian())
                values = @byteSwap(values);
            const converted: Vec = switch (sample_type) {
                .float32 => @bitCast(values),
                .float64 => @floatCast(
                    @as(@Vector(lanes, f64), @bitCast(values)),
                ),
                .unsigned8 => (@as(Vec, @floatFromInt(values))
                    - @as(Vec, @splat(128.0)))
                    / @as(Vec, @splat(128.0)),
                else => @as(Vec, @floatFromInt(values))
                    / @as(
                        Vec,
                        @splat(
                            @as(
                                f32,
                                @floatFromInt(
                                    @as(u64, 1) << (@bitSizeOf(Int) - 1),
                                ),
                            ),
                        ),
                    ),
            };
            output[index..][0..lanes].* = converted;
        }
    }
    for (output[index..], index..) |*sample, sample_index| {
        sample.* = decodeSample(bytes[sample_index * size..][0..size], format);
    }
}

pub fn reset(_: *Self) Error!void {}

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
