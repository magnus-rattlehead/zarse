//! Adapted from audiophile/flac, Copyright (c) 2026 Eric Freed (MIT).
//! Source: https://codeberg.org/audiophile/flac/commit/8681b5be53bc950101f4f33e291b7f1e1f117059
const std = @import("std");
const shared = @import("../bit_reader.zig");
const Self = @This();
pub const Error = shared.Error;
pub const ReadUnaryError = shared.ReadUnaryError;
pub const ReadUt8Error = Error
    || error{ Utf8OverlongEncoding, Utf8ExpectedContinuation };

bits: shared.SliceReader(.big),

pub fn init(bytes: []const u8) Self {
    return .{ .bits = .init(bytes) };
}

pub fn takeBits(self: *Self, comptime T: type, n: u16) Error!T {
    if (n > @bitSizeOf(T)) return error.InvalidBitCount;
    if (n > self.bits.remaining()) return error.EndOfStream;
    if (n <= 32) return @intCast(try self.bits.read(@intCast(n)));
    const high: u64 = try self.bits.read(@intCast(n - 32));
    return @intCast((high << 32) | try self.bits.read(32));
}

pub fn takeByte(self: *Self) Error!u8 {
    return self.takeBits(u8, 8);
}

pub fn takeInt(self: *Self, comptime T: type) Error!T {
    return self.takeBits(T, @bitSizeOf(T));
}

pub fn takeSignExtended(self: *Self, comptime T: type, n: u16) Error!T {
    if (n == 0 or n > @bitSizeOf(T)) return error.InvalidBitCount;
    const value = try self.takeBits(u64, n);
    const shift: u6 = @intCast(64 - n);
    return @intCast(@as(i64, @bitCast(value << shift)) >> shift);
}

pub fn discardBits(self: *Self, n: usize) Error!void {
    try self.bits.advance(n);
}

pub fn takeUnary(
    self: *Self,
    comptime T: type,
    comptime sentinel: u1,
) ReadUnaryError!T {
    var count: T = 0;
    while (true) {
        const width: u4 = @intCast(8 - self.bits.position % 8);
        const value: u8 = @intCast(try self.bits.peek(width));
        const mask: u8 = @intCast((@as(u16, 1) << width) - 1);
        const candidates = (if (sentinel == 1) value else ~value) & mask;
        const run = @clz(candidates) - (8 - width);
        const increment = std.math.cast(T, run)
            orelse return error.UnaryOverflow;
        count = std.math.add(T, count, increment)
            catch return error.UnaryOverflow;
        const terminated = run < width;
        try self.bits.advance(run + @as(u4, @intFromBool(terminated)));
        if (terminated) return count;
    }
}

pub fn byteCount(self: *const Self) usize {
    return (self.bits.position + 7) / 8;
}

pub fn headerChecksum(self: *const Self) u8 {
    var crc = Crc8.init();
    crc.update(self.bits.bytes[0..self.byteCount()]);
    return crc.final();
}

pub fn frameChecksum(self: *const Self) u16 {
    var crc = Crc16.init();
    crc.update(self.bits.bytes[0..self.byteCount()]);
    return crc.final();
}

/// crc-8 with polynomial used by flac
const Crc8 = std.hash.crc.Crc(
    u8,
    .{
        .initial = 0,
        .polynomial = 0x07,
        .reflect_input = false,
        .reflect_output = false,
        .xor_output = 0,
    },
);

/// crc-16 with polynomial used by flac
const Crc16 = std.hash.crc.Crc(
    u16,
    .{
        .initial = 0,
        .polynomial = 0x8005,
        .reflect_input = false,
        .reflect_output = false,
        .xor_output = 0,
    },
);

/// read a utf-8 integer within max_bits
pub fn takeUtf8Int(self: *Self, max_bits: u6) ReadUt8Error!u64 {
    const first = try self.takeBits(u8, 8);

    var value: u64 = 0;
    var continuation_count: u3 = undefined;

    if ((first & 0x80) == 0) {
        value = first;
        continuation_count = 0;
    } else {
        const leading = @clz(~first);
        // flac permits up to seven bytes, beyond standard utf-8
        if (leading < 2 or leading > 7) return error.Utf8OverlongEncoding;
        continuation_count = @intCast(leading - 1);
        value = first & (@as(u8, 0x7f) >> @as(u3, @intCast(leading)));
    }

    for (0..continuation_count) |_| {
        const b = try self.takeBits(u8, 8);

        if ((b & 0xC0) != 0x80) return error.Utf8ExpectedContinuation;

        value = (value << 6) | (b & 0x3F);
    }

    // reject non-minimal encodings as well as values beyond flac limits
    const minimum = [_]u64{
        0,
        0x80,
        0x800,
        0x10000,
        0x200000,
        0x4000000,
        0x80000000,
    };
    if (value < minimum[continuation_count]) return error.Utf8OverlongEncoding;

    if (value >= (@as(u64, 1) << max_bits)) return error.Utf8OverlongEncoding;

    return value;
}
