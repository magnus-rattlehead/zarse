//! bit extraction from slices
const std = @import("std");
const Reader = std.Io.Reader;

/// all readers report short input and invalid widths explicitly, failed primitive
/// reads, peeks and skips leave the cursor unchanged
/// compound encodings may consume a valid prefix before reporting an error
pub const Error = Reader.Error || error{ InvalidBitCount };

/// error set used when reading unary encoded integers
pub const ReadUnaryError = error{
    /// the encoded value is too large to fit in the resulting integer type
    UnaryOverflow,
}
    || Error;

/// peek up to 32 bits without moving the cursor
/// short input returns EndOfStream; widths above 32 return InvalidBitCount
pub fn peek(
    comptime order: std.builtin.Endian,
    input: []const u8,
    position: usize,
    width: u6,
) Error!u32 {
    if (width > 32) return error.InvalidBitCount;
    const first = position / 8;
    if (first > input.len) return error.EndOfStream;
    const offset: u3 = @truncate(position);
    const total = @as(u6, offset) + width;
    // round up bytes needed for width plus current bit offset
    const bytes: usize = (@as(usize, total) + 7) / 8;
    if (bytes > input.len - first) return error.EndOfStream;
    if (width == 0) return 0;
    const mask = (@as(u64, 1) << width) - 1;
    // enough bytes for one wide load, mask keeps only requested bits
    if (input.len - first >= 8) {
        const value = std.mem.readInt(u64, input[first..][0..8], order);
        const trailing: u6 = if (order == .big)
            @intCast(64 - @as(u7, total))
        else
            offset;
        return @intCast((value >> trailing) & mask);
    }
    // round end up to a byte, 32 bits starting inside a byte can span five bytes
    const end = first + bytes;
    // assemble enough bytes to cover value and partial byte offset
    var word: u64 = 0;
    for (input[first..end], 0..) |byte, byte_index| {
        if (order == .big) {
            word = (word << 8) | byte;
        } else {
            word |= @as(u64, byte) << @as(u6, @intCast(byte_index * 8));
        }
    }
    // shift away bits after requested field, then mask away bits before it
    const shift: u6 = if (order == .big)
        @intCast(bytes * 8 - total)
    else
        offset;
    return @intCast((word >> shift) & mask);
}

/// a checked cursor, reduce limit to enforce
/// a packet or granule boundary
pub fn SliceReader(comptime order: std.builtin.Endian) type {
    return struct {
        const Self = @This();
        bytes: []const u8 = &.{},
        position: usize = 0,
        limit: usize = 0,

        pub fn init(input: []const u8) Self {
            return .{ .bytes = input, .limit = input.len * 8 };
        }

        pub fn remaining(self: Self) usize {
            return @min(self.limit, self.bytes.len * 8) -| self.position;
        }

        pub fn peek(self: Self, width: u6) Error!u32 {
            if (width > 32) return error.InvalidBitCount;
            try self.checkAvailable(width);
            return peekBits(order, self.bytes, self.position, width);
        }

        pub fn read(self: *Self, width: u6) Error!u32 {
            const value = try self.peek(width);
            self.position += width;
            return value;
        }

        pub fn advance(self: *Self, count: usize) Error!void {
            try self.checkAvailable(count);
            self.position += count;
        }

        fn checkAvailable(self: Self, count: usize) Error!void {
            const end = @min(self.limit, self.bytes.len * 8);
            if (self.position > end or count > end - self.position)
                return error.EndOfStream;
        }
    };
}

const peekBits = peek;
