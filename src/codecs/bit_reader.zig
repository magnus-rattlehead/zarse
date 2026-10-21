//! Shared bit extraction for borrowed slices and codec-owned byte readers.
//! Streaming operations adapted from audiophile/flac, Copyright (c) 2026 Eric Freed (MIT).
//! Source: https://codeberg.org/audiophile/flac/commit/8681b5be53bc950101f4f33e291b7f1e1f117059
const std = @import("std");
const assert = std.debug.assert;
const Reader = std.Io.Reader;

/// All readers report short input and invalid widths explicitly. Failed primitive
/// reads, peeks and skips leave the logical cursor and checksum state unchanged.
/// Compound encodings may consume a valid prefix before reporting an error.
pub const Error = Reader.Error || error{ InvalidBitCount };

/// Error set used when reading unary encoded integers.
pub const ReadUnaryError = error{
    /// The encoded value is too large to fit in the resulting integer type.
    UnaryOverflow,
}
    || Error;

/// Read up to 32 bits from borrowed input without moving a cursor.
/// Historical contract:
/// A missing field returns null; each caller chooses its own failure policy.
/// Short input now returns EndOfStream, and widths above 32 return InvalidBitCount.
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

/// Borrowed input with a checked cursor. A caller may reduce limit to enforce
/// a packet or granule boundary without copying or padding the encoded bytes.
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

/// Bit operations for a codec-owned reader with buffer/count fields and a
/// takeByteRaw byte hook. Export selected methods as declarations in that
/// reader; the hook can update checksums or other codec-specific state.
/// These operations borrow the source and never allocate.
pub fn Methods(comptime endian: std.builtin.Endian) type {
    return struct {
        /// Lookup for table of mask values to isolate buffered bits.
        const low_bit_mask = [9]u8{
            0b00000000,
            0b00000001,
            0b00000011,
            0b00000111,
            0b00001111,
            0b00011111,
            0b00111111,
            0b01111111,
            0b11111111,
        };

        /// Discards bits in the buffer, leaving the reader aligned on a byte.
        /// Does nothing if already aligned.
        pub inline fn alignToByte(self: anytype) void {
            self.buffer = 0;
            self.count = 0;
        }

        /// Comptime selects the optimal way to convert an integer of type `Unsigned`
        /// to an integer of type `T`;
        ///
        /// * `T` - Target signed or unsigned integer type to convert to.
        /// * `Unsigned` - An unsigned integer to convert from.
        /// * `value` - The value to convert,
        ///
        /// Returns the value as type of `T`.
        inline fn bitResult(
            comptime T: type,
            comptime Unsigned: type,
            value: Unsigned,
        ) T {
            if (comptime T == Unsigned) return value;
            // This uses comptime branching, so this reduces down to only
            // the necessary final branch at comptime.
            if (comptime @typeInfo(T).int.signedness == .signed) {
                if (comptime @bitSizeOf(T) < 8) {
                    return @bitCast(
                        @as(@Int(.unsigned, @bitSizeOf(T)), @truncate(value)),
                    );
                } else {
                    return @bitCast(value);
                }
            } else {
                return if (comptime @bitSizeOf(T) < 8)
                    @truncate(value)
                else
                    value;
            }
        }

        /// Reads the specified number of bits and returns them as an integer of the given type.
        ///
        /// * `T` - The desired integer type of the return value.
        /// * `n` - The number of bits to read.
        ///
        /// Asserts that `n <= @bitSizeOf(T)`
        ///
        /// Note that for signed integers, the result is merely bit-cast from an
        /// unsigned integer of the same width as `T`, which will likely not
        /// handle the sign bit as expected. See `takeSignExtended` to properly
        /// handle both widening and preserving the sign of the integer.
        pub fn takeBits(self: anytype, comptime T: type, n: u16) Error!T {
            if (n > @bitSizeOf(T)) return error.InvalidBitCount;
            try self.ensureBits(n);
            // Use an unsigned integer with a minimum bit-width of 8.
            const Unsigned = @Int(.unsigned, @max(8, @bitSizeOf(T)));

            // Check if already enough bits in the buffer.
            if (n <= self.count) {
                const result = removeBits(self, @intCast(n));
                return bitResult(T, Unsigned, result);
            }

            // Drain bits in the buffer first.
            var bits_used: u16 = self.count;
            var result: Unsigned = removeBits(self, self.count);

            // Read all the full bytes possible and put their bits where they belong
            const bytes_remaining = (n - bits_used) / 8;

            for (0..bytes_remaining) |_| {
                const byte = try self.takeByteRaw();
                switch (endian) {
                    .big => {
                        // shifting u8 by 8 is illegal
                        if (Unsigned == u8) result = 0 else result <<= 8;
                        result |= byte;
                    },
                    .little => {
                        const pos = @as(Unsigned, byte) << @intCast(bits_used);
                        result |= pos;
                    },
                }
                bits_used += 8;
            }

            const bits_remaining = n - bits_used;
            const keep = 8 - bits_remaining;

            if (bits_remaining == 0) {
                // Evenly divisible
                return bitResult(T, Unsigned, result);
            }

            // Partial byte
            const final_byte = try self.takeByteRaw();
            switch (endian) {
                .big => {
                    result <<= @intCast(bits_remaining);
                    result |= final_byte >> @intCast(keep);
                    self.buffer = final_byte & low_bit_mask[keep];
                },
                .little => {
                    const pos = @as(
                        Unsigned,
                        final_byte & low_bit_mask[bits_remaining],
                    )
                        << @intCast(bits_used);
                    result |= pos;
                    self.buffer = final_byte >> @intCast(bits_remaining);
                },
            }

            self.count = @intCast(keep);
            return bitResult(T, Unsigned, result);
        }

        /// Convenience function for removing bits from the appropriate part
        /// of the buffer based on the defined endian.
        ///
        /// * `n` - The number of bits to remove.
        ///
        /// Returns the value of the remove bits.
        fn removeBits(self: anytype, n: u4) u8 {
            if (n == 8) {
                self.count = 0;
                return self.buffer;
            }

            const keep = self.count - n;
            const bits = switch (endian) {
                .big => self.buffer >> @intCast(keep),
                .little => self.buffer & low_bit_mask[n],
            };
            switch (endian) {
                .big => self.buffer &= low_bit_mask[keep],
                .little => self.buffer >>= @intCast(n),
            }

            self.count = keep;
            return bits;
        }

        /// Reads a single byte from current bit position.
        pub fn takeByte(self: anytype) Error!u8 {
            return self.takeBits(u8, 8);
        }

        /// Reads an integer of the given type from current bit position.
        pub fn takeInt(self: anytype, comptime T: type) Error!T {
            return self.takeBits(T, @bitSizeOf(T));
        }

        /// Reads the specified number of bits, and returns the value as a
        /// sign-extended integer of the given type.
        ///
        /// * `T` - The signed-integer type to return.
        /// * `n` - The number of bits to read.
        ///
        /// Asserts that `n > 0`
        /// Asserts that `n <= @bitSizeOf(T)`
        /// Asserts that `T` is a signed integer (comptime).
        ///
        /// Returns the signed-extended integer.
        pub fn takeSignExtended(
            self: anytype,
            comptime T: type,
            n: u16,
        ) Error!T {
            comptime {
                assert(@typeInfo(T) == .int);
                assert(@typeInfo(T).int.signedness == .signed);
            }
            if (n == 0 or n > @bitSizeOf(T)) return error.InvalidBitCount;

            if (n <= @bitSizeOf(T)) {
                @branchHint(.likely);
                const Unsigned = @Int(.unsigned, @bitSizeOf(T));
                const Shift = std.math.Log2Int(T);

                const value = try self.takeBits(Unsigned, n);
                // Shift the value to the very top of the 32-bit boundary,
                // then arithmetically right shift it back down to sign-extend.
                const shift_amt: Shift = @intCast(@bitSizeOf(T) - n);
                return @as(T, @bitCast(value << shift_amt)) >> shift_amt;
            } else {
                @branchHint(.unlikely);
                const Shift = std.math.Log2Int(i64);

                const value = try self.takeBits(u64, n);
                // Shift the value to the very top of the 64-bit boundary,
                // then arithmetically right shift it back down to sign-extend.
                const shift_amt: Shift = @intCast(64 - n);
                const extended = @as(i64, @bitCast(value << shift_amt))
                    >> shift_amt;
                return @intCast(extended);
            }
        }

        /// Reads bits one at a time until the given sentinel is reached,
        /// and returns the number of non-sentinel bits read.
        ///
        /// * `T` - The desired integer type to return the count as.
        /// * `sentinel` - The terminating value.
        ///
        /// Returns the number of leading bits that were not the sentinel.
        pub fn takeUnary(
            self: anytype,
            comptime T: type,
            comptime sentinel: u1,
        ) ReadUnaryError!T {
            var count: T = 0;

            while (try self.takeBits(u1, 1) != sentinel) : (count += 1) {
                if (count == std.math.maxInt(T)) {
                    return error.UnaryOverflow;
                }
            }
            return count;
        }

        /// Discards the given number of bits and advanced the read position.
        ///
        /// * `n` - The number of bits to discard.
        pub fn discardBits(self: anytype, n: usize) Error!void {
            try self.ensureBits(n);
            // Consume through the bit reader so skipped payload also enters CRC-16.
            var remaining = n;
            while (remaining >= 32) : (remaining -= 32)
                _ = try self.takeBits(u32, 32);
            if (remaining != 0) _ = try self.takeBits(u32, @intCast(remaining));
        }
    };
}
