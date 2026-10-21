//! Adapted from audiophile/flac, Copyright (c) 2026 Eric Freed (MIT).
//! Source: https://codeberg.org/audiophile/flac/commit/8681b5be53bc950101f4f33e291b7f1e1f117059
const std = @import("std");
const Reader = std.Io.Reader;
const Endian = std.builtin.Endian;
const bit_reader = @import("../bit_reader.zig");

// Backwards compatibility for breaking changes in 0.16 -> 0.17 transition
const Crc = blk: {
    const version = @import("builtin").zig_version;
    if (version.major < 1 and version.minor < 17) {
        break :blk std.hash.crc.Crc;
    }
    break :blk std.hash.crc.Generic;
};

/// CRC-8 with polynomial used by FLAC.
const Crc8 = Crc(
    u8,
    .{
        .initial = 0,
        .polynomial = 0x07,
        .reflect_input = false,
        .reflect_output = false,
        .xor_output = 0,
    },
);

/// CRC-16 with polynomial used by FLAC.
const Crc16 = Crc(
    u16,
    .{
        .initial = 0,
        .polynomial = 0x8005,
        .reflect_input = false,
        .reflect_output = false,
        .xor_output = 0,
    },
);

/// Provides a wrapper for a `std.Io.Reader` to read at bit granularity.
///
/// * `endian` - The endianness of the source data being read.
/// * `checksum` - Flag indicating if CRC checksums will be computed.
pub fn BitReader(comptime endian: Endian, comptime checksum: bool) type {
    return struct {
        /// Error set for read operations.
        pub const Error = bit_reader.Error;

        /// Error set used when reading UTF-8 encoded integers.
        pub const ReadUt8Error = error{
            /// The encoded value is too large to fit in the resulting integer type.
            Utf8OverlongEncoding,
            /// Expected continuation bit to be set.
            Utf8ExpectedContinuation,
        }
            || Error;

        /// Reference to this generic BitReader type.
        const Self = @This();

        const Shared = bit_reader.Methods(endian);
        pub const ReadUnaryError = bit_reader.ReadUnaryError;
        pub const alignToByte = Shared.alignToByte;
        pub const takeBits = Shared.takeBits;
        pub const takeByte = Shared.takeByte;
        pub const takeInt = Shared.takeInt;
        pub const takeSignExtended = Shared.takeSignExtended;
        pub const takeUnary = Shared.takeUnary;
        pub const discardBits = Shared.discardBits;

        /// The underlying reader providing the source data.
        child: *Reader,
        /// A buffer containing unconsumed bits from last byte read.
        buffer: u8 = 0,
        /// The number of valid bits in the buffer.
        count: u4 = 0,
        /// A running counter for tracking the number of bytes read.
        /// Note that is counter is reset and reused between frames.
        byte_count: usize = 0,
        /// Flag indicating if the CRC-8 will be computed.
        /// The CRC-8 is only used when reading frame headers.
        /// The CRC-16 is used for the entire frame.
        crc8_enabled: bool = false,
        /// The CRC-8 state.
        crc8: Crc8 = .init(),
        /// The CRC-16 state.
        crc16: Crc16 = .init(),

        /// Resets the CRC hashes to their initial state, and enables the checks.
        ///
        /// This function is a no-op when when checksums are disabled via a build options.
        pub inline fn resetChecksum(self: *Self) void {
            self.crc8 = .init();
            self.crc16 = .init();
        }

        /// Reserve a complete field before consuming bytes or updating CRCs.
        /// The FLAC adapter supplies a fixed reader over a complete packet.
        pub fn ensureBits(self: *Self, n: usize) Error!void {
            const missing = n -| self.count;
            const bytes = missing / 8 + @intFromBool(missing % 8 != 0);
            if (bytes > self.child.buffer.len) return error.EndOfStream;
            _ = try self.child.peek(bytes);
        }

        /// Returns a single byte from the underlying reader.
        /// Updates CRC states when enabled.
        pub inline fn takeByteRaw(self: *Self) !u8 {
            // Only update checksums when enabled via build option.
            if (checksum) {
                // No need to bounds check here
                var array: [1]u8 = undefined;
                array[0] = try self.child.takeByte();
                // Increment the byte count
                self.byte_count += 1;
                // Update CRC with value
                if (self.crc8_enabled) {
                    // CRC-8 is only used for frame header
                    self.crc8.update(&array);
                }
                self.crc16.update(&array);
                return array[0];
            } else {
                const byte = try self.child.takeByte();
                self.byte_count += 1;
                return byte;
            }
        }

        /// Reads a UTF-8 encoded integer.
        ///
        /// * `max_bits` - The maximum number of bits to consume before value is overlong.
        ///
        /// Returns the decoded integer.
        pub fn takeUtf8Int(self: *Self, max_bits: u6) ReadUt8Error!u64 {
            const first = try self.takeBits(u8, 8);

            var value: u64 = 0;
            var continuation_count: u3 = undefined;

            if ((first & 0x80) == 0) {
                value = first;
                continuation_count = 0;
            } else if ((first & 0xE0) == 0xC0) {
                value = first & 0x1F;
                continuation_count = 1;
            } else if ((first & 0xF0) == 0xE0) {
                value = first & 0x0F;
                continuation_count = 2;
            } else if ((first & 0xF8) == 0xF0) {
                value = first & 0x07;
                continuation_count = 3;
            } else if ((first & 0xFC) == 0xF8) {
                value = first & 0x03;
                continuation_count = 4;
            } else if ((first & 0xFE) == 0xFC) {
                value = first & 0x01;
                continuation_count = 5;
            } else if (first == 0xFE) {
                value = 0;
                continuation_count = 6;
            } else {
                return error.Utf8OverlongEncoding;
            }

            for (0..continuation_count) |_| {
                const b = try self.takeBits(u8, 8);

                if ((b & 0xC0) != 0x80) return error.Utf8ExpectedContinuation;

                value = (value << 6) | (b & 0x3F);
            }

            // Reject non-minimal encodings as well as values beyond FLAC limits.
            const minimum = [_]u64{
                0,
                0x80,
                0x800,
                0x10000,
                0x200000,
                0x4000000,
                0x80000000,
            };
            if (value < minimum[continuation_count])
                return error.Utf8OverlongEncoding;

            // Enforce FLAC limits.
            if (value >= (@as(u64, 1) << max_bits))
                return error.Utf8OverlongEncoding;

            return value;
        }
    };
}
