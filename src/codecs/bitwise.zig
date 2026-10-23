//! ogg bit packing; writers own storage and readers borrow input
const std = @import("std");
const bit_reader = @import("bit_reader.zig");

/// shared bit cursor, readers borrow buffer and writers own allocation
const State = struct {
    // whole bytes before cursor
    endByte: usize = 0,
    // bit offset within cursor byte
    endBit: u3 = 0,
    buffer: []const u8 = &.{},
    allocation: ?[]u8 = null,
    allocator: ?std.mem.Allocator = null,
    failed: bool = false,
};

pub const WriteError = error{
    InvalidBitCount,
    InvalidTruncation,
    ShortSource,
    NotWriter,
    Overflow,
    NoSpaceLeft,
};

/// append up to 32 bits in selected order
fn writeValue(
    comptime most_significant_first: bool,
    state: *State,
    value: u32,
    width: u6,
) WriteError!void {
    if (!valid(state)) return error.NotWriter;
    if (width > 32) return error.InvalidBitCount;
    if (width == 0) return;
    const total: u6 = width + @as(u6, state.endBit);
    const end = std.math.add(usize, state.endByte, total / 8)
        catch return error.Overflow;
    const used = std.math.add(usize, end, @intFromBool(total % 8 != 0))
        catch return error.Overflow;
    if (used >= state.buffer.len) return error.NoSpaceLeft;
    // discard value bits outside requested width
    const masked = value & lowMask(width);
    const memory = state.allocation.?;
    // 64 bits hold the value plus its partial byte offset
    const word = if (most_significant_first)
        @as(u64, masked) << @as(u6, @intCast(40 - total))
    else
        @as(u64, masked) << state.endBit;
    const first: u8 = @truncate(
        if (most_significant_first) word >> 32 else word,
    );
    // preserve bits already written in partial leading byte
    memory[state.endByte] |= first;
    var byte_index: u6 = 1;
    while (byte_index <= total / 8) : (byte_index += 1) {
        // remove leading or trailing offset bits before applying width mask
        const shift: u6 = if (most_significant_first)
            (4 - byte_index) * 8
        else
            byte_index * 8;
        memory[state.endByte + byte_index] = @truncate(word >> shift);
    }
    // advance whole bytes and retain low three bits as bit offset
    state.endByte += total / 8;
    state.endBit = @truncate(total);
}

/// mask containing width low bits, width at most 32
fn lowMask(width: u6) u32 {
    if (width == 0) return 0;
    const shift: u5 = @intCast(32 - width);
    const mask: u32 = std.math.maxInt(u32);
    return mask >> shift;
}

/// shorten written bit stream and clear unused bits in final byte
fn trunc(
    comptime most_significant_first: bool,
    state: *State,
    bits: usize,
) WriteError!void {
    if (!valid(state)) return error.NotWriter;
    if (bits > countBits(state)) return error.InvalidTruncation;
    state.endByte = bits / 8;
    state.endBit = @truncate(bits);
    // retain written side of partial byte for configured bit order
    const keep: u8 = if (most_significant_first)
        @truncate(@as(u16, 0xff00) >> state.endBit)
    else
        @truncate(lowMask(state.endBit));
    if (state.allocation) |memory| memory[state.endByte] &= keep;
}

/// append zero bits until next byte boundary
fn writeAlign(
    comptime most_significant_first: bool,
    state: *State,
) WriteError!void {
    if (state.endBit != 0)
        try writeValue(
            most_significant_first,
            state,
            0,
            8 - @as(u6, state.endBit),
        );
}

/// append source bits, support overlap with owned output
fn writeCopy(
    comptime most_significant_first: bool,
    state: *State,
    source: []const u8,
    bits: usize,
) WriteError!void {
    if (!valid(state)) return error.NotWriter;
    const bytes = bits / 8;
    const trailing: u3 = @truncate(bits);
    if (bytes > source.len or (bytes == source.len and trailing != 0))
        return error.ShortSource;
    const total = std.math.add(usize, bits, state.endBit)
        catch return error.Overflow;
    const end = std.math.add(usize, state.endByte, total / 8)
        catch return error.Overflow;

    if (bits == 0) return;
    const used = std.math.add(usize, end, @intFromBool(total % 8 != 0))
        catch return error.Overflow;
    if (used >= state.buffer.len) return error.NoSpaceLeft;
    const source_address = @intFromPtr(source.ptr);
    const buffer_address = @intFromPtr(state.buffer.ptr);
    var offset: ?usize = null;
    if (
        source_address >= buffer_address
            and source_address - buffer_address < state.buffer.len
    ) {
        const start = source_address - buffer_address;
        if (source.len > state.buffer.len - start) return error.ShortSource;
        offset = start;
    }
    if (state.endBit != 0 and offset != null) {
        copyAliased(most_significant_first, state, source, bits, offset.?);
        return;
    }
    const tail = if (trailing != 0) source[bytes] else 0;
    // unaligned destination needs bit writes to merge boundary bytes
    if (state.endBit != 0) {
        for (0..bytes) |byte_index| {
            const byte = source[byte_index];
            try writeValue(most_significant_first, state, byte, 8);
        }
    } else {
        const destination = state.allocation.?[state.endByte..][0..bytes];
        @memmove(destination, source[0..bytes]);
        state.endByte += bytes;
        state.allocation.?[state.endByte] = 0;
    }
    // copy remaining source bits from side selected by bit order
    if (trailing != 0) {
        const value = if (most_significant_first)
            tail >> @as(u3, @intCast(8 - @as(u4, trailing)))
        else
            tail;
        try writeValue(most_significant_first, state, value, trailing);
    }
}

fn copyAliased(
    comptime most_significant_first: bool,
    state: *State,
    source: []const u8,
    bits: usize,
    offset: usize,
) void {
    const memory = state.allocation.?;
    const backwards = state.endByte >= offset;
    for (0..bits) |step| {
        const bit = if (backwards) bits - 1 - step else step;
        const source_bit: u3 = @truncate(bit);
        const source_shift = if (most_significant_first)
            7 - source_bit
        else
            source_bit;
        const value = (source[bit / 8] >> source_shift) & 1;
        const destination = bit + @as(usize, state.endBit);
        const destination_bit: u3 = @truncate(destination);
        const shift = if (most_significant_first)
            7 - destination_bit
        else
            destination_bit;
        const mask = @as(u8, 1) << shift;
        const byte = &memory[state.endByte + destination / 8];
        byte.* = (byte.* & ~mask) | (value << shift);
    }
    const total = bits + @as(usize, state.endBit);
    state.endByte += total / 8;
    state.endBit = @truncate(total);
    const keep: u8 = if (most_significant_first)
        @truncate(@as(u16, 0xff00) >> state.endBit)
    else
        @truncate(lowMask(state.endBit));
    memory[state.endByte] &= keep;
}

/// peek a complete field, returning an error when it is unavailable
fn look(
    comptime most_significant_first: bool,
    state: *const State,
    width: u6,
) bit_reader.Error!u32 {
    if (width > 32) return error.InvalidBitCount;
    if (state.failed or state.endByte > state.buffer.len)
        return error.EndOfStream;
    return bit_reader.peek(
        if (most_significant_first) .big else .little,
        state.buffer[state.endByte..],
        state.endBit,
        width,
    );
}

/// read a complete field; errors leave the cursor unchanged
fn readValue(
    comptime most_significant_first: bool,
    state: *State,
    width: u6,
) bit_reader.Error!u32 {
    const value = try look(most_significant_first, state, width);
    try advanceCursor(state, width);
    return value;
}

/// writer still owns usable storage
fn valid(state: *const State) bool {
    return state.allocator != null and !state.failed;
}

/// rewind valid writer, retain capacity
fn resetState(state: *State) void {
    if (!valid(state)) return;
    state.endByte = 0;
    state.endBit = 0;
    if (state.allocation) |memory| memory[0] = 0;
}

/// release writer allocation and clear cursor state
fn clear(state: *State) void {
    if (state.allocation) |memory| state.allocator.?.free(memory);
    state.* = .{};
}

/// skip only when the entire requested range is available
fn advanceCursor(state: *State, bits: usize) bit_reader.Error!void {
    if (state.failed or state.endByte > state.buffer.len)
        return error.EndOfStream;
    var reader = bit_reader.SliceReader(.little)
        .init(state.buffer[state.endByte..]);
    reader.position = state.endBit;
    try reader.advance(bits);
    // partial final byte must also exist in input
    // advance whole bytes and retain low three bits as bit offset
    state.endByte += reader.position / 8;
    state.endBit = @truncate(reader.position);
}

/// byte count including any partial final byte
fn countBytes(state: *const State) usize {
    return state.endByte + @intFromBool(state.endBit != 0);
}

/// bit position from beginning of buffer
fn countBits(state: *const State) usize {
    return state.endByte * 8 + state.endBit;
}

/// return written bytes for writers, or the full input for readers
fn getBytes(state: *const State) []const u8 {
    return if (state.allocator != null)
        state.buffer[0..countBytes(state)]
    else
        state.buffer;
}

/// borrow unchanged input with a comptime bit order; little reads lsb first
/// reads, peeks and skips return errors without moving the cursor on failure
pub fn BitReader(comptime order: std.builtin.Endian) type {
    return struct {
        const Self = @This();
        state: State,

        /// borrow unchanged input starting at bit zero
        pub fn init(input: []const u8) Self {
            return .{ .state = .{ .buffer = input } };
        }

        /// failed peeks leave the cursor unchanged
        pub fn peek(self: *const Self, width: u6) bit_reader.Error!u32 {
            return look(order == .big, &self.state, width);
        }

        /// peek one bit without moving the cursor
        pub fn peekBit(self: *const Self) bit_reader.Error!u1 {
            const value = try self.peek(1);
            return @intCast(value);
        }

        /// read a complete field; errors leave the cursor unchanged
        pub fn read(self: *Self, width: u6) bit_reader.Error!u32 {
            return readValue(order == .big, &self.state, width);
        }

        /// read one bit; errors leave the cursor unchanged
        pub fn readBit(self: *Self) bit_reader.Error!u1 {
            const value = try self.read(1);
            return @intCast(value);
        }

        /// skip bits; errors leave the cursor unchanged
        pub fn advance(self: *Self, bits: usize) bit_reader.Error!void {
            try advanceCursor(&self.state, bits);
        }

        /// skip one bit using same bounds checks
        pub fn advanceBit(self: *Self) bit_reader.Error!void {
            try self.advance(1);
        }

        /// number of bits consumed
        pub fn bitCount(self: *const Self) usize {
            return countBits(&self.state);
        }

        /// consumed bytes rounded up to include partial byte
        pub fn byteCount(self: *const Self) usize {
            return countBytes(&self.state);
        }

        /// whether the input is exhausted or the cursor is marked failed
        pub fn isExhausted(self: *const Self) bool {
            return self.state.failed
                or self.state.endByte == self.state.buffer.len;
        }

        /// borrow original input including bytes already consumed
        pub fn bytes(self: *const Self) []const u8 {
            return self.state.buffer;
        }
    };
}

/// own output storage
pub fn BitWriter(comptime order: std.builtin.Endian) type {
    return struct {
        const Self = @This();
        state: State = .{},

        pub fn init(allocator: std.mem.Allocator) Self {
            return .{ .state = .{ .allocator = allocator } };
        }

        pub fn reserve(
            self: *Self,
            capacity: usize,
        ) (std.mem.Allocator.Error || error{ NotWriter, Overflow })!void {
            const allocator = self.state.allocator
                orelse return error.NotWriter;
            const size = std.math.add(usize, capacity, 1)
                catch return error.Overflow;
            if (size <= self.state.buffer.len) return;
            const allocation: []u8 = self.state.allocation orelse &.{};
            const memory = try allocator.realloc(allocation, size);
            if (self.state.allocation == null) memory[0] = 0;
            self.state.allocation = memory;
            self.state.buffer = memory;
        }

        /// transfer output storage and cursor; leave self safe to deinit
        pub fn take(self: *Self) Self {
            const owned = self.*;
            self.* = .{};
            return owned;
        }

        /// free owned output and clear state
        pub fn deinit(self: *Self) void {
            clear(&self.state);
        }

        /// whether writer still owns usable output storage
        pub fn isValid(self: *const Self) bool {
            return valid(&self.state);
        }

        /// discard written bits while retaining output allocation
        pub fn reset(self: *Self) void {
            resetState(&self.state);
        }

        /// append low width bits of value in configured bit order
        pub fn write(self: *Self, value: u32, width: u6) WriteError!void {
            try writeValue(order == .big, &self.state, value, width);
        }

        /// append source bits, source may alias output storage
        pub fn copy(
            self: *Self,
            source: []const u8,
            bits: usize,
        ) WriteError!void {
            try writeCopy(order == .big, &self.state, source, bits);
        }

        /// keep first bits of output, reject extension or invalid writer
        pub fn truncate(self: *Self, bits: usize) WriteError!void {
            try trunc(order == .big, &self.state, bits);
        }

        /// pad output with zeros to next byte boundary
        pub fn alignToByte(self: *Self) WriteError!void {
            try writeAlign(order == .big, &self.state);
        }

        /// number of bits written
        pub fn bitCount(self: *const Self) usize {
            return countBits(&self.state);
        }

        /// written bytes rounded up to include partial byte
        pub fn byteCount(self: *const Self) usize {
            return countBytes(&self.state);
        }

        /// borrow written bytes until mutation or cleanup
        pub fn bytes(self: *const Self) []const u8 {
            return getBytes(&self.state);
        }
    };
}
