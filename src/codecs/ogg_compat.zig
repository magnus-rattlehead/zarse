//! libogg bitpacking abi; libvorbis directly inspects its pointers and cursors
const std = @import("std");
const bits = @import("bitwise.zig");
const ogg_c = @cImport({
        @cInclude("ogg/ogg.h");
    });
const Buffer = ogg_c.oggpack_buffer;
const Reader = bits.BitReader(.little);
const Writer = bits.BitWriter(.little);

/// borrow c buffer and restore zig reader cursor
fn initReader(buffer: *const Buffer) Reader {
    var reader = Reader.init(
        if (buffer.buffer != null and buffer.storage > 0)
            buffer.buffer[0..@intCast(buffer.storage)]
        else
            &.{},
    );
    reader.state.endByte = @intCast(@max(0, buffer.endbyte));
    reader.state.endBit = @intCast(buffer.endbit);
    // libogg null cursor marks failed read
    reader.state.failed = buffer.ptr == null;
    return reader;
}

/// borrow c owned allocation for zig writer operations
fn initWriter(buffer: *const Buffer) Writer {
    const memory: ?[]u8 = if (buffer.buffer != null and buffer.storage > 0)
        buffer.buffer[0..@intCast(buffer.storage)]
    else
        null;
    return .{
        .state = .{
            .buffer = memory orelse &.{},
            .allocation = memory,
            .allocator = std.heap.c_allocator,
            .endByte = @intCast(@max(0, buffer.endbyte)),
            .endBit = @intCast(buffer.endbit),
            .failed = memory == null,
        },
    };
}

/// write cursor and failure sentinel back to c buffer
fn saveReader(buffer: *Buffer, reader: Reader) void {
    buffer.endbyte = @intCast(reader.state.endByte);
    buffer.endbit = reader.state.endBit;
    buffer.ptr = if (reader.state.failed or buffer.buffer == null)
        null
    else
        buffer.buffer + reader.state.endByte;
}

/// publish possibly reallocated output and cursor back to c buffer
fn saveWriter(buffer: *Buffer, writer: Writer) void {
    buffer.endbyte = @intCast(writer.state.endByte);
    buffer.endbit = writer.state.endBit;
    buffer.buffer = if (writer.state.allocation) |memory| memory.ptr else null;
    buffer.storage = @intCast(writer.state.buffer.len);
    buffer.ptr = if (buffer.buffer != null)
        buffer.buffer + writer.state.endByte
    else
        null;
}

/// convert optional unsigned bits to c result, missing value becomes negative one
fn toCLong(bits_value: ?u32) c_long {
    // preserve unsigned 32 bit values in signed c long results
    const unsigned_value = bits_value orelse return -1;
    return @bitCast(@as(c_ulong, unsigned_value));
}

/// borrow input for little endian bit reads, reset c cursor
export fn oggpack_readinit(
    buffer: *Buffer,
    bytes_buffer: [*c]u8,
    bytes: c_int,
) void {
    buffer.* = .{
        .endbyte = 0,
        .endbit = 0,
        .buffer = bytes_buffer,
        .ptr = bytes_buffer,
        .storage = @max(0, bytes),
    };
}

/// peek up to 32 bits, invalid width or short input returns negative one
export fn oggpack_look(buffer: *Buffer, width: c_int) c_long {
    if (width < 0 or width > 32) return -1;
    const reader = initReader(buffer);
    return toCLong(reader.peek(@intCast(width)));
}

/// peek one bit
export fn oggpack_look1(buffer: *Buffer) c_long {
    return oggpack_look(buffer, 1);
}

/// consume up to 32 bits, invalid width exhausts reader
export fn oggpack_read(buffer: *Buffer, width: c_int) c_long {
    var reader = initReader(buffer);
    var bits_value: ?u32 = null;
    if (width >= 0 and width <= 32) {
        bits_value = reader.read(@intCast(width));
    } else {
        // match libogg exhausted state, cursor one bit past buffer
        reader.state.endByte = reader.state.buffer.len;
        reader.state.endBit = 1;
        reader.state.failed = true;
    }
    saveReader(buffer, reader);
    return toCLong(bits_value);
}

/// consume one bit, failure returns negative one
export fn oggpack_read1(buffer: *Buffer) c_long {
    return oggpack_read(buffer, 1);
}

/// skip bits, negative count or short input exhausts reader
export fn oggpack_adv(buffer: *Buffer, count: c_int) void {
    var reader = initReader(buffer);
    if (count < 0) {
        // match libogg exhausted state, cursor one bit past buffer
        reader.state.endByte = reader.state.buffer.len;
        reader.state.endBit = 1;
        reader.state.failed = true;
    } else {
        reader.advance(@intCast(count));
    }
    saveReader(buffer, reader);
}

/// skip one bit using c reader failure convention
export fn oggpack_adv1(buffer: *Buffer) void {
    oggpack_adv(buffer, 1);
}

/// cursor byte count rounded up for partial byte
export fn oggpack_bytes(buffer: *Buffer) c_long {
    return buffer.endbyte + @divTrunc(buffer.endbit + 7, 8);
}

/// cursor bit count from start of buffer
export fn oggpack_bits(buffer: *Buffer) c_long {
    return buffer.endbyte * 8 + buffer.endbit;
}

/// borrow backing buffer until writer mutation or cleanup
export fn oggpack_get_buffer(buffer: *Buffer) [*c]u8 {
    return buffer.buffer;
}

/// allocate c writer storage, zero state on allocation failure
export fn oggpack_writeinit(buffer: *Buffer) void {
    const writer = Writer.init(std.heap.c_allocator) catch {
        buffer.* = std.mem.zeroes(Buffer);
        return;
    };
    saveWriter(buffer, writer);
}

/// zero for usable writer, negative one for missing storage
export fn oggpack_writecheck(buffer: *Buffer) c_int {
    return if (buffer.ptr == null or buffer.storage == 0) -1 else 0;
}

/// free c writer allocation and clear all fields
export fn oggpack_writeclear(buffer: *Buffer) void {
    var writer = initWriter(buffer);
    writer.deinit();
    buffer.* = std.mem.zeroes(Buffer);
}

/// append low value bits, invalid width clears writer
export fn oggpack_write(buffer: *Buffer, value: c_ulong, width: c_int) void {
    if (width < 0 or width > 32) {
        oggpack_writeclear(buffer);
        return;
    }
    var writer = initWriter(buffer);
    // zig writer clears allocation on failure, publish resulting state
    writer.write(@truncate(value), @intCast(width)) catch {};
    saveWriter(buffer, writer);
}

/// rewind valid writer while keeping allocation
export fn oggpack_reset(buffer: *Buffer) void {
    var writer = initWriter(buffer);
    writer.reset();
    saveWriter(buffer, writer);
}

/// keep first count bits, invalid truncation clears writer
export fn oggpack_writetrunc(buffer: *Buffer, count: c_long) void {
    if (count < 0) {
        oggpack_writeclear(buffer);
        return;
    }
    var writer = initWriter(buffer);
    writer.truncate(@intCast(count)) catch {
        writer.deinit();
    };
    saveWriter(buffer, writer);
}

/// append zero bits to byte boundary, failure clears storage
export fn oggpack_writealign(buffer: *Buffer) void {
    var writer = initWriter(buffer);
    writer.alignToByte() catch {};
    saveWriter(buffer, writer);
}

/// copy count bits from borrowed source, invalid source clears writer
export fn oggpack_writecopy(
    buffer: *Buffer,
    source: ?*anyopaque,
    count: c_long,
) void {
    if (count < 0 or (source == null and count > 0)) {
        oggpack_writeclear(buffer);
        return;
    }
    if (count == 0) return;
    var writer = initWriter(buffer);
    const source_pointer: [*]const u8 = @ptrCast(source.?);
    // round source bit count up to bytes, caller guarantees readable source
    const length = (@as(usize, @intCast(count)) + 7) / 8;
    writer.copy(source_pointer[0..length], @intCast(count)) catch {};
    saveWriter(buffer, writer);
}
