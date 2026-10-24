//! shared input buffering operations
const std = @import("std");
const types = @import("types.zig");

/// discard consumed bytes while retaining incomplete input
pub fn compact(buffer: []u8, length: *usize, consumed: *usize) void {
    if (consumed.* == 0) return;
    const remaining = length.* - consumed.*;
    std.mem.copyForwards(
        u8,
        buffer[0..remaining],
        buffer[consumed.*..length.*],
    );
    length.* = remaining;
    consumed.* = 0;
}

/// compact and refill state with source, buffer, length, consumed, offset, and eof fields
/// return whether new bytes were read; existing buffer slices may be invalidated
pub fn refill(state: anytype) types.Error!bool {
    if (state.eof) return false;
    // move buffer origin by discarded bytes, unread audio keeps absolute position
    state.offset = std.math.add(u64, state.offset, state.consumed)
        catch return error.InvalidData;
    compact(state.buffer, &state.length, &state.consumed);
    if (state.length == state.buffer.len) return error.LimitExceeded;
    const space = state.buffer[state.length..];
    const count = try state.source.reader.readSliceShort(space);
    state.length += count;
    // readSliceShort fills the requested space unless it reaches eof
    state.eof = count < space.len;
    return count != 0;
}

/// fill output or report truncated input; preserve the reader's other errors
pub fn readRequired(reader: anytype, out: []u8) !void {
    if (try reader.readSliceShort(out) != out.len) return error.TruncatedData;
}
