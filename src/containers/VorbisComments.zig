//! borrow vendor and comment fields for vorbis, opus, and flac
//! keep input unchanged; callers handle signatures, framing, and padding
const std = @import("std");
const Error = @import("../types.zig").ValidationError;
const Self = @This();

vendor: []const u8,
/// unread fields and trailer; only the trailer remains after iteration
remaining: []const u8,
comments_left: u32,

/// read vendor and count after the signature; iterate to validate all lengths
pub fn init(bytes: []const u8) Error!Self {
    var remaining = bytes;
    const vendor = try readField(&remaining);
    // counts and lengths are little endian u32 values
    if (remaining.len < 4) return error.InvalidData;
    const count = std.mem.readInt(u32, remaining[0..4], .little);
    remaining = remaining[4..];
    // each comment needs at least its four byte length field
    if (count > remaining.len / 4) return error.InvalidData;
    return .{
        .vendor = vendor,
        .remaining = remaining,
        .comments_left = count,
    };
}

/// borrow the next comment, or null at the end; empty comments remain valid
/// validate lengths here; callers interpret text and field names
pub fn next(self: *Self) Error!?[]const u8 {
    if (self.comments_left == 0) return null;
    const comment = try readField(&self.remaining);
    self.comments_left -= 1;
    return comment;
}

/// borrow length prefixed field and advance remaining slice
fn readField(remaining: *[]const u8) Error![]const u8 {
    if (remaining.len < 4) return error.InvalidData;
    const length = std.mem.readInt(u32, remaining.*[0..4], .little);
    // check payload fits before slicing, empty field allowed
    if (length > remaining.len - 4) return error.InvalidData;
    const field = remaining.*[4..][0..length];
    remaining.* = remaining.*[4 + @as(usize, length)..];
    return field;
}
