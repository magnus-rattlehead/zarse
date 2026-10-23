//! borrow input with optional seeking and size
//! copies share one reader, give the pipeline exclusive access until cleanup
//! keep reader, buffer and input in place, caller owns and closes input
const std = @import("std");
const types = @import("types.zig");
const Source = @This();

reader: *std.Io.Reader,
seek_to: ?*const fn (*std.Io.Reader, u64) types.SeekError!void = null,
get_size: ?*const fn (*std.Io.Reader) types.SizeError!u64 = null,

/// borrow sequential input; keep the reader, buffer and input alive while decoding
pub fn fromReader(reader: *std.Io.Reader) Source {
    return .{ .reader = reader };
}

/// borrow a file reader; streaming files decode sequentially
pub fn fromFile(reader: *std.Io.File.Reader) Source {
    return .{
        .reader = &reader.interface,
        .seek_to = fileSeek,
        .get_size = fileSize,
    };
}

/// borrow a fixed reader; keep its bytes unchanged
pub fn fromFixed(reader: *std.Io.Reader) Source {
    return .{ .reader = reader, .seek_to = fixedSeek, .get_size = fixedSize };
}

/// seek absolutely; unsupported seeks leave input unchanged
pub fn seekTo(self: Source, offset: u64) types.SourceSeekError!void {
    // missing callback leaves reader position unchanged
    const seek = self.seek_to orelse return error.Unseekable;
    try seek(self.reader, offset);
}

/// return input size, or null if unknown
pub fn getSize(self: Source) types.SizeError!?u64 {
    const size = self.get_size orelse return null;
    return size(self.reader)
        catch |err| switch (err) {
            // file may expose callbacks but still be a streaming input
            error.Unseekable => null,
            else => return err,
        };
}

/// seek the file and update its buffered position
fn fileSeek(reader: *std.Io.Reader, offset: u64) types.SeekError!void {
    // recover file reader owning this interface
    const file: *std.Io.File.Reader = @fieldParentPtr("interface", reader);
    try file.seekTo(offset);
}

/// get file size, map streaming input to unseekable
fn fileSize(reader: *std.Io.Reader) types.SizeError!u64 {
    // recover file reader owning this interface
    const file: *std.Io.File.Reader = @fieldParentPtr("interface", reader);
    return file.getSize()
        catch |err| switch (err) {
            error.Streaming => error.Unseekable,
            else => |size_error| size_error,
        };
}

/// set the cursor; reject offsets beyond the buffer
fn fixedSeek(reader: *std.Io.Reader, offset: u64) types.SeekError!void {
    if (offset > reader.end) return error.EndOfStream;
    reader.seek = @intCast(offset);
}

/// return the full buffer length
fn fixedSize(reader: *std.Io.Reader) types.SizeError!u64 {
    return reader.end;
}
