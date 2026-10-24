//! validate ogg pages and assemble packets for one audio stream
const std = @import("std");
const input_helpers = @import("../../input.zig");
const Source = @import("../../Source.zig");
const Error = @import("../../types.zig").Error;
const framing = @import("framing.zig");
const Self = @This();

pub const Location = framing.Location;
pub const Packet = struct { bytes: []const u8, location: Location };
/// packet and its index into the current page metadata
pub const PacketView = struct {
    bytes: []const u8,
    location: Location,
    index: usize,
};
allocator: std.mem.Allocator,
input: Input,
limit: usize,
assembly: []u8 = &.{},
length: usize = 0,
partial_offset: usize = 0,
partial_location: ?Location = null,
packet_capacity: usize = 0,
page_buffer: []u8 = &.{},
// logical stream identifier fixed by first page
serial: ?u32 = null,
sequence: u32 = 0,
ended: bool = false,
finished: bool = false,
resuming: bool = false,
// skip page laces before indexed packet when resuming seek
skipSegments: u8 = 0,
allowMissingPacketStart: bool = false,
// discard continued broadcast fragment whose start was unavailable
discardingPacket: bool = false,
packets: [255]Packet = undefined,
count: usize = 0,
packetCursor: usize = 0,
granule: i64 = -1,
flags: u8 = 0,

/// reserve a partial packet plus one page
/// cleanup is required even on failure
pub fn initBuffers(self: *Self, capacity: usize) Error!void {
    self.packet_capacity = @min(capacity, self.limit);
    if (self.packet_capacity == 0) return error.LimitExceeded;
    const capacity_bytes = std.math.add(
        usize,
        self.packet_capacity,
        framing.maxBodyBytes,
    )
        catch return error.LimitExceeded;
    self.assembly = try self.allocator.alloc(u8, capacity_bytes);
    self.page_buffer = try self.allocator.alloc(u8, framing.maxPageBytes);
}
/// read packets completed on one page; reject missing stream ends,
/// final fragments, chained streams and multiplexed streams
pub fn nextPage(self: *Self) Error!bool {
    if (self.finished) return false;
    self.count = 0;
    self.packetCursor = 0;
    // require physical eof after stream end, chaining unsupported
    if (self.ended) {
        var trailing: [1]u8 = undefined;
        if (try self.input.read(&trailing) != 0)
            return error.UnsupportedChunkLayout;
        self.finished = true;
        return false;
    }
    // keep only the unfinished packet from the previous page
    const remaining = self.length - self.partial_offset;
    std.mem.copyForwards(
        u8,
        self.assembly[0..remaining],
        self.assembly[self.partial_offset..self.length],
    );
    self.length = remaining;
    self.partial_offset = 0;
    const offset = self.input.position;
    // read the fixed header, lace table, then body
    const header = self.page_buffer[0..27];
    try self.input.required(header);
    if (
        !std.mem.eql(u8, header[0..4], "OggS")
            or header[4] != 0
            or header[5] & ~@as(u8, 7) != 0
    )
        return error.InvalidData;
    const headerLength = 27 + @as(usize, header[26]);
    const segments = self.page_buffer[27..headerLength];
    try self.input.required(segments);
    var length: usize = 0;
    // sum lace lengths, including fragments across pages
    for (segments) |segment| length += segment;
    const body = self.page_buffer[headerLength..][0..length];
    try self.input.required(body);
    const page = framing.Page{
        .header = self.page_buffer[0..headerLength],
        .body = body,
    };
    if (!page.verifyChecksum()) return error.InvalidData;
    self.flags = page.flags();
    self.granule = page.granulePosition();
    // reject multiplexing or restarting stream, page sequence wraps naturally
    if (self.serial) |expected| {
        if (page.serial() != expected or page.isBeginning())
            return error.UnsupportedChunkLayout;
        if (page.sequence() != self.sequence) return error.InvalidData;
    } else {
        if (!page.isBeginning() or page.isContinued() or page.sequence() != 0)
            return error.InvalidData;
        self.serial = page.serial();
    }
    self.sequence = page.sequence() +% 1;
    if (!self.resuming) {
        // continuation flag must match pending data outside a seek
        if (
            self.allowMissingPacketStart
                and !self.hasPartialPacket()
                and page.isContinued()
        )
            self.discardingPacket = true;
        if (
            page.isContinued() != (self.hasPartialPacket()
                    or self.discardingPacket)
        )
            return error.InvalidData;
    }
    self.allowMissingPacketStart = false;
    if (self.skipSegments > page.segments().len) return error.InvalidData;
    var cursor: usize = 0;
    for (segments[0..self.skipSegments]) |size| cursor += size;
    for (segments[self.skipSegments..], self.skipSegments..) |size, index| {
        const fragment = body[cursor..][0..size];
        cursor += size;
        if (self.discardingPacket) {
            if (size < 255) self.discardingPacket = false;
            continue;
        }
        if (self.partial_location == null) {
            self.partial_location = .{
                .offset = offset,
                .sequence = page.sequence(),
                .segment = @intCast(index),
            };
        }
        if (self.length - self.partial_offset + size > self.packet_capacity)
            return error.LimitExceeded;
        @memcpy(self.assembly[self.length..][0..size], fragment);
        self.length += size;
        if (size < 255) {
            self.packets[self.count] = .{
                .bytes = self.assembly[self.partial_offset..self.length],
                .location = self.partial_location.?,
            };
            self.count += 1;
            self.partial_offset = self.length;
            self.partial_location = null;
        }
    }
    self.skipSegments = 0;
    self.resuming = false;
    self.ended = page.isEnd();
    if (self.ended and (self.hasPartialPacket() or self.discardingPacket))
        return error.TruncatedData;
    // check granules even for fragments and packets skipped during seeking
    const completed = page.packetCount();
    if (
        (completed == 0 and self.granule != -1)
            or (completed != 0 and self.granule < 0)
    )
        return error.InvalidData;
    return true;
}
/// whether unfinished packet bytes need a later page
pub fn hasPartialPacket(self: *const Self) bool {
    return self.partial_location != null;
}
/// whether current page has unread completed packets
pub fn hasPacket(self: *const Self) bool {
    return self.packetCursor < self.count;
}
/// packet bytes remain valid until the next page load or seek
pub fn nextPacket(self: *Self) ?PacketView {
    if (!self.hasPacket()) return null;
    const index = self.packetCursor;
    self.packetCursor += 1;
    return .{
        .bytes = self.bytes(index),
        .location = self.packets[index].location,
        .index = index,
    };
}
pub fn bytes(self: *const Self, index: usize) []const u8 {
    return self.packets[index].bytes;
}
/// reset assembly and resume at indexed packet boundary
pub fn seek(self: *Self, location: Location) Error!void {
    try self.input.seek(location.offset);
    self.length = 0;
    self.partial_offset = 0;
    self.partial_location = null;
    self.sequence = location.sequence;
    // resume at the packet first lace, skipping earlier segments
    self.skipSegments = location.segment;
    self.resuming = true;
    self.ended = false;
    self.finished = false;
    self.allowMissingPacketStart = false;
    self.discardingPacket = false;
    self.count = 0;
    self.packetCursor = 0;
}
/// leave input open
pub fn deinit(self: *Self) void {
    self.allocator.free(self.assembly);
    self.allocator.free(self.page_buffer);
    self.assembly = &.{};
    self.page_buffer = &.{};
}

/// replay the detection prefix and track input offsets
pub const Input = struct {
    source: Source,
    /// replay bytes already consumed by format detection
    prefix: [4]u8,
    prefixPosition: usize = 0,
    /// byte offset from input start, including the prefix
    position: u64 = 0,

    /// read prefix then source; a short count means end of input
    pub fn read(self: *Input, out: []u8) Error!usize {
        var bytesRead: usize = 0;
        // replay detection bytes before reading remaining source
        if (self.prefixPosition < 4) {
            bytesRead = @min(out.len, 4 - self.prefixPosition);
            @memcpy(
                out[0..bytesRead],
                self.prefix[self.prefixPosition..][0..bytesRead],
            );
            self.prefixPosition += bytesRead;
        }
        bytesRead += try self.source.reader.readSliceShort(out[bytesRead..]);
        self.position = std.math.add(u64, self.position, bytesRead)
            catch return error.InvalidData;
        return bytesRead;
    }
    /// read prefix then source using the shared reader interface
    pub const readSliceShort = read;

    /// fill the buffer or report truncated input
    pub fn required(self: *Input, out: []u8) Error!void {
        return input_helpers.readRequired(self, out);
    }
    /// seek absolutely and disable prefix replay; requires seek support
    pub fn seek(self: *Input, offset: u64) Error!void {
        try self.source.seekTo(offset);
        self.prefixPosition = 4;
        self.position = offset;
    }
};
