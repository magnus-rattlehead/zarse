//! read one audio stream; shared framing assembles packets
const std = @import("std");
const Source = @import("../../Source.zig");
const Error = @import("../../types.zig").Error;
const framing = @import("framing.zig");
const Self = @This();

pub const Location = framing.Location;
/// borrowed assembled packet and location of its first segment
pub const Packet = struct { bytes: []const u8, location: Location };
/// borrowed packet with index into current page metadata
pub const PacketView = struct {
    bytes: []const u8,
    location: Location,
    index: usize,
};
allocator: std.mem.Allocator,
input: Input,
limit: usize,
stream: ?framing.Stream = null,
sync: ?framing.Sync = null,
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

/// reserve a partial packet plus one page and its laces
/// cleanup is required even on failure
pub fn initBuffers(self: *Self, capacity: usize) Error!void {
    if (self.stream != null or self.sync != null) return error.InvalidState;
    const packetCapacity = @min(capacity, self.limit);
    if (packetCapacity == 0) return error.LimitExceeded;
    // retain one partial packet while inserting maximum size page
    const bodyCapacity = std.math.add(
        usize,
        packetCapacity,
        framing.maxBodyBytes,
    )
        catch return error.LimitExceeded;
    self.stream = framing.Stream.init(self.allocator, 0)
        catch |err| return mapError(err);
    self.stream.?.reserve(bodyCapacity, packetCapacity / 255 + 257)
        catch |err| return mapError(err);
    // preallocated framing storage cannot grow during playback
    self.stream.?.bounded = true;
    self.stream.?.maxPacketBytes = packetCapacity;
    self.sync = framing.Sync.init(self.allocator);
    self.sync.?.reserve(framing.maxPageBytes) catch |err| return mapError(err);
    self.sync.?.bounded = true;
}
/// read exact bytes into synchronizer, map framing errors to shared errors
fn required(self: *Self, size: usize) Error!void {
    const buffer = self.sync.?.buffer(size) catch |err| return mapError(err);
    try self.input.required(buffer[0..size]);
    self.sync.?.commit(size) catch |err| return mapError(err);
}
/// convert framing overflow into configured resource limit error
fn mapError(err: framing.Error) Error {
    return switch (err) {
        error.Overflow => error.LimitExceeded,
        else => |other| other,
    };
}
/// borrow packets completed on one page; reject missing stream ends
/// final fragments, and chained or multiplexed streams
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
    const offset = self.input.position;
    self.sync.?.reset();
    // read the fixed header, lace table, then body
    try self.required(27);
    const header = self.sync.?.data.items[0..27];
    if (
        !std.mem.eql(u8, header[0..4], "OggS")
            or header[4] != 0
            or header[5] & ~@as(u8, 7) != 0
    )
        return error.InvalidData;
    const segmentCount = header[26];
    try self.required(segmentCount);
    var length: usize = 0;
    // sum lace lengths, including fragments across pages
    for (self.sync.?.data.items[27..]) |segment| length += segment;
    try self.required(length);
    // verify the page checksum before accepting it
    const page = switch (self.sync.?.pageSeek()) {
        .page => |page| page,
        else => return error.InvalidData,
    };
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
        self.stream.?.serial = page.serial();
    }
    self.sequence = page.sequence() +% 1;
    if (!self.resuming) {
        // the continuation flag must match pending data outside a seek
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
    if (self.discardingPacket) {
        for (page.segments()) |segment| {
            if (segment < 255) {
                self.discardingPacket = false;
                break;
            }
        }
    }
    self.stream.?.pageIn(
        page,
        .{
            .offset = offset,
            .startSegment = self.skipSegments,
            .resuming = self.resuming,
        },
    )
        catch |err| return mapError(err);
    // collect completed packets while leaving trailing fragment queued
    while (true)
        switch (self.stream.?.packetOut()) {
            .needInput => break,
            .hole => return error.InvalidData,
            .packet => |packet| {
                self.packets[self.count] = .{
                    .bytes = packet.bytes,
                    .location = packet.location orelse return error.InvalidData,
                };
                self.count += 1;
            },
        };
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
    return if (self.stream) |*stream| stream.hasPartialPacket() else false;
}
/// whether current page has unread completed packets
pub fn hasPacket(self: *const Self) bool {
    return self.packetCursor < self.count;
}
/// borrow next completed packet until next page load or seek
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
/// borrow packet bytes by current page index
pub fn bytes(self: *const Self, index: usize) []const u8 {
    return self.packets[index].bytes;
}
/// reset assembly and resume at indexed packet boundary
pub fn seek(self: *Self, location: Location) Error!void {
    try self.input.seek(location.offset);
    self.stream.?.reset();
    self.stream.?.nextSequence = location.sequence;
    self.sync.?.reset();
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
/// free packet assembly and synchronizer storage, leave source open
pub fn deinit(self: *Self) void {
    if (self.stream) |*stream| stream.deinit();
    if (self.sync) |*sync| sync.deinit();
    self.stream = null;
    self.sync = null;
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
    /// fill the buffer or report truncated input
    pub fn required(self: *Input, out: []u8) Error!void {
        if (try self.read(out) != out.len) return error.TruncatedData;
    }
    /// seek absolutely and disable prefix replay; requires seek support
    pub fn seek(self: *Input, offset: u64) Error!void {
        try self.source.seekTo(offset);
        self.prefixPosition = 4;
        self.position = offset;
    }
};
