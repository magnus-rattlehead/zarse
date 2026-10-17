//! ogg framing derived from libogg 1.3.6
//! pages and packets borrow storage; stream state owns allocations
const std = @import("std");

pub const Error = std.mem.Allocator.Error
    || error{ InvalidData, InvalidState, Overflow, LimitExceeded };
// a page holds at most 255 segments of 255 bytes each
pub const maxBodyBytes = 255 * 255;
// 27 fixed header bytes plus up to 255 laces
pub const maxPageBytes = 27 + 255 + maxBodyBytes;
pub const Crc = std.hash.crc.Crc(
    u32,
    .{
        .polynomial = 0x04c11db7,
        .initial = 0,
        .reflect_input = false,
        .reflect_output = false,
        .xor_output = 0,
    },
);

/// borrow a page with validated layout
/// verify checksums separately or use the synchronizer
pub const Page = struct {
    header: []const u8,
    body: []const u8,

    /// validate borrowed page layout, checksum and version checked separately
    pub fn init(header: []const u8, body: []const u8) Error!Page {
        // header, signature 4 bytes, version 1 bytes, flags 1 bytes, granule 8 bytes
        // serial 4 bytes, sequence 4 bytes, crc 4 bytes, segment count 1 bytes
        if (header.len < 27 or !std.mem.eql(u8, header[0..4], "OggS"))
            return error.InvalidData;
        // byte 26 counts laces after the fixed header
        if (header.len != 27 + @as(usize, header[26])) return error.InvalidData;
        var length: usize = 0;
        for (header[27..]) |segment| length += segment;
        if (body.len != length) return error.InvalidData;
        return .{ .header = header, .body = body };
    }
    /// encoded page version, supported version is zero
    pub fn version(self: Page) u8 {
        return self.header[4];
    }
    /// continuation, beginning and end flags from page header
    pub fn flags(self: Page) u8 {
        // bits 0, 1 and 2 mark continuation, stream start, and stream end
        return self.header[5];
    }
    /// first segment continues packet begun on earlier page
    pub fn isContinued(self: Page) bool {
        return self.flags() & 1 != 0;
    }
    /// page starts logical stream
    pub fn isBeginning(self: Page) bool {
        return self.flags() & 2 != 0;
    }
    /// page ends logical stream
    pub fn isEnd(self: Page) bool {
        return self.flags() & 4 != 0;
    }
    /// codec end position for last completed packet, negative one if none
    pub fn granulePosition(self: Page) i64 {
        // multibyte header fields are little endian
        return std.mem.readInt(i64, self.header[6..14], .little);
    }
    /// identifier of logical stream owning this page
    pub fn serial(self: Page) u32 {
        // the serial identifies the logical stream
        return std.mem.readInt(u32, self.header[14..18], .little);
    }
    /// wrapping page sequence within logical stream
    pub fn sequence(self: Page) u32 {
        // page sequence numbers wrap at 32 bits
        return std.mem.readInt(u32, self.header[18..22], .little);
    }
    /// borrow lacing table, each byte gives segment length
    pub fn segments(self: Page) []const u8 {
        return self.header[27..];
    }
    /// count terminating laces, includes packets begun on previous page
    pub fn packetCount(self: Page) usize {
        var count: usize = 0;
        // 255 continues a packet; a smaller lace ends it
        // exact multiples of 255 need a zero terminator
        for (self.segments()) |segment| count += @intFromBool(segment < 255);
        return count;
    }
    /// calculate crc with a zero checksum field; leave input unchanged
    pub fn checksum(self: Page) u32 {
        var crc = Crc.init();
        crc.update(self.header[0..22]);
        crc.update(&.{ 0, 0, 0, 0 });
        crc.update(self.header[26..]);
        crc.update(self.body);
        return crc.final();
    }
    /// compare encoded crc with calculation over header and body
    pub fn verifyChecksum(self: Page) bool {
        return self.checksum() == std.mem.readInt(
                u32,
                self.header[22..26],
                .little,
            );
    }
    /// the mutable header must be this page borrowed header
    pub fn setChecksum(self: Page, header: []u8) Error!void {
        if (header.ptr != self.header.ptr or header.len != self.header.len)
            return error.InvalidData;
        std.mem.writeInt(u32, header[22..26], self.checksum(), .little);
    }
};

/// absolute page offset, sequence and first lace for seeking to packet
pub const Location = struct { offset: u64, sequence: u32, segment: u8 };
/// borrowed encoded bytes and logical stream timing markers
pub const Packet = struct {
    bytes: []const u8,
    beginning: bool = false,
    end: bool = false,
    granule: i64 = -1,
    number: i64 = 0,
    location: ?Location = null,
};

/// synchronize pages; report each gap once; views expire on buffer access
/// reserve, reset, or cleanup, but survive page extraction
pub const Sync = struct {
    allocator: std.mem.Allocator,
    data: std.ArrayList(u8) = .empty,
    // bytes already extracted, removed on next buffer access
    returned: usize = 0,
    unsynced: bool = false,
    headerBytes: usize = 0,
    bodyBytes: usize = 0,
    bounded: bool = false,
    // writable bytes remaining in current buffer reservation
    committedCapacity: usize = 0,

    /// page scan result, skipped byte count describes one search step
    pub const SeekResult = union(enum) {
        needInput,
        skipped: usize,
        page: Page,
    };
    /// queued output or need for more input, hole means missing data
    pub const Result = union(enum) { needInput, hole, page: Page };

    /// create empty synchronizer, storage allocated on demand
    pub fn init(allocator: std.mem.Allocator) Sync {
        return .{ .allocator = allocator };
    }
    /// free synchronization storage and clear parser state
    pub fn deinit(self: *Sync) void {
        self.data.deinit(self.allocator);
        self.data = .empty;
        self.reset();
    }
    /// discard buffered pages and sync state, keep capacity
    pub fn reset(self: *Sync) void {
        self.data.clearRetainingCapacity();
        self.returned = 0;
        self.unsynced = false;
        self.headerBytes = 0;
        self.bodyBytes = 0;
        self.committedCapacity = 0;
    }
    /// reserve synchronization bytes, may invalidate borrowed views
    pub fn reserve(self: *Sync, bytes: usize) Error!void {
        try self.data.ensureTotalCapacityPrecise(self.allocator, bytes);
    }
    /// borrow writable space; commit only the bytes filled
    pub fn buffer(self: *Sync, size: usize) Error![]u8 {
        // retain unread bytes before exposing writable tail
        if (self.returned != 0) {
            const remaining = self.data.items[self.returned..];
            std.mem.copyForwards(
                u8,
                self.data.items[0..remaining.len],
                remaining,
            );
            self.data.items.len = remaining.len;
            self.returned = 0;
        }
        const needed = std.math.add(usize, self.data.items.len, size)
            catch return error.Overflow;
        // bounded playback rejects growth, unbounded sync reserves extra headroom
        if (needed > self.data.capacity) {
            if (self.bounded) return error.LimitExceeded;
            const capacity = std.math.add(usize, needed, 4096)
                catch return error.Overflow;
            try self.reserve(capacity);
        }
        self.committedCapacity = self.data.capacity - self.data.items.len;
        return self.data.unusedCapacitySlice();
    }
    /// publish filled bytes from most recent buffer reservation
    pub fn commit(self: *Sync, size: usize) Error!void {
        if (size > self.committedCapacity) return error.InvalidState;
        self.data.items.len += size;
        self.committedCapacity -= size;
    }
    /// return verified page, skipped byte count or request for more input
    pub fn pageSeek(self: *Sync) SeekResult {
        const bytes = self.data.items[self.returned..];
        if (bytes.len < 27) return .needInput;
        // cache complete header lengths while waiting for remaining body bytes
        if (self.headerBytes == 0 and std.mem.eql(u8, bytes[0..4], "OggS")) {
            const headerLength = 27 + @as(usize, bytes[26]);
            if (bytes.len < headerLength) return .needInput;
            self.headerBytes = headerLength;
            self.bodyBytes = 0;
            for (bytes[27..headerLength]) |segment| self.bodyBytes += segment;
        }
        if (self.headerBytes != 0) {
            const length = self.headerBytes + self.bodyBytes;
            if (bytes.len < length) return .needInput;
            const page = Page{
                .header = bytes[0..self.headerBytes],
                .body = bytes[self.headerBytes..length],
            };
            // advance logical cursor, returned page keeps borrowing storage
            if (page.verifyChecksum()) {
                self.returned += length;
                self.unsynced = false;
                self.headerBytes = 0;
                self.bodyBytes = 0;
                return .{ .page = page };
            }
        }
        self.headerBytes = 0;
        self.bodyBytes = 0;
        // search next possible capture prefix after invalid header or checksum
        const skip = if (std.mem.indexOfScalar(u8, bytes[1..], 'O')) |index|
            index + 1
        else
            bytes.len;
        self.returned += skip;
        return .{ .skipped = skip };
    }
    /// return verified page or report one hole per contiguous sync loss
    pub fn pageOut(self: *Sync) Result {
        while (true)
            switch (self.pageSeek()) {
                .needInput => return .needInput,
                .page => |page| return .{ .page = page },
                .skipped => {
                    // report first gap only, suppress repeats until valid page restores sync
                    if (!self.unsynced) {
                        self.unsynced = true;
                        return .hole;
                    }
                },
            };
    }
};

/// encode or decode one logical stream; use separate instances
/// views expire on insertion, reserve, reset, or cleanup
/// encoded headers also expire on the next page output
pub const Stream = struct {
    /// segment length in low byte plus internal start, end and hole flags
    const Lace = struct {
        value: u16,
        granule: i64 = -1,
        location: ?Location = null,
    };
    allocator: std.mem.Allocator,
    serial: u32,
    body: std.ArrayList(u8) = .empty,
    laces: std.ArrayList(Lace) = .empty,
    // bytes belonging to returned packets or pages, removed on compact
    bodyReturned: usize = 0,
    laceReturned: usize = 0,
    // lace boundary after last complete packet, remainder needs more input
    completeLaces: usize = 0,
    nextSequence: ?u32 = 0,
    packetNumber: i64 = 0,
    granule: i64 = 0,
    beginning: bool = false,
    ended: bool = false,
    bounded: bool = false,
    maxPacketBytes: usize = std.math.maxInt(usize),
    header: [282]u8 = undefined,
    // one state cannot mix encoding and decoding before reset
    mode: enum { unset, encode, decode } = .unset,

    /// queued output or need for more input, hole means missing data
    pub const Result = union(enum) { needInput, hole, packet: Packet };
    /// optional source position and indexed resume boundary
    pub const PageOptions = struct {
        /// source offset for packet locations
        offset: ?u64 = null,
        /// resume at a packet boundary after validating the full page
        startSegment: u8 = 0,
        resuming: bool = false,
    };

    /// allocate stream body and laces for one logical stream
    pub fn init(allocator: std.mem.Allocator, serial: u32) Error!Stream {
        var self = Stream{ .allocator = allocator, .serial = serial };
        errdefer self.deinit();
        try self.reserve(16 * 1024, 1024);
        return self;
    }
    /// free body and lace storage, reset stream queues
    pub fn deinit(self: *Stream) void {
        self.body.deinit(self.allocator);
        self.laces.deinit(self.allocator);
        self.body = .empty;
        self.laces = .empty;
        self.reset();
    }
    /// reserve body and lace capacity, may invalidate borrowed views
    pub fn reserve(
        self: *Stream,
        bodyBytes: usize,
        segments: usize,
    ) Error!void {
        try self.body.ensureTotalCapacityPrecise(self.allocator, bodyBytes);
        try self.laces.ensureTotalCapacityPrecise(self.allocator, segments);
    }
    /// discard queued data and stream mode, keep serial and capacity
    pub fn reset(self: *Stream) void {
        self.body.clearRetainingCapacity();
        self.laces.clearRetainingCapacity();
        self.bodyReturned = 0;
        self.laceReturned = 0;
        self.completeLaces = 0;
        self.nextSequence = null;
        self.packetNumber = 0;
        self.granule = 0;
        self.beginning = false;
        self.ended = false;
        self.mode = .unset;
    }
    /// reset queues and choose new logical stream serial
    pub fn resetSerial(self: *Stream, serial: u32) void {
        self.reset();
        self.serial = serial;
    }
    /// whether end of stream has been queued or observed
    pub fn isEnd(self: *const Stream) bool {
        return self.ended;
    }
    /// whether trailing laces belong to unfinished packet
    pub fn hasPartialPacket(self: *const Stream) bool {
        return self.laces.items.len > self.completeLaces;
    }
    /// sum byte lengths of trailing unfinished packet
    pub fn partialBytes(self: *const Stream) usize {
        var length: usize = 0;
        for (self.laces.items[self.completeLaces..]) |lace|
            length += lace.value & 255;
        return length;
    }
    /// discard returned body and laces, preserve queued complete and partial packets
    fn compact(self: *Stream) void {
        if (self.bodyReturned != 0) {
            const remaining = self.body.items[self.bodyReturned..];
            std.mem.copyForwards(
                u8,
                self.body.items[0..remaining.len],
                remaining,
            );
            self.body.items.len = remaining.len;
            self.bodyReturned = 0;
        }
        if (self.laceReturned != 0) {
            const remaining = self.laces.items[self.laceReturned..];
            std.mem.copyForwards(
                Lace,
                self.laces.items[0..remaining.len],
                remaining,
            );
            self.laces.items.len = remaining.len;
            self.completeLaces -= self.laceReturned;
            self.laceReturned = 0;
        }
    }
    /// enforce reserved capacity in bounded mode, grow otherwise
    fn ensure(self: *Stream, bodyBytes: usize, segments: usize) Error!void {
        if (self.bounded) {
            if (
                bodyBytes > self.body.capacity
                    or segments > self.laces.capacity
            )
                return error.LimitExceeded;
        } else {
            try self.body.ensureTotalCapacity(self.allocator, bodyBytes);
            try self.laces.ensureTotalCapacity(self.allocator, segments);
        }
    }
    /// copy a packet; input must not alias stream storage
    pub fn packetIn(self: *Stream, packet: Packet) Error!void {
        try self.slicesIn(&.{packet.bytes}, packet.end, packet.granule);
    }
    /// copy packet slices; input must not alias stream storage
    pub fn slicesIn(
        self: *Stream,
        slices: []const []const u8,
        end: bool,
        granule: i64,
    ) Error!void {
        if (self.mode == .decode) return error.InvalidState;
        var bytes: usize = 0;
        for (slices) |slice|
            bytes = std.math.add(usize, bytes, slice.len)
                catch return error.Overflow;
        if (bytes > self.maxPacketBytes) return error.LimitExceeded;
        // terminating lace required even when packet size multiple of 255
        const segments = bytes / 255 + 1;
        self.compact();
        const bodyLength = std.math.add(usize, self.body.items.len, bytes)
            catch return error.Overflow;
        const laceLength = std.math.add(usize, self.laces.items.len, segments)
            catch return error.Overflow;
        const number = std.math.add(i64, self.packetNumber, 1)
            catch return error.Overflow;
        try self.ensure(bodyLength, laceLength);
        self.mode = .encode;
        for (slices) |slice| self.body.appendSliceAssumeCapacity(slice);
        // split payload into laces, mark first lace and retain final granule
        for (0..segments) |index|
            self.laces.appendAssumeCapacity(
                .{
                    .value = (if (index + 1 == segments)
                        @as(u16, @intCast(bytes % 255))
                    else
                        255)
                        | (if (index == 0) @as(u16, 0x100) else 0),
                    .granule = if (index + 1 == segments)
                        granule
                    else
                        self.granule,
                },
            );
        self.granule = granule;
        self.packetNumber = number;
        self.ended = self.ended or end;
    }
    /// check layout here; verify checksums before insertion
    pub fn pageIn(
        self: *Stream,
        incoming: Page,
        options: PageOptions,
    ) Error!void {
        if (self.mode == .encode) return error.InvalidState;
        const page = try Page.init(incoming.header, incoming.body);
        if (page.serial() != self.serial or page.version() != 0)
            return error.InvalidData;
        if (options.startSegment > page.segments().len)
            return error.InvalidData;
        if (options.startSegment != 0 and !options.resuming)
            return error.InvalidState;
        self.compact();
        errdefer self.reset();
        // allocate before changing queues so failures remain resettable
        const bodyLength = std.math.add(
            usize,
            self.body.items.len,
            page.body.len,
        )
            catch return error.Overflow;
        const laceLength = std.math.add(
            usize,
            self.laces.items.len,
            page.segments().len + 1,
        )
            catch return error.Overflow;
        try self.ensure(bodyLength, laceLength);
        self.mode = .decode;
        if (
            self.nextSequence == null
                or page.sequence() != self.nextSequence.?
        ) {
            // discard unfinished packets after a gap and queue a hole marker
            const partial = self.partialBytes();
            self.body.items.len -= partial;
            self.laces.items.len = self.completeLaces;
            if (self.nextSequence != null) {
                self.laces.appendAssumeCapacity(.{ .value = 0x400 });
                self.completeLaces += 1;
            }
        }
        // indexed seek skips bytes preceding chosen packet start
        var segment: usize = options.startSegment;
        var cursor: usize = 0;
        for (page.segments()[0..segment]) |length| cursor += length;
        var bos = page.isBeginning();
        if (
            !options.resuming
                and page.isContinued()
                and !self.hasPartialPacket()
        ) {
            // skip an incomplete packet tail through its terminating lace
            bos = false;
            while (segment < page.segments().len) {
                const length = page.segments()[segment];
                cursor += length;
                segment += 1;
                if (length < 255) break;
            }
        }
        var partial = self.partialBytes();
        var saved: ?usize = null;
        for (page.segments()[segment..], segment..) |length, index| {
            partial = std.math.add(usize, partial, length)
                catch return error.Overflow;
            if (partial > self.maxPacketBytes) return error.LimitExceeded;
            // record location only on first segment, packet may span pages
            const start = !self.hasPartialPacket();
            self.body.appendSliceAssumeCapacity(page.body[cursor..][0..length]);
            cursor += length;
            self.laces.appendAssumeCapacity(
                .{
                    .value = @as(u16, length)
                        | (if (bos) @as(u16, 0x100) else 0),
                    .location = if (start and options.offset != null) .{
                        .offset = options.offset.?,
                        .sequence = page.sequence(),
                        .segment = @intCast(index),
                    } else null,
                },
            );
            bos = false;
            if (length < 255) {
                saved = self.laces.items.len - 1;
                self.completeLaces = self.laces.items.len;
                partial = 0;
            }
        }
        // only the last completed packet receives the page granule
        if (saved) |index|
            self.laces.items[index].granule = page.granulePosition();
        if (page.isEnd()) {
            self.ended = true;
            if (self.laces.items.len != 0)
                self.laces.items[self.laces.items.len - 1].value |= 0x200;
        }
        self.nextSequence = page.sequence() +% 1;
    }
    /// peeking consumes hole markers but leaves packets queued
    pub fn packetPeek(self: *Stream) Result {
        return self.extract(false);
    }
    /// borrow and consume one complete packet or hole marker
    pub fn packetOut(self: *Stream) Result {
        return self.extract(true);
    }
    /// borrow one packet; advance consumes it; peeking also consumes holes
    fn extract(self: *Stream, advance: bool) Result {
        // wait for a complete packet; trailing fragments need another page
        if (self.mode != .decode or self.laceReturned >= self.completeLaces)
            return .needInput;
        const first = self.laces.items[self.laceReturned];
        // 0x400 marks missing pages; consume the marker even when peeking
        if (first.value & 0x400 != 0) {
            self.laceReturned += 1;
            self.packetNumber +%= 1;
            return .hole;
        }
        var index = self.laceReturned;
        var bytes: usize = 0;
        var end = false;
        // sum lace lengths, 255 continues a packet, anything smaller ends it
        while (true) : (index += 1) {
            const lace = self.laces.items[index];
            bytes += lace.value & 255;
            // 0x200 marks end of stream
            end = end or lace.value & 0x200 != 0;
            if (lace.value & 255 < 255) break;
        }
        // borrow contiguous assembled packet bytes
        const packet = Packet{
            .bytes = self.body.items[self.bodyReturned..][0..bytes],
            // 0x100 marks the first packet in decode mode
            .beginning = first.value & 0x100 != 0,
            .end = end,
            // only the last packet completed on a page gets its granule
            .granule = self.laces.items[index].granule,
            .number = self.packetNumber,
            // retain the packet start for seeking
            .location = first.location,
        };
        if (advance) {
            // consume bytes and laces; peeking leaves both cursors unchanged
            self.bodyReturned += bytes;
            self.laceReturned = index + 1;
            self.packetNumber +%= 1;
        }
        return .{ .packet = packet };
    }
    /// borrow next ready encoded page using default fill target
    pub fn pageOut(self: *Stream) ?Page {
        return self.pageOutFill(4096);
    }
    /// emit encoded page when ready, force first or final page
    pub fn pageOutFill(self: *Stream, fill: usize) ?Page {
        return self.output(self.ended or !self.beginning, fill);
    }
    /// force next encoded page using default fill target
    pub fn flush(self: *Stream) ?Page {
        return self.flushFill(4096);
    }
    /// force encoded page even below requested fill target
    pub fn flushFill(self: *Stream, fill: usize) ?Page {
        return self.output(true, fill);
    }
    /// build borrowed page when ready, force permits output below fill target
    fn output(self: *Stream, force: bool, fill: usize) ?Page {
        if (self.mode != .encode or self.laces.items.len == 0) return null;
        // at most 255 laces per page
        const maximum = @min(self.laces.items.len, 255);
        var count: usize = 0;
        // negative one means no packet finishes on this page
        var granule: i64 = -1;
        var forced = force;
        if (!self.beginning) {
            // first page, take the first packet, up to the lace limit
            // beginning becomes true after emission
            granule = 0;
            while (count < maximum) {
                const value = self.laces.items[count].value & 255;
                count += 1;
                if (value < 255) break;
            }
        } else {
            var accumulated: usize = 0;
            var packets: usize = 0;
            var justCompleted: usize = 0;
            while (count < maximum) : (count += 1) {
                // prefer a packet boundary past fill with at least four
                // packets
                if (accumulated > fill and justCompleted >= 4) {
                    forced = true;
                    break;
                }
                const lace = self.laces.items[count];
                accumulated += lace.value & 255;
                // only completed packets qualify as stopping points
                if (lace.value & 255 < 255) {
                    granule = lace.granule;
                    packets += 1;
                    justCompleted = packets;
                } else justCompleted = 0;
            }
            // a full lace table forces output, even mid packet
            if (count == 255) forced = true;
        }
        // keep queued data until ready or forced
        if (!forced) return null;
        // 27 fixed bytes, then one byte per lace
        const header = self.header[0..27 + count];
        @memcpy(header[0..4], "OggS");
        header[4] = 0; // ogg version
        // flags, continuation bit 0, stream start bit 1, stream end bit 2
        // unset 0x100 means continuation, final page marks stream end
        header[5] = @as(
            u8,
            if (self.laces.items[0].value & 0x100 == 0) 1 else 0,
        )
            | @as(u8, if (!self.beginning) 2 else 0)
            | @as(
                u8,
                if (self.ended and count == self.laces.items.len) 4 else 0,
            );
        // little endian, granule at byte 6, serial at byte 14, sequence at byte 18
        std.mem.writeInt(i64, header[6..14], granule, .little);
        std.mem.writeInt(u32, header[14..18], self.serial, .little);
        const sequence = self.nextSequence orelse 0;
        std.mem.writeInt(u32, header[18..22], sequence, .little);
        self.nextSequence = sequence +% 1;
        // zero four crc bytes at byte 22 for calculation; byte 26 counts laces
        @memset(header[22..26], 0);
        header[26] = @intCast(count);
        var length: usize = 0;
        for (self.laces.items[0..count], 0..) |lace, index| {
            // store lengths only; discard internal flags
            header[27 + index] = @truncate(lace.value);
            length += lace.value & 255;
        }
        // borrow the body and checksum header, laces, and body
        const page = Page{
            .header = header,
            .body = self.body.items[self.bodyReturned..][0..length],
        };
        std.mem.writeInt(u32, header[22..26], page.checksum(), .little);
        // remove laces; leave borrowed body bytes in place until compaction
        const remaining = self.laces.items[count..];
        std.mem.copyForwards(
            Lace,
            self.laces.items[0..remaining.len],
            remaining,
        );
        self.laces.items.len = remaining.len;
        self.bodyReturned += length;
        self.beginning = true;
        return page;
    }
};
