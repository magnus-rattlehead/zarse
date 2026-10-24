//! read native flac frames, stream metadata, and seek positions
const std = @import("std");
const types = @import("../../types.zig");
const input_helpers = @import("../../input.zig");
const Source = @import("../../Source.zig");
const PacketSource = @import("../../PacketSource.zig");
const flac = @import("../../codecs/flac.zig");
const BitReader = @import("../../codecs/flac/bit_reader.zig");
const SeekIndex = @import("../seek_index.zig").Index(u64);
const Self = @This();

pub const Error = types.Error;

allocator: std.mem.Allocator,
source: Source,
/// true only when initialization built an index using seek and size support
seekable: bool = false,
options: types.Options,
config: types.FlacConfig = undefined,
metadata: types.Info = undefined,
buffer: []u8 = &.{},
// valid buffer bytes, unread audio starts at consumed
length: usize = 0,
consumed: usize = 0,
// absolute source byte offset represented by buffer start
offset: u64 = 0,
// first encoded frame after the signature and all metadata blocks
audio_offset: u64 = 0,
// pcm position after emitted encoded frames, one sample per channel
frame: u64 = 0,
// pcm positions paired with absolute encoded frame offsets
index: SeekIndex = .{},
// physical eof may still leave complete or partial frames in the buffer
eof: bool = false,

/// recognize native flac signature, full metadata validation during init
pub fn probe(prefix: []const u8) bool {
    return std.mem.eql(u8, prefix, "fLaC");
}

/// parse stream metadata after the four byte prefix
/// allocate packet storage and scan seekable input for frame positions
/// sequential sources start at the first audio frame, errors may advance input
pub fn init(
    allocator: std.mem.Allocator,
    source: Source,
    prefix: [4]u8,
    options: types.Options,
) Error!PacketSource {
    if (!probe(&prefix)) return error.UnsupportedFormat;
    const self = try allocator.create(Self);
    self.* = .{ .allocator = allocator, .source = source, .options = options };
    errdefer self.deinit();
    try self.readMetadata();
    self.buffer = try allocator.alloc(u8, options.bufferCapacity());
    if (self.buffer.len == 0) return error.LimitExceeded;
    // input needs both seeking and a known size before initialization scans it
    const seekable = source.seek_to != null and try source.getSize() != null;
    if (seekable) try self.buildIndex();
    self.seekable = seekable;
    return .{
        .backend = .{ .flac = self },
        .encoding = .{ .flac = self.config },
        .seekable = seekable,
    };
}

/// read mandatory streaminfo, skip other metadata without retaining payloads
/// stop at the first audio byte and record its absolute source offset
fn readMetadata(self: *Self) Error!void {
    const reader = self.source.reader;
    // shared detection already consumed the four byte fLaC signature
    var offset: u64 = 4;
    var first = true;
    while (true) {
        // metadata header, one last-block bit, seven type bits, big endian u24 length
        var header_bytes: [4]u8 = undefined;
        try input_helpers.readRequired(reader, &header_bytes);
        const header = header_bytes[0];
        const length = std.mem.readInt(u24, header_bytes[1..4], .big);
        const kind = header & 0x7f;
        // payload length excludes the four byte metadata header
        offset = std.math.add(u64, offset, @as(u64, length) + 4)
            catch return error.InvalidData;
        if (first) {
            // streaminfo must be first and occupies exactly 34 payload bytes
            if (kind != 0 or length != 34) return error.InvalidData;
            // fixed stack storage lets even a one byte source buffer supply
            // fields that the bit reader needs to inspect contiguously
            var bytes: [34]u8 = undefined;
            try input_helpers.readRequired(reader, &bytes);
            var bits: BitReader = .init(&bytes);
            self.config = types.FlacConfig.parse(&bits)
                catch |err| return flac.mapError(err);
            first = false;
        } else {
            // streaminfo cannot repeat, type 127 is forbidden
            // reserved types and optional metadata are skipped by declared length
            if (kind == 0 or kind == 127) return error.InvalidData;
            // discard incrementally so large tags and pictures need no allocation
            reader.discardAll(length) catch |err| return flac.mapError(err);
        }
        // high bit marks the last metadata block, audio follows immediately
        if (header & 0x80 != 0) break;
    }
    self.audio_offset = offset;
    self.offset = offset;
    self.metadata = .{
        .format = .flac,
        .channels = self.config.channels,
        .sample_rate = self.config.sample_rate,
        // zero sample count explicitly means unknown, discovered while reading
        .total_frames = if (self.config.sample_count == 0)
            null
        else
            self.config.sample_count,
    };
}

/// read one complete frame with pcm timing, null means validated end of input
/// packet bytes remain valid until the next advancing operation
/// used during both indexing and playback
fn takeFrame(self: *Self) Error!?types.Packet {
    while (true) {
        if (self.length == self.consumed and self.eof) {
            // clean eof requires no partial frame and the declared pcm count
            if (self.metadata.total_frames) |total| {
                if (self.frame < total) return error.TruncatedData;
                if (self.frame > total) return error.InvalidData;
            }
            // unknown totals become known after indexing or sequential playback
            self.metadata.total_frames = self.frame;
            return null;
        }
        // flac has no frame length field; walk encoded subframes and checksums
        // without reconstructing pcm to locate the exact end of this packet
        const parsed = flac.Frame.scan(
            self.config,
            self.buffer[self.consumed..self.length],
        ) catch |err| {
            if (err != error.EndOfStream) return flac.mapError(err);
            // scan exhaustion needs more bytes, physical eof makes it truncation
            if (self.eof) return error.TruncatedData;
            // preserve the partial frame, then scan again after refilling
            try self.fill();
            continue;
        };
        // fixed frame numbers and variable sample numbers must both stay contiguous
        if (parsed.firstSample() != self.frame) return error.InvalidData;
        const start = self.frame;
        const end = std.math.add(u64, start, parsed.block_size)
            catch return error.InvalidData;
        if (self.metadata.total_frames) |total| {
            // declared counts describe the whole stream, no trailing pcm trim
            if (end > total) return error.InvalidData;
        }
        const bytes = self.buffer[self.consumed..][0..parsed.encoded_bytes];
        self.consumed += bytes.len;
        self.frame = end;
        return .{ .bytes = bytes, .duration = parsed.block_size };
    }
}

/// emit a complete frame or end event after validating physical eof
/// packet bytes remain valid until the next advance
pub fn next(self: *Self) Error!PacketSource.Event {
    return if (try self.takeFrame()) |packet| .{ .packet = packet } else .end;
}

/// compact consumed bytes and refill, retain any incomplete encoded frame
/// invalidates buffer slices; a frame larger than the buffer exceeds the limit
fn fill(self: *Self) Error!void {
    // readSliceShort fills the requested space unless it reaches eof
    _ = try input_helpers.refill(self);
}

/// index complete audio frames during initialization, restore first frame on success
/// validate declared counts or discover unknown totals, errors may advance input
fn buildIndex(self: *Self) Error!void {
    while (true) {
        const start = self.frame;
        const packet = try self.takeFrame() orelse break;
        // refilling may compact the buffer, calculate absolute offset afterward
        // consumed points past this packet, subtract its length to locate the header
        const location = std.math.add(
            u64,
            self.offset,
            self.consumed - packet.bytes.len,
        )
            catch return error.InvalidData;
        // spare index capacity counts toward the configured byte limit
        try self.index.append(
            self.allocator,
            self.options.max_index_bytes,
            .{ .frame = start, .location = location },
        );
    }
    // keep the completed index and total, rewind only buffered input and pcm cursor
    try self.seekInput(self.audio_offset);
    self.frame = 0;
}

/// return stream properties, total_frames counts pcm frames per channel
/// sequential input may leave the total unknown until physical eof
pub fn info(self: *const Self) types.Info {
    return self.metadata;
}

/// seek to an indexed frame at or before target, no codec preroll required
/// return pcm origin, caller resets codec and discards samples before target
/// errors may advance input
pub fn seek(self: *Self, frame: u64) Error!PacketSource.SeekResult {
    try PacketSource.validateSeek(
        self.seekable,
        frame,
        0,
        self.metadata.total_frames,
    );
    // an empty indexed stream still permits seeking to its zero frame origin
    const entry = self.index.atOrBefore(frame)
        orelse SeekIndex.Entry{ .frame = 0, .location = self.audio_offset };
    const origin = entry.frame;
    try self.seekInput(entry.location);
    self.frame = entry.frame;
    return .{ .frame = origin };
}

/// seek to absolute byte offset, discard buffered input, retain metadata and index
fn seekInput(self: *Self, offset: u64) Error!void {
    // only clear reader state after source accepts the seek
    try self.source.seekTo(offset);
    self.offset = offset;
    self.length = 0;
    self.consumed = 0;
    self.eof = false;
}

/// leave input open
pub fn deinit(self: *Self) void {
    const allocator = self.allocator;
    self.index.deinit(allocator);
    allocator.free(self.buffer);
    allocator.destroy(self);
}
