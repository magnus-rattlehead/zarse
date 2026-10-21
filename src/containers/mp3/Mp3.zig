//! read mp3 frames, metadata, trim bounds, and seek positions
//! source: https://github.com/lieff/minimp3
//! commit: ea99364f61c14656440e8d77e9c233ccf3124633 (cc0-1.0).
const std = @import("std");
const types = @import("../../types.zig");
const input_helpers = @import("../../input.zig");
const Source = @import("../../Source.zig");
const PacketSource = @import("../../PacketSource.zig");
const framing = @import("framing.zig");
const SeekIndex = @import("../seek_index.zig").Index(u64);
const Self = @This();

pub const Error = types.Error;

allocator: std.mem.Allocator,
source: Source,
/// true only when initialization built an index using seek and size support
seekable: bool = false,
options: types.Options,
buffer: []u8 = &.{},
// valid buffer bytes, unread audio starts at consumed
length: usize = 0,
consumed: usize = 0,
// absolute source byte offset represented by buffer start
offset: u64 = 0,
// untrimmed pcm position after emitted encoded frames
frame: u64 = 0,
free_format_bytes: ?u16 = null,
metadata: types.Info,
presentation: types.Presentation = .{},
/// adjusted trailing pcm padding, retained even when xing omits a frame count
end_padding: u32 = 0,
index: SeekIndex = .{},
configured: bool = false,
eof: bool = false,
// trailing tags stripped once after physical eof
tail_checked: bool = false,

/// optional declared frame count and adjusted presentation trims
const VbrTag = struct {
    encoded_frames: ?u32 = null,
    /// trim counts in pcm frames, including the decoder delay adjustment
    delay: u32 = 0,
    padding: i32 = 0,
};

/// recognize id3v2 or possible layer iii header, full validation during init
pub fn probe(prefix: []const u8) bool {
    if (prefix.len < 4) return false;
    if (std.mem.startsWith(u8, prefix, "ID3")) return true;

    // share header validation with frame reading
    _ = framing.Header.parse(prefix) catch return false;
    return true;
}

/// open borrowed input after four byte prefix, retain first audio frame
/// scan seekable input for frame positions, errors may advance input
pub fn init(
    allocator: std.mem.Allocator,
    source: Source,
    prefix: [4]u8,
    options: types.Options,
) Error!PacketSource {
    const capacity = options.bufferCapacity();
    if (capacity < 10) return error.LimitExceeded; // complete id3v2 header
    const self = try allocator.create(Self);
    self.* = .{
        .allocator = allocator,
        .source = source,
        .options = options,
        .metadata = .{
            .format = .mp3,
            .sample_rate = 0,
            .channels = 0,
            .total_frames = null,
        },
    };
    errdefer self.deinit();
    self.buffer = try allocator.alloc(u8, capacity);
    @memcpy(self.buffer[0..prefix.len], &prefix);
    self.length = prefix.len;
    try self.skipId3v2(); // skip song metadata

    var first = try self.initialFrame();
    self.free_format_bytes = first.free_format_bytes;
    const header = try framing.Header.parse(
        self.buffer[self.consumed..self.length],
    ); // copy validated four byte header
    self.metadata.sample_rate = header.sampleRate();
    self.metadata.channels = header.channels();

    // inspect first frame for xing or info stream metadata
    if (
        try readVbrTag(self.buffer[self.consumed..][0..first.length], header)
    ) |tag| {
        // skip encoded metadata frame
        self.consumed += first.length;
        if (tag.encoded_frames) |count| { // count encoded mp3 frames
            self.metadata.total_frames = @as(u64, count)
                * header.frameSamples();
        }
        // delay includes encoder leading samples and decoder alignment adjustment
        self.presentation.start = tag.delay;
        self.end_padding = @intCast(@max(0, tag.padding));
        // follow minimp3 guarded trim subtraction; a nonzero playable count
        // limits playback; zero leaves reading uncapped until physical eof
        if (self.metadata.total_frames) |total| {
            const padding: u64 = self.end_padding;
            var playable = total;
            if (playable >= tag.delay) playable -= tag.delay;
            if (padding != 0 and playable >= padding) playable -= padding;
            if (playable != 0)
                self.presentation.end = @as(u64, tag.delay) + playable;
        }
        // locate first actual audio frame after metadata frame
        first = try self.initialFrame();
        const audio_header = try framing.Header.parse(
            self.buffer[self.consumed..self.length],
        );
        if (
            !header.compatible(audio_header)
                or header.channels() != audio_header.channels()
        )
            return error.InvalidData;
        // retain for later frame reading
        self.free_format_bytes = first.free_format_bytes;
    }
    // sequential sources continue from current buffer
    const seekable = source.seek_to != null and try source.getSize() != null;
    if (seekable) try self.buildIndex();
    self.seekable = seekable;
    return .{
        .backend = .{ .mp3 = self },
        .encoding = .mp3,
        .format = .{
            .sample_rate = self.metadata.sample_rate,
            .channels = self.metadata.channels,
        },
        .presentation = self.presentation,
        .seekable = seekable,
        .max_payload_bytes = options.max_payload_bytes,
    };
}

/// accept packet framing once, one encoded frame per packet
pub fn configure(self: *Self, scheme: types.InputScheme) Error!void {
    if (self.configured) return error.InvalidState;
    // encoded frame sizes can vary so fixed size schemes like .samples and .blocks are rejected
    if (scheme != .packets) return error.UnsupportedEncoding;
    self.configured = true;
}

/// emit frame with untrimmed pcm timing, announce trim bounds before affected packets
/// packet bytes borrow input buffer until next advancing operation
pub fn next(self: *Self) Error!PacketSource.Event {
    if (!self.configured) return error.InvalidState;
    // stop at the declared playable end
    // keep the crossing packet whole since the decoder trims its pcm
    if (self.presentation.end) |limit| {
        if (self.frame >= limit) return .end;
    }
    const current = try self.bufferedFrame(0)
        orelse return self.finishStream(self.frame);
    const start = std.math.cast(i64, self.frame)
        orelse return error.InvalidData;
    const end = std.math.add(u64, self.frame, current.samples)
        catch return error.InvalidData;

    // unknown end requires lookahead to locate trailing padding
    if (self.presentation.end == null and self.end_padding != 0) {
        var ahead = current.length; // encoded byte offset from the unread cursor
        var samples: u64 = current.samples; // pcm duration inspected so far
        // enough future samples ensure padding cannot reach this packet
        const needed = @as(u64, current.samples) + self.end_padding;
        while (samples < needed) {
            // retain inspected frames; insufficient buffer size returns LimitExceeded
            const following = try self.bufferedFrame(ahead) orelse {
                // eof before enough future samples, padding may overlap this packet
                const total = std.math.add(u64, self.frame, samples)
                    catch return error.InvalidData;
                // announce trim bounds before emitting the affected packet
                return self.finishStream(total);
            };
            ahead += following.length; // move lookahead to next encoded frame
            samples += following.samples; // extend the known pcm duration
        }
    }

    // lookahead can compact the buffer; take the packet slice only afterward
    const bytes = self.buffer[self.consumed..][0..current.length];
    self.consumed += current.length;
    self.frame = end;
    return .{
        .packet = .{
            .bytes = bytes,
            .timestamp = .{
                .value = start,
                // one timestamp tick is one pcm frame, regardless of channel count
                .time_base = .{
                    .numerator = 1,
                    .denominator = self.metadata.sample_rate,
                },
            },
            .duration = current.samples,
        },
    };
}

/// encoded byte length and pcm duration per channel
const BufferedFrame = struct { length: usize, samples: u16 };

/// inspect buffered frame ahead of cursor, retain it for later emission
/// after init establishes sync, require each frame at previous frame end
fn bufferedFrame(self: *Self, ahead: usize) Error!?BufferedFrame {
    if (!try self.ensureAudio(ahead + 4)) {
        if (self.length - self.consumed != ahead) return error.TruncatedData;
        return null;
    }
    const current = framing.frameInfo(
        self.buffer[self.consumed + ahead..self.length],
        self.free_format_bytes,
    ) catch |err| {
        const bytes = self.buffer[self.consumed + ahead..self.length];
        // a trailing tag may cross a read boundary, so eof might not be known yet
        // preserve the tail and refill until skipId3v1 can inspect its footer
        if (
            std.mem.startsWith(u8, bytes, "TAG")
                or std.mem.startsWith(u8, bytes, "APET")
        ) {
            while (!self.eof) _ = try self.fill();
            _ = try self.ensureAudio(0);
            if (self.length - self.consumed == ahead) return null;
        }
        return err;
    };
    const header = current.header;
    // the current reader keeps one fixed format and one free format frame size
    // bitrate, crc presence, padding, and stereo mode may vary between packets
    if (
        header.sampleRate() != self.metadata.sample_rate
            or header.channels() != self.metadata.channels
    )
        return error.InvalidData;
    if (header.isFreeFormat() != (self.free_format_bytes != null))
        return error.InvalidData;
    // retain current packet and inspected future frames until packet emission
    if (!try self.ensureAudio(ahead + current.length))
        return error.TruncatedData;
    return .{ .length = current.length, .samples = header.frameSamples() };
}

/// preserve declared playback limit or publish discovered trim bounds once
fn finishStream(self: *Self, total: u64) Error!PacketSource.Event {
    // lookahead may reach eof while affected audio still buffered
    if (self.presentation.end == null) {
        self.metadata.total_frames = total;
        // an early eof can leave no playable samples; ignore padding that cannot
        // be subtracted, minimp3 does the same thing
        var playable = total;
        if (playable >= self.presentation.start)
            playable -= self.presentation.start;
        if (self.end_padding != 0 and playable >= self.end_padding)
            playable -= self.end_padding;
        self.presentation.end = self.presentation.start + playable;
        return .{ .presentation = self.presentation };
    }
    return .end;
}

/// return collected metadata, total_frames counts decoded pcm frames per channel
/// count includes presentation delay and padding
pub fn info(self: *const Self) types.Info {
    return self.metadata;
}

/// report whether an index supports frame seeking
pub fn canSeek(self: *const Self) bool {
    return self.seekable;
}

/// seek at or before untrimmed target with reservoir and synthesis history
/// return decoded origin, caller resets codec and discards samples before target
/// errors may advance input, requires configuration and a populated index
pub fn seek(self: *Self, frame: u64) Error!PacketSource.SeekResult {
    try PacketSource.validateSeek(
        self.configured,
        self.canSeek(),
        frame,
        0,
        self.presentation.end orelse self.metadata.total_frames,
    );
    var selected = self.index.indexAtOrBefore(frame)
        orelse return error.InvalidState;
    const entries = self.index.entries.items;
    if (entries[selected].frame > frame) return error.OutOfRange;

    // two earlier encoded frames rebuild overlap and synthesis history
    selected -|= 2;
    // layer iii can reference up to 511 bytes of earlier main data
    var reservoir: usize = 511;
    while (selected != 0 and reservoir != 0) {
        selected -= 1;
        try self.seekInput(entries[selected].location);
        const previous = try self.bufferedFrame(0)
            orelse return error.TruncatedData;
        const header = try framing.Header.parse(
            self.buffer[self.consumed..self.length],
        );
        // reservoir contains main data after header, optional crc and side information
        reservoir -|= previous.length - header.minimumFrameBytes();
    }

    const anchor = entries[selected];
    const origin = std.math.cast(i64, anchor.frame)
        orelse return error.InvalidData;
    // discard inspection bytes and restart exactly at selected encoded header
    try self.seekInput(anchor.location);
    self.frame = anchor.frame;
    return .{ .frame = origin };
}

/// seek to absolute byte offset and invalidate buffered input, leave format and trims
fn seekInput(self: *Self, offset: u64) Error!void {
    // only clear reader state after source accepts the seek
    try self.source.seekTo(offset);
    self.offset = offset;
    self.length = 0;
    self.consumed = 0;
    self.eof = false;
    self.tail_checked = false;
}

/// free input storage, the seek index, and reader state; leave input open
pub fn deinit(self: *Self) void {
    const allocator = self.allocator;
    allocator.free(self.buffer);
    self.index.deinit(allocator);
    allocator.destroy(self);
}

/// compact consumed bytes and refill; return whether new bytes were read
/// false can leave buffered bytes available for the caller to inspect
fn fill(self: *Self) Error!bool {
    return input_helpers.refill(self);
}

/// skip consecutive leading id3v2 tags, large tags consumed in chunks
fn skipId3v2(self: *Self) Error!void {
    while (try self.ensureBytes(3)) {
        if (!std.mem.eql(u8, self.buffer[self.consumed..][0..3], "ID3")) return;
        // id3v2, three signature bytes, one each for version, revision and flags, four for size
        if (!try self.ensureBytes(10)) return error.TruncatedData;
        const header = self.buffer[self.consumed..][0..10];
        const version = header[3];
        if (version < 2 or version > 4 or header[4] == 0xff)
            return error.InvalidData;
        // reserved flag bits must be zero, low 6, 5, or 4 bits by version
        const reserved: u8 = switch (version) {
            2 => 0x3f,
            3 => 0x1f,
            4 => 0x0f,
            else => unreachable,
        };
        if (header[5] & reserved != 0) return error.InvalidData;
        var payload: usize = 0;
        // synchsafe size, each byte contributes 7 bits; its high bit is zero
        for (header[6..10]) |byte| {
            if (byte & 0x80 != 0) return error.InvalidData;
            payload = (payload << 7) | byte;
        }
        // only id3v2.4 defines the optional ten byte footer
        const footer: usize = if (version == 4 and header[5] & 0x10 != 0)
            10
        else
            0;
        var remaining = 10 + payload + footer;
        while (remaining != 0) {
            // discard only available bytes, then refill for the rest of the tag
            const count = @min(remaining, self.length - self.consumed);
            self.consumed += count;
            remaining -= count;
            if (remaining != 0 and !try self.fill()) return error.TruncatedData;
        }
    }
}

/// borrow buffered audio prefix after trimming recognized trailing metadata
fn skipId3v1(bytes: []const u8) []const u8 {
    var end = bytes.len;
    // id3v1 occupies 128 bytes; an optional TAG+ extension precedes it
    if (end >= 128 and std.mem.eql(u8, bytes[end - 128..][0..3], "TAG")) {
        end -= 128;
        if (end >= 227 and std.mem.eql(u8, bytes[end - 227..][0..4], "TAG+"))
            end -= 227;
    }
    // ape footer, 8 byte signature, four version bytes, little endian four tag size bytes, ...
    // the declared size includes the 32 byte footer and must fit this slice
    if (end >= 32 and std.mem.eql(u8, bytes[end - 32..][0..8], "APETAGEX")) {
        const size = std.mem.readInt(u32, bytes[end - 20..][0..4], .little);
        if (size >= 32 and size <= end) end -= size;
    }
    return bytes[0..end];
}

/// read xing or info metadata from complete frame, null when absent
/// malformed recognized fields return error, trim counts in pcm frames per channel
fn readVbrTag(bytes: []const u8, header: framing.Header) Error!?VbrTag {
    // layout, four byte mpeg header, optional two byte crc, side information, xing or info
    var cursor = header.minimumFrameBytes();
    if (cursor > bytes.len or bytes.len - cursor < 4) return null;
    const marker = bytes[cursor..][0..4];
    if (!std.mem.eql(u8, marker, "Xing") and !std.mem.eql(u8, marker, "Info"))
        return null;
    if (bytes.len - cursor < 8) return error.InvalidData;
    // four marker bytes followed by four flag bytes, stored in big endian byte order
    // bits 1, 2, 4 and 8 select frame count, byte count, seek table, and quality
    const flags = std.mem.readInt(u32, bytes[cursor + 4..][0..4], .big);
    if (flags & ~@as(u32, 0x0f) != 0) return error.InvalidData;
    cursor += 8;
    var result: VbrTag = .{};
    if (flags & 1 != 0) {
        if (bytes.len - cursor < 4) return error.InvalidData;
        result.encoded_frames = std.mem.readInt(
            u32,
            bytes[cursor..][0..4],
            .big,
        );
        cursor += 4;
    }
    // advance over present fields in wire order to locate the encoder extension
    const fields = [_]struct { mask: u32, size: usize }{
        .{ .mask = 2, .size = 4 }, // byte count
        .{ .mask = 4, .size = 100 }, // seek table
        .{ .mask = 8, .size = 4 }, // quality field
    };
    for (fields) |field| {
        if (flags & field.mask == 0) continue;
        if (bytes.len - cursor < field.size) return error.InvalidData;
        cursor += field.size;
    }
    // follow minimp3 nonzero byte check for a lame style encoder extension
    // require its 36 byte layout
    if (cursor < bytes.len and bytes[cursor] != 0) {
        if (bytes.len - cursor < 36) return error.InvalidData;
        // three bytes starting at 21 pack 12 bits each for delay and padding
        // the middle byte holds delay low 4 bits and padding high 4 bits
        const trim = bytes[cursor + 21..][0..3];
        // match minimp3 528 synthesis delay samples plus one alignment sample
        result.delay = (@as(u32, trim[0]) << 4 | trim[1] >> 4) + 529;
        result.padding = @as(i32, (@as(u16, trim[1] & 0x0f) << 8) | trim[2])
            - 529;
    }
    return result;
}

/// index complete audio frames before configuration, restore first frame on success
/// preserve declared playback bounds, errors may advance input
fn buildIndex(self: *Self) Error!void {
    if (self.configured or self.frame != 0 or self.index.entries.items.len != 0)
        return error.InvalidState;
    // init already skipped leading tags and any xing or info metadata frame
    const start = std.math.add(u64, self.offset, self.consumed)
        catch return error.InvalidData;
    var index: SeekIndex = .{};
    // keep partial index local so failed scans release all allocated entries
    errdefer index.deinit(self.allocator);
    var total: u64 = 0;
    // scan all physical audio frames
    while (try self.bufferedFrame(0)) |current| {
        // refilling may compact buffer, calculate absolute offset afterward
        const location = std.math.add(u64, self.offset, self.consumed)
            catch return error.InvalidData;
        try index.append(
            self.allocator,
            self.options.max_index_bytes,
            .{
                .frame = total, // pcm position before this encoded frame
                .location = location, // absolute source byte offset of its header
            },
        );
        total = std.math.add(u64, total, current.samples)
            catch return error.InvalidData;
        self.consumed += current.length;
    }
    // restore first audio frame for playback
    try self.seekInput(start);
    _ = try self.bufferedFrame(0) orelse return error.TruncatedData;
    // unknown end can now be announced at opening, declared limits stay authoritative
    if (self.presentation.end == null) _ = try self.finishStream(total);
    self.index = index;
}

/// ensure contiguous unread bytes, false if eof leaves too few
/// may move buffer and invalidate borrowed slices, oversized requests return error
fn ensureBytes(self: *Self, count: usize) Error!bool {
    // lookahead must retain all bytes from unread cursor through requested end
    if (count > self.buffer.len) return error.LimitExceeded;
    while (self.length - self.consumed < count) {
        if (!try self.fill()) return false;
    }
    return true;
}

/// ensure bytes, then exclude trailing metadata once eof is known
/// recheck the available count because removing tags can shorten the slice
fn ensureAudio(self: *Self, count: usize) Error!bool {
    _ = try self.ensureBytes(count);
    // remove tags before deciding whether final audio request is complete
    if (self.eof and !self.tail_checked) {
        self.length = self.consumed
            + skipId3v1(self.buffer[self.consumed..self.length]).len;
        self.tail_checked = true;
    }
    return self.length - self.consumed >= count;
}

/// confirm a following compatible header, or an exact single frame at eof
/// free format requires three compatible headers to infer the frame size
/// return the selected frame buffer offset and length; leave it unconsumed
fn initialFrame(self: *Self) Error!framing.Match {
    while (try self.ensureAudio(4)) {
        // framing inspects bytes only, this reader decides when to request more
        switch (
            try framing.findFrame(
                self.buffer[self.consumed..self.length],
                self.free_format_bytes,
            )
        ) {
            .skip => |count| self.consumed += count,
            .need_more => |request| {
                self.consumed += request.offset;
                if (!try self.ensureAudio(request.count)) {
                    // unresolved free format may be false sync before another frame
                    if (request.resync) {
                        self.consumed += 1;
                        continue;
                    }
                    return error.TruncatedData;
                }
            },
            .frame => |current| {
                self.consumed += current.offset;
                if (!current.confirmed) {
                    // read ahead to establish source end for an exact slice fit
                    if (try self.ensureAudio(current.length + 4)) continue;
                    if (self.length - self.consumed != current.length)
                        return error.TruncatedData;
                }
                return .{
                    .offset = self.consumed,
                    .length = current.length,
                    .free_format_bytes = current.free_format_bytes,
                    .confirmed = current.confirmed,
                };
            },
        }
    }
    return if (self.length != self.consumed)
        error.TruncatedData
    else
        error.InvalidData;
}
