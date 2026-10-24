//! read one opus or vorbis stream; framing handles pages and packets
//! codec headers, timing, trimming, and seeking are handled here
//! opus mapping: https://www.rfc-editor.org/rfc/rfc7845.html
const std = @import("std");
const types = @import("../../types.zig");
const Source = @import("../../Source.zig");
const PacketSource = @import("../../PacketSource.zig");
const Pages = @import("Pages.zig");
const SeekIndex = @import("../seek_index.zig").Index(Pages.Location);
const VorbisComments = @import("../VorbisComments.zig");
const Self = @This();

pub const Error = types.Error;
const codec_options = @import("build_options");
const Vorbis = if (codec_options.vorbis)
    @import("../../codecs/vorbis.zig")
else
    struct {};
allocator: std.mem.Allocator,
pages: Pages,
options: types.Options,
config: types.OpusConfig = .{},
is_vorbis: bool = false,
vorbis_headers: if (codec_options.vorbis) ?Vorbis.Headers else void = if (
    codec_options.vorbis
)
    null
else {
},
// previous vorbis transform size, zero before first overlap packet
previous_block: u16 = 0,
/// shift wire granules onto a nonnegative decode timeline
granule_shift: i128 = 0,
/// first audio page may contain only a zero output packet
initial_prime: []const u8 = &.{},
prime_location: ?Pages.Location = null,
prime_pending: bool = false,
metadata: types.Info = undefined,
/// sample interval retained after leading and trailing trimming
presentation: types.Presentation = .{},
/// granule position before the first decoded sample
origin: u64 = 0,
/// next packet untrimmed position on the granule timeline
frame: u64 = 0,
/// pair untrimmed positions with packet locations
index: SeekIndex = .{},
/// per packet sample counts for the current page
durations: [255]u16 = undefined,
headers: u8 = 0,
first_audio: bool = true,
/// send final trim bounds before packets on the end page
bounds_pending: bool = false,
seekable: bool = false,

/// recognize the prefix; validate pages when opening
pub fn probe(prefix: []const u8) bool {
    return std.mem.startsWith(u8, prefix, "OggS");
}

/// open after the four byte prefix; keep input unchanged
/// allocate packet storage and any seek index; scan seekable input
/// packets and tags must fit the configured buffer; errors may move input
pub fn init(
    allocator: std.mem.Allocator,
    source: Source,
    prefix: [4]u8,
    options: types.Options,
) Error!PacketSource {
    if (!codec_options.opus and !codec_options.vorbis)
        return error.UnsupportedEncoding;
    const self = try allocator.create(Self);
    self.* = .{
        .allocator = allocator,
        .pages = .{
            .allocator = allocator,
            .input = .{ .source = source, .prefix = prefix },
            .limit = options.max_payload_bytes,
        },
        .options = options,
    };
    errdefer self.deinit();
    try self.pages.initBuffers(options.bufferCapacity());
    self.seekable = source.seek_to != null and try source.getSize() != null;
    if (!try self.loadAudioPage()) return error.InvalidData;
    // scan packet locations and durations, then restore playback origin
    if (self.seekable) {
        if (self.prime_location) |location| {
            try self.index.append(
                allocator,
                options.max_index_bytes,
                .{ .frame = 0, .location = location },
            );
        }
        while (true) {
            while (self.pages.nextPacket()) |packet| {
                try self.index.append(
                    allocator,
                    options.max_index_bytes,
                    .{ .frame = self.frame, .location = packet.location },
                );
                self.frame = std.math.add(
                    u64,
                    self.frame,
                    self.durations[packet.index],
                )
                    catch return error.InvalidData;
            }
            if (!try self.loadAudioPage()) break;
        }
        self.metadata.total_frames = self.frame - self.origin;
        _ = try self.seekIndexed(self.origin);
    }
    return .{
        .backend = .{ .ogg = self },
        .encoding = if (self.is_vorbis) .vorbis
        else .{ .opus = self.config },
        .presentation = self.presentation,
        .decoded_origin = self.origin,
        .seekable = self.seekable,
    };
}

/// emit trim bounds, a packet, or end of input
/// packet bytes expire on the next advancing source operation
pub fn next(self: *Self) Error!PacketSource.Event {
    // vorbis overlap packet must reach codec before first audible packet
    if (self.prime_pending) {
        self.prime_pending = false;
        return .{ .packet = .{ .bytes = self.initial_prime, .duration = 0 } };
    }
    if (!self.pages.hasPacket()) {
        if (!try self.loadAudioPage()) {
            return .end;
        }
    }
    // announce final trim before returning any packet affected by it
    if (self.bounds_pending) {
        self.bounds_pending = false;
        return .{ .presentation = self.presentation };
    }
    const packet = self.pages.nextPacket().?;
    const frame = self.frame;
    self.frame = std.math.add(u64, frame, self.durations[packet.index])
        catch return error.InvalidData;
    return .{
        .packet = .{
            .bytes = packet.bytes,
            .duration = self.durations[packet.index],
        },
    };
}

/// return metadata with untrimmed counts; the decoder reports trimmed counts
pub fn info(self: *const Self) types.Info {
    return self.metadata;
}

/// seek at or before an untrimmed target with codec preroll subtracted
/// decoder resets codec history and discards samples before the target
pub fn seek(self: *Self, target: u64) Error!PacketSource.SeekResult {
    const end = self.presentation.end
        orelse if (self.metadata.total_frames) |total|
            std.math.add(u64, self.origin, total) catch return error.InvalidData
        else
            null;
    try PacketSource.validateSeek(self.seekable, target, self.origin, end);
    return self.seekIndexed(target);
}

/// restore an indexed anchor during opening or playback
fn seekIndexed(self: *Self, target: u64) Error!PacketSource.SeekResult {
    if (self.is_vorbis) return self.seekVorbis(target);
    const entry = self.index.atOrBefore(target)
        orelse return error.InvalidState;
    const origin = entry.frame;
    try self.pages.seek(entry.location);
    self.frame = entry.frame;
    self.bounds_pending = false;
    return .{ .frame = origin };
}

/// leave input open
pub fn deinit(self: *Self) void {
    if (codec_options.vorbis) {
        if (self.vorbis_headers) |*headers| {
            headers.deinit();
        }
    }
    if (self.initial_prime.len != 0) self.allocator.free(self.initial_prime);
    self.pages.deinit();
    self.index.deinit(self.allocator);
    const allocator = self.allocator;
    allocator.destroy(self);
}

/// validate the identification header; copy mapping, pre skip, and gain
fn parseOpusHead(self: *Self, packet_bytes: []const u8) Error!void {
    // opus head: 8 signature bytes, version, channels, u16 pre skip,
    // u32 input rate, i16 gain, then mapping family
    if (
        packet_bytes.len < 19
            or !std.mem.eql(u8, packet_bytes[0..8], "OpusHead")
    ) {
        return error.UnsupportedEncoding;
    }
    // versions 1 to 15 share the supported major version
    if (packet_bytes[8] == 0 or packet_bytes[8] > 15) {
        return error.UnsupportedEncoding;
    }
    const channels = packet_bytes[9];
    if (channels == 0) return error.InvalidData;
    if (channels > 8) return error.UnsupportedEncoding;
    self.metadata = .{
        .format = .ogg,
        .sample_rate = 48000,
        .channels = channels,
        .total_frames = null,
    };
    // pre skip at byte 10, informational input rate at byte 12
    // decoding and granules always use 48 khz
    self.presentation.start = std.mem.readInt(
        u16,
        packet_bytes[10..12],
        .little,
    );
    self.config.gain_q8 = std.mem.readInt(i16, packet_bytes[16..18], .little);
    // family 0 implies mono or stereo; family 1 carries a channel map
    switch (packet_bytes[18]) {
        0 => {
            if (
                channels > 2
                    or (packet_bytes[8] == 1 and packet_bytes.len != 19)
            ) {
                return error.InvalidData;
            }
            self.config.coupled_streams = channels - 1;
        },
        1 => {
            // family 1, stream counts at 19 and 20, then one map byte per channel
            if (packet_bytes.len < 21 + @as(usize, channels)) {
                return error.InvalidData;
            }
            if (
                packet_bytes[8] == 1
                    and packet_bytes.len != 21 + @as(usize, channels)
            ) {
                return error.InvalidData;
            }
            self.config.streams = packet_bytes[19];
            self.config.coupled_streams = packet_bytes[20];
            @memcpy(
                self.config.mapping[0..channels],
                packet_bytes[21..][0..channels],
            );
        },
        else => return error.UnsupportedEncoding,
    }
    try self.config.validate(
        .{ .sample_rate = self.metadata.sample_rate, .channels = channels },
    );
}

/// validate vendor and comment lengths; allow trailing padding
fn parseOpusTags(packet_bytes: []const u8) Error!void {
    if (!std.mem.startsWith(u8, packet_bytes, "OpusTags")) {
        return error.InvalidData;
    }
    // read little endian u32 lengths for the vendor and each comment
    // with a u32 comment count after the vendor
    var comments = try VorbisComments.init(packet_bytes[8..]);
    while (try comments.next()) |_| {}
}

/// read duration from the first stream toc, in 48 khz samples per channel
/// check framing and the 120 ms limit; the codec validates stream payloads
fn opusDuration(packet_bytes: []const u8) Error!u16 {
    if (packet_bytes.len == 0) return error.InvalidData;
    const toc = packet_bytes[0];
    // toc bit 7, celt uses 2.5, 5, 10 or 20 ms frames
    // bits 6 to 5 both set, hybrid uses 10 or 20 ms; silk uses 10, 20, 40 or 60 ms
    const samples_per_frame: u16 = if (toc & 128 != 0)
        @as(u16, 120) << @as(u2, @truncate(toc >> 3))
    else if (toc & 0x60 == 0x60)
        (if (toc & 8 != 0) @as(u16, 960) else 480)
    else
        switch ((toc >> 3) & 3) {
            0 => 480,
            1 => 960,
            2 => 1920,
            else => 2880,
        };
    // low toc bits select one frame, two, or a count in the next byte
    // low six bits hold count, upper bits are flags
    const frame_count: u16 = switch (toc & 3) {
        0 => 1,
        1, 2 => 2,
        else => if (packet_bytes.len < 2)
            return error.InvalidData
        else
            packet_bytes[1] & 63,
    };
    const packet_samples: u32 = @as(u32, samples_per_frame) * frame_count;
    // 120 ms max means 5760 samples at 48 khz
    if (frame_count == 0 or packet_samples > 5760) return error.InvalidData;
    return @intCast(packet_samples);
}

/// read an audio page and calculate untrimmed positions
/// granule ends its last completed packet; the final page may trim output
fn loadAudioPage(self: *Self) Error!bool {
    if (self.is_vorbis) return self.loadVorbisPage();
    return self.loadOpusPage();
}

/// identify the codec from the first stream packet
fn readFirstHeader(self: *Self, bytes: []const u8) Error!void {
    if (std.mem.startsWith(u8, bytes, "\x01vorbis")) {
        if (!codec_options.vorbis) return error.UnsupportedEncoding;
        // identification header: type, signature, version, channels, rate,
        // three bitrate hints, block sizes and framing; native headers validate fields
        if (bytes.len != 30) return error.InvalidData;
        self.is_vorbis = true;
        self.vorbis_headers = .{ .allocator = self.allocator };
        const headers = &self.vorbis_headers.?;
        try self.addVorbisHeader(bytes);
        self.metadata = .{
            .format = .ogg,
            .sample_rate = std.math.cast(u32, headers.info.sample_rate)
                orelse return error.InvalidData,
            .channels = std.math.cast(u16, headers.info.channels)
                orelse return error.InvalidData,
            .total_frames = null,
        };
    } else {
        if (!codec_options.opus) return error.UnsupportedEncoding;
        try self.parseOpusHead(bytes);
    }
}

/// validate and retain the header for codec init
fn addVorbisHeader(self: *Self, bytes: []const u8) Error!void {
    if (!codec_options.vorbis) return error.UnsupportedEncoding;
    if (self.headers == 1) {
        // header types, identification 0x01, comment 0x03, setup 0x05
        // each is followed by the six byte signature
        if (!std.mem.startsWith(u8, bytes, "\x03vorbis"))
            return error.InvalidData;
        var comments = try VorbisComments.init(bytes[7..]);
        while (try comments.next()) |_| {}
        // vorbis comments require a trailing framing bit
        if (comments.remaining.len == 0 or comments.remaining[0] & 1 == 0)
            return error.InvalidData;
    }
    try self.vorbis_headers.?.add(bytes);
}

/// read headers then derive packet durations and final trim from page granule
fn loadOpusPage(self: *Self) Error!bool {
    while (try self.pages.nextPage()) {
        while (self.headers < 2 and self.pages.hasPacket()) {
            const packet = self.pages.nextPacket().?;
            if (self.headers == 0) {
                // identification packet must be alone on the stream start page
                if (
                    self.pages.flags != 2
                        or self.pages.count != 1
                        or self.pages.hasPartialPacket()
                        or self.pages.granule != 0
                ) {
                    return error.InvalidData;
                }
                try self.readFirstHeader(packet.bytes);
                if (self.is_vorbis) {
                    self.headers = 1;
                    return self.loadVorbisPage();
                }
            } else {
                try parseOpusTags(packet.bytes);
                // comment packet must end its page
                if (
                    self.pages.hasPacket()
                        or self.pages.hasPartialPacket()
                        or self.pages.granule != 0
                ) {
                    return error.InvalidData;
                }
                // discard a leading broadcast fragment before establishing the origin
                self.pages.allowMissingPacketStart = true;
            }
            self.headers += 1;
        }
        if (!self.pages.hasPacket()) {
            if (
                self.pages.ended
                    or (self.pages.count != 0 and self.pages.granule != 0)
            ) {
                return error.InvalidData;
            }
            continue;
        }
        // sum untrimmed durations before checking page end granule
        var page_samples: u64 = 0;
        for (self.pages.packetCursor..self.pages.count) |packet_index| {
            self.durations[packet_index] = try opusDuration(
                self.pages.bytes(packet_index),
            );
            page_samples += self.durations[packet_index];
        }
        const granule: u64 = @intCast(self.pages.granule);
        if (self.first_audio) {
            // cropped streams may start at a nonzero granule origin
            self.origin = if (granule >= page_samples)
                granule - page_samples
            else if (self.pages.ended)
                0
            else
                return error.InvalidData;
            self.frame = self.origin;
            self.first_audio = false;
            // offset presentation bounds by the stream granule origin
            self.presentation.start = std.math.add(
                u64,
                self.origin,
                self.presentation.start,
            )
                catch return error.InvalidData;
        }
        const page_end = std.math.add(u64, self.frame, page_samples)
            catch return error.InvalidData;
        // final granule may trim current page, cannot extend beyond decoded samples
        if (self.pages.ended) {
            if (
                granule > page_end
                    or granule < self.frame
                    or granule < self.presentation.start
            ) {
                return error.InvalidData;
            }
            if (self.presentation.end) |expected| {
                if (granule != expected) return error.InvalidData;
            }
            self.presentation.end = granule;
            self.bounds_pending = true;
            self.metadata.total_frames = page_end - self.origin;
        } else if (granule != page_end) return error.InvalidData;
        return true;
    }
    return false;
}

/// read headers before audio; use codec block sizes to calculate timing
fn loadVorbisPage(self: *Self) Error!bool {
    if (!codec_options.vorbis) return error.UnsupportedEncoding;
    while (try self.pages.nextPage()) {
        while (self.headers < 3 and self.pages.hasPacket()) {
            const packet = self.pages.nextPacket().?;
            try self.addVorbisHeader(packet.bytes);
            self.headers += 1;
            if (
                self.headers == 3
                    and (self.pages.hasPacket()
                        or self.pages.hasPartialPacket())
            )
                return error.InvalidData;
        }
        if (!self.pages.hasPacket()) {
            if (
                self.pages.ended
                    or (self.pages.count != 0 and self.pages.granule != 0)
            )
                return error.InvalidData;
            continue;
        }
        // sum untrimmed durations before checking page end granule
        var page_samples: u64 = 0;
        for (self.pages.packetCursor..self.pages.count) |packet_index| {
            const block_size = try self.vorbis_headers.?.blockSize(
                self.pages.bytes(packet_index),
            );
            self.durations[packet_index] = 0;
            if (self.previous_block != 0) {
                // each adjacent block contributes a quarter of its size to output
                self.durations[packet_index] = self.previous_block / 4
                    + block_size / 4;
            }
            // first packet primes overlap for subsequent output
            self.previous_block = block_size;
            page_samples += self.durations[packet_index];
        }
        // save a lone overlap packet until initial timing is known
        if (self.first_audio and page_samples == 0 and !self.pages.ended) {
            if (
                self.initial_prime.len != 0
                    or self.pages.count != 1
                    or self.pages.granule != 0
            )
                return error.InvalidData;
            const packet = self.pages.nextPacket().?;
            self.initial_prime = try self.allocator.dupe(u8, packet.bytes);
            self.prime_location = packet.location;
            self.prime_pending = true;
            continue;
        }
        const wire: i128 = self.pages.granule;
        if (self.first_audio) {
            // initial end page trims tail, otherwise short granule trims start
            // larger granule supplies broadcast offset
            self.granule_shift = if (self.pages.ended)
                @min(0, @as(i128, page_samples) - wire)
            else
                @as(i128, page_samples) - wire;
            self.presentation.start = if (
                !self.pages.ended
                    and self.granule_shift > 0
            )
                @intCast(self.granule_shift)
            else
                0;
            if (self.presentation.start != 0) {
                // leading trimming cannot exceed the first overlap span
                var span: u16 = 0;
                for (self.pages.packetCursor..self.pages.count) |packet_index| {
                    if (self.durations[packet_index] != 0) {
                        span = self.durations[packet_index];
                        break;
                    }
                }
                if (self.presentation.start > span) return error.InvalidData;
            }
            self.frame = 0;
            self.first_audio = false;
        } else if (
            self.frame == 0
                and page_samples == 0
                and !self.pages.ended
        ) {
            // a standalone first overlap page keeps granule zero after seeking
            // regardless of the later timeline offset
            return true;
        }
        const page_end = std.math.add(u64, self.frame, page_samples)
            catch return error.InvalidData;
        // translate signed wire timeline into nonnegative decode positions
        const granule = std.math.cast(u64, wire + self.granule_shift)
            orelse return error.InvalidData;
        // final granule may trim current page, cannot extend beyond decoded samples
        if (self.pages.ended) {
            if (
                granule > page_end
                    or granule < self.frame
                    or granule < self.presentation.start
            )
                return error.InvalidData;
            if (self.presentation.end) |expected| {
                if (granule != expected) return error.InvalidData;
            }
            self.presentation.end = granule;
            self.bounds_pending = true;
            self.metadata.total_frames = page_end;
        } else if (granule != page_end) return error.InvalidData;
        return true;
    }
    return false;
}

/// seek with preceding overlap packet, return origin of first produced samples
fn seekVorbis(self: *Self, target: u64) Error!PacketSource.SeekResult {
    const entries = self.index.entries.items;
    // include the preceding packet to restore overlap before the target
    // at zero, keep the first packet even when two entries share frame zero
    const selected = self.index.indexAtOrBefore(target)
        orelse return error.InvalidState;
    // last packet at or before target, preceding packet restores overlap
    const prime = selected -| 1;
    const anchor = entries[selected].frame;
    const origin = anchor;
    try self.pages.seek(entries[prime].location);
    self.frame = anchor;
    self.previous_block = 0;
    self.prime_pending = false;
    self.bounds_pending = false;
    return .{ .frame = origin };
}
