//! parse mpeg audio headers and locate complete layer iii frames
//! source: https://github.com/lieff/minimp3
//! commit: ea99364f61c14656440e8d77e9c233ccf3124633 (cc0-1.0)
const types = @import("../../types.zig");

pub const Error = types.ParseError;
// upper bound used for encoded frame validation and free format search
pub const max_frame_bytes = 2304;

/// candidate encoded frame position and inferred free format size
pub const Match = struct {
    offset: usize,
    length: usize,
    /// inferred unpadded free format size
    free_format_bytes: ?u16 = null,
    /// following compatible header found, false for frame filling supplied slice
    confirmed: bool = false,
};

/// frame header and encoded length, payload may still need more input
pub const FrameInfo = struct { header: Header, length: usize };

/// byte inspection result, caller owns refilling and source end decisions
pub const FindResult = union(enum) {
    frame: Match,
    /// discard prefix, retain candidate and request count bytes from its start
    need_more: struct {
        offset: usize,
        count: usize,
        /// unresolved free format candidate may be skipped if source cannot supply more
        resync: bool = false,
    },
    /// discard scanned prefix, retain possible split header bytes
    skip: usize,
};

/// validated four byte layer iii header, accessors assume parse succeeded
pub const Header = struct {
    bytes: [4]u8,

    /// mpeg version controlling rate scaling and frame duration
    pub const Version = enum { mpeg1, mpeg2, mpeg25 };
    /// two bit channel mode from encoded header
    pub const Mode = enum(u2) { stereo, joint_stereo, dual_channel, mono };

    /// validate and copy a four byte layer iii header
    pub fn parse(bytes: []const u8) Error!Header {
        if (bytes.len < 4) return error.TruncatedData;
        // sync word, all 8 bits of byte 0 and the high 3 bits of byte 1 are set
        if (bytes[0] != 0xff or bytes[1] & 0xe0 != 0xe0)
            return error.InvalidData;
        // version bits 4 to 3 cannot be 01; layer bits 2 to 1 cannot be 00
        if (bytes[1] & 0x18 == 0x08 or bytes[1] & 0x06 == 0)
            return error.InvalidData;
        // bitrate index 1111 and sample rate index 11 are reserved
        if (bytes[2] & 0xf0 == 0xf0 or bytes[2] & 0x0c == 0x0c)
            return error.InvalidData;
        // layer bits 01 mean layer iii; valid layers i and ii are outside this reader
        if (bytes[1] & 0x06 != 0x02) return error.UnsupportedFormat;
        return .{ .bytes = bytes[0..4].* };
    }

    /// extract the two version bits, 11 means mpeg 1, 10 means mpeg 2, 00 means mpeg 2.5
    pub fn version(self: Header) Version {
        // shift bits 4 to 3 down, then keep only those two bits with binary mask 11
        return switch ((self.bytes[1] >> 3) & 3) {
            3 => .mpeg1,
            2 => .mpeg2,
            0 => .mpeg25,
            else => unreachable, // parse rejects the reserved version
        };
    }

    /// extract byte 3 high two bits, stereo, joint stereo, dual channel, mono
    pub fn mode(self: Header) Mode {
        return @enumFromInt(self.bytes[3] >> 6);
    }

    /// extract joint stereo extension, bit 1 enables mid side, bit 0 enables intensity
    pub fn modeExtension(self: Header) u2 {
        // bits 5 to 4 hold extension, interpretation depends on channel mode
        return @truncate(self.bytes[3] >> 4);
    }

    /// mono has one channel; every other channel mode has two
    pub fn channels(self: Header) u16 {
        return if (self.mode() == .mono) 1 else 2;
    }

    /// report whether a two byte crc follows the header
    pub fn hasCrc(self: Header) bool {
        // low protection bit is inverted, zero means crc present, one means absent
        return self.bytes[1] & 1 == 0;
    }

    /// bitrate index zero means frame size must be inferred from header spacing
    pub fn isFreeFormat(self: Header) bool {
        // 0xf0 selects the high four bits, which hold the bitrate index
        return self.bytes[2] & 0xf0 == 0;
    }

    /// whether frame has one extra encoded byte, separate from trailing pcm trim
    pub fn hasPadding(self: Header) bool {
        // binary mask 00000010 selects byte 2 padding bit
        return self.bytes[2] & 2 != 0;
    }

    /// identify joint stereo frames with mid side coding enabled
    pub fn isMidSideStereo(self: Header) bool {
        // extension flags only apply to joint stereo frames
        return self.mode() == .joint_stereo and self.modeExtension() & 2 != 0;
    }

    /// identify joint stereo frames with intensity coding enabled
    pub fn isIntensityStereo(self: Header) bool {
        // low extension bit enables intensity stereo, both flags can be set
        return self.mode() == .joint_stereo and self.modeExtension() & 1 != 0;
    }

    /// look up kilobits per second, zero marks free format
    pub fn bitrateKbps(self: Header) u32 {
        const mpeg1: usize = @intFromBool(self.version() == .mpeg1);
        // table axes, mpeg 2 or 2.5 or mpeg 1, layer zero for layer iii, bitrate index
        // values store half the bitrate; shifting byte 2 extracts its high nibble
        return 2 * @as(u32, halfrate[mpeg1][0][self.bytes[2] >> 4]);
    }

    /// look up sample rate in hertz, scaling the mpeg 1 base table by version
    pub fn sampleRate(self: Header) u32 {
        // right shifts 0, 1 and 2 divide by 1, 2 and 4 for mpeg 1, 2 and 2.5
        const shift: u5 = switch (self.version()) {
            .mpeg1 => 0,
            .mpeg2 => 1,
            .mpeg25 => 2,
        };
        // bits 3 to 2 select 44100, 48000, or 32000 before version scaling
        return g_hz[(self.bytes[2] >> 2) & 3] >> shift;
    }

    /// index nine sample rates in upstream order, mpeg 2.5 then 2 then 1
    pub fn sampleRateIndex(self: Header) usize {
        // each version has three rates, ordered by raw header index
        const base: usize = switch (self.version()) {
            .mpeg25 => 0,
            .mpeg2 => 3,
            .mpeg1 => 6,
        };
        return base + ((self.bytes[2] >> 2) & 3);
    }

    /// map nine sample rates to eight scalefactor band tables
    pub fn bandTableIndex(self: Header) usize {
        // 11025 and 12000 share first table, shift all later indices down one
        return self.sampleRateIndex() -| 1;
    }

    /// pcm frames per channel per encoded frame, mpeg 1 has 1152, mpeg 2 or 2.5 have 576
    pub fn frameSamples(self: Header) u16 {
        return if (self.version() == .mpeg1) 1152 else 576;
    }

    /// return total encoded bytes, including padding; free_format_bytes is the
    /// inferred unpadded size, return null until free format size known
    pub fn frameBytes(self: Header, free_format_bytes: ?u16) ?u32 {
        // multiply pcm frames by kbps and 125, divide by hertz to get encoded bytes
        // integer division rounds down before the optional padding byte is added
        const unpadded: u32 = if (self.isFreeFormat())
            free_format_bytes orelse return null
        else
            @as(u32, self.frameSamples())
                * self.bitrateKbps()
                * 125
                / self.sampleRate();
        return unpadded + @intFromBool(self.hasPadding());
    }

    /// compare validated headers for stream sync, allow bitrate and channel mode changes
    pub fn compatible(self: Header, other: Header) bool {
        // xor marks differing bits; 0xfe compares byte 1 except its crc bit
        if ((self.bytes[1] ^ other.bytes[1]) & 0xfe != 0) return false;
        // 0x0c selects sample rate index bits
        if ((self.bytes[2] ^ other.bytes[2]) & 0x0c != 0) return false;
        // frame size discovery must use same method for both headers
        return self.isFreeFormat() == other.isFreeFormat();
    }

    /// side information bytes, excludes frame header and optional crc
    pub fn sideInfoBytes(self: Header) u8 {
        // mpeg 1 uses 17 mono or 32 stereo bytes, later versions use 9 or 17
        return if (self.version() == .mpeg1)
            (if (self.channels() == 1) @as(u8, 17) else 32)
        else
            (if (self.channels() == 1) @as(u8, 9) else 17);
    }

    /// min encoded length for header, optional crc and side information
    pub fn minimumFrameBytes(self: Header) usize {
        return 4
            + @as(usize, self.sideInfoBytes())
            + @as(usize, if (self.hasCrc()) 2 else 0);
    }
};

/// inspect buffered header and bounded frame size, payload may be incomplete
/// free format needs inferred unpadded size
pub fn frameInfo(bytes: []const u8, free_format_bytes: ?u16) Error!FrameInfo {
    const header = try Header.parse(bytes);
    const length: usize = header.frameBytes(free_format_bytes)
        orelse return error.InvalidData;
    if (length < header.minimumFrameBytes() or length > max_frame_bytes)
        return error.InvalidData;
    return .{ .header = header, .length = length };
}

/// search supplied bytes for complete layer iii frame
/// infer free format from three headers, exact slice fit leaves confirmed false
pub fn findFrame(bytes: []const u8, free_format_bytes: ?u16) Error!FindResult {
    var offset: usize = 0;
    while (bytes.len - offset >= 4) : (offset += 1) {
        const candidate = bytes[offset..];
        const header = Header.parse(candidate) catch continue;
        var free_format = free_format_bytes;
        if (header.isFreeFormat() and free_format == null) {
            // test possible header spacing
            var distance = header.minimumFrameBytes();
            while (distance <= max_frame_bytes) : (distance += 1) {
                if (candidate.len < distance + 4)
                    return .{
                        .need_more = .{
                            .offset = offset,
                            .count = distance + 4,
                            .resync = true,
                        },
                    };
                const second = Header.parse(candidate[distance..])
                    catch continue;
                if (!header.compatible(second)) continue;
                // remove first padding byte and account for second frame padding
                const unpadded = distance - @intFromBool(header.hasPadding());
                const third = distance
                    + unpadded
                    + @intFromBool(second.hasPadding());
                if (candidate.len < third + 4)
                    return .{
                        .need_more = .{
                            .offset = offset,
                            .count = third + 4,
                            .resync = true,
                        },
                    };
                const third_header = Header.parse(candidate[third..])
                    catch continue;
                if (!header.compatible(third_header)) continue;
                free_format = @intCast(unpadded);
                break;
            }
            if (free_format == null) continue;
        }
        const current = try frameInfo(candidate, free_format);
        if (candidate.len < current.length)
            return .{
                .need_more = .{ .offset = offset, .count = current.length },
            };
        // caller decides whether exact slice fit reaches source end
        const confirmed = candidate.len > current.length;
        if (confirmed) {
            if (candidate.len - current.length < 4)
                return .{
                    .need_more = .{
                        .offset = offset,
                        .count = current.length + 4,
                    },
                };
            // validate next boundary, playback checks each later frame independently
            const following = frameInfo(
                candidate[current.length..],
                free_format,
            )
                catch continue;
            if (!header.compatible(following.header)) continue;
        }
        return .{
            .frame = .{
                .offset = offset,
                .length = current.length,
                .free_format_bytes = free_format,
                .confirmed = confirmed,
            },
        };
    }
    // preserve up to three bytes that could begin a split header
    return .{ .skip = offset };
}

pub const halfrate: [2][3][15]u8 = .{
    .{
        .{ 0, 4, 8, 12, 16, 20, 24, 28, 32, 40, 48, 56, 64, 72, 80 },
        .{ 0, 4, 8, 12, 16, 20, 24, 28, 32, 40, 48, 56, 64, 72, 80 },
        .{ 0, 16, 24, 28, 32, 40, 48, 56, 64, 72, 80, 88, 96, 112, 128 },
    },
    .{
        .{ 0, 16, 20, 24, 28, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160 },
        .{ 0, 16, 24, 28, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192 },
        .{ 0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224 },
    },
};
pub const g_hz: [3]u32 = .{ 44100, 48000, 32000 };
