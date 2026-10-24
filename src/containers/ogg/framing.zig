//! ogg framing derived from libogg 1.3.6
const std = @import("std");

// a page holds at most 255 segments of 255 bytes each
pub const maxBodyBytes = 255 * 255;
// 27 fixed header bytes plus up to 255 laces
pub const maxHeaderBytes = 27 + 255;
pub const maxPageBytes = maxHeaderBytes + maxBodyBytes;
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

/// a page with validated layout
/// verify checksums separately
pub const Page = struct {
    header: []const u8,
    body: []const u8,

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
        return std.mem.readInt(u32, self.header[14..18], .little);
    }
    /// wrapping page sequence within logical stream
    pub fn sequence(self: Page) u32 {
        return std.mem.readInt(u32, self.header[18..22], .little);
    }
    /// each lace gives a segment length in bytes
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
};

/// absolute page offset, sequence and first lace for seeking to packet
pub const Location = struct { offset: u64, sequence: u32, segment: u8 };
