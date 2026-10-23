//! identification header and stream params
//! ref: stb_vorbis.c start_decoder() first packet, libvorbis info.c
const std = @import("std");
const root = @import("root.zig");

const Error = root.Error;

/// packet type bytes of the three vorbis headers
pub const packet_identification = 0x01;
pub const packet_comment = 0x03;
pub const packet_setup = 0x05;

/// six byte signature shared by every vorbis header
pub const signature = "vorbis";

/// stream params and the block sizes derived from the header
pub const Info = struct {
    /// coded channel count, one to 255
    channels: u8 = 0,
    /// sample rate in hertz
    sample_rate: u32 = 0,
    /// max bitrate hint, may be zero or negative
    bitrate_maximum: i32 = 0,
    /// nominal bitrate hint, may be zero or negative
    bitrate_nominal: i32 = 0,
    /// min bitrate hint, may be zero or negative
    bitrate_minimum: i32 = 0,
    /// short block size, the power of two exponent
    blocksize_0_log2: u4 = 0,
    /// long block size, the power of two exponent
    blocksize_1_log2: u4 = 0,

    /// short block size in samples
    pub fn blocksize0(self: Info) usize {
        return @as(usize, 1) << self.blocksize_0_log2;
    }

    /// long block size in samples
    pub fn blocksize1(self: Info) usize {
        return @as(usize, 1) << self.blocksize_1_log2;
    }

    /// parse the identification header packet
    /// packet must hold the type byte, the signature, and the fields
    pub fn readIdentification(packet: []const u8) Error!Info {
        // type byte, six signature bytes, version, channels, rate, three
        // bitrate hints, block sizes and the framing bit
        if (packet.len < 30) return error.TruncatedData;
        try validatePrefix(packet, packet_identification);
        // vorbis version is fixed at zero
        if (!std.mem.eql(u8, packet[7..11], &[_]u8{ 0, 0, 0, 0 })) {
            return error.UnsupportedEncoding;
        }
        const channels = packet[11];
        if (channels == 0 or channels > root.max_channels) {
            return error.InvalidData;
        }
        const sample_rate = std.mem.readInt(u32, packet[12..16], .little);
        if (sample_rate == 0) return error.InvalidData;
        const bitrate_maximum = std.mem.readInt(i32, packet[16..20], .little);
        const bitrate_nominal = std.mem.readInt(i32, packet[20..24], .little);
        const bitrate_minimum = std.mem.readInt(i32, packet[24..28], .little);
        // low nibble holds the short block exponent, the high nibble the
        // long block exponent
        const sizes = packet[28];
        const blocksize_0_log2: u4 = @truncate(sizes);
        const blocksize_1_log2: u4 = @truncate(sizes >> 4);
        if (
            blocksize_0_log2 < root.min_block_size_log2
                or blocksize_0_log2 > root.max_block_size_log2
                or blocksize_1_log2 < root.min_block_size_log2
                or blocksize_1_log2 > root.max_block_size_log2
                or blocksize_0_log2 > blocksize_1_log2
        ) {
            return error.InvalidData;
        }
        // framing bit must be set
        if (packet[29] & 1 == 0) return error.InvalidData;
        return .{
            .channels = channels,
            .sample_rate = sample_rate,
            .bitrate_maximum = bitrate_maximum,
            .bitrate_nominal = bitrate_nominal,
            .bitrate_minimum = bitrate_minimum,
            .blocksize_0_log2 = blocksize_0_log2,
            .blocksize_1_log2 = blocksize_1_log2,
        };
    }
};

/// validate the type and signature; a short prefix returns TruncatedData
pub fn validatePrefix(packet: []const u8, packet_type: u8) Error!void {
    if (packet.len < 7) return error.TruncatedData;
    if (packet[0] != packet_type or !std.mem.eql(u8, packet[1..7], signature)) {
        return error.InvalidData;
    }
}
