//! native vorbis decoder, headers and setup feed packet decode and lapped synthesis
//! source: https://github.com/nothings/stb/blob/master/stb_vorbis.c
//! spec: https://www.xiph.org/vorbis/doc/Vorbis_I_spec.html
const std = @import("std");

/// errors raised inside the decoder core
pub const Error = error{
    /// header or packet violates the vorbis spec
    InvalidData,
    /// packet ended before the required fields were read
    TruncatedData,
    /// stream uses a feature this decoder does not implement
    UnsupportedEncoding,
    /// a decoded value exceeded the decoder bounds
    LimitExceeded,
    /// decoder state does not allow the requested operation
    InvalidState,
}
    || std.mem.Allocator.Error
    || @import("../bit_reader.zig").Error;

/// lsb first packet reader
pub const Reader = @import("../bit_reader.zig").SliceReader(.little);

/// channel count accepted by the spec
pub const max_channels = 255;
/// smallest block size exponent accepted by the spec
pub const min_block_size_log2 = 6;
/// largest block size exponent accepted by the spec
pub const max_block_size_log2 = 13;
/// longest huffman code length in a codebook
pub const max_code_length = 32;

/// stream params taken from the identification header
pub const Info = @import("headers.zig").Info;
/// packet type byte of the comment header
pub const packet_comment = @import("headers.zig").packet_comment;
/// six byte signature shared by every vorbis header
pub const signature = @import("headers.zig").signature;
/// vector quantization book
pub const Codebook = @import("codebook.zig").Codebook;
/// spectral floor curve
pub const Floor = @import("floor.zig").Floor;
/// spectral residue decoder
pub const Residue = @import("residue.zig").Residue;
/// channel mapping and coupling
pub const Mapping = @import("mapping.zig").Mapping;
/// packet mode with its block flag and mapping index
pub const Mode = @import("mapping.zig").Mode;
/// transform tables and the inverse mdct
pub const mdct = @import("mdct.zig");
/// packet decoder and lapped synthesis
pub const Decoder = @import("decoder.zig").Decoder;

/// setup tables without synthesis buffers
pub const Setup = @import("setup.zig").Setup;
/// validate a header's type and signature before accessing its payload
pub const validateHeaderPrefix = @import("headers.zig").validatePrefix;

/// number of bits needed to represent a value; zero consumes no bits
pub fn ilog(value: u32) u6 {
    return @as(u6, 32) - @clz(value);
}
