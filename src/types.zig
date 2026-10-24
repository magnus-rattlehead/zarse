//! audio metadata, sample formats, packets, options, and errors
const std = @import("std");

pub const Format = enum { wav, wave64, ogg, mp3, flac };

/// sample storage width and numeric representation
pub const PcmSampleType = enum {
    unsigned8,
    signed8,
    signed16,
    signed24,
    signed32,
    signed64,
    float32,
    float64,

    pub fn byteSize(self: PcmSampleType) usize {
        return switch (self) {
            .unsigned8, .signed8 => 1,
            .signed16 => 2,
            .signed24 => 3,
            .signed32, .float32 => 4,
            .signed64, .float64 => 8,
        };
    }
};

/// store interleaved pcm; reduced precision integers use the highest bits
/// null valid_bits uses all storage bits
pub const PcmFormat = struct {
    sample_type: PcmSampleType,
    byte_order: std.builtin.Endian,
    valid_bits: ?u16 = null,
};

pub const DecodingScheme = union(enum) {
    pcm: PcmFormat,
    opus: OpusConfig,
    vorbis,
    flac: FlacConfig,
    mp3,
    alaw,
    mulaw,
    ima_adpcm: ImaAdpcmConfig,
    ms_adpcm: MsAdpcmConfig,
    unsupported,
};

pub const ImaAdpcmConfig = struct {
    block_align: u16,
    samples_per_block: u16,
    byte_order: std.builtin.Endian = .little,
};

/// microsoft adpcm predictor coefficients, in file order
pub const AdpcmCoefficient = struct { first: i16, second: i16 };

pub const MsAdpcmConfig = struct {
    block_align: u16,
    samples_per_block: u16,
    byte_order: std.builtin.Endian = .little,
    /// number of active coefficients; predictor selectors are u8
    coefficient_count: u16,
    coefficients: [256]AdpcmCoefficient = @splat(.{ .first = 0, .second = 0 }),
};

pub const FlacConfig = @import("codecs/flac/StreamInfo.zig");

/// configure opus mapping families 0 and 1 in opus and vorbis channel order
pub const OpusConfig = struct {
    // coded streams, coupled streams produce two channels each
    streams: u8 = 1,
    coupled_streams: u8 = 0,
    // map output channels to coded channels, 255 means silence
    mapping: [8]u8 = .{ 0, 1, 2, 3, 4, 5, 6, 7 },
    /// signed fixed point db gain, 256 units per db
    gain_q8: i16 = 0,

    /// validate the output format, coded stream topology, and channel mapping
    pub fn validate(
        self: OpusConfig,
        format: AudioFormat,
    ) ValidationError!void {
        if (
            format.sample_rate != 48000
                or format.channels == 0
                or format.channels > 8
        )
            return error.InvalidData;
        // coupled streams add one extra coded channel each
        const coded_channels = @as(u16, self.streams) + self.coupled_streams;
        if (
            self.streams == 0
                or self.coupled_streams > self.streams
                or coded_channels > 255
        )
            return error.InvalidData;
        for (self.mapping[0..format.channels]) |channel_index| {
            if (channel_index != 255 and channel_index >= coded_channels)
                return error.InvalidData;
        }
    }
};

pub const DataRange = struct {
    offset: u64 = 0,
    length: u64 = 0,

    /// return the exclusive end offset; reject overflow
    pub fn end(self: DataRange) ValidationError!u64 {
        return std.math.add(u64, self.offset, self.length)
            catch error.InvalidData;
    }
};

pub const Segment = union(enum) {
    data: DataRange,
    silence: u64,

    pub fn frameCount(self: Segment, alignment: u32) ValidationError!u64 {
        return switch (self) {
            .data => |range| if (range.length % alignment != 0)
                error.InvalidData
            else
                range.length / alignment,
            .silence => |frames| frames,
        };
    }
};

pub const Info = struct {
    format: Format,
    sample_rate: u32,
    channels: u16,
    /// wave speaker bits; null preserves unspecified source channel order
    channel_mask: ?u32 = null,
    /// total pcm frames, one sample per channel; null means unknown
    total_frames: ?u64,

    /// return duration, or null for unknown counts
    /// sample rate must be nonzero
    pub fn durationSeconds(self: Info) ?f64 {
        const frames = self.total_frames orelse return null;
        return @as(f64, @floatFromInt(frames))
            / @as(f64, @floatFromInt(self.sample_rate));
    }
};

pub const ValidationError = error{ InvalidData };

pub const ParseError = error{
    UnsupportedFormat,
    UnsupportedChunkLayout,
    LimitExceeded,
    InvalidData,
    TruncatedData,
};

pub const DecodeError = error{
    UnsupportedEncoding,
    InvalidData,
    TruncatedData,
    InvalidBufferSize,
    OutOfRange,
    LimitExceeded,
    InvalidState,
};

pub const ReadError = std.Io.Reader.Error;
pub const SeekError = std.Io.File.Reader.SeekError;
pub const SourceSeekError = SeekError || error{ Unseekable };
pub const SizeError = std.Io.File.Reader.SizeError || error{ Unseekable };

pub const IoError = ReadError || SeekError || SizeError;

pub const ParserError = ParseError || IoError;

pub const DecoderError = ParseError
    || DecodeError
    || IoError
    || std.mem.Allocator.Error;

pub const Error = DecoderError;

/// caller-selected working capacity and limits for payloads and seek history
/// the library sizes codec state, scratch and sample storage internally
/// buffer_bytes is not a total memory budget
pub const Options = struct {
    /// limit packet and config bytes, excluding c codec allocations
    max_payload_bytes: usize = 64 * 1024 * 1024,
    /// limit seek index storage, including spare capacity
    max_index_bytes: usize = 64 * 1024 * 1024,
    /// required capacity for encoded input assembly and retained format metadata
    /// capped by max_payload_bytes; insufficient capacity returns LimitExceeded
    /// choose before opening; flac frames, ogg packets and fixed pcm units must fit
    /// mp3 also needs space for framing lookahead
    /// separate from the source reader buffer and caller's decoded output buffer
    buffer_bytes: usize,

    /// working storage size capped by payload limit
    pub fn bufferCapacity(self: Options) usize {
        return @min(self.buffer_bytes, self.max_payload_bytes);
    }
};

/// encoded unit boundaries expected by codec, sizes in bytes
pub const InputScheme = union(enum) {
    // one interleaved pcm frame per unit
    samples: u32,
    // one encoded block per unit, may decode to multiple frames
    blocks: u32,
    // container supplies variable sized packet boundaries
    packets,

    /// return bytes per unit, or null for packet framing
    pub fn unitBytes(self: InputScheme) ?u32 {
        return switch (self) {
            .samples, .blocks => |size| size,
            .packets => null,
        };
    }
};

/// decoded frame interval, inclusive start and exclusive end, null end means unknown
pub const Presentation = struct { start: u64 = 0, end: ?u64 = null };

/// sample rate and channel layout of decoded output
pub const AudioFormat = struct {
    sample_rate: u32,
    channels: u16,
    /// wave speaker bits; null preserves unspecified source channel order
    channel_mask: ?u32 = null,
};

/// packet bytes remain valid until the next advancing source operation
pub const Packet = struct {
    bytes: []const u8,
    /// pcm frames per channel, null when unknown
    duration: ?u64 = null,
};
