//! audio metadata, sample formats, packets, options, and errors
const std = @import("std");

/// identify the audio container
pub const Format = enum { wav, wave64, ogg, mp3 };

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

    /// return storage bytes per sample
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

/// identify the encoding; decoder support varies
pub const DecodingScheme = union(enum) {
    pcm: PcmFormat,
    opus: OpusConfig,
    vorbis: VorbisConfig,
    mp3,
    alaw,
    mulaw,
    unsupported,
};

/// borrow identification, comment, and setup packets in stream order
/// keep them unchanged until codec initialization copies their state
pub const VorbisConfig = struct { headers: [3][]const u8 };

/// configure opus mapping families 0 and 1 in opus and vorbis channel order
pub const OpusConfig = struct {
    // coded streams, coupled streams produce two channels each
    streams: u8 = 1,
    coupled_streams: u8 = 0,
    // map output channels to coded channels, 255 means silence
    mapping: [8]u8 = .{ 0, 1, 2, 3, 4, 5, 6, 7 },
    /// signed fixed point db gain, 256 units per db
    gain_q8: i16 = 0,
};

/// locate an absolute byte range in the source
pub const DataRange = struct {
    offset: u64 = 0,
    length: u64 = 0,

    /// return the exclusive end offset; reject overflow
    pub fn end(self: DataRange) ValidationError!u64 {
        return std.math.add(u64, self.offset, self.length)
            catch error.InvalidData;
    }
};

pub const AudioData = @import("AudioData.zig");
pub const Segment = AudioData.Segment;

/// store parsed metadata and borrowed audio segments
pub const ParsedAudio = struct {
    info: Info,
    scheme: DecodingScheme,
    data: AudioData,
};

/// describe the audio stream
pub const Info = struct {
    format: Format,
    sample_rate: u32,
    channels: u16,
    /// count frames, each with one sample per channel; null means unknown
    total_frames: ?u64,
    /// record whether the output sample rate has changed
    variable_sample_rate: bool = false,

    /// return duration, or null for unknown counts or a changed rate
    /// sample rate must be nonzero
    pub fn durationSeconds(self: Info) ?f64 {
        if (self.variable_sample_rate) return null;
        const frames = self.total_frames orelse return null;
        return @as(f64, @floatFromInt(frames))
            / @as(f64, @floatFromInt(self.sample_rate));
    }
};

/// report invalid metadata, sample formats, or byte ranges
pub const ValidationError = error{ InvalidData };

/// report container detection and parsing errors
pub const ParseError = error{
    UnsupportedFormat,
    UnsupportedChunkLayout,
    InvalidData,
    TruncatedData,
};

/// report decoding and decoder usage errors
pub const DecodeError = error{
    UnsupportedEncoding,
    InvalidData,
    TruncatedData,
    InvalidBufferSize,
    OutOfRange,
    LimitExceeded,
    InvalidState,
    FormatChanged,
};

pub const ReadError = std.Io.Reader.Error;
pub const SeekError = std.Io.File.Reader.SeekError;
/// include errors from sources without seek support
pub const SourceSeekError = SeekError || error{ Unseekable };
pub const SizeError = std.Io.File.Reader.SizeError || error{ Unseekable };

/// report source access errors
pub const IoError = ReadError || SeekError || SizeError;

/// include parsing and source access errors
pub const ParserError = ParseError || IoError;

/// include parsing, decoding, source access, and allocation errors
pub const DecoderError = ParseError
    || DecodeError
    || IoError
    || std.mem.Allocator.Error;

pub const Error = DecoderError;

/// bounds for working buffers, payloads and seek history
pub const Options = struct {
    /// limit packet and configuration bytes, excluding c codec allocations
    max_payload_bytes: usize = 64 * 1024 * 1024,
    /// limit seek index storage, including spare capacity
    max_index_bytes: usize = 64 * 1024 * 1024,
    /// working storage for packet assembly, metadata and fixed units
    /// capped by max_payload_bytes, reject inputs that cannot fit
    buffer_bytes: usize = 64 * 1024,
    /// reserve history for this channel count or the initial count, if larger
    /// reject later format changes that exceed the reservation
    reserve_channels: u16 = 8,

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

/// seconds per timestamp tick, numerator divided by denominator
pub const TimeBase = struct { numerator: u32, denominator: u32 };
/// map media timestamps to frame positions for seeking
pub const Timestamp = struct { value: i64, time_base: TimeBase };

/// decoded frame interval, inclusive start and exclusive end, null end means unknown
pub const Presentation = struct { start: u64 = 0, end: ?u64 = null };

/// sample rate and channel layout of decoded output
pub const AudioFormat = struct {
    sample_rate: u32,
    channels: u16,
    /// wave speaker bits; null preserves unspecified source channel order
    channel_mask: ?u32 = null,

    /// compare sample rate and channel layout
    pub fn eql(self: AudioFormat, other: AudioFormat) bool {
        return self.sample_rate == other.sample_rate
            and self.channels == other.channels
            and self.channel_mask == other.channel_mask;
    }
};

/// borrow encoded bytes; submission must consume or copy them before returning
pub const Packet = struct {
    bytes: []const u8,
    timestamp: ?Timestamp = null,
    // duration in timestamp ticks, null when unknown
    duration: ?u64 = null,
};

/// borrow interleaved f32 samples until the next codec operation
pub const AudioBlock = struct {
    samples: []const f32,
    format: AudioFormat,
    timestamp: ?Timestamp = null,

    /// count frames; samples must be channel aligned
    pub fn frameCount(self: AudioBlock) usize {
        return self.samples.len / self.format.channels;
    }
};

/// packet accepted or output must be received before retrying
pub const Submit = enum { accepted, receive_output_first };
/// decoded output, format change or next required pipeline action
pub const Receive = union(enum) {
    audio: AudioBlock,
    need_input,
    format_changed: AudioFormat,
    finished,
};
