//! parse riff, rifx, rf64, bw64, and wave64 with a shared chunk walker
const std = @import("std");
const types = @import("../../types.zig");
const input_helpers = @import("../../input.zig");
const Source = @import("../../Source.zig");
pub const Error = types.ParserError;
pub const ParseError = types.ParserError;

pub const ChunkHeader = struct {
    id: [4]u8,
    /// payload bytes, excluding header and padding
    size: u64,
};

pub fn readRequired(source: Source, out: []u8) ParseError!void {
    return input_helpers.readRequired(source.reader, out);
}

/// rf64 or bw64 size overrides and ordered size table
pub const DataSize64 = struct {
    data_size: u64,
    /// rf64 samples per channel; zero is unspecified, null for bw64
    sample_count: ?u64,
    /// 12-byte entries: fourcc followed by a little endian u64
    table: types.DataRange,
};

/// resolved chunk payload and next aligned header offset
pub const Chunk = struct {
    header: ChunkHeader,
    data: types.DataRange,
    next_offset: u64,
};

/// resolve chunk offsets and padding within end
/// header size must already include any ds64 override
pub fn frame(
    header: ChunkHeader,
    offset: u64,
    end: u64,
    kind: types.Format,
) Error!Chunk {
    // riff uses fourcc and u32; wave64 uses guid and u64 with inclusive size
    const header_size: u64 = if (kind == .wave64) 24 else 8;
    if (offset > end or end - offset < header_size) return error.InvalidData;
    const payload_offset = offset + header_size;
    if (header.size > end - payload_offset) return error.InvalidData;
    const payload_end = payload_offset + header.size;
    // wave64 data uses its declared size; other chunks align to eight bytes
    // with optional final padding; riff pads odd payload lengths
    const padding = if (kind == .wave64)
        (if (std.mem.eql(u8, &header.id, "data"))
            0
        else
            @min((8 - (header.size & 7)) & 7, end - payload_end))
    else
        header.size & 1;
    if (padding > end - payload_end) return error.InvalidData;
    return .{
        .header = header,
        .data = .{ .offset = payload_offset, .length = header.size },
        .next_offset = payload_end + padding,
    };
}

/// preserve known and unknown wave encoding tags
pub const FormatTag = enum(u16) {
    pcm = 0x0001,
    ms_adpcm = 0x0002,
    ieee_float = 0x0003,
    alaw = 0x0006,
    mulaw = 0x0007,
    ima_adpcm = 0x0011,
    extensible = 0xfffe,
    _,
};

/// format fields and codec settings
pub const FormatChunk = struct {
    audio_format: FormatTag,
    channels: u16,
    sample_rate: u32,
    /// average encoded bytes per second
    byte_rate: u32,
    /// encoded bytes per block
    block_align: u16,
    /// declared bits per sample; encoding determines meaning
    bits_per_sample: u16,
    /// extra format data, or null if absent
    extension: ?FormatExtension = null,
    byte_order: std.builtin.Endian = .little,

    /// preserve an explicit speaker mask, zero means unspecified source order
    pub fn channelMask(self: FormatChunk) ?u32 {
        const extension = self.extension orelse return null;
        return switch (extension.payload) {
            .extensible => |ext| if (ext.channel_mask == 0)
                null
            else
                ext.channel_mask,
            .codec, .ima_adpcm, .ms_adpcm => null,
        };
    }

    /// resolve standard subtypes; preserve unknown tags
    /// return null for custom or missing subtypes
    pub fn encoding(self: FormatChunk) ?FormatTag {
        if (self.audio_format != .extensible) return self.audio_format;
        const extension = self.extension orelse return null;
        const ext = switch (extension.payload) {
            .extensible => |ext| ext,
            .codec, .ima_adpcm, .ms_adpcm => return null,
        };
        // recognize standard wave subtype namespace before interpreting embedded tag
        const guid = ext.sub_format;
        // guid fields, data2 at byte 4, data3 at byte 6, data4 at byte 8
        if (
            std.mem.readInt(u16, guid[4..6], self.byte_order) != 0
                or std.mem.readInt(u16, guid[6..8], self.byte_order) != 0x0010
                or !std.mem.eql(
                    u8,
                    guid[8..],
                    "\x80\x00\x00\xaa\x00\x38\x9b\x71",
                )
        )
            return null;
        const tag = std.mem.readInt(u32, guid[0..4], self.byte_order);
        if (tag > std.math.maxInt(u16)) return null;
        return @enumFromInt(@as(u16, @intCast(tag)));
    }

    /// check whether block_align describes one decoded frame
    pub fn fixedWidth(self: FormatChunk) bool {
        const tag = self.encoding();
        return tag == .pcm
            or tag == .ieee_float
            or tag == .alaw
            or tag == .mulaw;
    }

    /// select decoding for a validated format and byte order
    /// return unsupported for unknown encodings or sample layouts
    pub fn decodingScheme(
        fmt: FormatChunk,
        byte_order: std.builtin.Endian,
    ) types.DecodingScheme {
        const tag = fmt.encoding() orelse return .unsupported;
        switch (tag) {
            .alaw => return .alaw,
            .mulaw => return .mulaw,
            .ima_adpcm, .ms_adpcm => {
                const extension = fmt.extension orelse return .unsupported;
                return switch (extension.payload) {
                    .ima_adpcm => |config| .{ .ima_adpcm = config },
                    .ms_adpcm => |config| .{ .ms_adpcm = config },
                    else => .unsupported,
                };
            },
            .pcm, .ieee_float => {},
            else => return .unsupported,
        }

        // derive physical sample width from complete interleaved frame size
        const container_bits = @as(u32, fmt.block_align / fmt.channels) * 8;
        // extensible format separates stored width from significant bits
        var valid_bits = fmt.bits_per_sample;
        if (fmt.audio_format == .extensible) {
            valid_bits = fmt
                .extension
                .?
                .payload
                .extensible
                .valid_bits_per_sample;
        } else if (
            container_bits != (@as(u32, fmt.bits_per_sample) + 7) / 8 * 8
        ) {
            // reject wider pcm storage that could conceal mislabeled floats
            return .unsupported;
        }
        const sample_type: types.PcmSampleType = if (tag == .ieee_float)
            switch (container_bits) {
                32 => .float32,
                64 => .float64,
                else => return .unsupported,
            }
        else
            switch (container_bits) {
                8 => .unsigned8,
                16 => .signed16,
                24 => .signed24,
                32 => .signed32,
                64 => .signed64,
                else => return .unsupported,
            };
        return .{
            .pcm = .{
                .sample_type = sample_type,
                .byte_order = byte_order,
                .valid_bits = valid_bits,
            },
        };
    }
};

pub const FormatExtension = struct {
    payload: union(enum) {
        codec: void,
        ima_adpcm: types.ImaAdpcmConfig,
        ms_adpcm: types.MsAdpcmConfig,
        extensible: ExtensibleFormat,
    },
};

pub const ExtensibleFormat = struct {
    /// significant bits within the sample storage width
    valid_bits_per_sample: u16,
    /// speaker positions; zero means unspecified
    channel_mask: u32,
    /// subtype guid in file byte order; resolve through the encoding helper
    sub_format: [16]u8,
};

/// validate format fields; padded allows trailing eight byte alignment
fn parseFormatPayload(
    state: *Reader,
    data: types.DataRange,
    byte_order: std.builtin.Endian,
    padded: bool,
) types.Error!FormatChunk {
    const chunk = ChunkHeader{ .id = "fmt ".*, .size = data.length };
    const offset = data.offset;
    _ = try data.end();
    if (chunk.size < 16)
        return error.InvalidData; // 16 bytes for base fmt fields
    try state.moveTo(offset);
    var bytes: [40]u8 = undefined;
    try state.read(bytes[0..16]);
    var fmt = FormatChunk{
        .byte_order = byte_order,
        .audio_format = @enumFromInt(
            std.mem.readInt(u16, bytes[0..2], byte_order),
        ),
        .channels = std.mem.readInt(u16, bytes[2..4], byte_order),
        .sample_rate = std.mem.readInt(u32, bytes[4..8], byte_order),
        .byte_rate = std.mem.readInt(u32, bytes[8..12], byte_order),
        .block_align = std.mem.readInt(u16, bytes[12..14], byte_order),
        .bits_per_sample = std.mem.readInt(u16, bytes[14..16], byte_order),
    };
    if (
        fmt.channels == 0
            or fmt.sample_rate == 0
            or fmt.block_align == 0
            or fmt.byte_rate == 0
    ) {
        return error.InvalidData;
    }
    if (chunk.size != 16) {
        // 18 bytes for base fmt fields and 16 bit extension size
        if (chunk.size < 18) return error.InvalidData;
        try state.read(bytes[16..18]);
        const size = std.mem.readInt(u16, bytes[16..18], byte_order);
        const format_size = @as(u64, size) + 18;
        if (format_size > chunk.size) return error.InvalidData;
        // allow up to seven bytes of wave64 alignment padding
        if (
            format_size != chunk.size
                and (!padded or chunk.size != (format_size + 7) & ~@as(u64, 7))
        )
            return error.InvalidData;
        if (fmt.audio_format == .extensible) {
            // extensible fields at byte 18: u16 valid bits, u32 channel mask, 16 byte guid
            if (size < 22) return error.InvalidData;
            try state.read(bytes[18..40]);
            fmt.extension = .{
                .payload = .{
                    .extensible = .{
                        .valid_bits_per_sample = std.mem.readInt(
                            u16,
                            bytes[18..20],
                            byte_order,
                        ),
                        .channel_mask = std.mem.readInt(
                            u32,
                            bytes[20..24],
                            byte_order,
                        ),
                        .sub_format = bytes[24..40].*,
                    },
                },
            };
        } else if (fmt.audio_format == .ima_adpcm) {
            if (size < 2) return error.InvalidData;
            try state.read(bytes[0..2]);
            const samples = std.mem.readInt(u16, bytes[0..2], byte_order);
            if (samples == 0) return error.InvalidData;
            fmt.extension = .{
                .payload = .{
                    .ima_adpcm = .{
                        .block_align = fmt.block_align,
                        .samples_per_block = samples,
                        .byte_order = byte_order,
                    },
                },
            };
        } else if (fmt.audio_format == .ms_adpcm) {
            // wSamplesPerBlock, wNumCoef, then signed coefficient pairs
            if (size < 4) return error.InvalidData;
            try state.read(bytes[0..4]);
            var config = types.MsAdpcmConfig{
                .block_align = fmt.block_align,
                .samples_per_block = std.mem.readInt(
                    u16,
                    bytes[0..2],
                    byte_order,
                ),
                .byte_order = byte_order,
                .coefficient_count = std.mem.readInt(
                    u16,
                    bytes[2..4],
                    byte_order,
                ),
            };
            if (
                config.samples_per_block < 2
                    or config.coefficient_count == 0
                    or config.coefficient_count > config.coefficients.len
                    or @as(u32, config.coefficient_count) * 4 > size - 4
            )
                return error.InvalidData;
            for (
                config.coefficients[0..config.coefficient_count],
            ) |*coefficient| {
                try state.read(bytes[0..4]);
                coefficient.* = .{
                    .first = std.mem.readInt(i16, bytes[0..2], byte_order),
                    .second = std.mem.readInt(i16, bytes[2..4], byte_order),
                };
            }
            fmt.extension = .{ .payload = .{ .ms_adpcm = config } };
        } else {
            fmt.extension = .{ .payload = .{ .codec = {} } };
        }
    } else if (
        fmt.audio_format != .pcm
            and fmt.audio_format != .ieee_float
            and fmt.audio_format != .alaw
            and fmt.audio_format != .mulaw
    ) {
        return error.InvalidData;
    }

    const encoding = fmt.encoding();
    if (encoding == .ima_adpcm or encoding == .ms_adpcm) {
        if (fmt.bits_per_sample != 4 or fmt.channels > 2)
            return error.InvalidData;
        // extensible adpcm needs its own extension interpretation before mapping
    }
    if (fmt.fixedWidth()) {
        if (fmt.bits_per_sample == 0 or fmt.block_align % fmt.channels != 0) {
            return error.InvalidData;
        }
        // derive physical sample width from complete interleaved frame size
        const container_bits = @as(u32, fmt.block_align / fmt.channels) * 8;
        if (fmt.audio_format == .extensible) {
            const valid_bits = fmt
                .extension
                .?
                .payload
                .extensible
                .valid_bits_per_sample;
            if (
                fmt.bits_per_sample != container_bits
                    or valid_bits == 0
                    or valid_bits > container_bits
            ) {
                return error.InvalidData;
            }
        } else if (fmt.bits_per_sample > container_bits) {
            return error.InvalidData;
        }
        // allow legacy pcm precision below the storage width
        if (encoding == .ieee_float) {
            if (
                (fmt.bits_per_sample != 32 and fmt.bits_per_sample != 64)
                    or container_bits != fmt.bits_per_sample
            )
                return error.InvalidData;
        }
        if (encoding == .alaw or encoding == .mulaw) {
            if (fmt.bits_per_sample != 8 or container_bits != 8) {
                return error.InvalidData;
            }
        }
        // fixed width byte rate must agree with sample rate and frame size
        if (@as(u64, fmt.sample_rate) * fmt.block_align != fmt.byte_rate) {
            return error.InvalidData;
        }
    }
    return fmt;
}

/// share chunk walking between metadata scans and sequential playback
pub const Reader = struct {
    const Self = @This();
    const ReadError = types.Error;
    /// declared decoded frame count paired with original chunk
    const Fact = struct { frames: u64 };

    source: Source,
    seekable: bool,
    allocator: std.mem.Allocator,
    options: types.Options,
    // absolute input cursor, detection already consumed first four bytes
    position: u64 = 4,
    end: u64 = 0,
    // next outer or wavl child header, may lie beyond current audio payload
    next_offset: u64 = 0,
    kind: types.Format = .wav,
    byte_order: std.builtin.Endian = .little,
    format: ?FormatChunk = null,
    table: []u8 = &.{},
    // bytes consumed from ordered ds64 override table
    table_offset: u64 = 0,
    ds64: ?DataSize64 = null,
    fact: ?Fact = null,
    audio_chunk: ?Chunk = null,
    wavl_end: ?u64 = null,
    // outer chunk cursor restored after final wavl child
    wavl_next: u64 = 0,
    frames: u64 = 0,
    finished: bool = false,

    /// open after the four byte prefix; sequential input retains the size table
    pub fn initPrefix(
        allocator: std.mem.Allocator,
        source: Source,
        options: types.Options,
        prefix: [4]u8,
        seekable: bool,
    ) ReadError!Self {
        var self = Self{
            .source = source,
            .seekable = seekable,
            .allocator = allocator,
            .options = options,
        };
        errdefer self.deinit();
        var bytes: [40]u8 = undefined;
        bytes[0..4].* = prefix;
        // riff header includes form type, rf64 and bw64 require immediate ds64
        if (!std.mem.eql(u8, &prefix, "riff")) {
            try self.read(bytes[4..12]);
            if (!std.mem.eql(u8, bytes[8..12], "WAVE")) {
                return error.UnsupportedFormat;
            }
            self.byte_order = if (std.mem.eql(u8, &prefix, "RIFX"))
                .big
            else
                .little;
            var riff_size: u64 = std.mem.readInt(
                u32,
                bytes[4..8],
                self.byte_order,
            );
            self.next_offset = 12;
            if (
                std.mem.eql(u8, &prefix, "RF64")
                    or std.mem.eql(u8, &prefix, "BW64")
            ) {
                if (riff_size != std.math.maxInt(u32)) {
                    return error.InvalidData;
                }
                var ds_header: [8]u8 = undefined;
                try self.read(&ds_header);
                const size = std.mem.readInt(u32, ds_header[4..8], .little);
                // ds64 starts with three u64 sizes or counts, then a u32 table count
                if (
                    !std.mem.eql(u8, ds_header[0..4], "ds64")
                        or size < 28
                        or size == std.math.maxInt(u32)
                ) {
                    return error.InvalidData;
                }
                if (
                    seekable
                        and @as(u64, size)
                            > (try source.getSize()
                                orelse return error.Unseekable)
                                - self.position
                ) {
                    return error.TruncatedData;
                }
                var ds_bytes: [28]u8 = undefined;
                try self.read(&ds_bytes);
                // each ds64 table entry contains fourcc and 64 bit payload size
                const table_length = @as(
                    u64,
                    std.mem.readInt(u32, ds_bytes[24..28], .little),
                )
                    * 12;
                if (table_length > size - 28) return error.InvalidData;
                self.ds64 = .{
                    .data_size = std.mem.readInt(u64, ds_bytes[8..16], .little),
                    .sample_count = if (std.mem.eql(u8, &prefix, "RF64"))
                        std.mem.readInt(u64, ds_bytes[16..24], .little)
                    else
                        null,
                    .table = .{
                        .offset = self.position,
                        .length = table_length,
                    },
                };
                if (!self.seekable) {
                    if (table_length > options.max_payload_bytes) {
                        return error.LimitExceeded;
                    }
                    self.table = try allocator.alloc(
                        u8,
                        @intCast(table_length),
                    );
                    try self.read(self.table);
                }
                // replace 32 bit placeholder with declared 64 bit riff size
                riff_size = std.mem.readInt(u64, ds_bytes[0..8], .little);
                self.next_offset = 20 + @as(u64, size) + (size & 1);
            }
            if (riff_size < 4) return error.InvalidData;
            self.end = std.math.add(u64, riff_size, 8)
                catch return error.InvalidData;
        } else {
            try self.read(bytes[4..40]);
            if (
                !std.mem.eql(u8, bytes[0..16], &wave64.riff_guid)
                    or !std.mem.eql(u8, bytes[24..40], &wave64.wave_guid)
            ) {
                return error.UnsupportedFormat;
            }
            // wave64 file size includes outer header
            self.kind = .wave64;
            self.end = std.mem.readInt(u64, bytes[16..24], .little);
            if (self.end < 40) return error.InvalidData;
            self.next_offset = 40;
        }
        if (
            self.seekable
                and self.end
                    > (try source.getSize() orelse return error.Unseekable)
        ) {
            return error.TruncatedData;
        }
        if (self.next_offset > self.end) return error.InvalidData;
        return self;
    }

    /// leave input open
    pub fn deinit(self: *Self) void {
        if (!self.seekable) {
            self.allocator.free(self.table);
        }
    }

    /// read exactly requested bytes and advance absolute position
    pub fn read(self: *Self, out: []u8) ReadError!void {
        try readRequired(self.source, out);
        self.position = std.math.add(u64, self.position, out.len)
            catch return error.InvalidData;
    }

    /// seek scan input or discard forward bytes for sequential input
    fn moveTo(self: *Self, offset: u64) ReadError!void {
        if (!self.seekable) {
            // sequential reader cannot revisit bytes already consumed
            if (offset < self.position) return error.InvalidState;
            var scratch: [8192]u8 = undefined;
            while (self.position < offset) {
                try self.read(
                    scratch[
                        0..@intCast(@min(scratch.len, offset - self.position))
                    ],
                );
            }
        } else {
            try self.source.seekTo(offset);
            self.position = offset;
        }
    }

    /// resolve chunk offsets and sizes for riff and wave64 headers
    fn readChunk(self: *Self, end: u64) ReadError!Chunk {
        const offset = self.position;
        const header_size: usize = if (self.kind == .wave64) 24 else 8;
        if (offset > end or end - offset < header_size) {
            return error.InvalidData;
        }
        var bytes: [24]u8 = undefined;
        try self.read(bytes[0..header_size]);
        if (self.kind == .wave64) {
            const size = std.mem.readInt(u64, bytes[16..24], .little);
            if (size < 24) return error.InvalidData;
            // only standard wave guids can be identified by fourcc
            const id: [4]u8 = if (
                std.mem.eql(u8, bytes[4..16], wave64.data_guid[4..])
            )
                bytes[0..4].*
            else .{ 0, 0, 0, 0 };
            // convert inclusive wave64 sizes to payload lengths
            return frame(
                .{ .id = id, .size = size - 24 },
                offset,
                end,
                .wave64,
            );
        }
        var header = ChunkHeader{
            .id = bytes[0..4].*,
            .size = std.mem.readInt(u32, bytes[4..8], self.byte_order),
        };
        if (self.ds64) |ds64| {
            if (std.mem.eql(u8, &header.id, "ds64")) {
                return error.InvalidData;
            }
            // all ones marks ds64 override, first data has dedicated size
            if (header.size == std.math.maxInt(u32)) {
                if (
                    std.mem.eql(u8, &header.id, "data")
                        and self.audio_chunk == null
                ) {
                    header.size = ds64.data_size;
                } else {
                    if (self.table_offset == ds64.table.length) {
                        return error.InvalidData;
                    }
                    var entry: [12]u8 = undefined;
                    if (!self.seekable) {
                        @memcpy(
                            &entry,
                            self.table[@intCast(self.table_offset)..][0..12],
                        );
                    } else {
                        try self.moveTo(ds64.table.offset + self.table_offset);
                        try self.read(&entry);
                        try self.moveTo(offset + 8);
                    }
                    // size table consumed in order, identifier must match current placeholder
                    if (!std.mem.eql(u8, &header.id, entry[0..4])) {
                        return error.InvalidData;
                    }
                    header.size = std.mem.readInt(u64, entry[4..12], .little);
                    self.table_offset += 12;
                }
            }
        }
        return frame(header, offset, end, .wav);
    }

    /// walk metadata and return audio segments; leave input at their data
    pub fn next(self: *Self) ReadError!?types.Segment {
        if (self.finished) return null;
        while (true) {
            // return to outer chunk walk after final audio list child
            if (self.wavl_end) |end| {
                if (self.next_offset == end) {
                    self.wavl_end = null;
                    self.next_offset = self.wavl_next;
                }
            }
            const end = self.wavl_end orelse self.end;
            if (self.next_offset > end) return error.InvalidData;
            if (
                self.next_offset == self.end
                    or (self.kind == .wave64
                        and self.audio_chunk != null
                        and end - self.next_offset < 24)
            ) {
                // allow a short trailing wave64 header after data
                if (self.next_offset == self.end) try self.moveTo(self.end);
                try self.validateEnd();
                self.finished = true;
                return null;
            }
            try self.moveTo(self.next_offset);
            const chunk = try self.readChunk(end);
            self.next_offset = chunk.next_offset;
            const id = chunk.header.id;
            if (std.mem.eql(u8, &id, "fmt ") and self.wavl_end == null) {
                if (self.format != null) return error.InvalidData;
                if (
                    !self.seekable
                        and chunk.data.length > self.options.bufferCapacity()
                )
                    return error.LimitExceeded;
                self.format = try parseFormatPayload(
                    self,
                    chunk.data,
                    self.byte_order,
                    self.kind == .wave64,
                );
            } else if (std.mem.eql(u8, &id, "fact") and self.wavl_end == null) {
                // fact count uses 64 bits in wave64, 32 bits in riff
                const size: usize = if (self.kind == .wave64) 8 else 4;
                if (self.fact != null or chunk.data.length < size) {
                    return error.InvalidData;
                }
                var bytes: [8]u8 = undefined;
                try self.read(bytes[0..size]);
                self.fact = .{
                    .frames = if (size == 8)
                        std.mem.readInt(u64, &bytes, .little)
                    else
                        std.mem.readInt(u32, bytes[0..4], self.byte_order),
                };
            } else if (std.mem.eql(u8, &id, "data")) {
                const format = self.format orelse return error.InvalidData;
                if (self.wavl_end == null) {
                    if (self.audio_chunk != null) {
                        return error.UnsupportedChunkLayout;
                    }
                    self.audio_chunk = chunk;
                }
                if (format.fixedWidth()) {
                    if (
                        self.kind != .wave64
                            and chunk.data.length % format.block_align != 0
                    ) {
                        return error.InvalidData;
                    }
                    self.frames = std.math.add(
                        u64,
                        self.frames,
                        chunk.data.length / format.block_align,
                    )
                        catch return error.InvalidData;
                }
                return .{ .data = chunk.data };
            } else if (std.mem.eql(u8, &id, "slnt") and self.wavl_end != null) {
                if (chunk.data.length != 4) return error.InvalidData;
                var bytes: [4]u8 = undefined;
                try self.read(&bytes);
                const count = std.mem.readInt(u32, &bytes, self.byte_order);
                self.frames = std.math.add(u64, self.frames, count)
                    catch return error.InvalidData;
                return .{ .silence = count };
            } else if (std.mem.eql(u8, &id, "LIST") and self.kind == .wav) {
                if (chunk.data.length < 4) return error.InvalidData;
                var kind: [4]u8 = undefined;
                try self.read(&kind);
                if (std.mem.eql(u8, &kind, "wavl")) {
                    if (
                        self.wavl_end != null
                            or self.audio_chunk != null
                            or self.ds64 != null
                    ) {
                        return error.UnsupportedChunkLayout;
                    }
                    if (self.format == null) return error.InvalidData;
                    self.audio_chunk = chunk;
                    self.wavl_end = try chunk.data.end();
                    self.wavl_next = self.next_offset;
                    self.next_offset = self.position;
                }
            }
        }
    }

    /// resolve padding from the fact count or complete frames
    pub fn wave64DataLength(self: *const Self) types.ValidationError!u64 {
        const chunk = self.audio_chunk.?;
        const alignment = self.format.?.block_align;
        // prefer declared frame count, otherwise round payload down to whole frames
        const length = if (self.fact) |fact|
            std.math.mul(u64, fact.frames, alignment)
                catch return error.InvalidData
        else
            chunk.data.length - chunk.data.length % alignment;
        // wave64 data sizes may include seven padding bytes
        if (length > chunk.data.length or chunk.data.length - length > 7) {
            return error.InvalidData;
        }
        if (length != chunk.data.length and chunk.header.size & 7 != 0) {
            return error.InvalidData;
        }
        return length;
    }

    /// require format and audio layout, reconcile final counts and wave64 padding
    fn validateEnd(self: *Self) types.ValidationError!void {
        const format = self.format orelse return error.InvalidData;
        if (self.audio_chunk == null) return error.InvalidData;
        if (self.kind == .wave64) {
            if (format.fixedWidth()) {
                const length = try self.wave64DataLength();
                self.frames = length / format.block_align;
            }
        } else {
            // non pcm or segmented riff needs declared count from fact or ds64
            const segmented = std.mem.eql(
                u8,
                &self.audio_chunk.?.header.id,
                "LIST",
            );
            if (
                (format.encoding() != .pcm or segmented)
                    and self.fact == null
                    and self.ds64 == null
            ) {
                return error.InvalidData;
            }
            if (
                segmented
                    and format.fixedWidth()
                    and self.fact.?.frames != self.frames
            ) {
                return error.InvalidData;
            }
        }
    }

    pub fn info(self: *const Self) types.Info {
        const fmt = self.format.?;
        var total: ?u64 = if (self.finished and fmt.fixedWidth())
            self.frames
        else
            null;
        if (self.finished and total == null) {
            if (self.ds64) |ds64| {
                if (ds64.sample_count) |count| {
                    if (count != 0) total = count;
                }
            }
            if (total == null) {
                if (self.fact) |fact| {
                    if (
                        self.ds64 == null
                            or fact.frames != std.math.maxInt(u32)
                    )
                        total = fact.frames;
                }
            }
        }
        return .{
            .format = self.kind,
            .sample_rate = fmt.sample_rate,
            .channels = fmt.channels,
            .channel_mask = fmt.channelMask(),
            .total_frames = total,
        };
    }
};

pub fn probe(prefix: []const u8) bool {
    return std.mem.eql(u8, prefix, "RIFF")
        or std.mem.eql(u8, prefix, "RIFX")
        or std.mem.eql(u8, prefix, "RF64")
        or std.mem.eql(u8, prefix, "BW64")
        or std.mem.eql(u8, prefix, "riff");
}

const wave64 = struct {
    const riff_guid = "riff\x2e\x91\xcf\x11\xa5\xd6\x28\xdb\x04\xc1\x00\x00".*;
    const wave_guid = "wave\xf3\xac\xd3\x11\x8c\xd1\x00\xc0\x4f\x8e\xdb\x8a".*;
    const data_guid = "data\xf3\xac\xd3\x11\x8c\xd1\x00\xc0\x4f\x8e\xdb\x8a".*;
};
