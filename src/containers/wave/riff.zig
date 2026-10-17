//! parse riff, rifx, rf64, bw64, and wave64 with a shared chunk walker
const std = @import("std");
const types = @import("../../types.zig");
const Source = @import("../../Source.zig");
const declarations = @This();
pub const Error = types.ParserError;
pub const ParseError = types.ParserError;

/// store a fourcc header with resolved payload size
pub const ChunkHeader = struct {
    id: [4]u8,
    /// payload bytes, excluding header and padding
    size: u64,
};

/// fill output or report truncated input
pub fn readRequired(source: Source, out: []u8) ParseError!void {
    return readExact(source.reader, out);
}

/// fill output, map physical eof to truncated data
fn readExact(
    reader: *std.Io.Reader,
    out: []u8,
) (types.ReadError || error{ TruncatedData })!void {
    reader.readSliceAll(out)
        catch |err| return switch (err) {
            error.EndOfStream => error.TruncatedData,
            else => |read_error| read_error,
        };
}

/// outer riff chunk size and form identifier
pub const RiffHeader = struct {
    chunk: ChunkHeader, // riff, rifx, rf64 or bw64; resolved file size minus 8
    format: [4]u8, // form type; interpreted by the caller

    /// rifx is big endian; other variants are little endian
    pub fn byteOrder(self: RiffHeader) std.builtin.Endian {
        return if (std.mem.eql(u8, &self.chunk.id, "RIFX")) .big else .little;
    }
};

/// rf64 or bw64 size overrides and borrowed ordered size table
pub const DataSize64 = struct {
    riff_size: u64,
    data_size: u64,
    /// rf64 samples per channel; zero is unspecified, null for bw64
    sample_count: ?u64,
    /// borrow 12 byte entries, fourcc then little endian u64
    table: types.DataRange,
};

/// resolved chunk payload and next aligned header offset
pub const Chunk = struct {
    header: ChunkHeader,
    data: types.DataRange,
    next_offset: u64,
};

/// read a header within the exclusive end bound; leave input at its payload
pub fn readHeader(
    source: Source,
    offset: u64,
    end: u64,
    byte_order: std.builtin.Endian,
) Error!ChunkHeader {
    // 8 bytes, fourcc plus u32 payload size
    if (offset > end or end - offset < 8) return error.InvalidData;
    try source.seekTo(offset);
    var bytes: [8]u8 = undefined;
    try readRequired(source, &bytes);
    return .{
        .id = bytes[0..4].*,
        .size = std.mem.readInt(u32, bytes[4..8], byte_order),
    };
}

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
    ieee_float = 0x0003,
    alaw = 0x0006,
    mulaw = 0x0007,
    extensible = 0xfffe,
    _,
};

/// store parsed format fields and borrowed extension ranges
pub const FormatChunk = struct {
    chunk: ChunkHeader, // fmt chunk identifier
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

    /// resolve standard subtypes; preserve unknown tags
    /// return null for custom or missing subtypes
    pub fn encoding(self: FormatChunk) ?FormatTag {
        if (self.audio_format != .extensible) return self.audio_format;
        const extension = self.extension orelse return null;
        const ext = switch (extension.payload) {
            .extensible => |ext| ext,
            .codec => return null,
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
            // wider pcm containers may hide mislabeled floats; reject
            // ambiguity
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

/// store extra format data after the base fields
pub const FormatExtension = struct {
    /// extension bytes, excluding the size field
    size: u16,
    payload: union(enum) {
        /// borrow codec data, possibly empty
        codec: types.DataRange,
        /// store extensible format fields
        extensible: ExtensibleFormat,
    },
};

/// describe the extensible sample layout and subtype
pub const ExtensibleFormat = struct {
    /// significant bits within the sample storage width
    valid_bits_per_sample: u16,
    /// speaker positions; zero means unspecified
    channel_mask: u32,
    /// subtype guid in file byte order; resolve through the encoding helper
    sub_format: [16]u8,
    /// borrow trailing extension data, possibly empty
    extra_data: types.DataRange = .{},
};

/// parse a format payload; leave input within it
/// keep input unchanged for borrowed ranges
pub fn parseFormat(
    source: Source,
    data: types.DataRange,
    byte_order: std.builtin.Endian,
) ParseError!FormatChunk {
    return parseFormatPayload(source, data, byte_order, false);
}

/// parse little endian wave64 format fields with alignment padding
pub fn parseWave64Format(
    source: Source,
    data: types.DataRange,
) ParseError!FormatChunk {
    return parseFormatPayload(source, data, .little, true);
}

/// validate format fields; padded allows trailing eight byte alignment
fn parseFormatPayload(
    source: Source,
    data: types.DataRange,
    byte_order: std.builtin.Endian,
    padded: bool,
) ParseError!FormatChunk {
    const chunk = ChunkHeader{ .id = "fmt ".*, .size = data.length };
    const offset = data.offset;
    _ = try data.end();
    if (chunk.size < 16)
        return error.InvalidData; // 16 bytes for base fmt fields
    try source.seekTo(offset);
    var bytes: [40]u8 = undefined;
    try readRequired(source, bytes[0..16]);
    var fmt = FormatChunk{
        .chunk = chunk,
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
        try readRequired(source, bytes[16..18]);
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
            // 22 bytes, valid bits 2 bytes, channel mask 4 bytes, guid 16 bytes
            if (size < 22) return error.InvalidData;
            // fixed extensible fields occupy 22 bytes starting at byte 18
            try readRequired(source, bytes[18..40]);
            fmt.extension = .{
                .size = size,
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
                        .extra_data = .{
                            .offset = offset + 40,
                            .length = size - 22,
                        },
                    },
                },
            };
        } else {
            fmt.extension = .{
                .size = size,
                .payload = .{
                    .codec = .{ .offset = offset + 18, .length = size },
                },
            };
        }
    } else if (
        fmt.audio_format != .pcm
            and fmt.audio_format != .ieee_float
            and fmt.audio_format != .alaw
            and fmt.audio_format != .mulaw
    ) {
        return error.InvalidData;
    }

    // validate sample layout for known encodings
    const encoding = fmt.encoding();
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

/// iterate data and silence chunks in a wavl list
pub const SegmentList = struct {
    /// child chunks, excluding the list header and wavl identifier
    children: types.DataRange,
    byte_order: std.builtin.Endian,

    /// iterate segments; keep source contents unchanged
    pub fn iterate(self: SegmentList, source: Source) SegmentIterator {
        return .{
            .source = source,
            .list = self,
            .offset = self.children.offset,
        };
    }
};

/// seekable cursor through borrowed wavl children
pub const SegmentIterator = struct {
    source: Source,
    list: SegmentList,
    offset: u64,

    /// return data or silence, skipping other chunks; null ends the list
    /// iteration repositions input
    pub fn next(self: *SegmentIterator) ParseError!?types.Segment {
        const end = try self.list.children.end();
        while (self.offset != end) {
            const header = try readHeader(
                self.source,
                self.offset,
                end,
                self.list.byte_order,
            );
            const chunk = try frame(header, self.offset, end, .wav);
            // advance past payload and alignment even when chunk ignored
            self.offset = chunk.next_offset;
            if (std.mem.eql(u8, &header.id, "data")) {
                return .{ .data = chunk.data };
            }
            if (std.mem.eql(u8, &header.id, "slnt")) {
                // four bytes for 32 bit frame count
                if (header.size != 4) return error.InvalidData;
                var count: [4]u8 = undefined;
                try readRequired(self.source, &count);
                return .{
                    .silence = std.mem.readInt(
                        u32,
                        &count,
                        self.list.byte_order,
                    ),
                };
            }
            if (std.mem.eql(u8, &header.id, "LIST")) {
                // 4 bytes hold the list type
                if (header.size < 4) return error.InvalidData;
                var kind: [4]u8 = undefined;
                try readRequired(self.source, &kind);
                // nested audio lists unsupported
                if (std.mem.eql(u8, &kind, "wavl")) {
                    return error.UnsupportedChunkLayout;
                }
            }
        }
        return null;
    }
};

/// share chunk walking between metadata scans and sequential playback
/// scans borrow ranges; streaming retains format and size table bytes
pub fn Reader(comptime streaming: bool) type {
    return struct {
        const Self = @This();
        const Allocator = if (streaming) std.mem.Allocator else void;
        const Options = if (streaming) types.Options else void;
        const ReadError = if (streaming) types.Error else types.ParserError;
        /// declared decoded frame count paired with original chunk
        const Fact = struct { chunk: Chunk, frames: u64 };

        source: Source,
        allocator: Allocator,
        options: Options,
        // absolute input cursor, detection already consumed first four bytes
        position: u64 = 4,
        end: u64 = 0,
        // next outer or wavl child header, may lie beyond current audio payload
        next_offset: u64 = 0,
        kind: types.Format = .wav,
        byte_order: std.builtin.Endian = .little,
        header: RiffHeader = undefined,
        format: ?FormatChunk = null,
        /// owned streaming format storage; seekable scans borrow ranges
        format_bytes: []u8 = &.{},
        table: []u8 = &.{},
        // bytes consumed from ordered ds64 override table
        table_offset: u64 = 0,
        ds64: ?DataSize64 = null,
        fact: ?Fact = null,
        audio_chunk: ?Chunk = null,
        layout: ?types.AudioData = null,
        first_data: ?u64 = null,
        wavl_end: ?u64 = null,
        // outer chunk cursor restored after final wavl child
        wavl_next: u64 = 0,
        frames: u64 = 0,
        finished: bool = false,

        /// open after the four byte prefix; streaming allocates format storage
        /// and the declared size table within configured limits
        pub fn initPrefix(
            allocator: Allocator,
            source: Source,
            options: Options,
            prefix: [4]u8,
        ) ReadError!Self {
            var self = Self{
                .source = source,
                .allocator = allocator,
                .options = options,
            };
            errdefer self.deinit();
            if (streaming) {
                self.format_bytes = try allocator.alloc(
                    u8,
                    options.bufferCapacity(),
                );
            }
            var bytes: [40]u8 = undefined;
            bytes[0..4].* = prefix;
            // riff header includes form type, rf64 and bw64 require immediate ds64
            if (wav.probe(&prefix)) {
                try self.read(bytes[4..12]);
                if (!std.mem.eql(u8, bytes[8..12], "WAVE")) {
                    return error.UnsupportedFormat;
                }
                self.byte_order = if (std.mem.eql(u8, &prefix, "RIFX"))
                    .big
                else
                    .little;
                self.header = .{
                    .chunk = .{
                        .id = prefix,
                        .size = std.mem.readInt(
                            u32,
                            bytes[4..8],
                            self.byte_order,
                        ),
                    },
                    .format = "WAVE".*,
                };
                self.next_offset = 12;
                if (
                    std.mem.eql(u8, &prefix, "RF64")
                        or std.mem.eql(u8, &prefix, "BW64")
                ) {
                    if (self.header.chunk.size != std.math.maxInt(u32)) {
                        return error.InvalidData;
                    }
                    var ds_header: [8]u8 = undefined;
                    try self.read(&ds_header);
                    const size = std.mem.readInt(u32, ds_header[4..8], .little);
                    // ds64 begins with three u64 sizes or counts and a u32 table
                    // count
                    if (
                        !std.mem.eql(u8, ds_header[0..4], "ds64")
                            or size < 28
                            or size == std.math.maxInt(u32)
                    ) {
                        return error.InvalidData;
                    }
                    if (
                        !streaming
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
                        .riff_size = std.mem.readInt(
                            u64,
                            ds_bytes[0..8],
                            .little,
                        ),
                        .data_size = std.mem.readInt(
                            u64,
                            ds_bytes[8..16],
                            .little,
                        ),
                        .sample_count = if (std.mem.eql(u8, &prefix, "RF64"))
                            std.mem.readInt(u64, ds_bytes[16..24], .little)
                        else
                            null,
                        .table = .{
                            .offset = self.position,
                            .length = table_length,
                        },
                    };
                    if (streaming) {
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
                    self.header.chunk.size = self.ds64.?.riff_size;
                    self.next_offset = 20 + @as(u64, size) + (size & 1);
                }
                if (self.header.chunk.size < 4) return error.InvalidData;
                self.end = std.math.add(u64, self.header.chunk.size, 8)
                    catch return error.InvalidData;
            } else if (wave64.probe(&prefix)) {
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
            } else return error.UnsupportedFormat;
            if (
                !streaming
                    and self.end
                        > (try source.getSize() orelse return error.Unseekable)
            ) {
                return error.TruncatedData;
            }
            if (self.next_offset > self.end) return error.InvalidData;
            return self;
        }

        /// free streaming metadata storage; leave input open
        pub fn deinit(self: *Self) void {
            if (streaming) {
                self.allocator.free(self.table);
                self.allocator.free(self.format_bytes);
            }
        }

        /// read exactly requested bytes and advance absolute position
        pub fn read(self: *Self, out: []u8) ReadError!void {
            try readExact(self.source.reader, out);
            self.position = std.math.add(u64, self.position, out.len)
                catch return error.InvalidData;
        }

        /// seek scan input or discard forward bytes for sequential input
        fn moveTo(self: *Self, offset: u64) ReadError!void {
            if (streaming) {
                // sequential reader cannot revisit bytes already consumed
                if (offset < self.position) return error.InvalidState;
                var scratch: [8192]u8 = undefined;
                while (self.position < offset) {
                    try self.read(
                        scratch[
                            0..@intCast(
                                @min(scratch.len, offset - self.position),
                            )
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
                        if (streaming) {
                            @memcpy(
                                &entry,
                                self.table[@intCast(self.table_offset)..][
                                    0..12
                                ],
                            );
                        } else {
                            try self.moveTo(
                                ds64.table.offset + self.table_offset,
                            );
                            try self.read(&entry);
                            try self.moveTo(offset + 8);
                        }
                        // size table consumed in order, identifier must match current placeholder
                        if (!std.mem.eql(u8, &header.id, entry[0..4])) {
                            return error.InvalidData;
                        }
                        header.size = std.mem.readInt(
                            u64,
                            entry[4..12],
                            .little,
                        );
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
                    if (streaming) {
                        if (
                            chunk.data.length > self.options.max_payload_bytes
                                or chunk.data.length > self.format_bytes.len
                        ) {
                            return error.LimitExceeded;
                        }
                        const bytes = self.format_bytes[
                            0..@intCast(chunk.data.length)
                        ];
                        try self.read(bytes);
                        // reuse seekable format parser on retained streaming bytes
                        var reader = std.Io.Reader.fixed(bytes);
                        self.format = try parseFormatPayload(
                            Source.fromFixed(&reader),
                            .{ .length = bytes.len },
                            self.byte_order,
                            self.kind == .wave64,
                        );
                    } else {
                        self.format = try parseFormatPayload(
                            self.source,
                            chunk.data,
                            self.byte_order,
                            self.kind == .wave64,
                        );
                    }
                } else if (
                    std.mem.eql(u8, &id, "fact")
                        and self.wavl_end == null
                ) {
                    // fact count uses 64 bits in wave64, 32 bits in riff
                    const size: usize = if (self.kind == .wave64) 8 else 4;
                    if (self.fact != null or chunk.data.length < size) {
                        return error.InvalidData;
                    }
                    var bytes: [8]u8 = undefined;
                    try self.read(bytes[0..size]);
                    self.fact = .{
                        .chunk = chunk,
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
                        self.layout = types.AudioData.single(
                            .{ .data = chunk.data },
                        );
                    }
                    // remember first payload for restoring position after metadata scan
                    if (self.first_data == null) {
                        self.first_data = chunk.data.offset;
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
                } else if (
                    std.mem.eql(u8, &id, "slnt")
                        and self.wavl_end != null
                ) {
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
                        self.layout = .{
                            .layout = .{
                                .wavl = .{
                                    .children = .{
                                        .offset = self.position,
                                        .length = chunk.data.length - 4,
                                    },
                                    .byte_order = self.byte_order,
                                },
                            },
                        };
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
                    self.layout.?.layout.single.data.length = length;
                    self.frames = length / format.block_align;
                }
            } else {
                // non pcm or segmented riff needs declared count from fact or ds64
                const segmented = self.layout.?.layout == .wavl;
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

        /// expose validated scan as borrowed riff metadata and audio ranges
        fn wavFile(self: Self) wav.WavFile {
            return .{
                .header = .{
                    .riff = self.header,
                    .ds64 = self.ds64,
                    .fmt = self.format.?,
                    .fact = if (self.fact) |fact| .{
                        .chunk = fact.chunk.header,
                        .sample_length = @intCast(fact.frames),
                        .extra_data = .{
                            .offset = fact.chunk.data.offset + 4,
                            .length = fact.chunk.data.length - 4,
                        },
                    } else null,
                    .data = self.audio_chunk.?.header,
                    .data_offset = self.audio_chunk.?.data.offset,
                },
                .data = self.layout.?,
                .decoded_frames = if (self.format.?.fixedWidth())
                    self.frames
                else
                    null,
            };
        }

        /// expose validated scan with inclusive wave64 sizes and borrowed audio ranges
        fn wave64File(self: Self) wave64.Wave64File {
            return .{
                .header = .{
                    .riff = .{ .id = wave64.riff_guid, .size = self.end },
                    .fmt = self.format.?,
                    .fact = if (self.fact) |fact| .{
                        .chunk = .{
                            .id = wave64.fact_guid,
                            .size = fact.chunk.header.size + 24,
                        },
                        .sample_length = fact.frames,
                        .extra_data = .{
                            .offset = fact.chunk.data.offset + 8,
                            .length = fact.chunk.data.length - 8,
                        },
                    } else null,
                    .data = .{
                        .id = wave64.data_guid,
                        .size = self.audio_chunk.?.header.size + 24,
                    },
                    .data_offset = self.audio_chunk.?.data.offset,
                },
                .data = self.layout.?,
                .decoded_frames = if (self.format.?.fixedWidth())
                    self.frames
                else
                    null,
            };
        }
    };
}

/// scan metadata using the playback chunk walker
fn scan(
    source: Source,
    expected: ?types.Format,
) types.ParserError!Reader(false) {
    try source.seekTo(0);
    var prefix: [4]u8 = undefined;
    if (try source.reader.readSliceShort(&prefix) != prefix.len) {
        return if (expected == null)
            error.UnsupportedFormat
        else
            error.TruncatedData;
    }
    var reader = try Reader(false).initPrefix({}, source, {}, prefix);
    if (expected) |kind| {
        if (reader.kind != kind) return error.UnsupportedFormat;
    }
    // scan every chunk so trailing metadata and count validation are included
    while (try reader.next()) |_| {}
    try source.seekTo(
        reader.first_data orelse reader.audio_chunk.?.data.offset,
    );
    return reader;
}

/// scan wav or wave64 metadata, leave input at first audio data
pub fn parse(source: Source) types.ParserError!types.ParsedAudio {
    const reader = try scan(source, null);
    return if (reader.kind == .wave64)
        reader.wave64File().audio()
    else
        reader.wavFile().audio();
}

/// expose riff metadata through the shared chunk walker
pub const wav = struct {
    /// chunk identifier and declared size, header inclusion depends on container
    pub const ChunkHeader = declarations.ChunkHeader;
    /// outer riff chunk size and form identifier
    pub const RiffHeader = declarations.RiffHeader;
    /// rf64 or bw64 size overrides and borrowed ordered size table
    pub const DataSize64 = declarations.DataSize64;
    pub const FormatTag = declarations.FormatTag;
    pub const FormatChunk = declarations.FormatChunk;
    pub const FormatExtension = declarations.FormatExtension;
    pub const ExtensibleFormat = declarations.ExtensibleFormat;
    pub const ParseError = declarations.ParseError;
    pub const SegmentList = declarations.SegmentList;
    /// seekable cursor through borrowed wavl children
    pub const SegmentIterator = declarations.SegmentIterator;
    pub const parseFormat = declarations.parseFormat;
    pub const parseWave64Format = declarations.parseWave64Format;

    /// declared decoded frames and borrowed trailing fact data
    pub const FactChunk = struct {
        chunk: wav.ChunkHeader, // fact chunk identifier, payload at least four bytes
        /// decoded samples per channel, including compressed audio
        sample_length: u32,
        /// borrow bytes after the sample count, if any
        extra_data: types.DataRange = .{},
    };

    /// collect wave metadata while walking chunks
    pub const WavHeader = struct {
        riff: wav.RiffHeader,
        ds64: ?wav.DataSize64 = null,
        fmt: wav.FormatChunk,
        /// declared decoded frame count; optional for pcm
        fact: ?FactChunk = null,
        // data or wavl list size, excluding header and padding
        data: wav.ChunkHeader,
        /// byte offset of the audio chunk payload
        data_offset: u64,
    };

    /// store wave metadata and source ranges
    pub const WavFile = struct {
        header: WavHeader,
        data: types.AudioData,
        /// count fixed width frames, including silence
        decoded_frames: ?u64,

        /// return metadata and encoding from a parsed file
        pub fn audio(self: WavFile) types.ParsedAudio {
            return .{
                .info = .{
                    .format = .wav,
                    .sample_rate = self.header.fmt.sample_rate,
                    .channels = self.header.fmt.channels,
                    .total_frames = self.frameCount(),
                },
                .scheme = self.header.fmt.decodingScheme(
                    self.header.riff.byteOrder(),
                ),
                .data = self.data,
            };
        }

        /// return total frames, or null if unknown; use a parsed file
        pub fn frameCount(self: WavFile) ?u64 {
            // prefer measured fixed width frames over declared count
            if (self.decoded_frames) |frames| return frames;
            // rf64 nonzero sample count takes priority over fact
            if (self.header.ds64) |ds64| {
                if (ds64.sample_count) |count| {
                    if (count != 0) return count;
                }
            }
            if (self.header.fact) |fact| {
                if (
                    self.header.ds64 == null
                        or fact.sample_length != std.math.maxInt(u32)
                ) {
                    return fact.sample_length;
                }
            }
            return null;
        }
    };

    /// dispatch short recognized prefixes so parsing can report truncation
    pub const probe_size = 12; // 12 bytes for chunk identifier, size and wave form type

    /// recognize wave prefixes, including truncated headers
    pub fn probe(prefix: []const u8) bool {
        if (prefix.len < 4) return false; // four bytes for fourcc
        const id = prefix[0..4];
        if (
            !std.mem.eql(u8, id, "RIFF")
                and !std.mem.eql(u8, id, "RIFX")
                and !std.mem.eql(u8, id, "RF64")
                and !std.mem.eql(u8, id, "BW64")
        )
            return false;
        // wave form type occupies four bytes starting at byte 8
        return prefix.len < probe_size
            or std.mem.eql(u8, prefix[8..12], "WAVE");
    }

    /// validate headers and layout; leave input at the first audio data
    pub fn parse(source: Source) wav.ParseError!WavFile {
        return (try scan(source, .wav)).wavFile();
    }
};

/// expose wave64 metadata with guid identifiers and inclusive sizes
pub const wave64 = struct {
    pub const ParseError = declarations.ParseError;
    pub const FormatChunk = declarations.FormatChunk;
    pub const Guid = [16]u8;
    pub const riff_guid: Guid = "riff\x2e\x91\xcf\x11\xa5\xd6\x28\xdb\x04\xc1\x00\x00".*;
    pub const wave_guid = guid("wave");
    pub const fmt_guid = guid("fmt ");
    pub const fact_guid = guid("fact");
    pub const data_guid = guid("data");

    /// build a wave64 guid from a four character identifier
    fn guid(comptime id: *const [4]u8) Guid {
        return (id.* ++ "\xf3\xac\xd3\x11\x8c\xd1\x00\xc0\x4f\x8e\xdb\x8a").*;
    }

    /// chunk identifier and declared size, header inclusion depends on container
    pub const ChunkHeader = struct {
        id: Guid,
        /// on disk bytes, including the 24 byte header
        size: u64,
    };

    /// declared decoded frames and borrowed trailing fact data
    pub const FactChunk = struct {
        chunk: wave64.ChunkHeader,
        sample_length: u64,
        extra_data: types.DataRange,
    };

    /// validated wave64 format and audio chunk metadata
    pub const Wave64Header = struct {
        riff: wave64.ChunkHeader,
        fmt: wave64.FormatChunk,
        fact: ?FactChunk,
        data: wave64.ChunkHeader,
        data_offset: u64,
    };

    /// wave64 metadata with borrowed audio ranges and optional decoded count
    pub const Wave64File = struct {
        header: Wave64Header,
        /// borrow one data segment, trimmed to complete fixed width frames
        data: types.AudioData,
        decoded_frames: ?u64,

        /// return total frames, or null if unknown; use a parsed file
        pub fn frameCount(self: Wave64File) ?u64 {
            // prefer measured fixed width frames over declared count
            if (self.decoded_frames) |frames| return frames;
            return if (self.header.fact) |fact| fact.sample_length else null;
        }

        /// return metadata and encoding from a parsed file
        pub fn audio(self: Wave64File) types.ParsedAudio {
            return .{
                .info = .{
                    .format = .wave64,
                    .sample_rate = self.header.fmt.sample_rate,
                    .channels = self.header.fmt.channels,
                    .total_frames = self.frameCount(),
                },
                .scheme = self.header.fmt.decodingScheme(.little),
                .data = self.data,
            };
        }
    };

    // 40 bytes, riff guid 16 bytes, size 8 bytes, wave guid 16 bytes
    pub const probe_size = 40;

    /// recognize wave64 prefixes, including truncated headers
    pub fn probe(prefix: []const u8) bool {
        if (prefix.len < 4) return false; // four bytes for riff prefix of guid
        // recognize available guid prefix so full parser can report truncation
        const riff_length = @min(prefix.len, 16);
        if (
            !std.mem.eql(u8, prefix[0..riff_length], riff_guid[0..riff_length])
        ) {
            return false;
        }
        if (prefix.len <= 24) return true; // wave guid starts at byte 24
        const wave_length = @min(prefix.len - 24, 16);
        return std.mem.eql(
            u8,
            prefix[24..][0..wave_length],
            wave_guid[0..wave_length],
        );
    }

    /// validate headers and layout; leave input at the first audio data
    pub fn parse(source: Source) wave64.ParseError!Wave64File {
        return (try scan(source, .wave64)).wave64File();
    }
};
