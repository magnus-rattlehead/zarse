//! libopus multistream decoder
const std = @import("std");
const types = @import("../types.zig");
const Codec = @import("../Codec.zig");
const libopus = @cImport({
        @cInclude("opus_multistream.h");
    });
const Self = @This();

pub const Error = types.Error;
pub const Config = types.OpusConfig;

// use extra seek history to settle energy prediction beyond the 80 ms
// minimum recommended by rfc 7845
pub const preroll_frames: u64 = 24000; // 500 ms at 48 khz

allocator: std.mem.Allocator,
/// owned decoder state; keep its address stable and aligned to 16 bytes
storage: []align(16) u8,
config: Config,
format: types.AudioFormat,

/// allocate a 48 khz decoder and output buffer in source channel order
/// libopus uses stack scratch while decoding
pub fn init(
    allocator: std.mem.Allocator,
    config: Config,
    format: types.AudioFormat,
    options: types.Options,
) Error!Codec {
    if (
        format.sample_rate != 48000
            or format.channels == 0
            or format.channels > 8
            or config.streams == 0
            or config.coupled_streams > config.streams
    ) {
        return error.InvalidData;
    }
    // query libopus state size for coded stream topology
    const state_bytes = libopus.opus_multistream_decoder_get_size(
        config.streams,
        config.coupled_streams,
    );
    if (state_bytes <= 0) return error.InvalidData;
    // own aligned storage so decoder address stays stable
    const storage = try allocator.alignedAlloc(
        u8,
        .@"16",
        @intCast(state_bytes),
    );
    errdefer allocator.free(storage);
    var unit = Self{
        .allocator = allocator,
        .storage = storage,
        .config = config,
        .format = format,
    };
    // map coded channels into the file output order
    if (
        libopus.opus_multistream_decoder_init(
            unit.state(),
            48000,
            format.channels,
            config.streams,
            config.coupled_streams,
            &config.mapping,
        ) != libopus.OPUS_OK
    ) {
        return error.InvalidData;
    }
    // 5760 samples per channel cover the maximum 120 ms packet
    const adapter = try Codec.UnitAdapter(Self)
        .init(allocator, unit, .packets, 5760, format, options);
    return adapter.interface(.opus);
}

/// decode one packet with header gain; return the frame count
/// output must hold 5760 samples per channel; the pipeline handles trimming
pub fn decodeUnit(self: *Self, bytes: []const u8, output: []f32) Error!usize {
    // libopus packet length must fit signed c integer
    const packet_length = std.math.cast(c_int, bytes.len)
        orelse return error.LimitExceeded;
    // zero disables forward error correction for this packet
    const decoded_samples = libopus.opus_multistream_decode_float(
        self.state(),
        bytes.ptr,
        packet_length,
        output.ptr,
        5760,
        0,
    );
    if (decoded_samples < 0) return error.InvalidData;
    const sample_count: usize = @intCast(decoded_samples);
    // convert fixed point db to amplitude, divide gain by 256 then db by 20
    const gain = std.math.pow(
        f32,
        10,
        @as(f32, @floatFromInt(self.config.gain_q8)) / (20 * 256),
    );
    // apply header gain only to samples produced by this packet
    for (output[0..sample_count * self.format.channels]) |*sample| {
        sample.* *= gain;
    }
    return sample_count;
}

/// reset history with the same channels and mapping
pub fn reset(self: *Self) void {
    const result = libopus.opus_multistream_decoder_init(
        self.state(),
        48000,
        self.format.channels,
        self.config.streams,
        self.config.coupled_streams,
        &self.config.mapping,
    );
    std.debug.assert(result == libopus.OPUS_OK);
}

/// free decoder state; the adapter frees its sample buffer
pub fn deinit(self: *Self) void {
    self.allocator.free(self.storage);
}

/// borrow decoder state from aligned storage
fn state(self: *Self) *libopus.OpusMSDecoder {
    return @ptrCast(self.storage.ptr);
}
