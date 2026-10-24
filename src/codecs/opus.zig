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
// min recommended by rfc 7845
pub const preroll_frames: u64 = 24000; // 500 ms at 48 khz

allocator: std.mem.Allocator,
/// keep the decoder address stable and aligned to 16 bytes
storage: []align(16) u8,
config: Config,
format: types.AudioFormat,

/// allocate a 48 khz decoder and output buffer in source channel order
pub fn init(
    allocator: std.mem.Allocator,
    config: Config,
    format: types.AudioFormat,
    options: types.Options,
) Error!Codec {
    try config.validate(format);
    // query libopus state size for coded stream topology
    const state_bytes = libopus.opus_multistream_decoder_get_size(
        config.streams,
        config.coupled_streams,
    );
    if (state_bytes <= 0) return error.InvalidData;
    const storage = try allocator.alignedAlloc(
        u8,
        .@"16",
        @intCast(state_bytes),
    );
    var unit = Self{
        .allocator = allocator,
        .storage = storage,
        .config = config,
        .format = format,
    };
    defer unit.deinit();
    // map coded channels into the file output order
    try checkResult(
        libopus.opus_multistream_decoder_init(
            unit.state(),
            48000,
            format.channels,
            config.streams,
            config.coupled_streams,
            &config.mapping,
        ),
    );
    // 5760 samples per channel cover the max 120 ms packet
    return Codec.UnitAdapter(Self)
        .init(allocator, unit.take(), .opus, .packets, 5760, format, options);
}

/// leave self empty and safe to deinit
pub fn take(self: *Self) Self {
    const owned = self.*;
    self.storage = &.{};
    return owned;
}

/// decode one packet with header gain; return the frame count
/// output must hold 5760 samples per channel; the pipeline handles trimming
/// input failures require reset before reuse
pub fn decodeUnit(self: *Self, bytes: []const u8, output: []f32) Error!usize {
    if (bytes.len == 0) return error.InvalidData;
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
    try checkResult(decoded_samples);
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
pub fn reset(self: *Self) Error!void {
    const result = libopus.opus_multistream_decoder_init(
        self.state(),
        48000,
        self.format.channels,
        self.config.streams,
        self.config.coupled_streams,
        &self.config.mapping,
    );
    try checkResult(result);
}

pub fn deinit(self: *Self) void {
    self.allocator.free(self.storage);
    self.storage = &.{};
}

fn state(self: *Self) *libopus.OpusMSDecoder {
    return @ptrCast(self.storage.ptr);
}

/// translate backend failures without hiding allocation or capacity errors
fn checkResult(result: c_int) Error!void {
    if (result >= 0) return;
    return switch (result) {
        libopus.OPUS_ALLOC_FAIL => error.OutOfMemory,
        libopus.OPUS_BUFFER_TOO_SMALL => error.InvalidBufferSize,
        libopus.OPUS_INVALID_PACKET => error.InvalidData,
        libopus.OPUS_BAD_ARG,
        libopus.OPUS_INTERNAL_ERROR,
        libopus.OPUS_INVALID_STATE,
        libopus.OPUS_UNIMPLEMENTED => error.InvalidState,
        else => error.InvalidState,
    };
}
