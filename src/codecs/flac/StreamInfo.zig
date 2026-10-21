//! Adapted from audiophile/flac, Copyright (c) 2026 Eric Freed (MIT).
//! Source: https://codeberg.org/audiophile/flac/commit/8681b5be53bc950101f4f33e291b7f1e1f117059
//! The streaminfo metadata block has information about the whole stream,
//! such as sample rate, number of channels, total number of samples, etc.
//!
//! It MUST be present as the first metadata block in the stream.
//! Other metadata blocks **MAY** follow.
//! There **MUST** be no more than one streaminfo metadata block per FLAC stream.

const std = @import("std");
const Writer = std.Io.Writer;
const assert = std.debug.assert;

const BitReader = @import("bit_reader.zig").BitReader(.big, true);

const StreamInfo = @This();

/// Maximum sample rate in (Hz).
pub const max_sample_rate = 655350;

/// Error set used when parsing stream info metadata blocks.
pub const Error = error{
    /// The sample rate is 0 or above `max_sample_rate`.
    InvalidSampleRate,
    /// The block size is less than 16 or min value is greater than the max.
    InvalidBlockSize,
    /// The frame size min value is greater than the max.
    InvalidFrameSize,
    /// The bits per sample is not between 4 and 32.
    InvalidBitDepth,
    InvalidChannelCount,
}
    || BitReader.Error;

/// An empty placeholder instance initialized to zero.
pub const empty: StreamInfo = .{
    .min_block_size = 0,
    .max_block_size = 0,
    .min_frame_size = 0,
    .max_frame_size = 0,
    .sample_rate = 0,
    .channels = 0,
    .bits_per_sample = 0,
    .sample_count = 0,
    .checksum = @splat(0),
};

/// A default placeholder to use when the stream is missing
/// the STREAMINFO metadata block.
///
/// Fields are set to the maximum limits to ensure that initial
/// buffers are setup to the accommodate worst-case situation.
pub const streamless: StreamInfo = .{
    // The minimum block size allowed by the spec is 16 samples.
    .min_block_size = 16,
    // In streamless mode, allocate or assume the absolute maximum
    // block size allowed by the specification (65,535 samples).
    .max_block_size = 65535,
    // Frame sizes are unknown until parsed.
    .min_frame_size = 0,
    .max_frame_size = 0,
    // Fall back to the maximum sample rate supported by FLAC (655,350 Hz).
    .sample_rate = 655350,
    // Assume the absolute maximum channel count (8 channels).
    .channels = 8,
    // Set to 32 bits per sample, the maximum supported BPS.
    .bits_per_sample = 32,
    // A value of 0 here is explicitly defined by the FLAC specification
    // to mean that the total sample count is completely unknown.
    .sample_count = 0,
    // Zero out the MD5 checksum array; it cannot be validated for live streams.
    .checksum = @splat(0),
};

/// The minimum block size (in samples) used in the
/// stream, excluding the last block.
min_block_size: u16,
/// The maximum block size (in samples) used in the stream.
max_block_size: u16,
/// The minimum frame size (in bytes) used in the stream.
/// A value of 0 signifies that the valuecis not known.
min_frame_size: u24,
/// The maximum frame size (in bytes) used in thecstream.
/// A value of 0 signifies that the value is not known.
max_frame_size: u24,
/// Sample rate in Hertz (Hz).
sample_rate: u32,
/// The number of multiplexed channels in the stream.
/// FLAC supports from 1 to 8 channels.
channels: u8,
/// The bit depth of the sample data.
/// FLAC supports from 4 to 32 bits.
bits_per_sample: u6,
/// Total number of interchannel samples in the stream.
///
/// A value of 0 here means the number of total samples is unknown.
sample_count: u36,
/// MD5 checksum of the unencoded audio data.
/// This allows the decoder to determine if an error exists in
/// the audio data even when, despite the error, the bitstream
/// itself is valid.
///
/// A value of 0 signifies that the value is not known.
checksum: [16]u8,

/// Parses the StreamInfo block from the stream.
///
/// * `br` - A bit reader for the FLAC stream.
///
/// Returns the parsed metadata block.
pub fn parse(br: *BitReader, strict: bool) Error!StreamInfo {
    // Ensure byte-alignment
    assert(br.count == 0);
    const info: StreamInfo = .{
        .min_block_size = try br.child.takeInt(u16, .big),
        .max_block_size = try br.child.takeInt(u16, .big),
        .min_frame_size = try br.child.takeInt(u24, .big),
        .max_frame_size = try br.child.takeInt(u24, .big),
        .sample_rate = try br.takeBits(u32, 20),
        .channels = 1 + try br.takeBits(u8, 3),
        .bits_per_sample = 1 + try br.takeBits(u6, 5),
        .sample_count = try br.takeBits(u36, 36),
        .checksum = (try br.child.takeArray(16)).*,
    };
    assert(br.count == 0);

    try info.validate(strict);
    return info;
}

pub fn validate(info: StreamInfo, strict: bool) Error!void {
    // Enforce the RFC 9639 spec
    if (info.min_block_size < 16 or info.max_block_size < 16) {
        return error.InvalidBlockSize;
    }
    // Ensure min does not exceed max
    if (info.min_block_size > info.max_block_size) {
        return error.InvalidBlockSize;
    }

    // Ensure min does not exceed max
    if (info.min_frame_size > info.max_frame_size) {
        return error.InvalidFrameSize;
    }

    // A value of 0 is explicitly invalid according to spec.
    if (
        info.sample_rate == 0
            or (strict and info.sample_rate > max_sample_rate)
    ) {
        return error.InvalidSampleRate;
    }

    // Bit depth must be between 4 and 32.
    // Value is read as u5 (max 31) and then 1 added, so cannot exceed 32.
    if (info.bits_per_sample < 4 or info.bits_per_sample > 32) {
        return error.InvalidBitDepth;
    }
    if (info.channels == 0 or info.channels > 8) {
        return error.InvalidChannelCount;
    }
}

/// Provides custom format implementation which can be used by `std.fmt`.
///
/// * `writer` - The writer instance to use for output.
pub fn format(self: StreamInfo, writer: *Writer) Writer.Error!void {
    try writer.writeAll("STREAMINFO");
    try writer.print("\n  -> channels: {d}", .{self.channels});
    try writer.print("\n  -> sample rate: {d} Hz", .{self.sample_rate});
    try writer.print("\n  -> bits per sample: {d}", .{self.bits_per_sample});
    if (self.min_block_size == self.max_block_size) {
        try writer.print(
            "\n  -> block size: {d} (fixed)",
            .{self.max_block_size},
        );
    } else {
        try writer.print(
            "\n  -> block size: {d}..{d}",
            .{ self.min_block_size, self.max_block_size },
        );
    }
    if (self.min_frame_size > 0 and self.max_block_size > 0) {
        try writer.print(
            "\n  -> frame size: {d}..{d}",
            .{ self.min_frame_size, self.max_frame_size },
        );
    }
    const md5 = std.fmt.bytesToHex(self.checksum, .lower);
    try writer.print("\n  -> md5: {s}", .{&md5});
}
