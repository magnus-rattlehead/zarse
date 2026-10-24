//! Adapted from audiophile/flac, Copyright (c) 2026 Eric Freed (MIT).
//! Source: https://codeberg.org/audiophile/flac/commit/8681b5be53bc950101f4f33e291b7f1e1f117059
//! streaminfo must occur once, as the first native flac metadata block

const std = @import("std");
const assert = std.debug.assert;

const BitReader = @import("bit_reader.zig");

const StreamInfo = @This();

pub const max_sample_rate = 655350;

pub const Error = error{
    /// the sample rate is 0 or above `max_sample_rate`
    InvalidSampleRate,
    /// the block size is less than 16 or min value is greater than the max
    InvalidBlockSize,
    /// the frame size min value is greater than the max
    InvalidFrameSize,
    /// the bits per sample is not between 4 and 32
    InvalidBitDepth,
    InvalidChannelCount,
}
    || BitReader.Error;

/// samples per channel, excluding the last block
min_block_size: u16,
/// samples per channel
max_block_size: u16,
/// encoded bytes; zero means unknown
min_frame_size: u24,
/// encoded bytes; zero means unknown
max_frame_size: u24,
/// sample rate in hz
sample_rate: u32,
/// 1..8 channels
channels: u8,
/// 4..32 bits per sample
bits_per_sample: u6,
/// total pcm frames; zero means unknown
sample_count: u36,

pub fn parse(br: *BitReader) Error!StreamInfo {
    assert(br.bits.position % 8 == 0);
    const info: StreamInfo = .{
        .min_block_size = try br.takeInt(u16),
        .max_block_size = try br.takeInt(u16),
        .min_frame_size = try br.takeInt(u24),
        .max_frame_size = try br.takeInt(u24),
        .sample_rate = try br.takeBits(u32, 20),
        .channels = 1 + try br.takeBits(u8, 3),
        .bits_per_sample = 1 + try br.takeBits(u6, 5),
        .sample_count = try br.takeBits(u36, 36),
    };
    assert(br.bits.position % 8 == 0);

    try br.discardBits(128);

    try info.validate();
    return info;
}

pub fn validate(info: StreamInfo) Error!void {
    if (info.min_block_size < 16 or info.max_block_size < 16) {
        return error.InvalidBlockSize;
    }
    if (info.min_block_size > info.max_block_size) {
        return error.InvalidBlockSize;
    }

    if (info.min_frame_size > info.max_frame_size) {
        return error.InvalidFrameSize;
    }

    if (info.sample_rate == 0 or info.sample_rate > max_sample_rate) {
        return error.InvalidSampleRate;
    }

    if (info.bits_per_sample < 4 or info.bits_per_sample > 32) {
        return error.InvalidBitDepth;
    }
    if (info.channels == 0 or info.channels > 8) {
        return error.InvalidChannelCount;
    }
}
