//! Adapted from audiophile/flac, Copyright (c) 2026 Eric Freed (MIT).
//! Source: https://codeberg.org/audiophile/flac/commit/8681b5be53bc950101f4f33e291b7f1e1f117059
//! Architectural representation of a decoded FLAC audio frame.
//!
//! A FLAC bitstream is divided into independent, sequentially decodable blocks
//! of audio data called frames. Each frame consists of a strict header (containing
//! sample rate, block size, and channel assignment), followed by a series of
//! independent subframes (one per encoded channel), and terminates with a 16-bit CRC.

const std = @import("std");
const assert = std.debug.assert;
const Allocator = std.mem.Allocator;
const Reader = std.Io.Reader;

const BitReader = @import("bit_reader.zig").BitReader(.big, true);
const ChannelLayout = @import("channel_layout.zig").ChannelLayout;
const StreamInfo = @import("StreamInfo.zig");
const Subframe = @import("Subframe.zig");

const Frame = @This();

/// The maximum number of distinct audio channels supported by FLAC.
pub const max_channels = 8;
/// The maximum possible block size.
pub const max_block_size = 65535;
/// Sync code for frame headers.
const sync_code = 0b111111111111100;

/// Error set used when reading/decoding a complete FLAC frame header and its payload.
pub const ReadError = error{
    /// The sample rate specified in the frame header is invalid, uses a reserved
    /// bit pattern, or cannot be resolved by the decoder.
    InvalidSampleRate,
    /// The channel assignment or channel layout bits represent an invalid,
    /// reserved, or unsupported configuration.
    InvalidChannelLayout,
    /// The number of channels in the frame differs from what is defined in
    /// the STREAMINFO metadata block.
    InvalidChannelCount,
    /// The actual number of bytes in a frame or less/more than the min/max
    /// specified in the STREAMINFO metadata block;
    InvalidFrameSize,
    /// The calculated 8-bit CRC of the frame header or the 16-bit CRC of the
    /// entire frame footer does not match the checksum stored within the stream,
    /// indicating data corruption.
    ///
    /// This error is impossible when checksums are disabled.
    ChecksumMismatch,
}
    || Subframe.ReadError
    || BitReader.ReadUt8Error;

/// Describes how to interpret the block size.
const BlockingStrategy = enum(u1) {
    /// Fixed block size, number is the FLAC frame number.
    fixed,
    /// Variable block size, number is the sample number.
    variable,
};

/// A heap allocated buffer for decoding and mixing.
///
/// This buffer is subdivided and used for each subframe, but is owned
/// by the parent frame to remain contiguous and reduce memory fragmentation.
buffer: []i64,
/// Byte length of the last scanned frame, including its checksum.
encoded_bytes: usize = 0,
/// An array of subframe slots, one used for each channel in the stream.
subframes: [max_channels]Subframe,
/// The metadata block, required by all FLAC files.
/// May not be present for FLAC used by other container formats.
info: StreamInfo,
/// The sampling rate, in Hertz (Hz).
sample_rate: u32,
/// The current block size of the frame.
/// This is the number of samples in each subframe.
block_size: u32,
/// Describes the channel layout of the frame.
channels: ChannelLayout,
/// The base number of bits used per sample.
sample_bits: u6,
/// Enforce strict compliance with the FLAC Stream Subset profile.
strict: bool,
/// Current frame or sample number encoded into the frame.
number: union(BlockingStrategy) {
    /// The FLAC frame number.
    fixed: u64,
    /// The first sample number in the block.
    variable: u64,
},

/// Initializes a Frame and allocates enough storage to contain the
/// maximum fully decoded frame in the stream.
///
/// * `allocator` - The memory allocator used for heap allocations.
/// * `info` - The STREAMINFO metadata block read from the stream.
/// * `strict` - Enforce strict compliance with the FLAC Stream Subset profile.
pub fn init(
    allocator: Allocator,
    info: StreamInfo,
    strict: bool,
) Allocator.Error!Frame {
    assert(info.channels > 0 and info.channels <= max_channels);

    // When using traditional subset, the maximum memory is known up-front.
    // Otherwise, the maximums need to be assumed, as it can change between frames.
    const buffer_len: usize, const stride: usize, const num_chans = if (
        strict
    ) .{
        @as(usize, info.max_block_size) * info.channels,
        info.max_block_size,
        info.channels,
    } else .{ max_channels * max_block_size, max_block_size, max_channels };

    var frame: Frame = .{
        .buffer = try allocator.alloc(i64, buffer_len),
        .subframes = undefined,
        .info = info,
        .block_size = 0,
        .channels = .none,
        .sample_bits = 0,
        .sample_rate = 0,
        .strict = strict,
        .number = .{ .fixed = 0 },
    };

    // Portion out sub-slices of the buffer memory to subframes.
    for (frame.subframes[0..num_chans], 0..) |*subframe, channel| {
        subframe.* = .init(channel);
        subframe.buffer = frame.buffer[channel * stride..][0..stride];
    }

    // Explicitly set unused channels to undefined.
    if (num_chans < frame.subframes.len) {
        for (frame.subframes[num_chans..]) |*sub| sub.* = undefined;
    }

    return frame;
}

/// Release all resources used by the FLAC frame.
///
/// * `allocator` - The original allocator used during initialization.
pub fn deinit(self: *Frame, allocator: Allocator) void {
    allocator.free(self.buffer);
    self.buffer = &.{};
    // Subframes borrow this allocation; remove their aliases as well.
    for (&self.subframes, 0..) |*subframe, channel| subframe.* = .init(channel);
    self.block_size = 0;
}

/// Transfer sample storage and its subframe views; leave self safe to deinit.
pub fn take(self: *Frame) Frame {
    const owned = self.*;
    self.buffer = &.{};
    for (&self.subframes, 0..) |*subframe, channel| subframe.* = .init(channel);
    self.block_size = 0;
    return owned;
}

/// Scan mode shares the structural parser without allocating sample storage.
pub fn scan(info: StreamInfo, bytes: []const u8) ReadError!Frame {
    var frame: Frame = .{
        .buffer = &.{},
        .subframes = undefined,
        .info = info,
        .sample_rate = 0,
        .block_size = 0,
        .channels = .none,
        .sample_bits = 0,
        .strict = true,
        .number = .{ .fixed = 0 },
    };
    for (&frame.subframes, 0..) |*sub, i| sub.* = .init(i);
    var reader: Reader = .fixed(bytes);
    var bits: BitReader = .{ .child = &reader };
    _ = try frame.read(&bits, false);
    // Scan results retain only header data and the encoded byte count.
    frame.encoded_bytes = bits.byte_count;
    return frame;
}

pub fn firstSample(self: *const Frame) u64 {
    return switch (self.number) {
        .fixed => |n| n * self.info.max_block_size,
        .variable => |n| n,
    };
}

/// Read one complete frame, optionally reconstructing samples into owned scratch.
/// All previous decoded subframes are invalidated when decode is true.
pub fn read(
    self: *Frame,
    br: *BitReader,
    comptime decode: bool,
) ReadError!usize {
    br.byte_count = 0;
    // The caller distinguishes empty input from a truncated header.
    try self.readHeader(br);
    const channels = self.channels.count();
    assert(br.count == 0);
    for (self.subframes[0..channels]) |*subframe|
        try subframe.read(br, self, decode);

    // Padding bits must be zero. They are already included in CRC-16.
    if (br.buffer != 0) return error.OutOfSync;
    br.alignToByte();
    const actual = br.crc16.final();
    const expected = try br.child.takeInt(u16, .big);
    br.byte_count += 2;
    if (actual != expected) return error.ChecksumMismatch;
    if (
        br.byte_count < self.info.min_frame_size
            or (self.info.max_frame_size != 0
                and br.byte_count > self.info.max_frame_size)
    )
        return error.InvalidFrameSize;

    if (decode) {
        if (self.channels.isSideLayout()) self.decorrelate();
        // Reconstructed channels must fit the declared source bit depth.
        const limit = @as(i64, 1) << @as(u6, self.sample_bits - 1);
        for (self.subframes[0..channels]) |sub| {
            for (sub.buffer[0..self.block_size]) |sample| {
                if (sample < -limit or sample >= limit)
                    return error.InvalidBitDepth;
            }
        }
    }
    return self.block_size;
}

/// Computes the required bits-per-sample for decoding the given channel.
///
/// * `channel` - The index of the channel (subframe).
/// * `wasted` - The number of wasted bits per sample, from the subframe header.
///
/// Returns the number of bits per sample.
pub fn bitsPerSample(
    self: *const Frame,
    channel: u8,
    wasted: u6,
) error{ InvalidBitDepth }!u6 {
    var bps = self.sample_bits;
    const extra = switch (self.channels) {
        .left_side, .mid_side => channel == 1,
        .right_side => channel == 0,
        else => false,
    };
    // Side channels use an extra bit
    if (extra) bps += 1;
    if (wasted >= bps) return error.InvalidBitDepth;
    bps -= wasted;
    return bps;
}

/// Decorrelates interchannel stereo streams to their final values.
///
/// The encoder will create mid and side signals based on the average and difference
/// (respectively) of the left and right channels. The encoder will then pass the best
/// form of the signal to the next stage.
fn decorrelate(self: *Frame) void {
    const chan0 = self.subframes[0].buffer[0..self.block_size];
    const chan1 = self.subframes[1].buffer[0..self.block_size];

    switch (self.channels) {
        .left_side => {
            // Right = Left - Side
            for (chan0, chan1) |left, *side| {
                side.* = left - side.*;
            }
        },
        .right_side => {
            // Left = Side + Right
            for (chan0, chan1) |*side, right| {
                side.* = side.* + right;
            }
        },
        .mid_side => {
            for (chan0, chan1) |*mid, *side| {
                const m = mid.*;
                const s = side.*;

                // Shift arithmetic down cleanly first
                const left = m + (s >> 1) + (s & 1);
                const right = left - s;

                // Write them out safely without stepping on your variables
                mid.* = left;
                side.* = right;
            }
        },
        else => unreachable,
    }
}

/// Reads and decodes a FLAC frame header from the stream, updating the values of this frame.
///
/// * `br` - A bit reader for the FLAC stream.
fn readHeader(self: *Frame, br: *BitReader) !void {
    // Initialize empty header and reset bit reader
    br.resetChecksum();
    br.crc8_enabled = true;
    br.alignToByte();

    // Ensure reader is still in sync.
    if (sync_code != try br.takeBits(u16, 15)) {
        return error.OutOfSync;
    }

    // Read blocking strategy, which determines if the number represents a sample or a frame.
    const blocking_stategy: BlockingStrategy = @enumFromInt(
        try br.takeBits(u1, 1),
    );

    // Just read raw for now, parse values at end of header
    // where more may need read depending on value.
    const block_size: u4 = try br.takeBits(u4, 4);
    const sample_rate: u4 = try br.takeBits(u4, 4);

    // Parse and validate channel assignments.
    switch (try br.takeBits(u4, 4)) {
        0b1011...0b1111 => {
            @branchHint(.cold);
            return error.InvalidChannelLayout;
        },
        else => |chan| {
            @branchHint(.likely);
            self.channels = @enumFromInt(1 + chan);
        },
    }
    if (self.strict and self.channels.count() != self.info.channels) {
        return error.InvalidChannelCount;
    }

    // Read and parse the sample bit depth.
    self.sample_bits = switch (try br.takeBits(u3, 3)) {
        // get from STREAMINFO metadata block
        0b000 => self.info.bits_per_sample,
        0b001 => 8,
        0b010 => 12,
        0b011 => {
            @branchHint(.cold);
            return error.InvalidBitDepth;
        },
        0b100 => 16,
        0b101 => 20,
        0b110 => 24,
        0b111 => 32,
    };
    if (self.strict and self.sample_bits != self.info.bits_per_sample) {
        return error.InvalidBitDepth;
    }

    // Should be byte-aligned after reserve bit if prior logic is correct
    assert(br.count == 1);
    if (try br.takeBits(u1, 1) != 0) return error.OutOfSync;
    assert(br.count == 0);

    // Read UTF-8 coded sample/frame number depending on block strategy
    self.number = switch (blocking_stategy) {
        .fixed => .{ .fixed = try br.takeUtf8Int(31) },
        .variable => .{ .variable = try br.takeUtf8Int(36) },
    };

    // Calculate the block size depending on previously read value.
    self.block_size = switch (block_size) {
        0b0000 => return error.InvalidBlockSize,
        0b0001 => 192,
        // 0b0010...0b0101 => |n| 576 * std.math.pow(u16, 2, n - 2),
        0b0010...0b0101 => |n| @as(u32, 576) << (n - 2),
        0b0110 => @as(u32, try br.takeByte()) + 1,
        0b0111 => @as(u32, try br.takeInt(u16)) + 1,
        // 0b1000...0b1111 => |n| 256 * std.math.pow(u16, 2, n - 8),
        0b1000...0b1111 => |n| @as(u32, 256) << (n - 8),
    };

    // Ensure block size is within bounds
    if (
        self.info.max_block_size > 0
            and self.block_size > self.info.max_block_size
    ) {
        return error.InvalidBlockSize;
    }

    // Calculate the sample rate depending on previously read value.
    self.sample_rate = switch (sample_rate) {
        // zig fmt: off
        // get from STREAMINFO metadata block
        0b0000 => self.info.sample_rate,
        0b0001 => 88200, // 88.2kHz
        0b0010 => 176400, // 176.4kHz
        0b0011 => 192000, // 192kHz
        0b0100 => 8000, // 8kHz
        0b0101 => 16000, // 16kHz
        0b0110 => 22050, // 22.05kHz
        0b0111 => 24000, // 24kHz
        0b1000 => 32000, // 32kHz
        0b1001 => 44100, // 44.1kHz
        0b1010 => 48000, // 48kHz
        0b1011 => 96000, // 96kHz
        // get 8 bit sample rate (in kHz) from end of header
        0b1100 => @as(u32, try br.takeByte()) * 1000,
        // get 16 bit sample rate (in Hz) from end of header
        0b1101 => @as(u32, try br.takeInt(u16)),
        // get 16 bit sample rate (in tens of Hz) from end of header
        0b1110 => @as(u32, try br.takeInt(u16)) * 10,
        // invalid, to prevent sync-fooling string of 1s
        0b1111 => return error.InvalidSampleRate,
        // zig fmt: on
    };

    if (self.sample_rate == 0 or self.sample_rate != self.info.sample_rate)
        return error.InvalidSampleRate;

    const actual = br.crc8.final();
    // The header CRC byte participates in CRC-16, but not in its own CRC-8.
    br.crc8_enabled = false;
    const expected = try br.takeByte();
    if (actual != expected) return error.ChecksumMismatch;
}
