//! Adapted from audiophile/flac, Copyright (c) 2026 Eric Freed (MIT).
//! Source: https://codeberg.org/audiophile/flac/commit/8681b5be53bc950101f4f33e291b7f1e1f117059
//! flac frame parsing and channel reconstruction

const std = @import("std");
const assert = std.debug.assert;
const Allocator = std.mem.Allocator;
const BitReader = @import("bit_reader.zig");
const ChannelLayout = @import("channel_layout.zig").ChannelLayout;
const StreamInfo = @import("StreamInfo.zig");
const Subframe = @import("Subframe.zig");

const Frame = @This();

pub const max_channels = 8;
const sync_code = 0b111111111111100;

pub const ReadError = error{
    InvalidSampleRate,
    InvalidChannelLayout,
    /// frame channel count differs from streaminfo
    InvalidChannelCount,
    /// encoded frame size lies outside the streaminfo bounds
    InvalidFrameSize,
    /// header crc-8 or frame crc-16 mismatch
    ChecksumMismatch,
}
    || Subframe.ReadError
    || BitReader.ReadUt8Error;

const BlockingStrategy = enum(u1) {
    /// fixed block size, number is the flac frame number
    fixed,
    /// variable block size, number is the sample number
    variable,
};

/// contiguous sample buffer, divided into one slice per channel
buffer: []i64,
/// byte length of the last scanned frame, including its checksum
encoded_bytes: usize = 0,
subframes: [max_channels]Subframe,
info: StreamInfo,
/// sample rate in hz
sample_rate: u32,
/// samples per channel
block_size: u32,
channels: ChannelLayout,
sample_bits: u6,
number: union(BlockingStrategy) {
    /// frame number
    fixed: u64,
    /// first sample number in the block
    variable: u64,
},

/// reserve space for the max decoded frame
pub fn init(allocator: Allocator, info: StreamInfo) Allocator.Error!Frame {
    assert(info.channels > 0 and info.channels <= max_channels);

    const stride = info.max_block_size;
    const num_chans = info.channels;
    const buffer_len = @as(usize, stride) * num_chans;

    var frame: Frame = .{
        .buffer = try allocator.alloc(i64, buffer_len),
        .subframes = undefined,
        .info = info,
        .block_size = 0,
        .channels = .none,
        .sample_bits = 0,
        .sample_rate = 0,
        .number = .{ .fixed = 0 },
    };

    for (frame.subframes[0..num_chans], 0..) |*subframe, channel| {
        subframe.* = .init(channel);
        subframe.buffer = frame.buffer[channel * stride..][0..stride];
    }

    if (num_chans < frame.subframes.len) {
        for (frame.subframes[num_chans..]) |*sub| sub.* = undefined;
    }

    return frame;
}

/// use the allocator supplied to init
pub fn deinit(self: *Frame, allocator: Allocator) void {
    allocator.free(self.buffer);
    self.buffer = &.{};
    for (&self.subframes, 0..) |*subframe, channel| subframe.* = .init(channel);
    self.block_size = 0;
}

/// leave self empty and safe to deinit
pub fn take(self: *Frame) Frame {
    const owned = self.*;
    self.buffer = &.{};
    for (&self.subframes, 0..) |*subframe, channel| subframe.* = .init(channel);
    self.block_size = 0;
    return owned;
}

/// scan mode shares the structural parser without allocating sample storage
pub fn scan(info: StreamInfo, bytes: []const u8) ReadError!Frame {
    var frame: Frame = .{
        .buffer = &.{},
        .subframes = undefined,
        .info = info,
        .sample_rate = 0,
        .block_size = 0,
        .channels = .none,
        .sample_bits = 0,
        .number = .{ .fixed = 0 },
    };
    for (&frame.subframes, 0..) |*sub, i| sub.* = .init(i);
    var bits: BitReader = .init(bytes);
    _ = try frame.read(&bits, false);
    // scan results retain only header data and the encoded byte count
    frame.encoded_bytes = bits.byteCount();
    return frame;
}

pub fn firstSample(self: *const Frame) u64 {
    return switch (self.number) {
        .fixed => |n| n * self.info.max_block_size,
        .variable => |n| n,
    };
}

/// read one frame; decode reconstructs samples and invalidates previous subframes
pub fn read(
    self: *Frame,
    br: *BitReader,
    comptime decode: bool,
) ReadError!usize {
    // the caller distinguishes empty input from a truncated header
    try self.readHeader(br);
    const channels = self.channels.count();
    assert(br.bits.position % 8 == 0);
    for (self.subframes[0..channels]) |*subframe|
        try subframe.read(br, self, decode);

    // padding bits must be zero, they are already included in crc-16
    const padding: u4 = @intCast((8 - br.bits.position % 8) % 8);
    if (try br.takeBits(u8, padding) != 0) return error.OutOfSync;
    const actual = br.frameChecksum();
    const expected = try br.takeInt(u16);
    if (actual != expected) return error.ChecksumMismatch;
    if (
        br.byteCount() < self.info.min_frame_size
            or (self.info.max_frame_size != 0
                and br.byteCount() > self.info.max_frame_size)
    )
        return error.InvalidFrameSize;

    if (decode) {
        if (self.channels.isSideLayout()) self.decorrelate();
        // reconstructed channels must fit the declared source bit depth
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

/// account for side-channel precision and wasted bits
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
    // side channels use an extra bit
    if (extra) bps += 1;
    if (wasted >= bps) return error.InvalidBitDepth;
    bps -= wasted;
    return bps;
}

/// reconstruct left and right channels from stereo decorrelation
fn decorrelate(self: *Frame) void {
    const chan0 = self.subframes[0].buffer[0..self.block_size];
    const chan1 = self.subframes[1].buffer[0..self.block_size];

    switch (self.channels) {
        .left_side => {
            // right = left - side
            for (chan0, chan1) |left, *side| {
                side.* = left - side.*;
            }
        },
        .right_side => {
            // left = side + right
            for (chan0, chan1) |*side, right| {
                side.* = side.* + right;
            }
        },
        .mid_side => {
            for (chan0, chan1) |*mid, *side| {
                const m = mid.*;
                const s = side.*;

                const left = m + (s >> 1) + (s & 1);
                const right = left - s;

                mid.* = left;
                side.* = right;
            }
        },
        else => unreachable,
    }
}

fn readHeader(self: *Frame, br: *BitReader) !void {
    if (sync_code != try br.takeBits(u16, 15)) {
        return error.OutOfSync;
    }

    // read blocking strategy, which determines if the number represents a sample or a frame
    const blocking_stategy: BlockingStrategy = @enumFromInt(
        try br.takeBits(u1, 1),
    );

    // just read raw for now, parse values at end of header
    // where more may need read depending on value
    const block_size: u4 = try br.takeBits(u4, 4);
    const sample_rate: u4 = try br.takeBits(u4, 4);

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
    if (self.channels.count() != self.info.channels) {
        return error.InvalidChannelCount;
    }

    self.sample_bits = switch (try br.takeBits(u3, 3)) {
        // get from streaminfo metadata block
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
    if (self.sample_bits != self.info.bits_per_sample) {
        return error.InvalidBitDepth;
    }

    assert(br.bits.position % 8 == 7);
    if (try br.takeBits(u1, 1) != 0) return error.OutOfSync;
    assert(br.bits.position % 8 == 0);

    self.number = switch (blocking_stategy) {
        .fixed => .{ .fixed = try br.takeUtf8Int(31) },
        .variable => .{ .variable = try br.takeUtf8Int(36) },
    };

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

    if (
        self.info.max_block_size > 0
            and self.block_size > self.info.max_block_size
    ) {
        return error.InvalidBlockSize;
    }

    self.sample_rate = switch (sample_rate) {
        // zig fmt: off
        // get from streaminfo metadata block
        0b0000 => self.info.sample_rate,
        0b0001 => 88200,
        0b0010 => 176400,
        0b0011 => 192000,
        0b0100 => 8000,
        0b0101 => 16000,
        0b0110 => 22050,
        0b0111 => 24000,
        0b1000 => 32000,
        0b1001 => 44100,
        0b1010 => 48000,
        0b1011 => 96000,
        // get 8 bit sample rate (in kHz) from end of header
        0b1100 => @as(u32, try br.takeByte()) * 1000,
        // get 16 bit sample rate (in hz) from end of header
        0b1101 => @as(u32, try br.takeInt(u16)),
        // get 16 bit sample rate (in tens of hz) from end of header
        0b1110 => @as(u32, try br.takeInt(u16)) * 10,
        // invalid, to prevent sync-fooling string of 1s
        0b1111 => return error.InvalidSampleRate,
        // zig fmt: on
    };

    if (self.sample_rate == 0 or self.sample_rate != self.info.sample_rate)
        return error.InvalidSampleRate;

    const actual = br.headerChecksum();
    // the header crc byte participates in crc-16, but not in its own crc-8
    const expected = try br.takeByte();
    if (actual != expected) return error.ChecksumMismatch;
}
