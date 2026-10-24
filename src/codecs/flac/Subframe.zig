//! Adapted from audiophile/flac, Copyright (c) 2026 Eric Freed (MIT).
//! Source: https://codeberg.org/audiophile/flac/commit/8681b5be53bc950101f4f33e291b7f1e1f117059
//! decode one channel of a flac frame

const std = @import("std");
const assert = std.debug.assert;

const BitReader = @import("bit_reader.zig");
const Frame = @import("Frame.zig");

const Subframe = @This();
const max_linear_predictor_coeffs = 32;

const Type = enum(u2) {
    /// one repeated sample, with no residual
    constant,
    /// uncompressed pcm samples
    verbatim,
    /// fixed-coefficient polynomial predictor
    fixed,
    /// custom linear predictor with up to 32 coefficients
    lpc,
};

/// warm-up samples followed by reconstructed samples in the frame buffer
buffer: []i64,
channel: u8,
subframe_type: Type,
/// predictor order: fixed 0..4, lpc 1..32, otherwise zero
order: u6,
/// trailing zero bits omitted during encoding; restored after prediction
wasted_bits: u6,

pub fn init(channel: usize) Subframe {
    assert(channel < Frame.max_channels);
    return .{
        .buffer = &.{},
        .channel = @intCast(channel),
        .subframe_type = .constant,
        .order = 0,
        .wasted_bits = 0,
    };
}

pub const ReadError = error{
    /// invalid sync or padding bits
    OutOfSync,
    InvalidSubframeType,
    /// predictor order or coefficient precision is out of range
    InvalidPredictor,
    InvalidBitDepth,
    InvalidBlockSize,
    UnknownResidualCodingMethod,
    /// lpc shift is negative or out of range
    InvalidShift,
}
    || BitReader.Error
    || BitReader.ReadUnaryError;

pub fn read(
    self: *Subframe,
    br: *BitReader,
    frame: *const Frame,
    comptime decode: bool,
) ReadError!void {
    try self.readHeader(br);
    const bps = try frame.bitsPerSample(self.channel, self.wasted_bits);

    if (frame.block_size < self.order) {
        @branchHint(.cold);
        return error.InvalidBlockSize;
    }

    assert(frame.block_size > 0);
    if (!decode) {
        switch (self.subframe_type) {
            .constant => try br.discardBits(bps),
            .verbatim => try br.discardBits(@as(usize, frame.block_size) * bps),
            .fixed, .lpc => {
                try br.discardBits(@as(usize, self.order) * bps);
                if (self.subframe_type == .lpc) {
                    const precision = try br.takeBits(u5, 4) + 1;
                    if (precision == 16) return error.InvalidPredictor;
                    const shift = try br.takeBits(u5, 5);
                    if (shift > 15) return error.InvalidShift;
                    try br.discardBits(@as(usize, self.order) * precision);
                }
                try decodeResiduals(br, null, frame.block_size, self.order);
            },
        }
        return;
    }
    const buffer = self.buffer[0..frame.block_size];
    try switch (self.subframe_type) {
        .constant => readConstant(br, buffer, bps),
        .verbatim => readVerbatim(br, buffer, bps),
        .fixed => readFixed(br, buffer, bps, self.order),
        .lpc => readLinearPrediction(br, buffer, bps, self.order),
    };
    const limit = @as(i64, 1) << @as(u6, bps - 1);
    for (buffer) |*sample| {
        if (sample.* < -limit or sample.* >= limit)
            return error.InvalidBitDepth;
        // restore before interchannel decorrelation: each encoded channel can
        // have a different wasted-bit count
        sample.* <<= self.wasted_bits;
    }
}

fn readHeader(self: *Subframe, br: *BitReader) ReadError!void {
    // zero bit padding, to prevent sync-fooling string of 1s
    if (try br.takeBits(u1, 1) != 0) {
        return error.OutOfSync;
    }

    self.order = 0;
    self.subframe_type = switch (try br.takeBits(u6, 6)) {
        0b000000 => .constant,
        0b000001 => .verbatim,
        // reserved bit ranges
        0b000010...0b000111, 0b001101...0b011111 => {
            @branchHint(.cold);
            return error.InvalidSubframeType;
        },
        0b001000...0b001100 => |n| D: {
            self.order = n & 0b111;
            if (self.order > 4) return error.InvalidPredictor; // reserved
            break :D .fixed;
        },
        0b100000...0b111111 => |n| D: {
            self.order = 1 + (n & 0b11111);
            break :D .lpc;
        },
    };

    const has_wasted_bits = try br.takeBits(u1, 1) == 1;
    self.wasted_bits = 0;
    if (has_wasted_bits) {
        const wasted = try br.takeUnary(u6, 1);
        if (wasted >= 32) return error.InvalidBitDepth;
        self.wasted_bits = wasted + 1;
    }
}

/// repeat a single encoded sample
fn readConstant(br: *BitReader, buffer: []i64, bps: u6) BitReader.Error!void {
    const sample = try br.takeSignExtended(i64, bps);
    @memset(buffer, sample);
}

fn readVerbatim(br: *BitReader, buffer: []i64, bps: u6) BitReader.Error!void {
    for (buffer) |*sample| {
        sample.* = try br.takeSignExtended(i64, bps);
    }
}

/// reconstruct samples using the fixed polynomial for order
fn readFixed(br: *BitReader, buffer: []i64, bps: u6, order: u6) !void {
    // zig fmt: off
    const fixed_coeffs = comptime [_][]const i64{
        &.{},
        &.{1},
        &.{ 2, -1 },
        &.{ 3, -3, 1 },
        &.{ 4, -6, 4, -1 },
    };
    // zig fmt: on
    assert(order < fixed_coeffs.len);

    // warm-up samples
    for (0..order) |i| {
        buffer[i] = try br.takeSignExtended(i64, bps);
    }

    try decodeResiduals(br, buffer, buffer.len, order);
    if (order > 0) {
        try restoreLinearPrediction(buffer, fixed_coeffs[order], 0);
    }
}

/// read predictor coefficients and reconstruct samples from residuals
fn readLinearPrediction(
    br: *BitReader,
    buffer: []i64,
    bps: u6,
    order: u6,
) !void {
    assert(order > 0);
    // warm-up samples
    for (0..order) |i| {
        buffer[i] = try br.takeSignExtended(i64, bps);
    }

    const precision = try br.takeBits(u5, 4) + 1;
    if (precision == 16) return error.InvalidPredictor;
    const shift: u5 = @bitCast(try br.takeBits(u5, 5));
    // the shift is actually signed two's-complement, but must be positive
    if (@as(i5, @bitCast(shift)) < 0) {
        return error.InvalidShift;
    }

    var coeff_buffer: [max_linear_predictor_coeffs]i64 = undefined;
    const coeffs = coeff_buffer[0..order];
    for (coeffs) |*coeff| {
        coeff.* = try br.takeSignExtended(i64, precision);
    }

    try decodeResiduals(br, buffer, buffer.len, order);
    try restoreLinearPrediction(buffer, coeffs, shift);
}

/// decode rice partitions after the warm-up samples; null buffer skips reconstruction
fn decodeResiduals(
    br: *BitReader,
    buffer: ?[]i64,
    sample_count: usize,
    warmup: u6,
) !void {
    const method: enum(u2) { rice1 = 0, rice2 = 1, _ } = @enumFromInt(
        try br.takeBits(u2, 2),
    );

    // determine number of bits for the param and escape value
    // base on which residual coding method is used
    const param_bits: u16, const escape_value: u5 = switch (method) {
        .rice1 => .{ 4, 15 },
        .rice2 => .{ 5, 31 },
        else => {
            @branchHint(.cold);
            return error.UnknownResidualCodingMethod;
        },
    };

    const partition_order = try br.takeBits(u4, 4);
    const num_partitions = @as(u32, 1) << partition_order;
    if (sample_count % num_partitions != 0) return error.InvalidBlockSize;
    const partition_size = sample_count / num_partitions;
    if (partition_size < warmup) return error.InvalidBlockSize;

    for (0..num_partitions) |i| {
        const start = i * partition_size + (if (i == 0) warmup else 0);
        const end = (i + 1) * partition_size;

        const param = try br.takeBits(u5, param_bits);
        if (param != escape_value) {
            for (start..end) |j| {
                const value = try decodeRice(br, param);
                if (buffer) |dest| dest[j] = value;
            }
        } else {
            const num_bits = try br.takeBits(u5, 5);
            if (buffer) |dest| {
                for (dest[start..end]) |*sample|
                    sample.* = if (num_bits == 0)
                        0
                    else
                        try br.takeSignExtended(i64, num_bits);
            } else {
                try br.discardBits((end - start) * num_bits);
            }
        }
    }
}

/// add predicted samples to residuals using previously reconstructed history
fn restoreLinearPrediction(
    buffer: []i64,
    coeffs: []const i64,
    shift: u5,
) error{ InvalidBitDepth }!void {
    const order = coeffs.len;
    if (buffer.len <= order) return;

    for (buffer[order..], order..) |*result, i| {
        var sum: i128 = 0;
        for (0..order) |j| {
            const sample = buffer[i - 1 - j];
            const coeff = coeffs[j];
            sum += @as(i128, sample) * coeff;
        }
        const value = (sum >> shift) + result.*;
        result.* = std.math.cast(i64, value)
            orelse return error.InvalidBitDepth;
    }
}

/// decode a rice integer with k remainder bits
fn decodeRice(br: *BitReader, k: u5) BitReader.ReadUnaryError!i64 {
    const quotient = try br.takeUnary(u64, 1);
    if (quotient > (@as(u32, std.math.maxInt(u32)) >> k))
        return error.UnaryOverflow;
    var value: u32 = @intCast(quotient << k);
    if (k > 0) {
        value |= try br.takeBits(u32, k);
    }

    const signed_value = @as(i64, @intCast(value >> 1));
    const sign_mask = -%@as(i64, @intCast(value & 1));
    return signed_value ^ sign_mask;
}
