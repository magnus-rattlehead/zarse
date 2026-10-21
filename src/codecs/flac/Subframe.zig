//! Adapted from audiophile/flac, Copyright (c) 2026 Eric Freed (MIT).
//! Source: https://codeberg.org/audiophile/flac/commit/8681b5be53bc950101f4f33e291b7f1e1f117059
//! Architectural representation of an individual FLAC subframe.
//!
//! Within a FLAC frame, each audio channel is compressed independently and stored
//! as a subframe. A subframe contains the audio data for exactly one channel over
//! the duration of the parent frame's `block_size`.

const std = @import("std");
const assert = std.debug.assert;

const BitReader = @import("bit_reader.zig").BitReader(.big, true);
const Frame = @import("Frame.zig");

const Subframe = @This();

/// The maximum number of bits per sample.
pub const max_sample_bits = 32;
/// The maximum number of coefficients used for LPC decoding.
const max_linear_predictor_coeffs = 32;

/// Describes the subframe type.
///
/// FLAC utilizes four distinct subframe categories, optimizing for space based on
/// the mathematical complexity of the underlying audio signal.
const Type = enum(u2) {
    /// The entire channel contains a single, unvarying sample value (e.g., complete
    /// silence or a digital constant DC offset). No residual signal is present.
    constant,
    /// The audio signal is uncompressed. Samples are stored raw as unaligned linear
    /// PCM values. This fallback is typically triggered when high-entropy noise
    /// makes predictive compression inefficient.
    verbatim,
    /// Samples are approximated using fixed-coefficient linear predictors.
    /// The polynomial formulas are hardcoded into the FLAC decoder specification.
    fixed,
    /// The encoder determines custom FIR filter coefficients (up to order 32) suited
    /// specifically to the audio block. The decoder reconstructs the signal by
    /// applying these coefficients to the historical warm-up samples, shifting the
    /// result by a specified quantization level.
    lpc,
};

/// Memory slice holding the decoded linear PCM samples for this subframe.
///
/// This buffer contains both the initial uncompressed 'warm-up' samples
/// (the count of which matches the `order` field) followed by the fully
/// reconstructed samples calculated from the decoded Rice residuals.
///
/// Note that this memory is owned and managed by the parent frame.
buffer: []i64,
/// The zero-indexed channel number associated with this subframe.
channel: u8,
/// Specifies the encoding strategy used to compress this subframe's block.
subframe_type: Type,
/// The predictor order used for signal reconstruction.
///
/// * For `.fixed`: Valid orders range from 0 to 4, mapping to the standard
///   hardcoded polynomial prediction curves.
/// * For `.lpc`: Specifies the number of custom FIR filter coefficients
///   present in the bitstream (ranging from 1 to 32).
/// * For `.constant` and `.verbatim`: This field is strictly 0.
order: u6,
/// The number of least-significant bits (LSBs) dropped per sample before encoding.
///
/// If the source audio signal exhibits a uniform sequence of lower-bit zeros
/// (e.g., a 24-bit audio file containing data that only occupies 16 bits of
/// dynamic range), the encoder shifts the entire signal right to discard the
/// dead space.
wasted_bits: u6,

/// Initializes a subframe with the given channel index.
///
/// * `channel` - A zero-based channel index this subframe represents.
///
/// Asserts the `channel < Frame.max_channels`.
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

/// Error set used when reading/decoding a subframe.
pub const ReadError = error{
    /// The bitstream read pointer has drifted out of alignment with the expected
    /// frame boundaries, or a frame/subframe sync code could not be validated.
    OutOfSync,
    /// The subframe type header contains a reserved or invalid bit pattern.
    InvalidSubframeType,
    /// The subframe uses a predictive encoding scheme, but the specified
    /// fixed polynomial order or custom LPC coefficient count is outside
    /// the valid operational limits (such as a fixed order > 4 or LPC order > 32).
    InvalidPredictor,
    /// The sample depth calculated for this specific subframe is invalid, or
    /// exceeds the maximum bit depth supported by the decoder implementation.
    InvalidBitDepth,
    /// The block size specified in the frame header is zero, greater than
    /// the maximum permitted 65535 samples.
    InvalidBlockSize,
    /// The residual coding method flag indicates an unsupported or
    /// unknown entropy coding layout.
    UnknownResidualCodingMethod,
    /// The LPC shift factor is mathematically invalid or negative.
    InvalidShift,
}
    || BitReader.Error
    || BitReader.ReadUnaryError;

/// Reads the underlying stream and decodes the results into `self`.
///
/// * `br` - A bit reader for the FLAC stream.
/// * `frame` - The parent frame this subframe is defined within.
pub fn read(
    self: *Subframe,
    br: *BitReader,
    frame: *const Frame,
    comptime decode: bool,
) ReadError!void {
    try self.readHeader(br);
    // Compute the bits per sample for this subframe, which depends
    // on the frame BPS, channels layout, and number of bits that it wastes.
    const bps = try frame.bitsPerSample(self.channel, self.wasted_bits);

    // Ensure that block size is valid
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
        // Restore before interchannel decorrelation: each encoded channel can
        // have a different wasted-bit count.
        sample.* <<= self.wasted_bits;
    }
}

/// Reads and decodes a FLAC subframe header from the stream, updating the values of this subframe.
///
/// * `br` - A bit reader for the FLAC stream.
fn readHeader(self: *Subframe, br: *BitReader) ReadError!void {
    // Zero bit padding, to prevent sync-fooling string of 1s
    if (try br.takeBits(u1, 1) != 0) {
        return error.OutOfSync;
    }

    // Determine subframe type
    self.order = 0;
    self.subframe_type = switch (try br.takeBits(u6, 6)) {
        0b000000 => .constant,
        0b000001 => .verbatim,
        // reserved bit ranges
        0b000010...0b000111, 0b001101...0b011111 => {
            @branchHint(.cold);
            return error.InvalidSubframeType;
        },
        // fixed
        0b001000...0b001100 => |n| D: {
            self.order = n & 0b111;
            if (self.order > 4) return error.InvalidPredictor; // reserved
            break :D .fixed;
        },
        // linear predictor
        0b100000...0b111111 => |n| D: {
            self.order = 1 + (n & 0b11111);
            break :D .lpc;
        },
    };

    // Wasted bits-per-sample flag
    const has_wasted_bits = try br.takeBits(u1, 1) == 1;
    // Read unary to determine the number of bits to waste
    self.wasted_bits = 0;
    if (has_wasted_bits) {
        const wasted = try br.takeUnary(u6, 1);
        if (wasted >= 32) return error.InvalidBitDepth;
        self.wasted_bits = wasted + 1;
    }
}

/// Decodes the subframe data into the given buffer.
///
/// The entire channel contains a single, unvarying sample value (e.g., complete
/// silence or a digital constant DC offset). No residual signal is present.
///
/// * `br` - A bit reader for the FLAC stream.
/// * `buffer` - The output buffer to receive the samples.
/// * `bps` - The bits per sample of the data.
fn readConstant(br: *BitReader, buffer: []i64, bps: u6) BitReader.Error!void {
    const sample = try br.takeSignExtended(i64, bps);
    @memset(buffer, sample);
}

/// Decodes the subframe data into the given buffer.
///
/// The audio signal is uncompressed. Samples are stored raw as unaligned linear
/// PCM values. This fallback is typically triggered when high-entropy noise
/// makes predictive compression inefficient.
///
/// * `br` - A bit reader for the FLAC stream.
/// * `buffer` - The output buffer to receive the samples.
/// * `bps` - The bits per sample of the data.
fn readVerbatim(br: *BitReader, buffer: []i64, bps: u6) BitReader.Error!void {
    for (buffer) |*sample| {
        sample.* = try br.takeSignExtended(i64, bps);
    }
}

/// Decodes the subframe data into the given buffer.
///
/// Samples are approximated using fixed-coefficient linear predictors.
/// The polynomial formulas are hardcoded into the FLAC decoder specification.
///
/// * `br` - A bit reader for the FLAC stream.
/// * `buffer` - The output buffer to receive the samples.
/// * `bps` - The bits per sample of the data.
/// * `order` - The index of the fixed coefficients to select.
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

    // Warm-up samples
    for (0..order) |i| {
        buffer[i] = try br.takeSignExtended(i64, bps);
    }

    try decodeResiduals(br, buffer, buffer.len, order);
    if (order > 0) {
        try restoreLinearPrediction(buffer, fixed_coeffs[order], 0);
    }
}

/// Decodes the subframe data into the given buffer.
///
/// The encoder determines custom FIR filter coefficients (up to order 32) suited
/// specifically to the audio block. The decoder reconstructs the signal by
/// applying these coefficients to the historical warm-up samples, shifting the
/// result by a specified quantization level.
///
/// * `br` - A bit reader for the FLAC stream.
/// * `buffer` - The output buffer to receive the samples.
/// * `bps` - The bits per sample of the data.
/// * `order` - The number of coefficients to read from the stream.
fn readLinearPrediction(
    br: *BitReader,
    buffer: []i64,
    bps: u6,
    order: u6,
) !void {
    assert(order > 0);
    // Warm-up samples
    for (0..order) |i| {
        buffer[i] = try br.takeSignExtended(i64, bps);
    }

    // Read the precision and shift values
    const precision = try br.takeBits(u5, 4) + 1;
    if (precision == 16) return error.InvalidPredictor;
    const shift: u5 = @bitCast(try br.takeBits(u5, 5));
    // The shift is actually signed two's-complement, but must be positive
    if (@as(i5, @bitCast(shift)) < 0) {
        return error.InvalidShift;
    }

    // Stack allocated storage for coefficients.
    var coeff_buffer: [max_linear_predictor_coeffs]i64 = undefined;
    // Take a sub-slice for needed amount, and read coefficient values into it.
    const coeffs = coeff_buffer[0..order];
    for (coeffs) |*coeff| {
        coeff.* = try br.takeSignExtended(i64, precision);
    }

    try decodeResiduals(br, buffer, buffer.len, order);
    try restoreLinearPrediction(buffer, coeffs, shift);
}

/// Parses and decodes the Rice-coded error (residual) signal from the bitstream.
///
/// This function reads the residual section of a fixed or LPC subframe. It parses
/// the residual coding method, reads the partition order, and iteratively decodes
/// each Rice partition using the bit reader, populating the `buffer` directly
/// after the warm-up sample region.
///
/// * `br` - A bit reader for the FLAC stream.
/// * `buffer` - The output buffer to receive the samples.
/// * `warmup`- The number of warmup samples in the buffer.
fn decodeResiduals(
    br: *BitReader,
    buffer: ?[]i64,
    sample_count: usize,
    warmup: u6,
) !void {
    const method: enum(u2) { rice1 = 0, rice2 = 1, _ } = @enumFromInt(
        try br.takeBits(u2, 2),
    );

    // Determine number of bits for the param and escape value
    // base on which residual coding method is used.
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

/// Applies a Finite Impulse Response (FIR) filter to reconstruct the original
/// audio signal from its linear prediction coefficients.
///
/// Reconstructs the true PCM sample values by applying the custom LPC filter
/// coefficients against the historical warm-up and previously restored samples,
/// adding the decoded residual error value back to the result.
///
/// * `br` - A bit reader for the FLAC stream.
/// * `coeffs` - The custom FIR filter coefficients whose length matches the predictor order.
/// * `shift` - The right-shift quantization factor (scale).
fn restoreLinearPrediction(
    buffer: []i64,
    coeffs: []const i64,
    shift: u5,
) error{ InvalidBitDepth }!void {
    const order = coeffs.len;
    if (buffer.len <= order) return;

    // Slice the buffer so result starts at buffer[order]
    for (buffer[order..], order..) |*result, i| {
        var sum: i128 = 0;
        // Look backward into decoded history
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

/// Decodes a single variable-length integer from the bitstream.
///
/// * `br` - A bit reader for the FLAC stream.
/// * `k` - The Rice parameter (the bit-width of the binary remainder chunk)/
fn decodeRice(br: *BitReader, k: u5) BitReader.ReadUnaryError!i64 {
    // Read the unary-encoded quotient
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
