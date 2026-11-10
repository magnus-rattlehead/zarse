// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2007-2008 CSIRO
// SPDX-FileCopyrightText: © 2007-2009 Xiph.Org Foundation
// SPDX-License-Identifier: MIT AND BSD-2-Clause

//! Band energies: how loud each band is, which CELT codes before anything
//! about the band's shape. RFC 6716, Section 4.3.2, following libopus's
//! `quant_bands.c` and `laplace.c`.
//!
//! Energies are in log2 units (6 dB each) and coded in three passes:
//!
//! 1. **Coarse**: to within 6 dB, as a prediction error from the band below
//!    and the same band in the previous frame, with a Laplace-like
//!    distribution (`laplaceDecode`);
//! 2. **Fine**: a few more bits for each band, as the bit allocation says;
//! 3. **Final**: one more bit for some bands, from what is left of the frame
//!    once everything else has been decoded.
//!
//! The energies persist from frame to frame, since the coarse pass predicts
//! from them.

const std = @import("std");
const RangeDecoder = @import("../RangeDecoder.zig");
const mode = @import("mode.zig");

/// The most fine-energy bits a band gets. `rate.h`.
pub const max_fine_bits = 8;

/// The mean energy of each band, which is added back once decoding is done.
/// The coded energies are relative to it.
pub const e_means = [25]f32{
    6.437500,
    6.250000,
    5.750000,
    5.312500,
    5.062500,
    4.812500,
    4.500000,
    4.375000,
    4.875000,
    4.687500,
    4.562500,
    4.437500,
    4.875000,
    4.625000,
    4.312500,
    4.500000,
    4.375000,
    4.625000,
    4.750000,
    4.437500,
    3.750000,
    3.750000,
    3.750000,
    3.750000,
    3.750000,
};

/// How much of the previous frame's energy each frame size predicts from:
/// 0.9, 0.8, 0.65 and 0.5. Section 4.3.2.1.
const pred_coef = [4]f32{
    29440.0 / 32768.0,
    26112.0 / 32768.0,
    21248.0 / 32768.0,
    16384.0 / 32768.0,
};
/// How much of the band below's prediction error carries into the next.
const beta_coef = [4]f32{
    30147.0 / 32768.0,
    22282.0 / 32768.0,
    12124.0 / 32768.0,
    6554.0 / 32768.0,
};
/// The same, for an intra frame, which predicts from nothing before it.
const beta_intra: f32 = 4915.0 / 32768.0;

/// The Laplace model for each frame size, inter or intra, and band: the
/// probability of a zero, and the decay, both in Q8. Section 4.3.2.1.
const e_prob_model = [4][2][42]u8{
    .{
        .{
            72,
            127,
            65,
            129,
            66,
            128,
            65,
            128,
            64,
            128,
            62,
            128,
            64,
            128,
            64,
            128,
            92,
            78,
            92,
            79,
            92,
            78,
            90,
            79,
            116,
            41,
            115,
            40,
            114,
            40,
            132,
            26,
            132,
            26,
            145,
            17,
            161,
            12,
            176,
            10,
            177,
            11,
        },
        .{
            24,
            179,
            48,
            138,
            54,
            135,
            54,
            132,
            53,
            134,
            56,
            133,
            55,
            132,
            55,
            132,
            61,
            114,
            70,
            96,
            74,
            88,
            75,
            88,
            87,
            74,
            89,
            66,
            91,
            67,
            100,
            59,
            108,
            50,
            120,
            40,
            122,
            37,
            97,
            43,
            78,
            50,
        },
    },
    .{
        .{
            83,
            78,
            84,
            81,
            88,
            75,
            86,
            74,
            87,
            71,
            90,
            73,
            93,
            74,
            93,
            74,
            109,
            40,
            114,
            36,
            117,
            34,
            117,
            34,
            143,
            17,
            145,
            18,
            146,
            19,
            162,
            12,
            165,
            10,
            178,
            7,
            189,
            6,
            190,
            8,
            177,
            9,
        },
        .{
            23,
            178,
            54,
            115,
            63,
            102,
            66,
            98,
            69,
            99,
            74,
            89,
            71,
            91,
            73,
            91,
            78,
            89,
            86,
            80,
            92,
            66,
            93,
            64,
            102,
            59,
            103,
            60,
            104,
            60,
            117,
            52,
            123,
            44,
            138,
            35,
            133,
            31,
            97,
            38,
            77,
            45,
        },
    },
    .{
        .{
            61,
            90,
            93,
            60,
            105,
            42,
            107,
            41,
            110,
            45,
            116,
            38,
            113,
            38,
            112,
            38,
            124,
            26,
            132,
            27,
            136,
            19,
            140,
            20,
            155,
            14,
            159,
            16,
            158,
            18,
            170,
            13,
            177,
            10,
            187,
            8,
            192,
            6,
            175,
            9,
            159,
            10,
        },
        .{
            21,
            178,
            59,
            110,
            71,
            86,
            75,
            85,
            84,
            83,
            91,
            66,
            88,
            73,
            87,
            72,
            92,
            75,
            98,
            72,
            105,
            58,
            107,
            54,
            115,
            52,
            114,
            55,
            112,
            56,
            129,
            51,
            132,
            40,
            150,
            33,
            140,
            29,
            98,
            35,
            77,
            42,
        },
    },
    .{
        .{
            42,
            121,
            96,
            66,
            108,
            43,
            111,
            40,
            117,
            44,
            123,
            32,
            120,
            36,
            119,
            33,
            127,
            33,
            134,
            34,
            139,
            21,
            147,
            23,
            152,
            20,
            158,
            25,
            154,
            26,
            166,
            21,
            173,
            16,
            184,
            13,
            184,
            10,
            150,
            13,
            139,
            15,
        },
        .{
            22,
            178,
            63,
            114,
            74,
            82,
            84,
            83,
            92,
            82,
            103,
            62,
            96,
            72,
            96,
            67,
            101,
            73,
            107,
            72,
            113,
            55,
            118,
            52,
            125,
            52,
            118,
            52,
            117,
            55,
            135,
            49,
            137,
            39,
            157,
            32,
            145,
            29,
            97,
            33,
            77,
            40,
        },
    },
};

/// What a band's coarse energy is when a frame has run out of bits: -1, 0
/// or 1, coded with a small table.
const small_energy_icdf = [3]u8{ 2, 1, 0 };

const laplace_log_minp = 0;
const laplace_minp = 1 << laplace_log_minp;
/// The least number of values that get the minimum probability, on each
/// side.
const laplace_nmin = 16;

fn laplaceGetFreq1(fs0: u32, decay: u32) u32 {
    const ft = 32768 - laplace_minp * (2 * laplace_nmin) - fs0;
    return ft * (16384 - decay) >> 15;
}

/// A value from a Laplace-like distribution: probability `fs`/32768 of
/// zero, falling away geometrically by `decay`/16384 on either side, and
/// never below the minimum. `laplace.c`.
pub fn laplaceDecode(dec: *RangeDecoder, fs_: u32, decay: u32) i32 {
    var fs = fs_;
    var val: i32 = 0;
    const fm = dec.decodeBin(15);
    var fl: u32 = 0;
    if (fm >= fs) {
        val += 1;
        fl = fs;
        fs = laplaceGetFreq1(fs, decay) + laplace_minp;
        // The decaying part of the distribution.
        while (fs > laplace_minp and fm >= fl + 2 * fs) {
            fs *= 2;
            fl += fs;
            fs = ((fs - 2 * laplace_minp) * decay) >> 15;
            fs += laplace_minp;
            val += 1;
        }
        // Everything beyond has the minimum probability.
        if (fs <= laplace_minp) {
            const di = (fm - fl) >> (laplace_log_minp + 1);
            val += @intCast(di);
            fl += 2 * di * laplace_minp;
        }
        if (fm < fl + fs) val = -val else fl += fs;
    }
    dec.update(fl, @min(fl + fs, 32768), 32768);
    return val;
}

/// The coarse energies of bands `[start, end)`, for `channels` channels,
/// into `old`, which holds the previous frame's and is updated in place.
/// `old` is laid out a channel at a time, `mode.nb_ebands` to each.
pub fn decodeCoarse(
    dec: *RangeDecoder,
    start: usize,
    end: usize,
    old: []f32,
    intra: bool,
    channels: usize,
    lm: usize,
) void {
    const prob_model = &e_prob_model[lm][@intFromBool(intra)];
    var prev = [2]f32{ 0, 0 };
    const coef: f32 = if (intra) 0 else pred_coef[lm];
    const beta: f32 = if (intra) beta_intra else beta_coef[lm];
    const budget: i32 = @intCast(dec.buf.len * 8);

    for (start..end) |i| {
        for (0..channels) |c| {
            const tell = dec.tell();
            const qi: i32 = if (budget - tell >= 15) blk: {
                const pi = 2 * @as(usize, @min(i, 20));
                break :blk laplaceDecode(
                    dec,
                    @as(u32, prob_model[pi]) << 7,
                    @as(u32, prob_model[pi + 1]) << 6,
                );
            } else if (budget - tell >= 2) blk: {
                const s: i32 = @intCast(dec.decodeIcdf(&small_energy_icdf, 2));
                break :blk (s >> 1) ^ -(s & 1);
            } else if (budget - tell >= 1)
                -@as(i32, @intFromBool(dec.decodeBitLogp(1)))
            else
                -1;
            const q: f32 = @floatFromInt(qi);
            const e = &old[i + c * mode.nb_ebands];
            e.* = @max(-9.0, e.*);
            e.* = coef * e.* + prev[c] + q;
            prev[c] = prev[c] + q - beta * q;
        }
    }
}

/// The fine energies: `fine_quant[i]` more bits for band `i`, each
/// channel. Section 4.3.2.2.
pub fn decodeFine(
    dec: *RangeDecoder,
    start: usize,
    end: usize,
    old: []f32,
    fine_quant: []const i32,
    channels: usize,
) void {
    for (start..end) |i| {
        if (fine_quant[i] <= 0) continue;
        const bits: u5 = @intCast(fine_quant[i]);
        for (0..channels) |c| {
            const q2: f32 = @floatFromInt(dec.decodeBits(bits));
            const scale: f32 = @floatFromInt(
                @as(u32, 1) << @intCast(14 - fine_quant[i]),
            );
            old[i + c * mode.nb_ebands] += (q2 + 0.5) * scale * (1.0 / 16384.0)
                - 0.5;
        }
    }
}

/// The last bits of the frame, one more for each band that can take it:
/// the higher-priority bands first. Section 4.3.2.2.
pub fn decodeFinal(
    dec: *RangeDecoder,
    start: usize,
    end: usize,
    old: []f32,
    fine_quant: []const i32,
    fine_priority: []const i32,
    bits_left_: i32,
    channels: usize,
) void {
    var bits_left = bits_left_;
    for (0..2) |prio| {
        var i = start;
        while (i < end and bits_left >= @as(i32, @intCast(channels)))
        : (i += 1) {
            if (fine_quant[i] >= max_fine_bits or fine_priority[i] != prio)
                continue;
            for (0..channels) |c| {
                const q2: f32 = @floatFromInt(dec.decodeBits(1));
                const scale: f32 = @floatFromInt(
                    @as(u32, 1) << @intCast(14 - fine_quant[i] - 1),
                );
                old[i + c * mode.nb_ebands] += (q2 - 0.5)
                    * scale
                    * (1.0 / 16384.0);
                bits_left -= 1;
            }
        }
    }
}

