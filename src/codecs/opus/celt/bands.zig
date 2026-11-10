// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2007-2008 CSIRO
// SPDX-FileCopyrightText: © 2007-2009 Xiph.Org Foundation
// SPDX-FileCopyrightText: © 2008-2009 Gregory Maxwell
// SPDX-License-Identifier: MIT AND BSD-2-Clause

//! The bands' shapes: RFC 6716, Section 4.3.4, following the decoder's side
//! of libopus's `bands.c`.
//!
//! Each band's normalized spectrum is decoded from the bits allocation gave
//! it. A band with more bits than one vector of pulses can use is split in
//! two, recursively, with the split's angle coded between the halves. A
//! stereo band is coded as mid and side the same way. A band with no bits
//! is folded -- a copy of a lower band's decoded spectrum, or noise -- so
//! that it is never silent. The time-frequency resolution of each band can
//! be changed with Haar transforms before any of that.

const std = @import("std");
const RangeDecoder = @import("../RangeDecoder.zig");
const mode = @import("mode.zig");
const rate = @import("rate.zig");
const pvq = @import("pvq.zig");
const simd = @import("../simd.zig");
const Arch = @import("../arch.zig").Arch;
const energy = @import("energy.zig");

const bitres = RangeDecoder.bitres;
/// How much less than their share of bits a split's angle gets.
const qtheta_offset = 4;
const qtheta_offset_twophase = 16;
/// The largest band: 22 bins of the widest band, at 20 ms.
const max_band = 176;

/// The random generator libopus folds noise with.
pub fn lcgRand(seed: u32) u32 {
    return 1664525 *% seed +% 1013904223;
}

/// `(a * b + 16384) >> 15`, each operand first narrowed to 16 bits as
/// libopus's `FRAC_MUL16` casts them.
fn fracMul16(a: i32, b: i32) i32 {
    const a16: i32 = @as(i16, @truncate(a));
    const b16: i32 = @as(i16, @truncate(b));
    return (16384 + a16 * b16) >> 15;
}

/// A cosine that comes out the same everywhere, since it decides how bits
/// are allocated: of `x` in units of pi/2 over 16384.
pub fn bitexactCos(x: i16) i16 {
    const tmp: i32 = (4096 + @as(i32, x) * x) >> 13;
    var x2: i32 = @as(i16, @truncate(tmp));
    x2 = (32767 - x2)
        + fracMul16(x2, (-7651 + fracMul16(x2, (8277 + fracMul16(-626, x2)))));
    return @truncate(1 + x2);
}

/// log2 of `isin / icos`, in 1/2048, bit-exactly.
pub fn bitexactLog2Tan(isin_: i32, icos_: i32) i32 {
    const lc: i32 = RangeDecoder.ilog(@intCast(icos_));
    const ls: i32 = RangeDecoder.ilog(@intCast(isin_));
    const icos = icos_ << @intCast(15 - lc);
    const isin = isin_ << @intCast(15 - ls);
    return (ls - lc) * (1 << 11)
        + fracMul16(isin, fracMul16(isin, -2597) + 7932)
        - fracMul16(icos, fracMul16(icos, -2597) + 7932);
}

/// The integer square root, rounded down.
pub fn isqrt32(val_: u32) u32 {
    var val = val_;
    var g: u32 = 0;
    var bshift: i32 = (@as(i32, RangeDecoder.ilog(val)) - 1) >> 1;
    var b: u32 = @as(u32, 1) << @intCast(bshift);
    while (true) {
        const t = ((g << 1) + b) << @intCast(bshift);
        if (t <= val) {
            g += b;
            val -= t;
        }
        b >>= 1;
        bshift -= 1;
        if (bshift < 0) break;
    }
    return g;
}

/// A split: the angle between its halves, and what follows from it.
const Split = struct {
    inv: bool,
    imid: i32,
    iside: i32,
    delta: i32,
    itheta: i32,
    qalloc: i32,
};

/// How finely a split's angle is coded.
fn computeQn(n: i32, b: i32, offset: i32, pulse_cap: i32, stereo: bool) i32 {
    const exp2_table8 = [8]i32{
        16384,
        17866,
        19483,
        21247,
        23170,
        25267,
        27554,
        30048,
    };
    var n2 = 2 * n - 1;
    if (stereo and n == 2) n2 -= 1;
    // Enough left over, for a stereo split at 16384, to code a pulse in the
    // side, which is not folded and would otherwise collapse.
    var qb = @divTrunc(b + n2 * offset, n2);
    qb = @min(b - pulse_cap - (4 << bitres), qb);
    qb = @min(8 << bitres, qb);
    if (qb < (1 << bitres >> 1)) return 1;
    const qn = exp2_table8[@intCast(qb & 0x7)] >> @intCast(14 - (qb >> bitres));
    return (qn + 1) >> 1 << 1;
}

/// The ordering of a Hadamard transform's outputs, for 2, 4, 8 and 16.
const ordery_table = [_]usize{
    1,
    0,
    3,
    0,
    2,
    1,
    7,
    0,
    4,
    3,
    6,
    1,
    5,
    2,
    15,
    0,
    8,
    7,
    12,
    3,
    11,
    4,
    14,
    1,
    9,
    6,
    13,
    2,
    10,
    5,
};

fn deinterleaveHadamard(
    x: []f32,
    n0: usize,
    stride: usize,
    hadamard: bool,
) void {
    var tmp: [max_band]f32 = undefined;
    const n = n0 * stride;
    if (hadamard) {
        const ordery = ordery_table[stride - 2..];
        for (0..stride) |i|
            for (0..n0) |j| {
                tmp[ordery[i] * n0 + j] = x[j * stride + i];
            };
    } else {
        for (0..stride) |i|
            for (0..n0) |j| {
                tmp[i * n0 + j] = x[j * stride + i];
            };
    }
    @memcpy(x[0..n], tmp[0..n]);
}

fn interleaveHadamard(x: []f32, n0: usize, stride: usize, hadamard: bool) void {
    var tmp: [max_band]f32 = undefined;
    const n = n0 * stride;
    if (hadamard) {
        const ordery = ordery_table[stride - 2..];
        for (0..stride) |i|
            for (0..n0) |j| {
                tmp[j * stride + i] = x[ordery[i] * n0 + j];
            };
    } else {
        for (0..stride) |i|
            for (0..n0) |j| {
                tmp[j * stride + i] = x[i * n0 + j];
            };
    }
    @memcpy(x[0..n], tmp[0..n]);
}

/// One level of a Haar transform across `stride` interleaved blocks.
pub fn haar1(x: []f32, n0_: usize, stride: usize) void {
    const n0 = n0_ >> 1;
    for (0..stride) |i|
        for (0..n0) |j| {
            const tmp1 = 0.70710678 * x[stride * 2 * j + i];
            const tmp2 = 0.70710678 * x[stride * (2 * j + 1) + i];
            x[stride * 2 * j + i] = tmp1 + tmp2;
            x[stride * (2 * j + 1) + i] = tmp1 - tmp2;
        };
}

/// Mid and side back to left and right.
fn stereoMerge(arch: Arch, x: []f32, y: []f32, mid: f32, n: usize) void {
    // |X+Y|^2 and |X-Y|^2 as |X|^2 + |Y|^2 +- 2 sum(xy).
    var xp: f32 = undefined;
    var side: f32 = undefined;
    simd.dualInnerProd(arch, y[0..n], x, y, &xp, &side);
    // For the mid's normalization.
    xp = mid * xp;
    const mid2 = mid;
    const el = mid2 * mid2 + side - 2 * xp;
    const er = mid2 * mid2 + side + 2 * xp;
    if (er < 6e-4 or el < 6e-4) {
        @memcpy(y[0..n], x[0..n]);
        return;
    }
    const lgain = 1.0 / @as(f32, @floatCast(@sqrt(@as(f64, el))));
    const rgain = 1.0 / @as(f32, @floatCast(@sqrt(@as(f64, er))));
    for (0..n) |j| {
        const l = mid * x[j];
        const r = y[j];
        x[j] = lgain * (l - r);
        y[j] = rgain * (l + r);
    }
}

/// In hybrid mode, enough of the first band's folding data to fold the
/// second.
fn specialHybridFolding(
    norm: []f32,
    norm2: []f32,
    start: usize,
    m: usize,
    dual_stereo: bool,
) void {
    const n1 = m
        * @as(usize, @intCast(mode.ebands[start + 1] - mode.ebands[start]));
    const n2 = m
        * @as(usize, @intCast(mode.ebands[start + 2] - mode.ebands[start + 1]));
    if (n2 <= n1) return;
    @memcpy(norm[n1..n2], norm[2 * n1 - n2..n1]);
    if (dual_stereo) @memcpy(norm2[n1..n2], norm2[2 * n1 - n2..n1]);
}

/// What stays the same from band to band, and what carries between
/// them. `band_ctx`.
const Context = struct {
    ec: *RangeDecoder,
    arch: Arch,
    /// The band being coded.
    i: usize = 0,
    intensity: usize,
    spread: pvq.Spread,
    tf_change: i32 = 0,
    /// Bits left in the frame, in eighths.
    remaining_bits: i32 = 0,
    seed: u32,
    /// Whether to leave the sign of an intensity-stereo side alone,
    /// which makes a mono downmix of the output cleaner.
    disable_inv: bool,
    avoid_split_noise: bool = false,
};

/// Decode the angle of a split. Section 4.3.4.1,
/// `compute_theta`.
fn computeTheta(
    ctx: *Context,
    n: i32,
    b: *i32,
    bb: u5,
    b0: u5,
    lm: i32,
    stereo: bool,
    fill: *u32,
) Split {
    const ec = ctx.ec;
    const pulse_cap = mode.log_n[ctx.i] + lm * (1 << bitres);
    const offset = (pulse_cap >> 1)
        - (if (stereo and n == 2)
            @as(i32, qtheta_offset_twophase)
        else
            qtheta_offset);
    var qn = computeQn(n, b.*, offset, pulse_cap, stereo);
    if (stereo and ctx.i >= ctx.intensity) qn = 1;

    var itheta: i32 = 0;
    var inv = false;
    // The angle between the halves, whose ratio of side to mid it is:
    // knowing both have unit norm and are orthogonal, it rescales
    // both.

    const tell: i32 = @intCast(ec.tellFrac());
    if (qn != 1) {
        if (stereo and n > 2) {
            // A step: probability 3 up to half-way, 1 after.
            const p0 = 3;
            const x0 = @divTrunc(qn, 2);
            const ft: u32 = @intCast(p0 * (x0 + 1) + x0);
            const fs: i32 = @intCast(ec.decode(ft));
            const xv = if (fs < (x0 + 1) * p0)
                @divTrunc(fs, p0)
            else
                x0 + 1 + (fs - (x0 + 1) * p0);
            const fl: i32 = if (xv <= x0)
                p0 * xv
            else
                (xv - 1 - x0) + (x0 + 1) * p0;
            const fh: i32 = if (xv <= x0)
                p0 * (xv + 1)
            else
                (xv - x0) + (x0 + 1) * p0;
            ec.update(@intCast(fl), @intCast(fh), ft);
            itheta = xv;
        } else if (b0 > 1 or stereo) {
            // Uniform.
            itheta = @intCast(ec.decodeUint(@intCast(qn + 1)));
        } else {
            // Triangular.
            const half = qn >> 1;
            const ft: i32 = (half + 1) * (half + 1);
            var fl: i32 = undefined;
            var fs: i32 = undefined;
            {
                const fm: i32 = @intCast(ec.decode(@intCast(ft)));
                if (fm < (half * (half + 1) >> 1)) {
                    itheta = (@as(
                        i32,
                        @intCast(isqrt32(8 * @as(u32, @intCast(fm)) + 1)),
                    )
                        - 1)
                        >> 1;
                    fs = itheta + 1;
                    fl = itheta * (itheta + 1) >> 1;
                } else {
                    itheta = (2 * (qn + 1)
                        - @as(
                            i32,
                            @intCast(
                                isqrt32(
                                    8 * @as(u32, @intCast(ft - fm - 1)) + 1,
                                ),
                            ),
                        ))
                        >> 1;
                    fs = qn + 1 - itheta;
                    fl = ft - ((qn + 1 - itheta) * (qn + 2 - itheta) >> 1);
                }
                ec.update(@intCast(fl), @intCast(fl + fs), @intCast(ft));
            }
        }
        itheta = @intCast(
            @as(u32, @intCast(itheta * 16384)) / @as(u32, @intCast(qn)),
        );
    } else if (stereo) {
        if (b.* > 2 << bitres and ctx.remaining_bits > 2 << bitres) {
            inv = ec.decodeBitLogp(2);
        } else {
            inv = false;
        }
        // Overridden, to keep a downmix clean.
        if (ctx.disable_inv) inv = false;
        itheta = 0;
    }
    const qalloc = @as(i32, @intCast(ec.tellFrac())) - tell;
    b.* -= qalloc;

    var imid: i32 = undefined;
    var iside: i32 = undefined;
    var delta: i32 = undefined;
    const all: u32 = (@as(u32, 1) << bb) - 1;
    if (itheta == 0) {
        imid = 32767;
        iside = 0;
        fill.* &= all;
        delta = -16384;
    } else if (itheta == 16384) {
        imid = 0;
        iside = 32767;
        fill.* &= all << bb;
        delta = 16384;
    } else {
        imid = bitexactCos(@intCast(itheta));
        iside = bitexactCos(@intCast(16384 - itheta));
        // The split of bits between mid and side that minimizes the
        // squared error in the band.
        delta = fracMul16((n - 1) << 7, bitexactLog2Tan(iside, imid));
    }
    return .{
        .inv = inv,
        .imid = imid,
        .iside = iside,
        .delta = delta,
        .itheta = itheta,
        .qalloc = qalloc,
    };
}

/// A band of one coefficient: just its sign. `quant_band_n1`.
fn decodeBandN1(ctx: *Context, x: []f32, y: ?[]f32, lowband_out: ?[]f32) u32 {
    for (0..@as(usize, if (y != null) 2 else 1)) |c| {
        const v = if (c == 0) x else y.?;
        var sign: u32 = 0;
        if (ctx.remaining_bits >= 1 << bitres) {
            sign = ctx.ec.decodeBits(1);
            ctx.remaining_bits -= 1 << bitres;
        }
        v[0] = if (sign != 0) -1.0 else 1.0;
    }
    if (lowband_out) |out| out[0] = x[0];
    return 1;
}

/// A mono band, or half of one: split in two while it has more bits
/// than a vector of pulses can use, coded as pulses when it has some,
/// folded when it has none. `quant_partition`.
fn decodePartition(
    ctx: *Context,
    x: []f32,
    n_: usize,
    b_: i32,
    bb_: u5,
    lowband: ?[]f32,
    lm_: i32,
    gain: f32,
    fill_: u32,
) u32 {
    var n = n_;
    var b = b_;
    var bb = bb_;
    var lm = lm_;
    var fill = fill_;
    const b0 = bb;
    var cm: u32 = 0;

    // Split if it needs more than one and a half bits more than a
    // vector can give.
    const cache = rate.cacheRow(ctx.i, lm);
    if (lm != -1 and b > @as(i32, cache[cache[0]]) + 12 and n > 2) {
        n >>= 1;
        const y = x[n..];
        lm -= 1;
        if (bb == 1) fill = (fill & 1) | (fill << 1);
        bb = (bb + 1) >> 1;

        const split = computeTheta(
            ctx,
            @intCast(n),
            &b,
            bb,
            b0,
            lm,
            false,
            &fill,
        );
        const mid: f32 = (1.0 / 32768.0) * @as(f32, @floatFromInt(split.imid));
        const side: f32 = (1.0 / 32768.0)
            * @as(f32, @floatFromInt(split.iside));
        var delta = split.delta;

        // More bits to low-energy short MDCTs than they would otherwise
        // get.
        if (b0 > 1 and (split.itheta & 0x3fff) != 0) {
            if (split.itheta > 8192) {
                // Pre-echo masking, roughly.
                delta -= delta >> @intCast(4 - lm);
            } else {
                // A forward-masking slope of 1.5 dB per 10 ms.
                delta = @min(
                    0,
                    delta
                        + (@as(i32, @intCast(n)) << bitres >> @intCast(5 - lm)),
                );
            }
        }
        var mbits: i32 = @max(0, @min(b, @divTrunc(b - delta, 2)));
        var sbits = b - mbits;
        ctx.remaining_bits -= split.qalloc;

        const next_lowband2: ?[]f32 = if (lowband) |l| l[n..] else null;
        var rebalance = ctx.remaining_bits;
        if (mbits >= sbits) {
            cm = decodePartition(
                ctx,
                x,
                n,
                mbits,
                bb,
                lowband,
                lm,
                gain * mid,
                fill,
            );
            rebalance = mbits - (rebalance - ctx.remaining_bits);
            if (rebalance > 3 << bitres and split.itheta != 0)
                sbits += rebalance - (3 << bitres);
            cm |= decodePartition(
                ctx,
                y,
                n,
                sbits,
                bb,
                next_lowband2,
                lm,
                gain * side,
                fill >> bb,
            )
                << (b0 >> 1);
        } else {
            cm = decodePartition(
                ctx,
                y,
                n,
                sbits,
                bb,
                next_lowband2,
                lm,
                gain * side,
                fill >> bb,
            )
                << (b0 >> 1);
            rebalance = sbits - (rebalance - ctx.remaining_bits);
            if (rebalance > 3 << bitres and split.itheta != 16384)
                mbits += rebalance - (3 << bitres);
            cm |= decodePartition(
                ctx,
                x,
                n,
                mbits,
                bb,
                lowband,
                lm,
                gain * mid,
                fill,
            );
        }
        return cm;
    }

    // No split.
    var q = rate.bits2pulses(ctx.i, lm, b);
    var curr_bits = rate.pulses2bits(ctx.i, lm, q);
    ctx.remaining_bits -= curr_bits;
    // Never past the budget.
    while (ctx.remaining_bits < 0 and q > 0) {
        ctx.remaining_bits += curr_bits;
        q -= 1;
        curr_bits = rate.pulses2bits(ctx.i, lm, q);
        ctx.remaining_bits -= curr_bits;
    }

    if (q != 0) {
        const k: u32 = @intCast(rate.getPulses(q));

        return pvq.algUnquant(x, n, k, ctx.spread, bb, ctx.ec, gain);
    }

    // No pulses: fill the band anyway.

    const cm_mask: u32 = @truncate((@as(u64, 1) << bb) - 1);
    fill &= cm_mask;
    if (fill == 0) {
        @memset(x[0..n], 0);
        return 0;
    }
    if (lowband) |l| {
        // The folded spectrum, about 48 dB above the noise it replaces.
        for (x[0..n], l[0..n]) |*xj, lj| {
            ctx.seed = lcgRand(ctx.seed);
            const tmp: f32 = 1.0 / 256.0;
            xj.* = lj + (if (ctx.seed & 0x8000 != 0) tmp else -tmp);
        }
        cm = fill;
    } else {
        // Noise.
        for (x[0..n]) |*xj| {
            ctx.seed = lcgRand(ctx.seed);
            xj.* = @floatFromInt(@as(i32, @bitCast(ctx.seed)) >> 20);
        }
        cm = cm_mask;
    }
    pvq.renormaliseVector(ctx.arch, x[0..n], gain);
    return cm;
}

/// A mono band, with its time-frequency resolution changed first as
/// the frame says, and changed back afterwards. `quant_band`.
fn decodeBand(
    ctx: *Context,
    x: []f32,
    n: usize,
    b: i32,
    bb_: u5,
    lowband_: ?[]f32,
    lm: i32,
    lowband_out: ?[]f32,
    gain: f32,
    lowband_scratch: ?[]f32,
    fill_: u32,
) u32 {
    var bb = bb_;
    var lowband = lowband_;
    var fill = fill_;
    const n0 = n;
    var n_b = n / bb;
    const long_blocks = bb == 1;
    var tf_change = ctx.tf_change;

    if (n == 1) return decodeBandN1(ctx, x, null, lowband_out);

    const recombine: u5 = if (tf_change > 0) @intCast(tf_change) else 0;
    // Work on a copy of the lowband, which the transforms change.
    if (lowband_scratch) |scratch|
        if (lowband) |l| {
            if (
                recombine != 0
                    or ((n_b & 1) == 0 and tf_change < 0)
                    or bb > 1
            ) {
                @memcpy(scratch[0..n], l[0..n]);
                lowband = scratch;
            }
        };

    // Recombining bands, for more frequency resolution.
    const bit_interleave_table = [16]u32{
        0,
        1,
        1,
        1,
        2,
        3,
        3,
        3,
        2,
        3,
        3,
        3,
        2,
        3,
        3,
        3,
    };
    for (0..recombine) |k| {
        if (lowband) |l|
            haar1(l, n >> @intCast(k), @as(usize, 1) << @intCast(k));
        fill = bit_interleave_table[fill & 0xf]
            | bit_interleave_table[fill >> 4] << 2;
    }
    bb >>= @intCast(recombine);
    n_b <<= recombine;

    // Dividing in time, for more time resolution.
    var time_divide: u5 = 0;
    while ((n_b & 1) == 0 and tf_change < 0) {
        if (lowband) |l| haar1(l, n_b, bb);
        fill |= fill << bb;
        bb <<= 1;
        n_b >>= 1;
        time_divide += 1;
        tf_change += 1;
    }
    const b0 = bb;
    const n_b0 = n_b;

    // Time order rather than frequency order.
    if (b0 > 1) {
        if (lowband) |l|
            deinterleaveHadamard(
                l,
                n_b >> recombine,
                @as(usize, b0) << recombine,
                long_blocks,
            );
    }

    var cm = decodePartition(ctx, x, n, b, bb, lowband, lm, gain, fill);

    // And back.
    if (b0 > 1)
        interleaveHadamard(
            x,
            n_b >> recombine,
            @as(usize, b0) << recombine,
            long_blocks,
        );
    n_b = n_b0;
    bb = b0;
    for (0..time_divide) |_| {
        bb >>= 1;
        n_b <<= 1;
        cm |= cm >> bb;
        haar1(x, n_b, bb);
    }
    const bit_deinterleave_table = [16]u32{
        0x00,
        0x03,
        0x0C,
        0x0F,
        0x30,
        0x33,
        0x3C,
        0x3F,
        0xC0,
        0xC3,
        0xCC,
        0xCF,
        0xF0,
        0xF3,
        0xFC,
        0xFF,
    };
    for (0..recombine) |k| {
        cm = bit_deinterleave_table[cm];
        haar1(x, n0 >> @intCast(k), @as(usize, 1) << @intCast(k));
    }
    bb <<= @intCast(recombine);

    // Scaled for folding from, later.
    if (lowband_out) |out| {
        const scale: f32 = @floatCast(@sqrt(@as(f64, @floatFromInt(n0))));
        for (out[0..n0], x[0..n0]) |*o, xj| o.* = scale * xj;
    }
    cm &= @truncate((@as(u64, 1) << bb) - 1);
    return cm;
}

/// A stereo band, as mid and side. `quant_band_stereo`.
fn decodeBandStereo(
    ctx: *Context,
    x: []f32,
    y: []f32,
    n: usize,
    b_: i32,
    bb: u5,
    lowband: ?[]f32,
    lm: i32,
    lowband_out: ?[]f32,
    lowband_scratch: ?[]f32,
    fill_: u32,
) u32 {
    var b = b_;
    var fill = fill_;
    if (n == 1) return decodeBandN1(ctx, x, y, lowband_out);
    const orig_fill = fill;

    const split = computeTheta(ctx, @intCast(n), &b, bb, bb, lm, true, &fill);
    const mid: f32 = (1.0 / 32768.0) * @as(f32, @floatFromInt(split.imid));
    const side: f32 = (1.0 / 32768.0) * @as(f32, @floatFromInt(split.iside));
    var cm: u32 = 0;

    if (n == 2) {
        // Mid and side are orthogonal, so the side is one bit: its
        // sign.
        var mbits = b;
        var sbits: i32 = 0;
        if (split.itheta != 0 and split.itheta != 16384) sbits = 1 << bitres;
        mbits -= sbits;
        const swap = split.itheta > 8192;
        ctx.remaining_bits -= split.qalloc + sbits;
        const x2 = if (swap) y else x;
        const y2 = if (swap) x else y;
        var sign: i32 = 0;
        if (sbits != 0) {
            sign = @intCast(ctx.ec.decodeBits(1));
        }
        sign = 1 - 2 * sign;
        // `orig_fill`, to fold the side even when itheta==16384
        // cleared fill's low bits.
        cm = decodeBand(
            ctx,
            x2,
            n,
            mbits,
            bb,
            lowband,
            lm,
            lowband_out,
            1.0,
            lowband_scratch,
            orig_fill,
        );
        const s: f32 = @floatFromInt(sign);
        y2[0] = -s * x2[1];
        y2[1] = s * x2[0];
        {
            x[0] = mid * x[0];
            x[1] = mid * x[1];
            y[0] = side * y[0];
            y[1] = side * y[1];
            var tmp = x[0];
            x[0] = tmp - y[0];
            y[0] = tmp + y[0];
            tmp = x[1];
            x[1] = tmp - y[1];
            y[1] = tmp + y[1];
        }
    } else {
        var mbits: i32 = @max(0, @min(b, @divTrunc(b - split.delta, 2)));
        var sbits = b - mbits;
        ctx.remaining_bits -= split.qalloc;
        var rebalance = ctx.remaining_bits;
        // The mid is left unscaled, since it is folded from later. The
        // side is never folded, so fill's high bits are zero for it.
        if (mbits >= sbits) {
            cm = decodeBand(
                ctx,
                x,
                n,
                mbits,
                bb,
                lowband,
                lm,
                lowband_out,
                1.0,
                lowband_scratch,
                fill,
            );
            rebalance = mbits - (rebalance - ctx.remaining_bits);
            if (rebalance > 3 << bitres and split.itheta != 0)
                sbits += rebalance - (3 << bitres);
            cm |= decodeBand(
                ctx,
                y,
                n,
                sbits,
                bb,
                null,
                lm,
                null,
                side,
                null,
                fill >> bb,
            );
        } else {
            cm = decodeBand(
                ctx,
                y,
                n,
                sbits,
                bb,
                null,
                lm,
                null,
                side,
                null,
                fill >> bb,
            );
            rebalance = sbits - (rebalance - ctx.remaining_bits);
            if (rebalance > 3 << bitres and split.itheta != 16384)
                mbits += rebalance - (3 << bitres);
            cm |= decodeBand(
                ctx,
                x,
                n,
                mbits,
                bb,
                lowband,
                lm,
                lowband_out,
                1.0,
                lowband_scratch,
                fill,
            );
        }
    }

    if (n != 2) stereoMerge(ctx.arch, x, y, mid, n);
    if (split.inv) for (y[0..n]) |*v| {
        v.* = -v.*;
    };
    return cm;
}

/// Decode every band's shape, `[start, end)`, into `x` and, for
/// stereo, `y`. `quant_all_bands`.
pub fn decodeAll(
    start: usize,
    end: usize,
    x: []f32,
    y: ?[]f32,
    collapse_masks: []u8,
    pulses: []const i32,
    short_blocks: bool,
    spread: pvq.Spread,
    dual_stereo_: bool,
    intensity: usize,
    tf_res: []const i32,
    total_bits: i32,
    balance_: i32,
    ec: *RangeDecoder,
    lm: u2,
    coded_bands: usize,
    seed: *u32,
    disable_inv: bool,
    arch: Arch,
) void {
    var dual_stereo = dual_stereo_;
    var balance = balance_;
    const channels: usize = if (y != null) 2 else 1;
    const m: usize = @as(usize, 1) << lm;
    const bb: u5 = if (short_blocks) @intCast(m) else 1;
    const norm_offset = m * @as(usize, @intCast(mode.ebands[start]));
    // The coded spectrum, for folding from. The last band is never
    // folded from.
    const norm_len = m * @as(usize, @intCast(mode.ebands[mode.nb_ebands - 1]))
        - norm_offset;
    var norm_buf: [2 * 8 * 78]f32 = undefined;
    const norm = norm_buf[0..norm_len];
    const norm2 = norm_buf[norm_len..2 * norm_len];
    // Borrow the last band's space for folding scratch.
    var lowband_scratch: ?[]f32 = x[
        m * @as(usize, @intCast(mode.ebands[mode.eff_ebands - 1]))..
    ];

    var lowband_offset: usize = 0;
    var update_lowband = true;
    var ctx: Context = .{
        .ec = ec,
        .arch = arch,
        .intensity = intensity,
        .spread = spread,
        .seed = seed.*,
        .disable_inv = disable_inv,
        // No noise in the first band of a transient.
        .avoid_split_noise = bb > 1,
    };

    for (start..end) |i| {
        ctx.i = i;
        const last = i == end - 1;
        const lo = m * @as(usize, @intCast(mode.ebands[i]));
        const n = m * @as(usize, @intCast(mode.ebands[i + 1])) - lo;
        const xb = x[lo..];
        const yb: ?[]f32 = if (y) |yy| yy[lo..] else null;

        const tell: i32 = @intCast(ec.tellFrac());
        // The bits for this band.
        if (i != start) balance -= tell;
        const remaining_bits = total_bits - tell - 1;
        ctx.remaining_bits = remaining_bits;
        var b: i32 = 0;
        if (i + 1 <= coded_bands) {
            const curr_balance = @divTrunc(
                balance,
                @as(i32, @intCast(@min(3, coded_bands - i))),
            );
            b = @max(
                0,
                @min(16383, @min(remaining_bits + 1, pulses[i] + curr_balance)),
            );
        }

        if (
            (((lo >= n)
                and (lo - n >= m * @as(usize, @intCast(mode.ebands[start]))))
                or (i == start + 1))
                and ((update_lowband) or (lowband_offset == 0))
        )
            lowband_offset = i;
        if (i == start + 1)
            specialHybridFolding(norm, norm2, start, m, dual_stereo);

        ctx.tf_change = tf_res[i];
        if (last) lowband_scratch = null;

        // A conservative estimate of the collapse masks of the bands
        // folded from.
        var x_cm: u32 = undefined;
        var y_cm: u32 = undefined;
        var effective_lowband: ?usize = null;
        if (
            lowband_offset != 0
                and (spread != .aggressive or bb > 1 or ctx.tf_change < 0)
        ) {
            // Never repeating content within one band.
            const lb_start = m
                * @as(usize, @intCast(mode.ebands[lowband_offset]));
            const eff = if (lb_start >= norm_offset + n)
                lb_start - norm_offset - n
            else
                0;
            effective_lowband = eff;
            var fold_start = lowband_offset;
            while (true) {
                fold_start -= 1;
                if (
                    m * @as(usize, @intCast(mode.ebands[fold_start])) <= eff
                            + norm_offset
                )
                    break;
            }
            var fold_end = lowband_offset - 1;
            while (true) {
                fold_end += 1;
                if (
                    !(fold_end < i
                        and m * @as(usize, @intCast(mode.ebands[fold_end]))
                            < eff + norm_offset + n)
                )
                    break;
            }
            x_cm = 0;
            y_cm = 0;
            var fold_i = fold_start;
            while (true) {
                x_cm |= collapse_masks[fold_i * channels + 0];
                y_cm |= collapse_masks[fold_i * channels + channels - 1];
                fold_i += 1;
                if (fold_i >= fold_end) break;
            }
        } else {
            // Folding from noise, which fills every block.
            x_cm = (@as(u32, 1) << bb) - 1;
            y_cm = x_cm;
        }

        if (dual_stereo and i == intensity) {
            // Dual stereo gives way to intensity.
            dual_stereo = false;
            for (
                norm[0..lo - norm_offset],
                norm2[0..lo - norm_offset],
            ) |*a, b2| {
                a.* = 0.5 * (a.* + b2);
            }
        }
        const out: ?[]f32 = if (last) null else norm[lo - norm_offset..];
        const lowband: ?[]f32 = if (effective_lowband) |e| norm[e..] else null;
        if (dual_stereo) {
            x_cm = decodeBand(
                &ctx,
                xb,
                n,
                @divTrunc(b, 2),
                bb,
                lowband,
                lm,
                out,
                1.0,
                lowband_scratch,
                x_cm,
            );
            const lowband2: ?[]f32 = if (effective_lowband) |e|
                norm2[e..]
            else
                null;
            const out2: ?[]f32 = if (last) null else norm2[lo - norm_offset..];
            y_cm = decodeBand(
                &ctx,
                yb.?,
                n,
                @divTrunc(b, 2),
                bb,
                lowband2,
                lm,
                out2,
                1.0,
                lowband_scratch,
                y_cm,
            );
        } else {
            if (yb) |yy| {
                {
                    x_cm = decodeBandStereo(
                        &ctx,
                        xb,
                        yy,
                        n,
                        b,
                        bb,
                        lowband,
                        lm,
                        out,
                        lowband_scratch,
                        x_cm | y_cm,
                    );
                }
            } else {
                x_cm = decodeBand(
                    &ctx,
                    xb,
                    n,
                    b,
                    bb,
                    lowband,
                    lm,
                    out,
                    1.0,
                    lowband_scratch,
                    x_cm | y_cm,
                );
            }
            y_cm = x_cm;
        }
        collapse_masks[i * channels + 0] = @truncate(x_cm);
        collapse_masks[i * channels + channels - 1] = @truncate(y_cm);
        balance += pulses[i] + tell;
        // The folding position moves on only while there is a bit a
        // sample.
        update_lowband = b > @as(i32, @intCast(n << bitres));
        // Only the first band needs protecting from split noise.
        // After it, there is folding.
        ctx.avoid_split_noise = false;
    }
    seed.* = ctx.seed;
}

/// For a transient, noise in the short blocks of a band that got no pulses
/// in any of them, so that its energy does not collapse. Section 4.3.5.
pub fn antiCollapse(
    x: []f32,
    collapse_masks: []const u8,
    lm: u2,
    channels: usize,
    size: usize,
    start: usize,
    end: usize,
    log_e: []const f32,
    prev1_log_e: []const f32,
    prev2_log_e: []const f32,
    pulses: []const i32,
    seed_: u32,
    arch: Arch,
) void {
    var seed = seed_;
    for (start..end) |i| {
        const n0: usize = @intCast(mode.ebands[i + 1] - mode.ebands[i]);
        // Depth in eighths of a bit.
        const depth: i32 = @intCast(
            (@as(u32, @intCast(1 + pulses[i])) / @as(u32, @intCast(n0))) >> lm,
        );
        const thresh: f32 = 0.5 * exp2(-0.125 * @as(f32, @floatFromInt(depth)));
        const sqrt_1: f32 = 1.0
            / @as(f32, @floatCast(@sqrt(@as(f64, @floatFromInt(n0 << lm)))));

        for (0..channels) |c| {
            var prev1 = prev1_log_e[c * mode.nb_ebands + i];
            var prev2 = prev2_log_e[c * mode.nb_ebands + i];
            if (channels == 1) {
                prev1 = @max(prev1, prev1_log_e[mode.nb_ebands + i]);
                prev2 = @max(prev2, prev2_log_e[mode.nb_ebands + i]);
            }
            var ediff = log_e[c * mode.nb_ebands + i] - @min(prev1, prev2);
            ediff = @max(0, ediff);
            // Twice, or 2 * sqrt(2) at 20 ms: short blocks do not have the
            // energy of long ones.
            var r: f32 = 2.0 * exp2(-ediff);
            if (lm == 3) r *= 1.41421356;
            r = @min(thresh, r);
            r = r * sqrt_1;

            const band = x[
                c * size + (@as(usize, @intCast(mode.ebands[i])) << lm)..
            ];
            var renormalize = false;
            for (0..@as(usize, 1) << lm) |k| {
                if (
                    collapse_masks[i * channels + c]
                        & (@as(u32, 1) << @intCast(k)) == 0
                ) {
                    // Collapsed: fill with noise.
                    for (0..n0) |j| {
                        seed = lcgRand(seed);
                        band[(j << lm) + k] = if (seed & 0x8000 != 0) r else -r;
                    }
                    renormalize = true;
                }
            }
            if (renormalize)
                pvq.renormaliseVector(arch, band[0..n0 << lm], 1.0);
        }
    }
}

/// `2^x`, as libopus's float `celt_exp2`: through double.
pub fn exp2(x: f32) f32 {
    return @floatCast(@exp(0.6931471805599453094 * @as(f64, x)));
}

/// The bands' shapes scaled by their energies: the spectrum the inverse MDCT
/// takes. Bins outside `[start, end)` are zero. `denormalise_bands`.
pub fn denormalise(
    x: []const f32,
    freq: []f32,
    band_log_e: []const f32,
    start_: usize,
    end_: usize,
    m: usize,
    silence: bool,
) void {
    var start = start_;
    var end = end_;
    const n = m * mode.short_mdct_size;
    var bound = m * @as(usize, @intCast(mode.ebands[end]));
    if (silence) {
        bound = 0;
        start = 0;
        end = 0;
    }
    const first = m * @as(usize, @intCast(mode.ebands[start]));
    @memset(freq[0..first], 0);
    for (start..end) |i| {
        const lo = m * @as(usize, @intCast(mode.ebands[i]));
        const hi = m * @as(usize, @intCast(mode.ebands[i + 1]));
        const lg = band_log_e[i] + energy.e_means[i];
        const g = exp2(@min(32.0, lg));
        for (freq[lo..hi], x[lo..hi]) |*f, xj| f.* = xj * g;
    }
    @memset(freq[bound..n], 0);
}
