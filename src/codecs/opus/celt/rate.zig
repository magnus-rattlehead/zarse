// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2007-2008 CSIRO
// SPDX-FileCopyrightText: © 2007-2009 Xiph.Org Foundation
// SPDX-License-Identifier: MIT AND BSD-2-Clause

//! Bit allocation: how the bits left in a frame after its energies are
//! shared out among the bands, for their fine energy and for their shape.
//! RFC 6716, Section 4.3.3, following libopus's `rate.c` and `rate.h`.
//!
//! None of this is transmitted except the few decisions the encoder is free
//! to make -- which bands to skip, the intensity stereo band, dual stereo --
//! so the decoder has to arrive at exactly the encoder's numbers from the
//! same inputs. Everything is integer arithmetic in eighths of a bit.

const std = @import("std");
const RangeDecoder = @import("../RangeDecoder.zig");
const mode = @import("mode.zig");
const energy = @import("energy.zig");

const bitres = RangeDecoder.bitres;

/// The fine energy offset: fine bits are given at log2(N)/2 plus this, in
/// eighths of a bit, below a band's fair share.
const fine_offset = 21;
/// Steps of the bisection between two allocation vectors.
const alloc_steps = 6;
/// The largest number of pulses the pulse cache goes to, as a log.
const log_max_pseudo = 6;

/// The cost of intensity stereo's band index, in eighths of a bit, for each
/// number of bands it could be.
const log2_frac_table = [24]u8{
    0,
    8,
    13,
    16,
    19,
    21,
    23,
    24,
    26,
    27,
    28,
    29,
    30,
    31,
    32,
    32,
    33,
    34,
    34,
    35,
    36,
    36,
    37,
    37,
};

/// How many pulses the `i`th pseudo-pulse count stands for: exact up to 8,
/// then growing exponentially.
pub fn getPulses(i: i32) i32 {
    return if (i < 8) i else (8 + (i & 7)) << @intCast((i >> 3) - 1);
}

/// The pulse cache's row for a band at an LM, which is -1 for half of a
/// 2.5 ms band split in two.
pub fn cacheRow(band: usize, lm: i32) []const u8 {
    const row: usize = @intCast(lm + 1);
    const index: usize = @intCast(
        mode.cache_index[row * mode.nb_ebands + band],
    );
    return mode.cache_bits[index..];
}

/// The pseudo-pulse count whose cost comes nearest `bits`.
pub fn bits2pulses(band: usize, lm: i32, bits_: i32) i32 {
    const cache = cacheRow(band, lm);
    var lo: i32 = 0;
    var hi: i32 = cache[0];
    const bits = bits_ - 1;
    for (0..log_max_pseudo) |_| {
        const mid = (lo + hi + 1) >> 1;
        if (@as(i32, cache[@intCast(mid)]) >= bits) hi = mid else lo = mid;
    }
    const below: i32 = if (lo == 0) -1 else cache[@intCast(lo)];
    return if (bits - below <= @as(i32, cache[@intCast(hi)]) - bits) lo else hi;
}

/// What a pseudo-pulse count costs, in eighths of a bit.
pub fn pulses2bits(band: usize, lm: i32, pulses: i32) i32 {
    const cache = cacheRow(band, lm);
    return if (pulses == 0) 0 else @as(i32, cache[@intCast(pulses)]) + 1;
}

/// The most bits each band can use, for `channels` channels: `celt.c`'s
/// `init_caps`.
pub fn initCaps(cap: []i32, lm: usize, channels: usize) void {
    for (0..mode.nb_ebands) |i| {
        const n: i32 = @as(i32, mode.ebands[i + 1] - mode.ebands[i])
            << @intCast(lm);
        const c: i32 = @intCast(channels);
        cap[i] = (@as(
            i32,
            mode.cache_caps[mode.nb_ebands * (2 * lm + channels - 1) + i],
        )
            + 64)
            * c
            * n
            >> 2;
    }
}

pub const Allocation = struct {
    /// Bands coded. The rest are skipped.
    coded_bands: usize,
    /// The first band coded as intensity stereo, or 0.
    intensity: usize,
    dual_stereo: bool,
    /// Bits over the caps, carried into the bands' rebalancing.
    balance: i32,
};

/// Unsigned division, as libopus's `celt_udiv`: its operands are cast to
/// unsigned before dividing.
fn udiv(n: i32, d: i32) i32 {
    return @bitCast(@as(u32, @bitCast(n)) / @as(u32, @bitCast(d)));
}

/// The bit allocation of Section 4.3.3, for bands `[start, end)`:
/// `pulses` gets each band's bits for its shape, `ebits` its fine energy
/// bits, and `fine_priority` whether it is first in line for the final
/// pass. `offsets` are the dynamic allocation's boosts, `cap` from
/// `initCaps`, `total` the bits available in eighths of a bit. The
/// skipping, intensity and dual stereo decisions are read from `dec`.
pub fn computeAllocation(
    start: usize,
    end: usize,
    offsets: []const i32,
    cap: []const i32,
    alloc_trim: i32,
    total_: i32,
    pulses: []i32,
    ebits: []i32,
    fine_priority: []i32,
    channels: usize,
    lm: usize,
    dec: *RangeDecoder,
) Allocation {
    const c: i32 = @intCast(channels);
    const lm_i: i32 = @intCast(lm);
    var total: i32 = @max(total_, 0);
    const len = mode.nb_ebands;
    var skip_start = start;
    // A bit to signal the end of skipped bands.
    const skip_rsv: i32 = if (total >= 1 << bitres) 1 << bitres else 0;
    total -= skip_rsv;
    // Bits for the intensity and dual stereo parameters.
    var intensity_rsv: i32 = 0;
    var dual_stereo_rsv: i32 = 0;
    if (channels == 2) {
        intensity_rsv = log2_frac_table[end - start];
        if (intensity_rsv > total) {
            intensity_rsv = 0;
        } else {
            total -= intensity_rsv;
            dual_stereo_rsv = if (total >= 1 << bitres) 1 << bitres else 0;
            total -= dual_stereo_rsv;
        }
    }

    var bits1: [len]i32 = undefined;
    var bits2: [len]i32 = undefined;
    var thresh: [len]i32 = undefined;
    var trim_offset: [len]i32 = undefined;

    for (start..end) |j| {
        const n: i32 = mode.ebands[j + 1] - mode.ebands[j];
        // Below this, a band is sure to get no bits for its shape.
        thresh[j] = @max(c << bitres, (3 * n << @intCast(lm) << bitres) >> 4);
        // The tilt of the allocation curve.
        const tilt = c
            * n
            * (alloc_trim - 5 - lm_i)
            * @as(i32, @intCast(end - j - 1));
        trim_offset[j] = tilt * (@as(i32, 1) << @intCast(lm + bitres)) >> 6;
        // Single-coefficient bands do better with one coarse value per
        // coefficient.
        if (n << @intCast(lm) == 1) trim_offset[j] -= c << bitres;
    }

    // Which two allocation vectors the total falls between.
    var lo: i32 = 1;
    var hi: i32 = mode.nb_alloc_vectors - 1;
    while (true) {
        var done = false;
        var psum: i32 = 0;
        const mid = (lo + hi) >> 1;
        var j = end;
        while (j > start) {
            j -= 1;
            const n: i32 = mode.ebands[j + 1] - mode.ebands[j];
            var bitsj: i32 = c
                * n
                * @as(
                    i32,
                    mode.alloc_vectors[@as(usize, @intCast(mid)) * len + j],
                )
                << @intCast(lm)
                >> 2;
            if (bitsj > 0) bitsj = @max(0, bitsj + trim_offset[j]);
            bitsj += offsets[j];
            if (bitsj >= thresh[j] or done) {
                done = true;
                psum += @min(bitsj, cap[j]);
            } else if (bitsj >= c << bitres) {
                psum += c << bitres;
            }
        }
        if (psum > total) hi = mid - 1 else lo = mid + 1;
        if (lo > hi) break;
    }
    hi = lo;
    lo -= 1;
    for (start..end) |j| {
        const n: i32 = mode.ebands[j + 1] - mode.ebands[j];
        var bits1j: i32 = c
            * n
            * @as(i32, mode.alloc_vectors[@as(usize, @intCast(lo)) * len + j])
            << @intCast(lm)
            >> 2;
        var bits2j: i32 = if (hi >= mode.nb_alloc_vectors)
            cap[j]
        else
            c
                * n
                * @as(
                    i32,
                    mode.alloc_vectors[@as(usize, @intCast(hi)) * len + j],
                )
                << @intCast(lm)
                >> 2;
        if (bits1j > 0) bits1j = @max(0, bits1j + trim_offset[j]);
        if (bits2j > 0) bits2j = @max(0, bits2j + trim_offset[j]);
        if (lo > 0) bits1j += offsets[j];
        bits2j += offsets[j];
        if (offsets[j] > 0) skip_start = j;
        bits2j = @max(0, bits2j - bits1j);
        bits1[j] = bits1j;
        bits2[j] = bits2j;
    }
    return interpBits2Pulses(
        start,
        end,
        skip_start,
        &bits1,
        &bits2,
        &thresh,
        cap,
        total,
        skip_rsv,
        intensity_rsv,
        dual_stereo_rsv,
        pulses,
        ebits,
        fine_priority,
        channels,
        lm,
        dec,
    );
}

fn interpBits2Pulses(
    start: usize,
    end: usize,
    skip_start: usize,
    bits1: []const i32,
    bits2: []const i32,
    thresh: []const i32,
    cap: []const i32,
    total_: i32,
    skip_rsv: i32,
    intensity_rsv_: i32,
    dual_stereo_rsv_: i32,
    bits: []i32,
    ebits: []i32,
    fine_priority: []i32,
    channels: usize,
    lm: usize,
    dec: *RangeDecoder,
) Allocation {
    const c: i32 = @intCast(channels);
    var total = total_;
    var intensity_rsv = intensity_rsv_;
    var dual_stereo_rsv = dual_stereo_rsv_;
    const alloc_floor: i32 = c << bitres;
    const stereo: u5 = @intFromBool(channels > 1);
    const log_m: i32 = @as(i32, @intCast(lm)) << bitres;

    // Bisect between the two allocation vectors.
    var lo: i32 = 0;
    var hi: i32 = 1 << alloc_steps;
    for (0..alloc_steps) |_| {
        const mid = (lo + hi) >> 1;
        var psum: i32 = 0;
        var done = false;
        var j = end;
        while (j > start) {
            j -= 1;
            const tmp = bits1[j] + (mid * bits2[j] >> alloc_steps);
            if (tmp >= thresh[j] or done) {
                done = true;
                psum += @min(tmp, cap[j]);
            } else if (tmp >= alloc_floor) {
                psum += alloc_floor;
            }
        }
        if (psum > total) hi = mid else lo = mid;
    }
    var psum: i32 = 0;
    {
        var done = false;
        var j = end;
        while (j > start) {
            j -= 1;
            var tmp = bits1[j] + (lo * bits2[j] >> alloc_steps);
            if (tmp < thresh[j] and !done) {
                tmp = if (tmp >= alloc_floor) alloc_floor else 0;
            } else done = true;
            tmp = @min(tmp, cap[j]);
            bits[j] = tmp;
            psum += tmp;
        }
    }

    // Which bands to skip, working back from the end.
    var coded_bands = end;
    while (true) : (coded_bands -= 1) {
        const j = coded_bands - 1;
        // Never the first band, nor one the dynamic allocation boosted.
        if (j <= skip_start) {
            // The bit reserved to end skipping goes back.
            total += skip_rsv;
            break;
        }
        // The left-over bits this band would get, including those taken back
        // from higher bands already skipped.
        const span: i32 = mode.ebands[coded_bands] - mode.ebands[start];
        var left = total - psum;
        const percoeff = udiv(left, span);
        left -= span * percoeff;
        const rem = @max(left - (mode.ebands[j] - mode.ebands[start]), 0);
        const band_width: i32 = mode.ebands[coded_bands] - mode.ebands[j];
        var band_bits = bits[j] + percoeff * band_width + rem;
        // A skip decision is coded only above the band's threshold, so that
        // there are always bits for the flag. Below it, the band is skipped.
        if (band_bits >= @max(thresh[j], alloc_floor + (1 << bitres))) {
            if (dec.decodeBitLogp(1)) break;
            // A bit spent saying this band is skipped.
            psum += 1 << bitres;
            band_bits -= 1 << bitres;
        }
        // Take back what this band was given.
        psum -= bits[j] + intensity_rsv;
        if (intensity_rsv > 0) intensity_rsv = log2_frac_table[j - start];
        psum += intensity_rsv;
        if (band_bits >= alloc_floor) {
            // Enough for a fine energy bit per channel.
            psum += alloc_floor;
            bits[j] = alloc_floor;
        } else {
            bits[j] = 0;
        }
    }

    // The intensity and dual stereo parameters.
    var intensity: usize = 0;
    if (intensity_rsv > 0) {
        intensity = start + dec.decodeUint(@intCast(coded_bands + 1 - start));
    }
    if (intensity <= start) {
        total += dual_stereo_rsv;
        dual_stereo_rsv = 0;
    }
    var dual_stereo = false;
    if (dual_stereo_rsv > 0) dual_stereo = dec.decodeBitLogp(1);

    // What is left, spread evenly.
    {
        const span: i32 = mode.ebands[coded_bands] - mode.ebands[start];
        var left = total - psum;
        const percoeff = udiv(left, span);
        left -= span * percoeff;
        for (start..coded_bands) |j|
            bits[j] += percoeff * (mode.ebands[j + 1] - mode.ebands[j]);
        for (start..coded_bands) |j| {
            const tmp = @min(left, mode.ebands[j + 1] - mode.ebands[j]);
            bits[j] += tmp;
            left -= tmp;
        }
    }

    var balance: i32 = 0;
    var j = start;
    while (j < coded_bands) : (j += 1) {
        const n0: i32 = mode.ebands[j + 1] - mode.ebands[j];
        const n = n0 << @intCast(lm);
        const bit = bits[j] + balance;
        var excess: i32 = undefined;
        if (n > 1) {
            excess = @max(bit - cap[j], 0);
            bits[j] = bit - excess;

            // The extra degree of freedom in stereo.
            const den: i32 = c * n
                + @as(
                    i32,
                    @intFromBool(
                        channels == 2
                            and n > 2
                            and !dual_stereo
                            and j < intensity,
                    ),
                );
            const nc_log_n = den * (mode.log_n[j] + log_m);

            // Fine bits at log2(N)/2 + FINE_OFFSET below the band's fair
            // share of the total.
            var offset = (nc_log_n >> 1) - den * fine_offset;
            // N=2 is the one point off the curve.
            if (n == 2) offset += den << bitres >> 2;
            // The second and third fine energy bits.
            if (bits[j] + offset < den * 2 << bitres) {
                offset += nc_log_n >> 2;
            } else if (bits[j] + offset < den * 3 << bitres) {
                offset += nc_log_n >> 3;
            }

            // Divide, rounding.
            ebits[j] = @max(0, bits[j] + offset + (den << (bitres - 1)));
            ebits[j] = udiv(ebits[j], den) >> bitres;
            // Not more than there are.
            if (c * ebits[j] > (bits[j] >> bitres))
                ebits[j] = bits[j] >> stereo >> bitres;
            // Beyond what PVQ can use.
            ebits[j] = @min(ebits[j], energy.max_fine_bits);
            // Rounded down or capped: first in line for the final pass.
            fine_priority[j] = @intFromBool(
                ebits[j] * (den << bitres) >= bits[j] + offset,
            );
            // The rest goes to the band's shape.
            bits[j] -= c * ebits[j] << bitres;
        } else {
            // A single coefficient: everything to fine energy but its sign.
            excess = @max(0, bit - (c << bitres));
            bits[j] = bit - excess;
            ebits[j] = 0;
            fine_priority[j] = 1;
        }

        // Fine energy cannot take part in the bands' rebalancing, so it is
        // rebalanced here.
        if (excess > 0) {
            const extra_fine = @min(
                excess >> (stereo + bitres),
                energy.max_fine_bits - ebits[j],
            );
            ebits[j] += extra_fine;
            const extra_bits = extra_fine * c << bitres;
            fine_priority[j] = @intFromBool(extra_bits >= excess - balance);
            excess -= extra_bits;
        }
        balance = excess;
    }

    // Skipped bands spend everything on fine energy.
    while (j < end) : (j += 1) {
        ebits[j] = bits[j] >> stereo >> bitres;
        bits[j] = 0;
        fine_priority[j] = @intFromBool(ebits[j] < 1);
    }

    return .{
        .coded_bands = coded_bands,
        .intensity = intensity,
        .dual_stereo = dual_stereo,
        .balance = balance,
    };
}
