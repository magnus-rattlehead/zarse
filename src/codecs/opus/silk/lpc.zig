// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2006-2011 Skype Limited
// SPDX-FileCopyrightText: © 2013 Koen Vos
// SPDX-License-Identifier: MIT AND BSD-3-Clause

//! SILK's linear prediction filters: from the normalized line spectral
//! frequencies the stream carries to the filter's coefficients, kept
//! stable, and bandwidth expansion. libopus's `NLSF2A.c`,
//! `NLSF_stabilize.c`, `LPC_fit.c`, `LPC_inv_pred_gain.c`,
//! `bwexpander.c`, `bwexpander_32.c`, `LPC_analysis_filter.c` and `sort.c`.

const std = @import("std");
const fix = @import("fix.zig");
const tables = @import("tables.zig");

/// The most coefficients a SILK filter has.
pub const max_order = 16;

/// Chirps the filter `ar`: the k-th coefficient scaled by `chirp^(k+1)`,
/// which widens its resonances. `silk_bwexpander`.
pub fn bwexpander(ar: []i16, chirp_q16_: i32) void {
    var chirp_q16 = chirp_q16_;
    const chirp_minus_one_q16 = chirp_q16 - 65536;
    // Not smulwb: its bias can make the filter unstable.
    for (ar[0..ar.len - 1]) |*a| {
        a.* = @truncate(fix.rshiftRound(fix.mul(chirp_q16, a.*), 16));
        chirp_q16 +%= fix.rshiftRound(
            fix.mul(chirp_q16, chirp_minus_one_q16),
            16,
        );
    }
    ar[ar.len - 1] = @truncate(
        fix.rshiftRound(fix.mul(chirp_q16, ar[ar.len - 1]), 16),
    );
}

/// `bwexpander` for 32-bit coefficients. `silk_bwexpander_32`.
pub fn bwexpander32(ar: []i32, chirp_q16_: i32) void {
    var chirp_q16 = chirp_q16_;
    const chirp_minus_one_q16 = chirp_q16 - 65536;
    for (ar[0..ar.len - 1]) |*a| {
        a.* = fix.smulww(chirp_q16, a.*);
        chirp_q16 +%= fix.rshiftRound(
            fix.mul(chirp_q16, chirp_minus_one_q16),
            16,
        );
    }
    ar[ar.len - 1] = fix.smulww(chirp_q16, ar[ar.len - 1]);
}

const qa = 24;
const a_limit: i32 = @intFromFloat(@as(f64, 0.99975) * (1 << qa) + 0.5);
/// `1 / MAX_PREDICTION_POWER_GAIN` in Q30.
const min_inv_gain_q30: i32 = @intFromFloat(
    @as(f32, 1.0 / 1e4) * @as(f64, 1 << 30) + 0.5,
);

inline fn mul32FracQ(a: i32, b: i32, q: comptime_int) i32 {
    return @truncate(fix.rshiftRound64(fix.smull(a, b), q));
}

/// The inverse of the filter's prediction gain, in Q30, or 0 if it is
/// unstable or too close to it. `silk_LPC_inverse_pred_gain_c`.
pub fn inversePredGain(a_q12: []const i16) i32 {
    const order = a_q12.len;
    var a_qa: [max_order]i32 = undefined;
    var dc_resp: i32 = 0;
    for (a_q12, 0..) |a, k| {
        dc_resp += a;
        a_qa[k] = fix.lshift(@as(i32, a), qa - 12);
    }
    // An unstable DC response needs no more work.
    if (dc_resp >= 4096) return 0;
    return inversePredGainQa(a_qa[0..order]);
}

fn inversePredGainQa(a_qa: []i32) i32 {
    var inv_gain_q30: i32 = 1 << 30;
    var k = a_qa.len - 1;
    while (k > 0) : (k -= 1) {
        if (a_qa[k] > a_limit or a_qa[k] < -a_limit) return 0;
        const rc_q31 = 0 -% fix.lshift(a_qa[k], 31 - qa);
        const rc_mult1_q30 = (@as(i32, 1) << 30) -% fix.smmul(rc_q31, rc_q31);
        inv_gain_q30 = fix.lshift(fix.smmul(inv_gain_q30, rc_mult1_q30), 2);
        if (inv_gain_q30 < min_inv_gain_q30) return 0;
        const mult2q = 32 - fix.clz32(fix.abs(rc_mult1_q30));
        const rc_mult2 = fix.inverse32VarQ(rc_mult1_q30, mult2q + 30);
        for (0..(k + 1) >> 1) |n| {
            const tmp1 = a_qa[n];
            const tmp2 = a_qa[k - n - 1];
            var t64 = fix.rshiftRound64(
                fix.smull(
                    fix.subSat32(tmp1, mul32FracQ(tmp2, rc_q31, 31)),
                    rc_mult2,
                ),
                mult2q,
            );
            if (t64 > fix.i32_max or t64 < fix.i32_min) return 0;
            a_qa[n] = @intCast(t64);
            t64 = fix.rshiftRound64(
                fix.smull(
                    fix.subSat32(tmp2, mul32FracQ(tmp1, rc_q31, 31)),
                    rc_mult2,
                ),
                mult2q,
            );
            if (t64 > fix.i32_max or t64 < fix.i32_min) return 0;
            a_qa[k - n - 1] = @intCast(t64);
        }
    }
    if (a_qa[0] > a_limit or a_qa[0] < -a_limit) return 0;
    const rc_q31 = 0 -% fix.lshift(a_qa[0], 31 - qa);
    const rc_mult1_q30 = (@as(i32, 1) << 30) -% fix.smmul(rc_q31, rc_q31);
    inv_gain_q30 = fix.lshift(fix.smmul(inv_gain_q30, rc_mult1_q30), 2);
    if (inv_gain_q30 < min_inv_gain_q30) return 0;
    return inv_gain_q30;
}

/// 32-bit coefficients in Q`q_in` to 16-bit ones in Q`q_out`, chirped until
/// they fit. `silk_LPC_fit`.
pub fn fit(
    a_qout: []i16,
    a_qin: []i32,
    q_out: comptime_int,
    q_in: comptime_int,
) void {
    const d = a_qout.len;
    var i: usize = 0;
    while (i < 10) : (i += 1) {
        var maxabs: i32 = 0;
        var idx: usize = 0;
        for (a_qin[0..d], 0..) |a, k| {
            const absval = fix.abs(a);
            if (absval > maxabs) {
                maxabs = absval;
                idx = k;
            }
        }
        maxabs = fix.rshiftRound(maxabs, q_in - q_out);
        if (maxabs > fix.i16_max) {
            maxabs = @min(maxabs, 163838);
            const chirp_q16 = @as(
                i32,
                @intFromFloat(@as(f64, 0.999) * 65536.0 + 0.5),
            )
                - @divTrunc(
                    fix.lshift(maxabs - fix.i16_max, 14),
                    (maxabs *% @as(i32, @intCast(idx + 1))) >> 2,
                );
            bwexpander32(a_qin[0..d], chirp_q16);
        } else {
            break;
        }
    }
    if (i == 10) {
        // Still too large: clipped.
        for (0..d) |k| {
            a_qout[k] = @intCast(
                fix.sat16(fix.rshiftRound(a_qin[k], q_in - q_out)),
            );
            a_qin[k] = fix.lshift(@as(i32, a_qout[k]), q_in - q_out);
        }
    } else {
        for (0..d) |k|
            a_qout[k] = @truncate(fix.rshiftRound(a_qin[k], q_in - q_out));
    }
}

const nlsf2a_qa = 16;

fn findPoly(out: []i32, c_lsf: []const i32, dd: usize) void {
    out[0] = 1 << nlsf2a_qa;
    out[1] = -c_lsf[0];
    for (1..dd) |k| {
        const ftmp = c_lsf[2 * k];
        out[k + 1] = fix.lshift(out[k - 1], 1)
            -% @as(
                i32,
                @truncate(
                    fix.rshiftRound64(fix.smull(ftmp, out[k]), nlsf2a_qa),
                ),
            );
        var n = k;
        while (n > 1) : (n -= 1) {
            out[n] +%= out[n - 2]
                -% @as(
                    i32,
                    @truncate(
                        fix.rshiftRound64(
                            fix.smull(ftmp, out[n - 1]),
                            nlsf2a_qa,
                        ),
                    ),
                );
        }
        out[1] -%= ftmp;
    }
}

/// The whitening filter for the normalized line spectral frequencies
/// `nlsf`, in Q15, into `a_q12`: 10 or 16 of them. `silk_NLSF2A`.
pub fn nlsf2a(a_q12: []i16, nlsf: []const i16) void {
    const d = nlsf.len;
    std.debug.assert(d == 10 or d == 16);
    // An order that keeps the polynomials accurate.
    const ordering16 = [16]u8{
        0,
        15,
        8,
        7,
        4,
        11,
        12,
        3,
        2,
        13,
        10,
        5,
        6,
        9,
        14,
        1,
    };
    const ordering10 = [10]u8{ 0, 9, 6, 3, 4, 5, 8, 1, 2, 7 };
    const ordering: []const u8 = if (d == 16) &ordering16 else &ordering10;

    // 2 cos(LSF), from the table by linear interpolation.
    var cos_lsf_qa: [max_order]i32 = undefined;
    for (0..d) |k| {
        const f_int: i32 = nlsf[k] >> (15 - 7);
        const f_frac: i32 = nlsf[k] - fix.lshift(f_int, 15 - 7);
        const cos_val: i32 = tables.lsfcostab_fix_q12[@intCast(f_int)];
        const delta: i32 = tables.lsfcostab_fix_q12[@intCast(f_int + 1)]
            - cos_val;
        cos_lsf_qa[ordering[k]] = fix.rshiftRound(
            fix.lshift(cos_val, 8) +% fix.mul(delta, f_frac),
            20 - nlsf2a_qa,
        );
    }

    const dd = d >> 1;
    var p: [max_order / 2 + 1]i32 = undefined;
    var q: [max_order / 2 + 1]i32 = undefined;
    // The even and odd polynomials, whose roots are interleaved.
    findPoly(&p, cos_lsf_qa[0..d], dd);
    findPoly(&q, cos_lsf_qa[1..d], dd);

    var a32_qa1: [max_order]i32 = undefined;
    for (0..dd) |k| {
        const ptmp = p[k + 1] +% p[k];
        const qtmp = q[k + 1] -% q[k];
        a32_qa1[k] = (0 -% qtmp) -% ptmp;
        a32_qa1[d - k - 1] = qtmp -% ptmp;
    }

    fit(a_q12[0..d], a32_qa1[0..d], 12, nlsf2a_qa + 1);
    var i: u5 = 0;
    while (inversePredGain(a_q12[0..d]) == 0 and i < 16) : (i += 1) {
        // Too close to unstable: expanded further, from the unscaled
        // coefficients, and measured again.
        bwexpander32(a32_qa1[0..d], 65536 - fix.lshift(2, i));
        for (0..d) |k|
            a_q12[k] = @truncate(
                fix.rshiftRound(a32_qa1[k], nlsf2a_qa + 1 - 12),
            );
    }
}

/// Moves the frequencies apart, and away from the ends, until each is at
/// least `delta_min` from the one before. `silk_NLSF_stabilize`.
pub fn nlsfStabilize(nlsf: []i16, delta_min: []const i16) void {
    const l = nlsf.len;
    var loops: usize = 0;
    while (loops < 20) : (loops += 1) {
        // The smallest distance.
        var min_diff: i32 = @as(i32, nlsf[0]) - delta_min[0];
        var big_i: usize = 0;
        for (1..l) |i| {
            const diff: i32 = @as(i32, nlsf[i])
                - (@as(i32, nlsf[i - 1]) + delta_min[i]);
            if (diff < min_diff) {
                min_diff = diff;
                big_i = i;
            }
        }
        const last: i32 = (1 << 15) - (@as(i32, nlsf[l - 1]) + delta_min[l]);
        if (last < min_diff) {
            min_diff = last;
            big_i = l;
        }
        if (min_diff >= 0) return;

        if (big_i == 0) {
            nlsf[0] = delta_min[0];
        } else if (big_i == l) {
            nlsf[l - 1] = @truncate((1 << 15) - @as(i32, delta_min[l]));
        } else {
            // Apart around the same center, within where the others allow.
            var min_center: i32 = 0;
            for (0..big_i) |k| min_center += delta_min[k];
            min_center += delta_min[big_i] >> 1;
            var max_center: i32 = 1 << 15;
            var k = l;
            while (k > big_i) : (k -= 1) max_center -= delta_min[k];
            max_center -= delta_min[big_i] >> 1;
            const center: i16 = @truncate(
                fix.limit(
                    fix.rshiftRound(@as(i32, nlsf[big_i - 1]) + nlsf[big_i], 1),
                    min_center,
                    max_center,
                ),
            );
            nlsf[big_i - 1] = @truncate(
                @as(i32, center) - (delta_min[big_i] >> 1),
            );
            nlsf[big_i] = @truncate(
                @as(i32, nlsf[big_i - 1]) + delta_min[big_i],
            );
        }
    }

    // Not settled: the simple way, which is further from the input.
    insertionSort(nlsf);
    nlsf[0] = @max(nlsf[0], delta_min[0]);
    for (1..l) |i| {
        nlsf[i] = @intCast(
            @max(
                @as(i32, nlsf[i]),
                fix.sat16(@as(i32, nlsf[i - 1]) + delta_min[i]),
            ),
        );
    }
    nlsf[l - 1] = @intCast(
        @min(@as(i32, nlsf[l - 1]), (1 << 15) - @as(i32, delta_min[l])),
    );
    var i = l - 1;
    while (i > 0) {
        i -= 1;
        nlsf[i] = @truncate(
            @min(@as(i32, nlsf[i]), @as(i32, nlsf[i + 1]) - delta_min[i + 1]),
        );
    }
}

/// `silk_insertion_sort_increasing_all_values_int16`.
fn insertionSort(a: []i16) void {
    for (1..a.len) |i| {
        const value = a[i];
        var j: usize = i;
        while (j > 0 and value < a[j - 1]) : (j -= 1) a[j] = a[j - 1];
        a[j] = value;
    }
}

/// The prediction error of `in` through the filter `b`, in Q12. The first
/// `b.len` outputs are zero. `silk_LPC_analysis_filter`.
pub fn analysisFilter(out: []i16, in: []const i16, b: []const i16) void {
    const d = b.len;
    for (d..in.len) |ix| {
        var sum: i32 = fix.smulbb(in[ix - 1], b[0]);
        // Wrapping, so that two wraps can cancel.
        for (1..d) |j| sum = fix.smlabbOvflw(sum, in[ix - 1 - j], b[j]);
        sum = fix.lshift(@as(i32, in[ix]), 12) -% sum;
        out[ix] = @intCast(fix.sat16(fix.rshiftRound(sum, 12)));
    }
    @memset(out[0..d], 0);
}
