// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2007-2008 CSIRO
// SPDX-FileCopyrightText: © 2007-2009 Xiph.Org Foundation
// SPDX-License-Identifier: MIT AND BSD-2-Clause

//! Finding the pitch of what was last decoded, which concealment repeats.
//! libopus's `pitch.c`, as its floating point build computes it.
//!
//! The signal is halved in rate, whitened, and correlated with itself: first
//! at a quarter of the rate over every lag, then at half the rate around
//! the two best, then refined between samples.

const std = @import("std");
const lpc = @import("lpc.zig");
const simd = @import("../simd.zig");
const Arch = @import("../arch.zig").Arch;

/// Both channels, summed, low-passed and halved in rate into `x_lp`, then
/// whitened by a fourth-order predictor with a zero added. `x_lp` is half of
/// `len`. `pitch_downsample`.
pub fn downsample(
    arch: Arch,
    x: []const []const f32,
    x_lp: []f32,
    len: usize,
) void {
    const half = len >> 1;
    for (x, 0..) |ch, c| {
        if (c == 0) {
            for (1..half) |i|
                x_lp[i] = 0.25 * ch[2 * i - 1]
                    + 0.25 * ch[2 * i + 1]
                    + 0.5 * ch[2 * i];
            x_lp[0] = 0.25 * ch[1] + 0.5 * ch[0];
        } else {
            for (1..half) |i|
                x_lp[i] += 0.25 * ch[2 * i - 1]
                    + 0.25 * ch[2 * i + 1]
                    + 0.5 * ch[2 * i];
            x_lp[0] += 0.25 * ch[1] + 0.5 * ch[0];
        }
    }

    var ac: [5]f32 = undefined;
    lpc.autocorrelation(arch, x_lp[0..half], &ac, &.{});
    // A noise floor at -40 dB.
    ac[0] *= 1.0001;
    // Lag windowing.
    for (1..5) |i| {
        const fi: f32 = @floatFromInt(i);
        ac[i] -= ac[i] * (0.008 * fi) * (0.008 * fi);
    }
    var coefs: [4]f32 = undefined;
    lpc.fromAutocorrelation(&coefs, &ac);
    var tmp: f32 = 1.0;
    for (&coefs) |*k| {
        tmp = 0.9 * tmp;
        k.* = k.* * tmp;
    }
    // Add a zero.
    const c1: f32 = 0.8;
    const lpc2 = [5]f32{
        coefs[0] + 0.8,
        coefs[1] + c1 * coefs[0],
        coefs[2] + c1 * coefs[1],
        coefs[3] + c1 * coefs[2],
        c1 * coefs[3],
    };
    fir5(x_lp[0..half], &lpc2);
}

/// A fifth-order FIR filter, in place. `celt_fir5`.
fn fir5(x: []f32, num: *const [5]f32) void {
    var mem = [5]f32{ 0, 0, 0, 0, 0 };
    for (x) |*v| {
        var sum = v.*;
        inline for (0..5) |k| sum = sum + num[k] * mem[k];
        mem[4] = mem[3];
        mem[3] = mem[2];
        mem[2] = mem[1];
        mem[1] = mem[0];
        mem[0] = v.*;
        v.* = sum;
    }
}

/// The two lags whose correlations, normalized by `y`'s energy there, are
/// highest. `find_best_pitch`.
fn findBest(xcorr: []const f32, y: []const f32, len: usize) [2]usize {
    var syy: f32 = 1;
    var best_num = [2]f32{ -1, -1 };
    var best_den = [2]f32{ 0, 0 };
    var best = [2]usize{ 0, 1 };
    for (y[0..len]) |v| syy = syy + v * v;
    for (xcorr, 0..) |xc, i| {
        if (xc > 0) {
            // Scaled to avoid both underflow and overflow in the square.
            const xcorr16 = xc * 1e-12;
            const num = xcorr16 * xcorr16;
            if (num * best_den[1] > best_num[1] * syy) {
                if (num * best_den[0] > best_num[0] * syy) {
                    best_num[1] = best_num[0];
                    best_den[1] = best_den[0];
                    best[1] = best[0];
                    best_num[0] = num;
                    best_den[0] = syy;
                    best[0] = i;
                } else {
                    best_num[1] = num;
                    best_den[1] = syy;
                    best[1] = i;
                }
            }
        }
        syy += y[i + len] * y[i + len] - y[i] * y[i];
        syy = if (1 > syy) 1 else syy;
    }
    return best;
}

/// The lag, in samples at the half rate `x_lp` is at, below `max_pitch`,
/// at which `y` best matches `x_lp`'s first `len`. `pitch_search`.
pub fn search(
    arch: Arch,
    x_lp: []const f32,
    y: []const f32,
    len: usize,
    max_pitch: usize,
) usize {
    const lag = len + max_pitch;
    var x_lp4_buf: [512]f32 = undefined;
    var y_lp4_buf: [1024]f32 = undefined;
    var xcorr_buf: [512]f32 = undefined;
    const x_lp4 = x_lp4_buf[0..len >> 2];
    const y_lp4 = y_lp4_buf[0..lag >> 2];
    const xcorr = xcorr_buf[0..max_pitch >> 1];

    // Halved in rate again.
    for (x_lp4, 0..) |*v, j| v.* = x_lp[2 * j];
    for (y_lp4, 0..) |*v, j| v.* = y[2 * j];

    // The coarse search, at a quarter of the rate.
    simd.pitchXcorr(arch, x_lp4, y_lp4, xcorr[0..max_pitch >> 2], len >> 2);
    var best = findBest(xcorr[0..max_pitch >> 2], y_lp4, len >> 2);

    // The finer one, at half, around the two best found.
    for (xcorr, 0..) |*xc, i| {
        xc.* = 0;
        const ii: isize = @intCast(i);
        if (
            @abs(ii - 2 * @as(isize, @intCast(best[0]))) > 2
                and @abs(ii - 2 * @as(isize, @intCast(best[1]))) > 2
        )
            continue;
        const sum = simd.innerProd(arch, x_lp[0..len >> 1], y[i..]);
        xc.* = if (-1 > sum) -1 else sum;
    }
    best = findBest(xcorr, y, len >> 1);

    // Refined by pseudo-interpolation.
    var offset: isize = 0;
    if (best[0] > 0 and best[0] < (max_pitch >> 1) - 1) {
        const a = xcorr[best[0] - 1];
        const b = xcorr[best[0]];
        const c = xcorr[best[0] + 1];
        if ((c - a) > 0.7 * (b - a)) {
            offset = 1;
        } else if ((a - c) > 0.7 * (b - c)) {
            offset = -1;
        }
    }
    return @intCast(2 * @as(isize, @intCast(best[0])) - offset);
}

