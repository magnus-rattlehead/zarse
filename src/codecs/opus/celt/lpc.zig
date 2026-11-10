// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2009-2010 Xiph.Org Foundation
// SPDX-License-Identifier: MIT AND BSD-2-Clause

//! Linear prediction, for concealment: the autocorrelation of a signal, the
//! predictor Levinson-Durbin makes from it, and the two filters that take a
//! signal to its prediction error and back. libopus's `celt_lpc.c`, as its
//! floating point build computes it.
//!
//! Each sum is added up in the order libopus adds it, since floating point
//! addition is not associative and concealment has to match the reference
//! sample for sample.

const std = @import("std");
const simd = @import("../simd.zig");
const Arch = @import("../arch.zig").Arch;

/// The predictor of order `lpc.len` for the autocorrelation `ac`, which has
/// one more value. `_celt_lpc`.
pub fn fromAutocorrelation(lpc: []f32, ac: []const f32) void {
    const p = lpc.len;
    @memset(lpc, 0);
    var err = ac[0];
    if (!(ac[0] > 1e-10)) return;
    for (0..p) |i| {
        // This iteration's reflection coefficient.
        var rr: f32 = 0;
        for (0..i) |j| rr += lpc[j] * ac[i - j];
        rr += ac[i + 1];
        const r = -(rr / err);
        lpc[i] = r;
        for (0..(i + 1) >> 1) |j| {
            const tmp1 = lpc[j];
            const tmp2 = lpc[i - 1 - j];
            lpc[j] = tmp1 + r * tmp2;
            lpc[i - 1 - j] = tmp2 + r * tmp1;
        }
        err = err - (r * r) * err;
        // Enough, at 30 dB of prediction gain.
        if (err <= 0.001 * ac[0]) break;
    }
}

/// The prediction error: `y[i] = x[i] + sum(num[k] * x[i - k - 1])`, where
/// `x` has `num.len` samples of history before `start`. `celt_fir_c`: four
/// outputs at a time through `xcorr_kernel`, and the rest one at a time.
pub fn fir(
    arch: Arch,
    x: []const f32,
    start: usize,
    num: []const f32,
    y: []f32,
) void {
    const ord = num.len;
    const n = y.len;
    var rnum: [24]f32 = undefined;
    for (0..ord) |i| rnum[i] = num[ord - i - 1];
    var i: usize = 0;
    while (i + 3 < n) : (i += 4) {
        var sum = [4]f32{
            x[start + i],
            x[start + i + 1],
            x[start + i + 2],
            x[start + i + 3],
        };
        simd.xcorrKernel(arch, rnum[0..ord], x[start + i - ord..], &sum, ord);
        y[i..][0..4].* = sum;
    }
    while (i < n) : (i += 1) {
        var sum = x[start + i];
        for (0..ord) |j| sum += rnum[j] * x[start + i + j - ord];
        y[i] = sum;
    }
}

/// The synthesis filter, in place: `x[i] -= sum(den[k] * y[i - k - 1])`,
/// with the outputs before the first in `mem`, most recent first, which is
/// left holding the last of this call's. `celt_iir`, unrolled by four as
/// libopus unrolls it, since that decides the order each sum is added in.
pub fn iir(arch: Arch, x: []f32, den: []const f32, mem: []f32) void {
    const ord = den.len;
    std.debug.assert(ord % 4 == 0 and ord <= 24);
    // The memory left behind is the last `ord` outputs.
    std.debug.assert(x.len >= ord);
    const n = x.len;
    var rden: [24]f32 = undefined;
    for (0..ord) |i| rden[i] = den[ord - i - 1];
    var y_buf: [24 + 2048]f32 = undefined;
    const y = y_buf[0..n + ord];
    for (0..ord) |i| y[i] = -mem[ord - i - 1];
    @memset(y[ord..], 0);

    var i: usize = 0;
    while (i + 3 < n) : (i += 4) {
        var sum = [4]f32{ x[i], x[i + 1], x[i + 2], x[i + 3] };
        simd.xcorrKernel(arch, rden[0..ord], y[i..], &sum, ord);
        // The kernel read zeros where this block's own outputs go. They are
        // added now that they are known.
        y[i + ord] = -sum[0];
        x[i] = sum[0];
        sum[1] += y[i + ord] * den[0];
        y[i + ord + 1] = -sum[1];
        x[i + 1] = sum[1];
        sum[2] += y[i + ord + 1] * den[0];
        sum[2] += y[i + ord] * den[1];
        y[i + ord + 2] = -sum[2];
        x[i + 2] = sum[2];
        sum[3] += y[i + ord + 2] * den[0];
        sum[3] += y[i + ord + 1] * den[1];
        sum[3] += y[i + ord] * den[2];
        y[i + ord + 3] = -sum[3];
        x[i + 3] = sum[3];
    }
    while (i < n) : (i += 1) {
        var sum = x[i];
        for (0..ord) |j| sum -= rden[j] * y[i + j];
        y[i + ord] = sum;
        x[i] = sum;
    }
    for (0..ord) |k| mem[k] = x[n - k - 1];
}

/// The autocorrelation of `x` at lags 0 to `ac.len - 1`, its first and last
/// `window.len` samples tapered by `window`. `_celt_autocorr`.
pub fn autocorrelation(
    arch: Arch,
    x: []const f32,
    ac: []f32,
    window: []const f32,
) void {
    const n = x.len;
    const lag = ac.len - 1;
    var xx_buf: [1024]f32 = undefined;
    const xx = xx_buf[0..n];
    @memcpy(xx, x);
    for (window, 0..) |w, i| {
        xx[i] = x[i] * w;
        xx[n - i - 1] = x[n - i - 1] * w;
    }
    const fast_n = n - lag;
    simd.pitchXcorr(arch, xx, xx, ac, fast_n);
    for (0..lag + 1) |k| {
        var d: f32 = 0;
        var i = k + fast_n;
        while (i < n) : (i += 1) d += xx[i] * xx[i - k];
        ac[k] += d;
    }
}
