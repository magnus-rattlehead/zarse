// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2007-2008 CSIRO
// SPDX-FileCopyrightText: © 2007-2009 Xiph.Org Foundation
// SPDX-FileCopyrightText: © 2014 Xiph.Org Foundation
// SPDX-FileCopyrightText: © 2014 Jean-Marc Valin
// SPDX-FileCopyrightText: © 2020 Amazon
// SPDX-License-Identifier: MIT AND BSD-2-Clause

//! The float kernels libopus has SIMD versions of, each at every `Arch`
//! level, added up in the order libopus's version for that level adds it
//! (on x86-64, the SSE ones at every level, as libopus presumes SSE there):
//! inner products and cross-correlations for CELT and its pitch analysis,
//! and the pitch post-filter.
//! libopus's `pitch.h` and `pitch.c`, `celt/x86/pitch_sse.c` and
//! `pitch_avx.c` and `celt.c`'s `comb_filter_const_c`.
//!
//! The vectors here are Zig's. Each lane does what libopus's intrinsic
//! does to it, so the results are the same whatever the CPU. It is only
//! how fast they come that depends on it.

const std = @import("std");
const Arch = @import("arch.zig").Arch;

const V4 = @Vector(4, f32);
const V8 = @Vector(8, f32);

fn load4(p: []const f32) V4 {
    return p[0..4].*;
}

fn load8(p: []const f32) V8 {
    return p[0..8].*;
}

// -- the four correlations at once ---------------------------------------------

/// `sum[k] += x[j] * y[j + k]` over `j` below `len`, for `k` from 0 to 3.
/// `y` holds `len + 3` samples. `xcorr_kernel`.
pub fn xcorrKernel(
    arch: Arch,
    x: []const f32,
    y: []const f32,
    sum: *[4]f32,
    len: usize,
) void {
    if (arch == .generic)
        xcorrKernelC(x, y, sum, len)
    else
        xcorrKernelSse(x, y, sum, len);
}

fn xcorrKernelC(x: []const f32, y: []const f32, sum: *[4]f32, len: usize) void {
    for (0..len) |j| {
        inline for (0..4) |k| sum[k] += x[j] * y[j + k];
    }
}

/// Two sums of four, taking turns, which the SSE version keeps for the
/// sake of its pipeline: `xcorr_kernel_sse`.
fn xcorrKernelSse(
    x: []const f32,
    y: []const f32,
    sum: *[4]f32,
    len: usize,
) void {
    var xsum1: V4 = sum.*;
    var xsum2: V4 = @splat(0);
    var j: usize = 0;
    while (j + 3 < len) : (j += 4) {
        xsum1 += @as(V4, @splat(x[j])) * load4(y[j..]);
        xsum2 += @as(V4, @splat(x[j + 1])) * load4(y[j + 1..]);
        xsum1 += @as(V4, @splat(x[j + 2])) * load4(y[j + 2..]);
        xsum2 += @as(V4, @splat(x[j + 3])) * load4(y[j + 3..]);
    }
    if (j < len) {
        xsum1 += @as(V4, @splat(x[j])) * load4(y[j..]);
        j += 1;
        if (j < len) {
            xsum2 += @as(V4, @splat(x[j])) * load4(y[j..]);
            j += 1;
            if (j < len) xsum1 += @as(V4, @splat(x[j])) * load4(y[j..]);
        }
    }
    sum.* = xsum1 + xsum2;
}

// -- inner products -------------------------------------------------------------

/// `x[i] * y[i]` summed over `x`. `celt_inner_prod`.
pub fn innerProd(arch: Arch, x: []const f32, y: []const f32) f32 {
    return if (arch == .generic) innerProdC(x, y) else innerProdSse(x, y);
}

fn innerProdC(x: []const f32, y: []const f32) f32 {
    var sum: f32 = 0;
    for (x, y[0..x.len]) |a, b| sum += a * b;
    return sum;
}

/// Four lanes, folded as `movehl` and `add_ss` fold them: lanes 0 and 2,
/// 1 and 3, then those two.
fn hsum4(v: V4) f32 {
    return (v[0] + v[2]) + (v[1] + v[3]);
}

/// `celt_inner_prod_sse`.
fn innerProdSse(x: []const f32, y: []const f32) f32 {
    const n = x.len;
    var sum: V4 = @splat(0);
    var i: usize = 0;
    while (i + 3 < n) : (i += 4) sum += load4(x[i..]) * load4(y[i..]);
    var xy = hsum4(sum);
    while (i < n) : (i += 1) xy += x[i] * y[i];
    return xy;
}

/// `x` with `y01` and with `y02`. `dual_inner_prod`.
pub fn dualInnerProd(
    arch: Arch,
    x: []const f32,
    y01: []const f32,
    y02: []const f32,
    xy1: *f32,
    xy2: *f32,
) void {
    const n = x.len;
    if (arch == .generic) {
        var a: f32 = 0;
        var b: f32 = 0;
        for (0..n) |i| {
            a += x[i] * y01[i];
            b += x[i] * y02[i];
        }
        xy1.* = a;
        xy2.* = b;
        return;
    }
    // `dual_inner_prod_sse`.
    var s1: V4 = @splat(0);
    var s2: V4 = @splat(0);
    var i: usize = 0;
    while (i + 3 < n) : (i += 4) {
        const xi = load4(x[i..]);
        s1 += xi * load4(y01[i..]);
        s2 += xi * load4(y02[i..]);
    }
    var a = hsum4(s1);
    var b = hsum4(s2);
    while (i < n) : (i += 1) {
        a += x[i] * y01[i];
        b += x[i] * y02[i];
    }
    xy1.* = a;
    xy2.* = b;
}

// -- cross-correlation ------------------------------------------------------------

/// `xcorr[k] = sum(x[j] * y[j + k])` over `j` below `len`, for each `k`
/// below `xcorr.len`. `celt_pitch_xcorr`: four lags at a time through
/// `xcorrKernel`, or eight through AVX2's, and the rest one at a time.
pub fn pitchXcorr(
    arch: Arch,
    x: []const f32,
    y: []const f32,
    xcorr: []f32,
    len: usize,
) void {
    const max_pitch = xcorr.len;
    var i: usize = 0;
    if (arch == .avx2) {
        while (i + 7 < max_pitch) : (i += 8)
            xcorrKernelAvx(x, y[i..], xcorr[i..][0..8], len);
    } else {
        while (i + 3 < max_pitch) : (i += 4) {
            var sum: [4]f32 = @splat(0);
            xcorrKernel(arch, x, y[i..], &sum, len);
            xcorr[i..][0..4].* = sum;
        }
    }
    while (i < max_pitch) : (i += 1)
        xcorr[i] = innerProd(arch, x[0..len], y[i..]);
}

/// Eight correlations at once, each in eight lanes of fused multiply-adds,
/// the lanes folded as AVX2's `permute2f128` and `hadd` fold them.
/// `xcorr_kernel_avx`.
fn xcorrKernelAvx(
    x: []const f32,
    y: []const f32,
    sum: *[8]f32,
    len: usize,
) void {
    var xsum: [8]V8 = @splat(@splat(0));
    var i: usize = 0;
    while (i + 7 < len) : (i += 8) {
        const x0 = load8(x[i..]);
        inline for (0..8) |k|
            xsum[k] = @mulAdd(V8, x0, load8(y[i + k..]), xsum[k]);
    }
    if (i != len) {
        // The rest, with the lanes past the end as zeros.
        const r = len - i;
        var xa: [8]f32 = @splat(0);
        for (0..r) |l| xa[l] = x[i + l];
        const xm: V8 = xa;
        inline for (0..8) |k| {
            var ya: [8]f32 = @splat(0);
            for (0..r) |l| ya[l] = y[i + k + l];
            xsum[k] = @mulAdd(V8, xm, ya, xsum[k]);
        }
    }
    // Each sum is ((l0 + l4) + (l1 + l5)) + ((l2 + l6) + (l3 + l7)).
    inline for (0..8) |k| {
        const v = xsum[k];
        sum[k] = ((v[0] + v[4]) + (v[1] + v[5]))
            + ((v[2] + v[6]) + (v[3] + v[7]));
    }
}

// -- the pitch post-filter ----------------------------------------------------------

/// `y[i] = x[i] + g10 x[i-t] + g11 (x[i-t+1] + x[i-t-1]) + g12 (x[i-t+2] +
/// x[i-t-2])` for `n` samples, where `x` and `y` are offsets into `mem`
/// and `out`. `comb_filter_const`.
pub fn combFilterConst(
    arch: Arch,
    out: []f32,
    y: usize,
    mem: []const f32,
    x: usize,
    t: usize,
    n: usize,
    g10: f32,
    g11: f32,
    g12: f32,
) void {
    if (arch == .generic) {
        var x4 = mem[x - t - 2];
        var x3 = mem[x - t - 1];
        var x2 = mem[x - t];
        var x1 = mem[x - t + 1];
        for (0..n) |i| {
            const x0 = mem[x + i - t + 2];
            out[y + i] = mem[x + i]
                + g10 * x2
                + g11 * (x1 + x3)
                + g12 * (x0 + x4);
            x4 = x3;
            x3 = x2;
            x2 = x1;
            x1 = x0;
        }
        return;
    }
    // `comb_filter_const_sse`: the two outer taps summed first, then added
    // to the rest, four samples at a time. Built without custom modes, as
    // libopus is, it leaves any samples past the last four alone. `n` is a
    // multiple of four whenever Opus calls it.
    const g10v: V4 = @splat(g10);
    const g11v: V4 = @splat(g11);
    const g12v: V4 = @splat(g12);
    var i: usize = 0;
    while (i + 3 < n) : (i += 4) {
        const p = x + i - t - 2;
        const x0v = load4(mem[p..]);
        const x1v = load4(mem[p + 1..]);
        const x2v = load4(mem[p + 2..]);
        const x3v = load4(mem[p + 3..]);
        const x4v = load4(mem[p + 4..]);
        var yi = load4(mem[x + i..]);
        yi += g10v * x2v;
        const yi2 = g11v * (x3v + x1v) + g12v * (x4v + x0v);
        out[y + i..][0..4].* = yi + yi2;
    }
}

// -- SILK ---------------------------------------------------------------------------

