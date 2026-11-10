// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2006-2011 Skype Limited
// SPDX-License-Identifier: MIT AND BSD-3-Clause

//! SILK's resampler, for the decoder: from the rate SILK coded at, 8, 12
//! or 16 kHz, to 48 kHz. libopus's
//! `resampler.c` and `resampler_private_*.c`.
//!
//! Doubling is a pair of all-pass filters. Other upsampling doubles and
//! then interpolates with a 12-phase FIR filter. Each keeps a
//! millisecond of delay, so that the output lines up with the input as
//! libopus's does.

const Resampler = @This();

const std = @import("std");
const fix = @import("fix.zig");
const tables = @import("tables.zig");

const max_batch_size_ms = 10;
const order_fir_12 = 8;

const up2_hq_0 = [3]i16{ 1746, 14986, 39083 - 65536 };
const up2_hq_1 = [3]i16{ 6854, 25769, 55542 - 65536 };

s_iir: [6]i32 = @splat(0),
fir_i16: [order_fir_12]i16 = @splat(0),
delay_buf: [48]i16 = @splat(0),
batch_size: usize = 0,
inv_ratio_q16: i32 = 0,
fs_in_khz: usize = 0,
fs_out_khz: usize = 0,
input_delay: usize = 0,

/// The decoder's resampler from `fs_in` to `fs_out`, in Hz: from 8, 12
/// or 16 kHz. `silk_resampler_init`.
pub fn init(fs_in: i32, fs_out: i32) Resampler {
    std.debug.assert(fs_out == 48000);
    const delay: usize = switch (fs_in) {
        8000 => 0,
        12000 => 4,
        16000 => 7,
        else => unreachable,
    };
    var s: Resampler = .{
        .input_delay = delay,
        .fs_in_khz = @intCast(@divTrunc(fs_in, 1000)),
        .fs_out_khz = 48,
    };
    s.batch_size = s.fs_in_khz * max_batch_size_ms;
    s.inv_ratio_q16 = @divTrunc(fs_in << 15, 48000) << 2;
    while (fix.smulww(s.inv_ratio_q16, 48000) < fs_in << 1)
        s.inv_ratio_q16 += 1;
    return s;
}

/// `in.len` samples at the input rate to `out`, at the output rate, which
/// is as long in time. `silk_resampler`.
pub fn process(s: *Resampler, out: []i16, in: []const i16) void {
    const n_samples = s.fs_in_khz - s.input_delay;
    // The first millisecond, from the delay buffer.
    @memcpy(s.delay_buf[s.input_delay..][0..n_samples], in[0..n_samples]);
    const rest_len = in.len - s.fs_in_khz;
    const rest = in[n_samples..][0..rest_len];
    var delayed = s.delay_buf;
    _ = s.iirFir(out, delayed[0..s.fs_in_khz]);
    _ = s.iirFir(out[s.fs_out_khz..], rest);

    @memcpy(s.delay_buf[0..s.input_delay], in[in.len - s.input_delay..]);
}

/// Doubled, by two all-pass filters for the even and odd outputs.
/// `silk_resampler_private_up2_HQ`.
fn up2Hq(st: *[6]i32, out: []i16, in: []const i16) void {
    for (in, 0..) |x, k| {
        const in32 = fix.lshift(@as(i32, x), 10);
        // Even output.
        var y = in32 -% st[0];
        var xx = fix.smulwb(y, up2_hq_0[0]);
        var out1 = st[0] +% xx;
        st[0] = in32 +% xx;
        y = out1 -% st[1];
        xx = fix.smulwb(y, up2_hq_0[1]);
        var out2 = st[1] +% xx;
        st[1] = out1 +% xx;
        y = out2 -% st[2];
        xx = fix.smlawb(y, y, up2_hq_0[2]);
        out1 = st[2] +% xx;
        st[2] = out2 +% xx;
        out[2 * k] = @intCast(fix.sat16(fix.rshiftRound(out1, 10)));
        // Odd output.
        y = in32 -% st[3];
        xx = fix.smulwb(y, up2_hq_1[0]);
        out1 = st[3] +% xx;
        st[3] = in32 +% xx;
        y = out1 -% st[4];
        xx = fix.smulwb(y, up2_hq_1[1]);
        out2 = st[4] +% xx;
        st[4] = out1 +% xx;
        y = out2 -% st[5];
        xx = fix.smlawb(y, y, up2_hq_1[2]);
        out1 = st[5] +% xx;
        st[5] = out2 +% xx;
        out[2 * k + 1] = @intCast(fix.sat16(fix.rshiftRound(out1, 10)));
    }
}

/// Doubled, then interpolated. `silk_resampler_private_IIR_FIR`.
fn iirFir(s: *Resampler, out_: []i16, in_: []const i16) usize {
    var buf: [2 * 16 * max_batch_size_ms + order_fir_12]i16 = undefined;
    @memcpy(buf[0..order_fir_12], &s.fir_i16);
    var out = out_;
    var in = in_;
    var written: usize = 0;
    var n_in: usize = 0;
    while (true) {
        n_in = @min(in.len, s.batch_size);
        up2Hq(&s.s_iir, buf[order_fir_12..], in[0..n_in]);
        const max_index_q16: i32 = fix.lshift(@as(i32, @intCast(n_in)), 16 + 1);
        var index_q16: i32 = 0;
        var o: usize = 0;
        while (index_q16 < max_index_q16) : (index_q16 += s.inv_ratio_q16) {
            const ti: usize = @intCast(fix.smulwb(index_q16 & 0xffff, 12));
            const p = buf[@intCast(index_q16 >> 16)..];
            const f = &tables.resampler_frac_fir_12;
            var res = fix.smulbb(p[0], f[ti][0]);
            res = fix.smlabb(res, p[1], f[ti][1]);
            res = fix.smlabb(res, p[2], f[ti][2]);
            res = fix.smlabb(res, p[3], f[ti][3]);
            res = fix.smlabb(res, p[4], f[11 - ti][3]);
            res = fix.smlabb(res, p[5], f[11 - ti][2]);
            res = fix.smlabb(res, p[6], f[11 - ti][1]);
            res = fix.smlabb(res, p[7], f[11 - ti][0]);
            out[o] = @intCast(fix.sat16(fix.rshiftRound(res, 15)));
            o += 1;
        }
        out = out[o..];
        written += o;
        in = in[n_in..];
        if (in.len > 0) {
            std.mem.copyForwards(
                i16,
                buf[0..order_fir_12],
                buf[n_in << 1..][0..order_fir_12],
            );
        } else break;
    }
    @memcpy(&s.fir_i16, buf[n_in << 1..][0..order_fir_12]);
    return written;
}

