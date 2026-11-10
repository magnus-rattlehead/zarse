// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2007-2008 CSIRO
// SPDX-FileCopyrightText: © 2007-2010 Xiph.Org Foundation
// SPDX-FileCopyrightText: © 2008 Gregory Maxwell
// SPDX-License-Identifier: MIT AND BSD-2-Clause

//! A CELT decoder: one channel or two, and what carries from one frame to
//! the next. RFC 6716, Section 4.3, following libopus's `celt_decoder.c`
//! and, for the pitch post-filter, `celt.c`.
//!
//! A frame is decoded in the order the encoder wrote it (Section 4.3,
//! Table 56): silence, the post-filter, transient and intra flags, coarse
//! energy, time-frequency changes, spreading, dynamic allocation, the
//! allocation trim, the bit allocation, fine energy, the bands' shapes,
//! anti-collapse and the last fine energy bits. Then the spectrum goes
//! through the inverse MDCT, the pitch post-filter and de-emphasis.

const Decoder = @This();

const std = @import("std");
const RangeDecoder = @import("../RangeDecoder.zig");
const mode = @import("mode.zig");
const energy = @import("energy.zig");
const rate = @import("rate.zig");
const pvq = @import("pvq.zig");
const bands = @import("bands.zig");
const mdct = @import("mdct.zig");
const lp = @import("lpc.zig");
const pitch = @import("pitch.zig");
const simd = @import("../simd.zig");
const Arch = @import("../arch.zig").Arch;

const bitres = RangeDecoder.bitres;
const nb_ebands = mode.nb_ebands;
const overlap = mode.overlap;

/// Samples kept for the post-filter's and concealment's look back.
pub const decode_buffer_size = 2048;
/// The shortest pitch period the post-filter uses.
const combfilter_minperiod = 15;
/// Order of the concealment's linear prediction.
pub const lpc_order = 24;
/// The longest pitch period concealment looks back over.
const max_period = 1024;
/// The range of pitch periods concealment searches, at 48 kHz.
const plc_pitch_lag_max = 720;
const plc_pitch_lag_min = 100;

pub const trim_icdf = [11]u8{ 126, 124, 119, 109, 87, 41, 19, 9, 4, 2, 0 };
/// None 21.875%, light 6.25%, normal 65.625%, aggressive 6.25%.
pub const spread_icdf = [4]u8{ 25, 23, 2, 0 };
pub const tapset_icdf = [3]u8{ 2, 1, 0 };

/// Which time-frequency change each band's flag selects, by frame size,
/// transient or not, and the frame's tf_select. `celt.c`.
pub const tf_select_table = [4][8]i8{
    .{ 0, -1, 0, -1, 0, -1, 0, -1 },
    .{ 0, -1, 0, -2, 1, 0, 1, -1 },
    .{ 0, -2, 0, -3, 2, 0, 1, -1 },
    .{ 0, -2, 0, -3, 3, 0, 1, -1 },
};

/// Output channels.
channels: usize,
/// Channels in the stream, which a frame may have fewer of.
stream_channels: usize,
/// The bands decoded: from 17 in hybrid mode, and to fewer than 21 at a
/// narrower bandwidth.
start: usize = 0,
end: usize = nb_ebands,
disable_inv: bool,
/// Which of libopus's kernels to compute with: see `Arch`.
arch: Arch = .generic,
/// The range coder's final state, for comparison with a reference decoder.
rng: u32 = 0,
/// Set when a frame's bits did not add up: it decoded, but not as the
/// encoder meant.
err: bool = false,
last_pitch_index: i32 = 0,
/// How long, in samples, since the last frame that arrived.
loss_duration: i32 = 0,
skip_plc: bool = true,
postfilter_period: i32 = 0,
postfilter_period_old: i32 = 0,
postfilter_gain: f32 = 0,
postfilter_gain_old: f32 = 0,
postfilter_tapset: usize = 0,
postfilter_tapset_old: usize = 0,
prefilter_and_fold: bool = false,
preemph_mem: [2]f32 = .{ 0, 0 },

/// Each channel's past output, and the frame being made at its end.
decode_mem: [2][decode_buffer_size + overlap]f32 = @splat(@splat(0)),
lpc: [2][lpc_order]f32 = @splat(@splat(0)),
/// Band energies: this frame's, and the two before for anti-collapse and
/// concealment, and the background noise level.
old_band_e: [2 * nb_ebands]f32 = @splat(0),
old_log_e: [2 * nb_ebands]f32 = @splat(-28),
old_log_e2: [2 * nb_ebands]f32 = @splat(-28),
background_log_e: [2 * nb_ebands]f32 = @splat(0),

/// A mono or stereo decoder with 48 kHz output.
pub fn init(channels: usize) error{ BadArg }!Decoder {
    if (channels < 1 or channels > 2) return error.BadArg;
    return .{
        .channels = channels,
        .stream_channels = channels,
        .disable_inv = channels == 1,
        .arch = Arch.detect(),
    };
}

/// Back to how `init` left it, keeping the settings: after a seek.
pub fn reset(d: *Decoder) void {
    const kept = d.*;
    d.* = .{
        .channels = kept.channels,
        .stream_channels = kept.stream_channels,
        .start = kept.start,
        .end = kept.end,
        .disable_inv = kept.disable_inv,
        .arch = kept.arch,
    };
}

pub const Error = error{
    /// The frame size is not one CELT has.
    BadArg,
    /// More bits were read than the frame had: it is corrupt.
    InternalError,
};

/// Decodes `data`, a frame of `frame_size` samples at 48 kHz, into `pcm`,
/// interleaved. `dec` is the
/// range decoder when the frame's start was already read for SILK, in
/// hybrid mode. Otherwise one is made. Returns the samples a channel.
pub fn decode(
    d: *Decoder,
    data: ?[]const u8,
    pcm: []f32,
    frame_size: usize,
    dec_: ?*RangeDecoder,
) Error!usize {
    const cc = d.channels;
    const c_stream = d.stream_channels;
    const start = d.start;
    const end = d.end;

    const lm: u2 = switch (frame_size) {
        120 => 0,
        240 => 1,
        480 => 2,
        960 => 3,
        else => return error.BadArg,
    };
    const m: usize = @as(usize, 1) << lm;
    const n = m * mode.short_mdct_size;
    const eff_end = @min(end, mode.eff_ebands);

    const len = if (data) |b| b.len else 0;
    if (len > 1275) return error.BadArg;

    if (data == null or len <= 1) {
        d.decodeLost(n, lm);
        d.deemphasis(pcm, n);
        return frame_size;
    }

    // Pitch-based concealment only after two frames in a row.
    if (d.loss_duration == 0) d.skip_plc = false;

    var own_dec: RangeDecoder = undefined;
    const dec = dec_ orelse blk: {
        own_dec = .init(data.?);
        break :blk &own_dec;
    };

    if (c_stream == 1) {
        for (0..nb_ebands) |i|
            d.old_band_e[i] = @max(
                d.old_band_e[i],
                d.old_band_e[nb_ebands + i],
            );
    }

    var total_bits: i32 = @intCast(len * 8);
    var tell = dec.tell();
    const silence = if (tell >= total_bits)
        true
    else if (tell == 1)
        dec.decodeBitLogp(15)
    else
        false;
    if (silence) {
        // As if every bit had been read.
        tell = @intCast(len * 8);
        dec.nbits_total += tell - dec.tell();
    }

    var postfilter_gain: f32 = 0;
    var postfilter_pitch: i32 = 0;
    var postfilter_tapset: usize = 0;
    if (start == 0 and tell + 16 <= total_bits) {
        if (dec.decodeBitLogp(1)) {
            const octave: u5 = @intCast(dec.decodeUint(6));
            postfilter_pitch = (@as(i32, 16) << octave)
                + @as(i32, @intCast(dec.decodeBits(4 + octave)))
                - 1;
            const qg: i32 = @intCast(dec.decodeBits(3));
            if (dec.tell() + 2 <= total_bits)
                postfilter_tapset = dec.decodeIcdf(&tapset_icdf, 2);
            postfilter_gain = 0.09375 * @as(f32, @floatFromInt(qg + 1));
        }
        tell = dec.tell();
    }

    var is_transient = false;
    if (lm > 0 and tell + 3 <= total_bits) {
        is_transient = dec.decodeBitLogp(3);
        tell = dec.tell();
    }
    const short_blocks = is_transient;

    const intra_ener = if (tell + 3 <= total_bits)
        dec.decodeBitLogp(3)
    else
        false;

    // After a loss, make the energy prediction safe, so that it cannot come
    // back loud.
    if (!intra_ener and d.loss_duration != 0) {
        for (0..2) |c| {
            var safety: f32 = 0;
            const missing = @min(10, d.loss_duration >> lm);
            if (lm == 0)
                safety = 1.5
            else if (lm == 1) safety = 0.5;
            for (start..end) |i| {
                const k = c * nb_ebands + i;
                if (d.old_band_e[k] < @max(d.old_log_e[k], d.old_log_e2[k])) {
                    // Falling already: continue the trend.
                    var e0 = d.old_band_e[k];
                    const e1 = d.old_log_e[k];
                    const e2 = d.old_log_e2[k];
                    const slope = @max(e1 - e0, 0.5 * (e2 - e0));
                    e0 -= @max(0, @as(f32, @floatFromInt(1 + missing)) * slope);
                    d.old_band_e[k] = @max(-20.0, e0);
                } else {
                    // Otherwise the least of the last frames.
                    d.old_band_e[k] = @min(
                        @min(d.old_band_e[k], d.old_log_e[k]),
                        d.old_log_e2[k],
                    );
                }
                // Shorter frames fluctuate more.
                d.old_band_e[k] -= safety;
            }
        }
    }

    energy.decodeCoarse(
        dec,
        start,
        end,
        &d.old_band_e,
        intra_ener,
        c_stream,
        lm,
    );

    var tf_res: [nb_ebands]i32 = undefined;
    tfDecode(start, end, is_transient, &tf_res, lm, dec);

    tell = dec.tell();
    var spread: pvq.Spread = .normal;
    if (tell + 4 <= total_bits)
        spread = @enumFromInt(dec.decodeIcdf(&spread_icdf, 5));

    var cap: [nb_ebands]i32 = undefined;
    rate.initCaps(&cap, lm, c_stream);

    // Dynamic allocation: boosts for bands the encoder thought needed them.
    var offsets: [nb_ebands]i32 = @splat(0);
    var dynalloc_logp: u5 = 6;
    total_bits <<= bitres;
    var tell_frac: i32 = @intCast(dec.tellFrac());
    for (start..end) |i| {
        const width: i32 = @as(i32, @intCast(c_stream))
            * (mode.ebands[i + 1] - mode.ebands[i])
            << lm;
        // Six bits, but at most one a sample and at least an eighth.
        const quanta = @min(width << bitres, @max(6 << bitres, width));
        var loop_logp = dynalloc_logp;
        var boost: i32 = 0;
        while (
            tell_frac + (@as(i32, loop_logp) << bitres) < total_bits
                and boost < cap[i]
        ) {
            const flag = dec.decodeBitLogp(loop_logp);
            tell_frac = @intCast(dec.tellFrac());
            if (!flag) break;
            boost += quanta;
            total_bits -= quanta;
            loop_logp = 1;
        }
        offsets[i] = boost;
        // Boosts make more boosts likely.
        if (boost > 0) dynalloc_logp = @max(2, dynalloc_logp - 1);
    }

    const alloc_trim: i32 = if (tell_frac + (6 << bitres) <= total_bits)
        @intCast(dec.decodeIcdf(&trim_icdf, 7))
    else
        5;

    var bits: i32 = (@as(i32, @intCast(len)) * 8 << bitres)
        - @as(i32, @intCast(dec.tellFrac()))
        - 1;
    const anti_collapse_rsv: i32 = if (
        is_transient
            and lm >= 2
            and bits >= (@as(i32, lm) + 2) << bitres
    )
        1 << bitres
    else
        0;
    bits -= anti_collapse_rsv;

    var pulses: [nb_ebands]i32 = undefined;
    var fine_quant: [nb_ebands]i32 = undefined;
    var fine_priority: [nb_ebands]i32 = undefined;
    const alloc = rate.computeAllocation(
        start,
        end,
        &offsets,
        &cap,
        alloc_trim,
        bits,
        &pulses,
        &fine_quant,
        &fine_priority,
        c_stream,
        lm,
        dec,
    );

    energy.decodeFine(dec, start, end, &d.old_band_e, &fine_quant, c_stream);

    // Make room for the frame at the end of the history.
    for (0..cc) |c| {
        const mem = &d.decode_mem[c];
        std.mem.copyForwards(
            f32,
            mem[0..decode_buffer_size - n + overlap],
            mem[n..decode_buffer_size + overlap],
        );
    }

    var collapse_masks: [2 * nb_ebands]u8 = undefined;
    var x_buf: [2 * 960]f32 = undefined;
    const x = x_buf[0..c_stream * n];
    bands.decodeAll(
        start,
        end,
        x[0..n],
        if (c_stream == 2) x[n..2 * n] else null,
        &collapse_masks,
        &pulses,
        short_blocks,
        spread,
        alloc.dual_stereo,
        alloc.intensity,
        &tf_res,
        @as(i32, @intCast(len)) * (8 << bitres) - anti_collapse_rsv,
        alloc.balance,
        dec,
        lm,
        alloc.coded_bands,
        &d.rng,
        d.disable_inv,
        d.arch,
    );

    const anti_collapse_on = anti_collapse_rsv > 0 and dec.decodeBits(1) != 0;

    energy.decodeFinal(
        dec,
        start,
        end,
        &d.old_band_e,
        &fine_quant,
        &fine_priority,
        @as(i32, @intCast(len * 8)) - dec.tell(),
        c_stream,
    );

    if (anti_collapse_on) {
        bands.antiCollapse(
            x,
            &collapse_masks,
            lm,
            c_stream,
            n,
            start,
            end,
            &d.old_band_e,
            &d.old_log_e,
            &d.old_log_e2,
            &pulses,
            d.rng,
            d.arch,
        );
    }

    if (silence) {
        for (d.old_band_e[0..c_stream * nb_ebands]) |*e| e.* = -28;
    }

    if (d.prefilter_and_fold) d.prefilterAndFold(n);

    d.synthesis(x, start, eff_end, c_stream, is_transient, lm, silence);

    for (0..cc) |c| {
        d.postfilter_period = @max(d.postfilter_period, combfilter_minperiod);
        d.postfilter_period_old = @max(
            d.postfilter_period_old,
            combfilter_minperiod,
        );
        const syn = decode_buffer_size - n;
        combFilter(
            d.arch,
            &d.decode_mem[c],
            syn,
            &d.decode_mem[c],
            syn,
            d.postfilter_period_old,
            d.postfilter_period,
            mode.short_mdct_size,
            d.postfilter_gain_old,
            d.postfilter_gain,
            d.postfilter_tapset_old,
            d.postfilter_tapset,
            overlap,
        );
        if (lm != 0) {
            combFilter(
                d.arch,
                &d.decode_mem[c],
                syn + mode.short_mdct_size,
                &d.decode_mem[c],
                syn + mode.short_mdct_size,
                d.postfilter_period,
                postfilter_pitch,
                n - mode.short_mdct_size,
                d.postfilter_gain,
                postfilter_gain,
                d.postfilter_tapset,
                postfilter_tapset,
                overlap,
            );
        }
    }
    d.postfilter_period_old = d.postfilter_period;
    d.postfilter_gain_old = d.postfilter_gain;
    d.postfilter_tapset_old = d.postfilter_tapset;
    d.postfilter_period = postfilter_pitch;
    d.postfilter_gain = postfilter_gain;
    d.postfilter_tapset = postfilter_tapset;
    if (lm != 0) {
        d.postfilter_period_old = d.postfilter_period;
        d.postfilter_gain_old = d.postfilter_gain;
        d.postfilter_tapset_old = d.postfilter_tapset;
    }

    if (c_stream == 1)
        @memcpy(d.old_band_e[nb_ebands..], d.old_band_e[0..nb_ebands]);
    if (!is_transient) {
        d.old_log_e2 = d.old_log_e;
        d.old_log_e = d.old_band_e;
    } else {
        for (&d.old_log_e, d.old_band_e) |*l, e| l.* = @min(l.*, e);
    }
    // The noise floor rises by at most 2.4 dB a second, except that a frame
    // after a gap takes the whole gap's worth.
    const max_background_increase: f32 = @as(
        f32,
        @floatFromInt(@min(160, d.loss_duration + @as(i32, @intCast(m)))),
    )
        * 0.001;
    for (&d.background_log_e, d.old_band_e) |*b, e|
        b.* = @min(b.* + max_background_increase, e);

    // In case start or end change.
    for (0..2) |c| {
        for (0..start) |i| {
            d.old_band_e[c * nb_ebands + i] = 0;
            d.old_log_e[c * nb_ebands + i] = -28;
            d.old_log_e2[c * nb_ebands + i] = -28;
        }
        for (end..nb_ebands) |i| {
            d.old_band_e[c * nb_ebands + i] = 0;
            d.old_log_e[c * nb_ebands + i] = -28;
            d.old_log_e2[c * nb_ebands + i] = -28;
        }
    }
    d.rng = dec.rng;

    d.deemphasis(pcm, n);
    d.loss_duration = 0;
    d.prefilter_and_fold = false;

    if (dec.tell() > 8 * @as(i32, @intCast(len))) return error.InternalError;
    if (dec.err) d.err = true;
    return frame_size;
}

/// Each band's time-frequency change. Section 4.3.1.
fn tfDecode(
    start: usize,
    end: usize,
    is_transient: bool,
    tf_res: []i32,
    lm: u2,
    dec: *RangeDecoder,
) void {
    var budget: u32 = @intCast(dec.buf.len * 8);
    var tell: u32 = @intCast(dec.tell());
    var logp: u5 = if (is_transient) 2 else 4;
    const tf_select_rsv = lm > 0 and tell + logp + 1 <= budget;
    budget -= @intFromBool(tf_select_rsv);
    var tf_changed: i32 = 0;
    var curr: i32 = 0;
    for (start..end) |i| {
        if (tell + logp <= budget) {
            curr ^= @intFromBool(dec.decodeBitLogp(logp));
            tell = @intCast(dec.tell());
            tf_changed |= curr;
        }
        tf_res[i] = curr;
        logp = if (is_transient) 4 else 5;
    }
    const t: usize = 4 * @as(usize, @intFromBool(is_transient));
    var tf_select: usize = 0;
    if (
        tf_select_rsv
            and tf_select_table[lm][
                t + @as(usize, @intCast(tf_changed))
            ] != tf_select_table[lm][t + 2 + @as(usize, @intCast(tf_changed))]
    ) {
        tf_select = @intFromBool(dec.decodeBitLogp(1));
    }
    for (start..end) |i|
        tf_res[i] = tf_select_table[lm][
            t + 2 * tf_select + @as(usize, @intCast(tf_res[i]))
        ];
}

/// The spectrum to time: denormalized, through the inverse MDCT, into the
/// end of each channel's history. `celt_synthesis`.
fn synthesis(
    d: *Decoder,
    x: []const f32,
    start: usize,
    eff_end: usize,
    c_stream: usize,
    is_transient: bool,
    lm: u2,
    silence: bool,
) void {
    const n = @as(usize, mode.short_mdct_size) << lm;
    const m = @as(usize, 1) << lm;
    const b_count: usize = if (is_transient) m else 1;
    const nb: usize = if (is_transient) mode.short_mdct_size else n;
    const shift: u2 = if (is_transient)
        mode.max_lm
    else
        @intCast(mode.max_lm - lm);
    const syn = decode_buffer_size - n;
    var freq: [960]f32 = undefined;

    if (d.channels == 2 and c_stream == 1) {
        // A mono stream to two channels. The inverse MDCT destroys its input,
        // so a copy is kept in the second channel's output meanwhile.
        bands.denormalise(x, &freq, &d.old_band_e, start, eff_end, m, silence);
        const freq2 = d.decode_mem[1][syn + overlap / 2..][0..n];
        @memcpy(freq2, freq[0..n]);
        for (0..b_count) |b|
            mdct.backward(
                freq2[b..],
                d.decode_mem[0][syn + nb * b..],
                &mode.window,
                overlap,
                shift,
                b_count,
            );
        for (0..b_count) |b|
            mdct.backward(
                freq[b..],
                d.decode_mem[1][syn + nb * b..],
                &mode.window,
                overlap,
                shift,
                b_count,
            );
    } else if (d.channels == 1 and c_stream == 2) {
        // A stereo stream to one channel.
        const freq2 = d.decode_mem[0][syn + overlap / 2..][0..n];
        bands.denormalise(x, &freq, &d.old_band_e, start, eff_end, m, silence);
        bands.denormalise(
            x[n..],
            freq2,
            d.old_band_e[nb_ebands..],
            start,
            eff_end,
            m,
            silence,
        );
        for (freq[0..n], freq2) |*f, f2| f.* = 0.5 * f.* + 0.5 * f2;
        for (0..b_count) |b|
            mdct.backward(
                freq[b..],
                d.decode_mem[0][syn + nb * b..],
                &mode.window,
                overlap,
                shift,
                b_count,
            );
    } else {
        for (0..d.channels) |c| {
            bands.denormalise(
                x[c * n..],
                &freq,
                d.old_band_e[c * nb_ebands..],
                start,
                eff_end,
                m,
                silence,
            );
            for (0..b_count) |b|
                mdct.backward(
                    freq[b..],
                    d.decode_mem[c][syn + nb * b..],
                    &mode.window,
                    overlap,
                    shift,
                    b_count,
                );
        }
    }
}

/// The pitch post-filter: a comb filter at the period the frame gives, its
/// taps' gains set by the tapset, cross-faded over the overlap from the
/// last frame's filter. `celt.c`'s `comb_filter`, from `mem` at `x`, with
/// the history before it, to `out` at `y`, which may be the same place.
pub fn combFilter(
    arch: Arch,
    out: []f32,
    y: usize,
    mem: []const f32,
    x: usize,
    t0_: i32,
    t1_: i32,
    n: usize,
    g0: f32,
    g1: f32,
    tapset0: usize,
    tapset1: usize,
    overlap_: usize,
) void {
    const gains = [3][3]f32{
        .{ 0.3066406250, 0.2170410156, 0.1296386719 },
        .{ 0.4638671875, 0.2680664062, 0.0 },
        .{ 0.7998046875, 0.1000976562, 0.0 },
    };
    const in_place = &out[y] == &mem[x];
    if (g0 == 0 and g1 == 0) {
        if (!in_place) std.mem.copyForwards(f32, out[y..y + n], mem[x..x + n]);
        return;
    }
    // A gain of zero sets the period to zero. It has to be at least 2.
    const t0: usize = @intCast(@max(t0_, combfilter_minperiod));
    const t1: usize = @intCast(@max(t1_, combfilter_minperiod));
    const g00 = g0 * gains[tapset0][0];
    const g01 = g0 * gains[tapset0][1];
    const g02 = g0 * gains[tapset0][2];
    const g10 = g1 * gains[tapset1][0];
    const g11 = g1 * gains[tapset1][1];
    const g12 = g1 * gains[tapset1][2];
    var x1 = mem[x - t1 + 1];
    var x2 = mem[x - t1];
    var x3 = mem[x - t1 - 1];
    var x4 = mem[x - t1 - 2];
    // An unchanged filter needs no cross-fade.
    const ov = if (g0 == g1 and t0 == t1 and tapset0 == tapset1)
        0
    else
        overlap_;
    var i: usize = 0;
    while (i < ov) : (i += 1) {
        const x0 = mem[x + i - t1 + 2];
        const f = mode.window[i] * mode.window[i];
        out[y + i] = mem[x + i]
            + ((1.0 - f) * g00) * mem[x + i - t0]
            + ((1.0 - f) * g01) * (mem[x + i - t0 + 1] + mem[x + i - t0 - 1])
            + ((1.0 - f) * g02) * (mem[x + i - t0 + 2] + mem[x + i - t0 - 2])
            + (f * g10) * x2
            + (f * g11) * (x1 + x3)
            + (f * g12) * (x0 + x4);
        x4 = x3;
        x3 = x2;
        x2 = x1;
        x1 = x0;
    }
    if (g1 == 0) {
        if (!in_place)
            std.mem.copyForwards(f32, out[y + ov..y + n], mem[x + ov..x + n]);
        return;
    }
    // The rest, with the new filter alone.
    simd.combFilterConst(
        arch,
        out,
        y + i,
        mem,
        x + i,
        t1,
        n - i,
        g10,
        g11,
        g12,
    );
}

/// Undoes the encoder's pre-emphasis, and scales to [-1, 1], into `pcm`
/// interleaved at 48 kHz. `deemphasis`.
fn deemphasis(d: *Decoder, pcm: []f32, n: usize) void {
    const coef0 = mode.preemph;
    const very_small: f32 = 1e-30;
    const scale: f32 = 1.0 / 32768.0;
    const cc = d.channels;
    const syn = decode_buffer_size - n;
    for (0..cc) |c| {
        const x = d.decode_mem[c][syn..][0..n];
        var mem = d.preemph_mem[c];
        for (0..n) |j| {
            const tmp = x[j] + very_small + mem;
            mem = coef0 * tmp;
            pcm[j * cc + c] = tmp * scale;
        }
        d.preemph_mem[c] = mem;
    }
}

/// A frame that did not arrive, concealed. `celt_decode_lost`.
///
/// After a frame that arrived, and while the loss is short, the last pitch
/// period is repeated: the signal's prediction error is found with a
/// predictor made from the last 1024 samples, extrapolated a period at a
/// time, fading as the signal was, and filtered back. Once that has gone on
/// for 100 ms, or after a single frame, or in hybrid mode, the
/// bands are filled with noise instead at their last energies, decaying to
/// the background level.
fn decodeLost(d: *Decoder, n: usize, lm: u2) void {
    const cc = d.channels;
    const start = d.start;
    const loss_duration = d.loss_duration;
    const syn = decode_buffer_size - n;

    const noise_based = loss_duration >= 40 or start != 0 or d.skip_plc;
    if (noise_based) {
        const end = d.end;
        const eff_end = @max(start, @min(end, mode.eff_ebands));
        for (0..cc) |c| {
            const mem = &d.decode_mem[c];
            std.mem.copyForwards(
                f32,
                mem[0..decode_buffer_size - n + overlap],
                mem[n..decode_buffer_size + overlap],
            );
        }
        if (d.prefilter_and_fold) d.prefilterAndFold(n);

        // Energy decays, to the background level at the least.
        const decay: f32 = if (loss_duration == 0) 1.5 else 0.5;
        for (0..cc) |c| {
            for (start..end) |i| {
                const e = &d.old_band_e[c * nb_ebands + i];
                const bg = d.background_log_e[c * nb_ebands + i];
                e.* = if (bg > e.* - decay) bg else e.* - decay;
            }
        }
        var x_buf: [2 * 960]f32 = undefined;
        const x = x_buf[0..cc * n];
        var seed = d.rng;
        for (0..cc) |c| {
            for (start..eff_end) |i| {
                const boffs = n * c
                    + (@as(usize, @intCast(mode.ebands[i])) << lm);
                const blen = @as(
                    usize,
                    @intCast(mode.ebands[i + 1] - mode.ebands[i]),
                )
                    << lm;
                for (x[boffs..][0..blen]) |*v| {
                    seed = bands.lcgRand(seed);
                    v.* = @floatFromInt(@as(i32, @bitCast(seed)) >> 20);
                }
                pvq.renormaliseVector(d.arch, x[boffs..][0..blen], 1.0);
            }
        }
        d.rng = seed;
        d.synthesis(x, start, eff_end, cc, false, lm, false);
        d.prefilter_and_fold = false;
        // Pitch-based concealment only after two frames in a row.
        d.skip_plc = true;
    } else {
        var fade: f32 = 1.0;
        var pitch_index: usize = undefined;
        if (loss_duration == 0) {
            pitch_index = plcPitchSearch(d.arch, d.decode_mem[0..cc]);
            d.last_pitch_index = @intCast(pitch_index);
        } else {
            pitch_index = @intCast(d.last_pitch_index);
            fade = 0.8;
        }
        // Two periods of excitation, to see whether it is decaying, but no
        // more than there is.
        const exc_length = @min(2 * pitch_index, max_period);
        const window = &mode.window;

        for (0..cc) |c| {
            const buf = &d.decode_mem[c];
            // The excitation, after `lpc_order` samples of history.
            var exc_buf: [max_period + lpc_order]f32 = undefined;
            @memcpy(
                &exc_buf,
                buf[decode_buffer_size - max_period - lpc_order..][
                    0..max_period + lpc_order
                ],
            );
            const exc = exc_buf[lpc_order..];
            const coefs = &d.lpc[c];

            if (loss_duration == 0) {
                // The predictor for the last `max_period` samples before
                // the first loss, to work on the prediction error.
                var ac: [lpc_order + 1]f32 = undefined;
                lp.autocorrelation(d.arch, exc, &ac, window);
                // A noise floor at -40 dB.
                ac[0] *= 1.0001;
                // Lag windowing, to keep Levinson-Durbin stable.
                for (1..lpc_order + 1) |i| {
                    const fi: f32 = @floatFromInt(i);
                    ac[i] -= ac[i]
                        * (@as(f32, 0.008) * @as(f32, 0.008))
                        * fi
                        * fi;
                }
                lp.fromAutocorrelation(coefs, &ac);
            }
            // The prediction error for the `exc_length` samples before the
            // loss. The filter cannot work in place.
            {
                var fir_tmp: [max_period]f32 = undefined;
                lp.fir(
                    d.arch,
                    &exc_buf,
                    lpc_order + max_period - exc_length,
                    coefs,
                    fir_tmp[0..exc_length],
                );
                @memcpy(
                    exc[max_period - exc_length..][0..exc_length],
                    fir_tmp[0..exc_length],
                );
            }

            // Whether the waveform is decaying, and how fast, so that
            // concealment does not add energy to a signal fading away.
            var decay: f32 = undefined;
            {
                var e1: f32 = 1;
                var e2: f32 = 1;
                const decay_length = exc_length >> 1;
                for (0..decay_length) |i| {
                    var e = exc[max_period - decay_length + i];
                    e1 += e * e;
                    e = exc[max_period - 2 * decay_length + i];
                    e2 += e * e;
                }
                e1 = if (e1 < e2) e1 else e2;
                decay = @floatCast(@sqrt(@as(f64, e1 / e2)));
            }

            // Room for the new frame. The overlap past the end is not used.
            std.mem.copyForwards(
                f32,
                buf[0..decode_buffer_size - n],
                buf[n..decode_buffer_size],
            );

            // The end of the excitation repeated a period at a time, each
            // period scaled by `decay` again, over the frame and the MDCT
            // overlap either side.
            const extrapolation_offset = max_period - pitch_index;
            const extrapolation_len = n + overlap;
            var attenuation = fade * decay;
            var s1: f32 = 0;
            var j: usize = 0;
            for (0..extrapolation_len) |i| {
                if (j >= pitch_index) {
                    j -= pitch_index;
                    attenuation = attenuation * decay;
                }
                buf[syn + i] = attenuation * exc[extrapolation_offset + j];
                // The energy of the signal whose excitation this copies.
                const tmp = buf[
                    decode_buffer_size
                        - max_period
                        - n
                        + extrapolation_offset
                        + j
                ];
                s1 += tmp * tmp;
                j += 1;
            }
            // Back to a signal, carrying on from the last samples decoded.
            {
                var lpc_mem: [lpc_order]f32 = undefined;
                for (&lpc_mem, 0..) |*v, i| v.* = buf[syn - 1 - i];
                lp.iir(
                    d.arch,
                    buf[syn..][0..extrapolation_len],
                    coefs,
                    &lpc_mem,
                );
            }

            // More energy than expected, which a change in the signal
            // during the window can cause, is attenuated. An explosion, or
            // a NaN, is silenced.
            var s2: f32 = 0;
            for (buf[syn..][0..extrapolation_len]) |v| s2 += v * v;
            if (!(s1 > 0.2 * s2)) {
                @memset(buf[syn..][0..extrapolation_len], 0);
            } else if (s1 < s2) {
                const ratio: f32 = @floatCast(
                    @sqrt(@as(f64, (s1 + 1) / (s2 + 1))),
                );
                for (0..overlap) |i| {
                    const g = 1.0 - window[i] * (1.0 - ratio);
                    buf[syn + i] = g * buf[syn + i];
                }
                for (overlap..extrapolation_len) |i|
                    buf[syn + i] = ratio * buf[syn + i];
            }
        }

        d.prefilter_and_fold = true;
    }
    // Saturated, to keep it from wrapping.
    d.loss_duration = @min(10000, loss_duration + (@as(i32, 1) << lm));
}

/// The pitch period of what was last decoded, at 48 kHz, between
/// `plc_pitch_lag_min` and `plc_pitch_lag_max`. `celt_plc_pitch_search`.
fn plcPitchSearch(
    arch: Arch,
    mem: []const [decode_buffer_size + overlap]f32,
) usize {
    var lp_pitch_buf: [decode_buffer_size >> 1]f32 = undefined;
    var channels: [2][]const f32 = undefined;
    for (mem, 0..) |*m, c| channels[c] = m[0..decode_buffer_size];
    pitch.downsample(
        arch,
        channels[0..mem.len],
        &lp_pitch_buf,
        decode_buffer_size,
    );
    const found = pitch.search(
        arch,
        lp_pitch_buf[plc_pitch_lag_max >> 1..],
        &lp_pitch_buf,
        decode_buffer_size - plc_pitch_lag_max,
        plc_pitch_lag_max - plc_pitch_lag_min,
    );
    return plc_pitch_lag_max - found;
}

/// After concealment, the concealed overlap is made ready to join the next
/// frame's: the post-filter, which the next frame reapplies, is undone, and
/// the MDCT's time-domain aliasing is simulated so that the overlap-add
/// cancels as it would between two decoded frames. `prefilter_and_fold`.
fn prefilterAndFold(d: *Decoder, n: usize) void {
    for (0..d.channels) |c| {
        const mem = &d.decode_mem[c];
        var etmp: [overlap]f32 = undefined;
        combFilter(
            d.arch,
            &etmp,
            0,
            mem,
            decode_buffer_size - n,
            d.postfilter_period_old,
            d.postfilter_period,
            overlap,
            -d.postfilter_gain_old,
            -d.postfilter_gain,
            d.postfilter_tapset_old,
            d.postfilter_tapset,
            0,
        );
        for (0..overlap / 2) |i| {
            mem[decode_buffer_size - n + i] = mode.window[i]
                * etmp[overlap - 1 - i]
                + mode.window[overlap - i - 1] * etmp[i];
        }
    }
}
