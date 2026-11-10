// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2006-2011 Skype Limited
// SPDX-License-Identifier: MIT AND BSD-3-Clause

//! A SILK decoder: one channel or two, and the stereo between them. RFC
//! 6716, Section 4.2, following libopus's `dec_API.c`,
//! `stereo_decode_pred.c` and `stereo_MS_to_LR.c`.
//!
//! The Opus layer calls `decode` once for each 10 or 20 ms SILK frame in a
//! packet. The first call for a packet reads the voice activity and
//! redundancy flags for all of them. A stereo stream is coded as mid and
//! side, with a prediction of the side from the mid, and turned back into
//! left and right here. Then each channel is resampled to the rate asked
//! for.

const Decoder = @This();

const std = @import("std");
const RangeDecoder = @import("../RangeDecoder.zig");
const fix = @import("fix.zig");
const tables = @import("tables.zig");
const Channel = @import("Channel.zig");

const max_frame_length = Channel.max_frame_length;
const stereo_interp_len_ms = 8;

/// What the Opus layer tells the decoder about a packet, and what it says
/// back. `silk_DecControlStruct`.
pub const Control = struct {
    /// Output channels.
    n_channels_api: usize,
    /// Channels in the stream.
    n_channels_internal: usize,
    /// The rate asked for: 8000, 12000, 16000, 24000 or 48000.
    api_sample_rate: i32,
    /// The rate SILK codes at: 8000, 12000 or 16000.
    internal_sample_rate: i32,
    /// 10, 20, 40 or 60, or 0 for a lost packet.
    payload_size_ms: i32,
    /// The pitch lag of the last frame, at 48 kHz, or 0 if it was not
    /// voiced, for the CELT concealment that may follow.
    prev_pitch_lag: i32 = 0,
};

const Stereo = struct {
    pred_prev_q13: [2]i32 = .{ 0, 0 },
    s_mid: [2]i16 = .{ 0, 0 },
    s_side: [2]i16 = .{ 0, 0 },
};

channels: [2]Channel,
stereo: Stereo = .{},
n_channels_api: usize = 0,
n_channels_internal: usize = 0,
prev_decode_only_middle: bool = false,

/// `silk_InitDecoder`.
pub fn init() Decoder {
    return .{ .channels = .{ .init(), .init() } };
}

/// `silk_ResetDecoder`.
pub fn reset(d: *Decoder) void {
    for (&d.channels) |*ch| ch.reset();
    d.stereo = .{};
    d.prev_decode_only_middle = false;
}

/// One SILK frame, 10 or 20 ms, into `out`, interleaved at the API rate
/// and channels. Returns the samples a channel. `silk_Decode`.
pub fn decode(
    d: *Decoder,
    ctrl: *Control,
    lost: Channel.LostFlag,
    new_packet: bool,
    dec: *RangeDecoder,
    out: []i16,
) error{ InvalidFrameSize }!usize {
    const chs = &d.channels;
    var decode_only_middle = false;
    var ms_pred_q13 = [2]i32{ 0, 0 };

    if (new_packet) {
        for (chs[0..ctrl.n_channels_internal]) |*ch| ch.n_frames_decoded = 0;
    }
    // Mono to stereo in the stream: the second channel starts afresh.
    if (ctrl.n_channels_internal > d.n_channels_internal) {
        chs[1] = .init();
    }

    const stereo_to_mono = ctrl.n_channels_internal == 1
        and d.n_channels_internal == 2
        and ctrl.internal_sample_rate == 1000 * chs[0].fs_khz;

    if (chs[0].n_frames_decoded == 0) {
        for (chs[0..ctrl.n_channels_internal]) |*ch| {
            switch (ctrl.payload_size_ms) {
                // A lost packet is taken as 10 ms.
                0, 10 => {
                    ch.n_frames_per_packet = 1;
                    ch.nb_subfr = 2;
                },
                20 => {
                    ch.n_frames_per_packet = 1;
                    ch.nb_subfr = 4;
                },
                40 => {
                    ch.n_frames_per_packet = 2;
                    ch.nb_subfr = 4;
                },
                60 => {
                    ch.n_frames_per_packet = 3;
                    ch.nb_subfr = 4;
                },
                else => return error.InvalidFrameSize,
            }
            const fs_khz_dec = (ctrl.internal_sample_rate >> 10) + 1;
            ch.setFs(fs_khz_dec, ctrl.api_sample_rate);
        }
    }

    if (
        ctrl.n_channels_api == 2
            and ctrl.n_channels_internal == 2
            and (d.n_channels_api == 1 or d.n_channels_internal == 1)
    ) {
        d.stereo.pred_prev_q13 = .{ 0, 0 };
        d.stereo.s_side = .{ 0, 0 };
        chs[1].resampler = chs[0].resampler;
    }
    d.n_channels_api = ctrl.n_channels_api;
    d.n_channels_internal = ctrl.n_channels_internal;

    if (lost != .packet_lost and chs[0].n_frames_decoded == 0) {
        // The first call for this packet: the voice activity and
        // redundancy flags of all its frames.
        for (chs[0..ctrl.n_channels_internal]) |*ch| {
            for (0..ch.n_frames_per_packet) |i|
                ch.vad_flags[i] = dec.decodeBitLogp(1);
            ch.lbrr_flag = dec.decodeBitLogp(1);
        }
        for (chs[0..ctrl.n_channels_internal]) |*ch| {
            ch.lbrr_flags = @splat(false);
            if (ch.lbrr_flag) {
                if (ch.n_frames_per_packet == 1) {
                    ch.lbrr_flags[0] = true;
                } else {
                    const symbol = dec.decodeIcdf(
                        tables.lbrr_flags_icdf_ptr[ch.n_frames_per_packet - 2],
                        8,
                    )
                        + 1;
                    for (0..ch.n_frames_per_packet) |i|
                        ch.lbrr_flags[i] = (symbol >> @intCast(i)) & 1 != 0;
                }
            }
        }
        if (lost == .decode_normal) {
            // The redundant copies are for a decoder that lost the last
            // packet. This one skips them.
            for (0..chs[0].n_frames_per_packet) |i| {
                for (0..ctrl.n_channels_internal) |n| {
                    if (!chs[n].lbrr_flags[i]) continue;
                    if (ctrl.n_channels_internal == 2 and n == 0) {
                        decodeStereoPred(dec, &ms_pred_q13);
                        if (!chs[1].lbrr_flags[i])
                            decode_only_middle = decodeMidOnly(dec);
                    }
                    const cond: Channel.CondCoding = if (
                        i > 0
                            and chs[n].lbrr_flags[i - 1]
                    )
                        .conditionally
                    else
                        .independently;
                    chs[n].decodeIndices(dec, i, true, cond);
                    var pulses: [max_frame_length + 16]i16 = undefined;
                    Channel.decodePulses(
                        dec,
                        &pulses,
                        chs[n].indices.signal_type,
                        chs[n].indices.quant_offset_type,
                        chs[n].frame_length,
                    );
                }
            }
        }
    }

    // The mid-side predictor.
    if (ctrl.n_channels_internal == 2) {
        const fi = chs[0].n_frames_decoded;
        if (
            lost == .decode_normal
                or (lost == .decode_lbrr and chs[0].lbrr_flags[fi])
        ) {
            decodeStereoPred(dec, &ms_pred_q13);
            // For redundancy, a mid-only flag only if the side has none.
            if (
                (lost == .decode_normal and !chs[1].vad_flags[fi])
                    or (lost == .decode_lbrr and !chs[1].lbrr_flags[fi])
            ) {
                decode_only_middle = decodeMidOnly(dec);
            } else {
                decode_only_middle = false;
            }
        } else {
            ms_pred_q13 = d.stereo.pred_prev_q13;
        }
    }

    // The first frame with a side after one without: its prediction starts
    // afresh.
    if (
        ctrl.n_channels_internal == 2
            and !decode_only_middle
            and d.prev_decode_only_middle
    ) {
        const side = &chs[1];
        @memset(&side.out_buf, 0);
        @memset(&side.slpc_q14_buf, 0);
        side.lag_prev = 100;
        side.last_gain_index = 10;
        side.prev_signal_type = Channel.SignalType.no_voice_activity;
        side.first_frame_after_reset = true;
    }

    // Each channel's output, with two samples of history in front for the
    // stereo filter.
    var tmp: [2][max_frame_length + 2]i16 = undefined;

    const has_side = if (lost == .decode_normal)
        !decode_only_middle
    else
        !d.prev_decode_only_middle
            or (ctrl.n_channels_internal == 2
                and lost == .decode_lbrr
                and chs[1].lbrr_flags[chs[1].n_frames_decoded]);

    var n_samples_out_dec: usize = 0;
    for (0..ctrl.n_channels_internal) |n| {
        if (n == 0 or has_side) {
            const frame_index = @as(isize, @intCast(chs[0].n_frames_decoded))
                - @as(isize, @intCast(n));
            const cond: Channel.CondCoding = if (frame_index <= 0)
                .independently
            else if (lost == .decode_lbrr)
                (if (chs[n].lbrr_flags[@intCast(frame_index - 1)])
                    .conditionally
                else
                    .independently)
            else if (
                n > 0
                    and d.prev_decode_only_middle
            )
            // A side frame skipped in this packet: its LTP state is
            // well defined, so it needs no scaling.
                .independently_no_ltp_scaling
            else
                .conditionally;

            chs[n].decodeFrame(dec, tmp[n][2..], lost, cond);
            n_samples_out_dec = chs[n].frame_length;
        } else {
            @memset(tmp[n][2..][0..n_samples_out_dec], 0);
        }
        chs[n].n_frames_decoded += 1;
    }

    if (ctrl.n_channels_api == 2 and ctrl.n_channels_internal == 2) {
        msToLr(
            &d.stereo,
            &tmp[0],
            &tmp[1],
            ms_pred_q13,
            chs[0].fs_khz,
            n_samples_out_dec,
        );
    } else {
        tmp[0][0..2].* = d.stereo.s_mid;
        d.stereo.s_mid = tmp[0][n_samples_out_dec..][0..2].*;
    }

    const n_samples_out: usize = @intCast(
        @divTrunc(
            @as(i32, @intCast(n_samples_out_dec)) * ctrl.api_sample_rate,
            chs[0].fs_khz * 1000,
        ),
    );

    var resampled: [960]i16 = undefined;
    for (0..@min(ctrl.n_channels_api, ctrl.n_channels_internal)) |n| {
        if (ctrl.n_channels_api == 2) {
            chs[n]
                .resampler.process(
                    resampled[0..n_samples_out],
                    tmp[n][1..][0..n_samples_out_dec],
                );
            for (0..n_samples_out) |i| out[n + 2 * i] = resampled[i];
        } else {
            chs[n]
                .resampler.process(
                    out[0..n_samples_out],
                    tmp[n][1..][0..n_samples_out_dec],
                );
        }
    }

    // Two channels out of a mono stream.
    if (ctrl.n_channels_api == 2 and ctrl.n_channels_internal == 1) {
        if (stereo_to_mono) {
            // The right channel's resampler, in case the stream had not
            // been collapsing to mono before.
            chs[1]
                .resampler.process(
                    resampled[0..n_samples_out],
                    tmp[0][1..][0..n_samples_out_dec],
                );
            for (0..n_samples_out) |i| out[1 + 2 * i] = resampled[i];
        } else {
            for (0..n_samples_out) |i| out[1 + 2 * i] = out[2 * i];
        }
    }

    // The pitch lag at 48 kHz.
    if (chs[0].prev_signal_type == Channel.SignalType.voiced) {
        const mult = [3]i32{ 6, 4, 3 };
        ctrl.prev_pitch_lag = chs[0].lag_prev
            * mult[@intCast((chs[0].fs_khz - 8) >> 2)];
    } else {
        ctrl.prev_pitch_lag = 0;
    }

    if (lost == .packet_lost) {
        // No gain clamping after a loss, so that the energy does not
        // bounce back if packets go missing while it is falling.
        for (chs[0..d.n_channels_internal]) |*ch| ch.last_gain_index = 10;
    } else {
        d.prev_decode_only_middle = decode_only_middle;
    }
    return n_samples_out;
}

/// The mid-side predictors. `silk_stereo_decode_pred`.
fn decodeStereoPred(dec: *RangeDecoder, pred_q13: *[2]i32) void {
    var ix: [2][3]i32 = undefined;
    const n: i32 = @intCast(dec.decodeIcdf(&tables.stereo_pred_joint_icdf, 8));
    ix[0][2] = @divTrunc(n, 5);
    ix[1][2] = n - 5 * ix[0][2];
    for (0..2) |k| {
        ix[k][0] = @intCast(dec.decodeIcdf(&tables.uniform3_icdf, 8));
        ix[k][1] = @intCast(dec.decodeIcdf(&tables.uniform5_icdf, 8));
    }
    // 0.5 / STEREO_QUANT_SUB_STEPS in Q16.
    const half_step_q16: i32 = @intFromFloat(0.5 / 5.0 * 65536.0 + 0.5);
    for (0..2) |k| {
        ix[k][0] += 3 * ix[k][2];
        const low_q13: i32 = tables.stereo_pred_quant_q13[@intCast(ix[k][0])];
        const step_q13 = fix.smulwb(
            @as(i32, tables.stereo_pred_quant_q13[@intCast(ix[k][0] + 1)])
                - low_q13,
            half_step_q16,
        );
        pred_q13[k] = fix.smlabb(low_q13, step_q13, 2 * ix[k][1] + 1);
    }
    // The second subtracted from the first, which suits applying them.
    pred_q13[0] -= pred_q13[1];
}

/// Whether only the mid channel was coded. `silk_stereo_decode_mid_only`.
fn decodeMidOnly(dec: *RangeDecoder) bool {
    return dec.decodeIcdf(&tables.stereo_only_code_mid_icdf, 8) != 0;
}

/// Mid and side, each with two samples of history in front, to left and
/// right. `silk_stereo_MS_to_LR`.
fn msToLr(
    st: *Stereo,
    x1: []i16,
    x2: []i16,
    pred_q13: [2]i32,
    fs_khz: i32,
    frame_length: usize,
) void {
    x1[0..2].* = st.s_mid;
    x2[0..2].* = st.s_side;
    st.s_mid = x1[frame_length..][0..2].*;
    st.s_side = x2[frame_length..][0..2].*;

    // The predictors, interpolated over the first 8 ms.
    var pred0 = st.pred_prev_q13[0];
    var pred1 = st.pred_prev_q13[1];
    const interp_len: usize = @intCast(stereo_interp_len_ms * fs_khz);
    const denom_q16 = @divTrunc(
        @as(i32, 1) << 16,
        stereo_interp_len_ms * fs_khz,
    );
    const delta0 = fix.rshiftRound(
        fix.smulbb(pred_q13[0] - st.pred_prev_q13[0], denom_q16),
        16,
    );
    const delta1 = fix.rshiftRound(
        fix.smulbb(pred_q13[1] - st.pred_prev_q13[1], denom_q16),
        16,
    );
    for (0..frame_length) |n| {
        if (n < interp_len) {
            pred0 += delta0;
            pred1 += delta1;
        } else if (n == interp_len) {
            pred0 = pred_q13[0];
            pred1 = pred_q13[1];
        }
        var sum = fix.lshift(
            (@as(i32, x1[n]) + x1[n + 2]) +% fix.lshift(@as(i32, x1[n + 1]), 1),
            9,
        );
        sum = fix.smlawb(fix.lshift(@as(i32, x2[n + 1]), 8), sum, pred0);
        sum = fix.smlawb(sum, fix.lshift(@as(i32, x1[n + 1]), 11), pred1);
        x2[n + 1] = @intCast(fix.sat16(fix.rshiftRound(sum, 8)));
    }
    // Kept, as libopus keeps them, in 16 bits.
    st.pred_prev_q13 = .{ fix.lo16(pred_q13[0]), fix.lo16(pred_q13[1]) };

    for (0..frame_length) |n| {
        const sum = @as(i32, x1[n + 1]) + x2[n + 1];
        const diff = @as(i32, x1[n + 1]) - x2[n + 1];
        x1[n + 1] = @intCast(fix.sat16(sum));
        x2[n + 1] = @intCast(fix.sat16(diff));
    }
}
