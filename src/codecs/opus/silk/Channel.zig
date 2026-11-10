// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2006-2011 Skype Limited
// SPDX-License-Identifier: MIT AND BSD-3-Clause

//! One channel of a SILK decoder: the mid or the side of a stereo stream,
//! or the only one. RFC 6716, Section 4.2, following libopus's
//! `silk_decoder_state` and the files that work on it: `decode_frame.c`,
//! `decode_indices.c`, `decode_pulses.c`, `shell_coder.c`, `code_signs.c`,
//! `gain_quant.c`, `NLSF_decode.c`, `NLSF_unpack.c`, `decode_pitch.c`,
//! `decode_parameters.c`, `decode_core.c`, `PLC.c`, `CNG.c`,
//! `init_decoder.c` and `decoder_set_fs.c`.
//!
//! A frame is side information -- signal type, gains, the spectral
//! envelope as line spectral frequencies, pitch and its filter -- and an
//! excitation of pulses. The excitation goes through the long-term
//! (pitch) predictor and then the short-term (LPC) one to make speech.

const Channel = @This();

const std = @import("std");
const RangeDecoder = @import("../RangeDecoder.zig");
const fix = @import("fix.zig");
const lpc = @import("lpc.zig");
const tables = @import("tables.zig");
const Resampler = @import("Resampler.zig");

pub const max_frames_per_packet = 3;
pub const max_nb_subfr = 4;
pub const max_fs_khz = 16;
pub const max_frame_length = 20 * max_fs_khz;
pub const max_sub_frame_length = 5 * max_fs_khz;
pub const max_lpc_order = lpc.max_order;
pub const ltp_order = 5;
const shell_codec_frame_length = 16;
const max_nb_shell_blocks = max_frame_length / shell_codec_frame_length;
const silk_max_pulses = 16;
const n_rate_levels = 10;
const nlsf_quant_max_amplitude = 4;

pub const SignalType = struct {
    pub const no_voice_activity = 0;
    pub const unvoiced = 1;
    pub const voiced = 2;
};

/// How a frame's parameters are coded: on their own, or relative to the
/// frame before.
pub const CondCoding = enum {
    independently,
    independently_no_ltp_scaling,
    conditionally,
};

/// What a call is to do: decode, conceal a lost frame, or decode the
/// redundant copy of one.
pub const LostFlag = enum { decode_normal, packet_lost, decode_lbrr };

/// A frame's side information, as coded. `SideInfoIndices`.
pub const Indices = struct {
    gains_indices: [max_nb_subfr]i32 = @splat(0),
    ltp_index: [max_nb_subfr]i32 = @splat(0),
    nlsf_indices: [max_lpc_order + 1]i32 = @splat(0),
    lag_index: i32 = 0,
    contour_index: i32 = 0,
    signal_type: i32 = 0,
    quant_offset_type: i32 = 0,
    nlsf_interp_coef_q2: i32 = 0,
    per_index: i32 = 0,
    ltp_scale_index: i32 = 0,
    seed: i32 = 0,
};

/// A frame's parameters, decoded. `silk_decoder_control`.
pub const Control = struct {
    pitch_l: [max_nb_subfr]i32 = @splat(0),
    gains_q16: [max_nb_subfr]i32 = @splat(0),
    /// For the first half of the frame and the second.
    pred_coef_q12: [2][max_lpc_order]i16 = @splat(@splat(0)),
    ltp_coef_q14: [ltp_order * max_nb_subfr]i16 = @splat(0),
    ltp_scale_q14: i32 = 0,
};

/// Packet loss concealment. `silk_PLC_struct`.
const Plc = struct {
    pitch_l_q8: i32 = 0,
    ltp_coef_q14: [ltp_order]i16 = @splat(0),
    prev_lpc_q12: [max_lpc_order]i16 = @splat(0),
    last_frame_lost: bool = false,
    rand_seed: i32 = 0,
    rand_scale_q14: i32 = 0,
    conc_energy: i32 = 0,
    conc_energy_shift: i32 = 0,
    prev_ltp_scale_q14: i32 = 0,
    prev_gain_q16: [2]i32 = @splat(0),
    fs_khz: i32 = 0,
    nb_subfr: usize = 0,
    subfr_length: usize = 0,
};

/// Comfort noise. `silk_CNG_struct`.
const Cng = struct {
    exc_buf_q14: [max_frame_length]i32 = @splat(0),
    smth_nlsf_q15: [max_lpc_order]i16 = @splat(0),
    synth_state: [max_lpc_order]i32 = @splat(0),
    smth_gain_q16: i32 = 0,
    rand_seed: i32 = 0,
    fs_khz: i32 = 0,
};

prev_gain_q16: i32 = 65536,
exc_q14: [max_frame_length]i32 = @splat(0),
slpc_q14_buf: [max_lpc_order]i32 = @splat(0),
out_buf: [max_frame_length + 2 * max_sub_frame_length]i16 = @splat(0),
lag_prev: i32 = 0,
last_gain_index: i32 = 0,
fs_khz: i32 = 0,
fs_api_hz: i32 = 0,
nb_subfr: usize = 0,
frame_length: usize = 0,
subfr_length: usize = 0,
ltp_mem_length: usize = 0,
lpc_order: usize = 0,
prev_nlsf_q15: [max_lpc_order]i16 = @splat(0),
first_frame_after_reset: bool = true,
pitch_lag_low_bits_icdf: []const u8 = &.{},
pitch_contour_icdf: []const u8 = &.{},
n_frames_decoded: usize = 0,
n_frames_per_packet: usize = 0,
ec_prev_signal_type: i32 = 0,
ec_prev_lag_index: i32 = 0,
vad_flags: [max_frames_per_packet]bool = @splat(false),
lbrr_flag: bool = false,
lbrr_flags: [max_frames_per_packet]bool = @splat(false),
resampler: Resampler = .{},
nlsf_cb: *const tables.NlsfCodebook = &tables.nlsf_cb_nb_mb,
indices: Indices = .{},
cng: Cng = .{},
loss_cnt: i32 = 0,
prev_signal_type: i32 = 0,
plc: Plc = .{},

/// A channel as `silk_init_decoder` leaves it.
pub fn init() Channel {
    var ch: Channel = .{};
    ch.reset();
    return ch;
}

/// Back to the start, as after `init`. `silk_reset_decoder`.
pub fn reset(ch: *Channel) void {
    ch.* = .{};
    ch.first_frame_after_reset = true;
    ch.prev_gain_q16 = 65536;
    ch.cngReset();
    ch.plcReset();
}

/// The internal rate, in kHz, and the rate asked for, in Hz.
/// `silk_decoder_set_fs`.
pub fn setFs(ch: *Channel, fs_khz: i32, fs_api_hz: i32) void {
    std.debug.assert(fs_khz == 8 or fs_khz == 12 or fs_khz == 16);
    ch.subfr_length = @intCast(5 * fs_khz);
    const frame_length = ch.nb_subfr * ch.subfr_length;

    if (ch.fs_khz != fs_khz or ch.fs_api_hz != fs_api_hz) {
        ch.resampler = .init(fs_khz * 1000, fs_api_hz);
        ch.fs_api_hz = fs_api_hz;
    }

    if (ch.fs_khz != fs_khz or frame_length != ch.frame_length) {
        if (fs_khz == 8) {
            ch.pitch_contour_icdf = if (ch.nb_subfr == max_nb_subfr)
                &tables.pitch_contour_nb_icdf
            else
                &tables.pitch_contour_10_ms_nb_icdf;
        } else {
            ch.pitch_contour_icdf = if (ch.nb_subfr == max_nb_subfr)
                &tables.pitch_contour_icdf
            else
                &tables.pitch_contour_10_ms_icdf;
        }
        if (ch.fs_khz != fs_khz) {
            ch.ltp_mem_length = @intCast(20 * fs_khz);
            if (fs_khz == 8 or fs_khz == 12) {
                ch.lpc_order = 10;
                ch.nlsf_cb = &tables.nlsf_cb_nb_mb;
            } else {
                ch.lpc_order = max_lpc_order;
                ch.nlsf_cb = &tables.nlsf_cb_wb;
            }
            ch.pitch_lag_low_bits_icdf = switch (fs_khz) {
                16 => &tables.uniform8_icdf,
                12 => &tables.uniform6_icdf,
                else => &tables.uniform4_icdf,
            };
            ch.first_frame_after_reset = true;
            ch.lag_prev = 100;
            ch.last_gain_index = 10;
            ch.prev_signal_type = SignalType.no_voice_activity;
            @memset(&ch.out_buf, 0);
            @memset(&ch.slpc_q14_buf, 0);
        }
        ch.fs_khz = fs_khz;
        ch.frame_length = frame_length;
    }
}

/// One frame, decoded or concealed, into `out`, `frame_length` samples.
/// `silk_decode_frame`.
pub fn decodeFrame(
    ch: *Channel,
    dec: *RangeDecoder,
    out: []i16,
    lost: LostFlag,
    cond_coding: CondCoding,
) void {
    const l = ch.frame_length;
    var ctrl: Control = .{};
    if (
        lost == .decode_normal
            or (lost == .decode_lbrr and ch.lbrr_flags[ch.n_frames_decoded])
    ) {
        var pulses: [
            max_frame_length + shell_codec_frame_length
        ]i16 = undefined;
        ch.decodeIndices(
            dec,
            ch.n_frames_decoded,
            lost == .decode_lbrr,
            cond_coding,
        );
        decodePulses(
            dec,
            &pulses,
            ch.indices.signal_type,
            ch.indices.quant_offset_type,
            l,
        );
        ch.decodeParameters(&ctrl, cond_coding);
        ch.decodeCore(&ctrl, out[0..l], pulses[0..l]);
        ch.updateOutBuf(out[0..l]);
        ch.plcUpdateIfRateChanged();
        ch.plcUpdate(&ctrl);

        ch.loss_cnt = 0;
        ch.prev_signal_type = ch.indices.signal_type;
        ch.first_frame_after_reset = false;
    } else {
        ch.plcUpdateIfRateChanged();
        ch.plcConceal(&ctrl, out[0..l]);
        ch.loss_cnt += 1;
        ch.updateOutBuf(out[0..l]);
    }
    ch.cngApply(&ctrl, out[0..l]);
    ch.plcGlueFrames(out[0..l]);
    ch.lag_prev = ctrl.pitch_l[ch.nb_subfr - 1];
}

fn updateOutBuf(ch: *Channel, out: []const i16) void {
    const mv_len = ch.ltp_mem_length - ch.frame_length;
    std.mem.copyForwards(
        i16,
        ch.out_buf[0..mv_len],
        ch.out_buf[ch.frame_length..][0..mv_len],
    );
    @memcpy(ch.out_buf[mv_len..][0..ch.frame_length], out);
}

// Side information.

/// The frame's side information. `silk_decode_indices`.
pub fn decodeIndices(
    ch: *Channel,
    dec: *RangeDecoder,
    frame_index: usize,
    decode_lbrr: bool,
    cond_coding: CondCoding,
) void {
    const ix: *Indices = &ch.indices;
    // Signal type and quantizer offset.
    var type_ix: i32 = if (decode_lbrr or ch.vad_flags[frame_index])
        @as(i32, @intCast(dec.decodeIcdf(&tables.type_offset_vad_icdf, 8))) + 2
    else
        @intCast(dec.decodeIcdf(&tables.type_offset_no_vad_icdf, 8));
    ix.signal_type = type_ix >> 1;
    ix.quant_offset_type = type_ix & 1;

    // Gains: the first on its own or as a delta, the rest as deltas.
    if (cond_coding == .conditionally) {
        ix.gains_indices[0] = @intCast(
            dec.decodeIcdf(&tables.delta_gain_icdf, 8),
        );
    } else {
        ix.gains_indices[0] = @as(
            i32,
            @intCast(
                dec.decodeIcdf(&tables.gain_icdf[@intCast(ix.signal_type)], 8),
            ),
        )
            << 3;
        ix.gains_indices[0] += @intCast(
            dec.decodeIcdf(&tables.uniform8_icdf, 8),
        );
    }
    for (1..ch.nb_subfr) |i|
        ix.gains_indices[i] = @intCast(
            dec.decodeIcdf(&tables.delta_gain_icdf, 8),
        );

    // The line spectral frequencies: a first-stage vector, and residuals.
    const cb = ch.nlsf_cb;
    const n_vectors: usize = @intCast(cb.n_vectors);
    ix.nlsf_indices[0] = @intCast(
        dec.decodeIcdf(
            cb.cb1_icdf[
                @as(usize, @intCast(ix.signal_type >> 1)) * n_vectors..
            ],
            8,
        ),
    );
    var ec_ix: [max_lpc_order]i32 = undefined;
    var pred_q8: [max_lpc_order]u8 = undefined;
    nlsfUnpack(&ec_ix, &pred_q8, cb, @intCast(ix.nlsf_indices[0]));
    for (0..@intCast(cb.order)) |i| {
        type_ix = @intCast(dec.decodeIcdf(cb.ec_icdf[@intCast(ec_ix[i])..], 8));
        if (type_ix == 0) {
            type_ix -= @intCast(dec.decodeIcdf(&tables.nlsf_ext_icdf, 8));
        } else if (type_ix == 2 * nlsf_quant_max_amplitude) {
            type_ix += @intCast(dec.decodeIcdf(&tables.nlsf_ext_icdf, 8));
        }
        ix.nlsf_indices[i + 1] = type_ix - nlsf_quant_max_amplitude;
    }
    ix.nlsf_interp_coef_q2 = if (ch.nb_subfr == max_nb_subfr)
        @intCast(dec.decodeIcdf(&tables.nlsf_interpolation_factor_icdf, 8))
    else
        4;

    if (ix.signal_type == SignalType.voiced) {
        // The pitch lag: relative to the last, or on its own.
        var absolute = true;
        if (
            cond_coding == .conditionally
                and ch.ec_prev_signal_type == SignalType.voiced
        ) {
            const delta: i32 = @intCast(
                dec.decodeIcdf(&tables.pitch_delta_icdf, 8),
            );
            if (delta > 0) {
                ix.lag_index = @as(
                    i16,
                    @truncate(ch.ec_prev_lag_index + (delta - 9)),
                );
                absolute = false;
            }
        }
        if (absolute) {
            ix.lag_index = @as(
                i32,
                @intCast(dec.decodeIcdf(&tables.pitch_lag_icdf, 8)),
            )
                * (ch.fs_khz >> 1);
            ix.lag_index += @intCast(
                dec.decodeIcdf(ch.pitch_lag_low_bits_icdf, 8),
            );
        }
        ch.ec_prev_lag_index = ix.lag_index;
        ix.contour_index = @intCast(dec.decodeIcdf(ch.pitch_contour_icdf, 8));

        // The long-term predictor's codebook and its vectors.
        ix.per_index = @intCast(dec.decodeIcdf(&tables.ltp_per_index_icdf, 8));
        for (0..ch.nb_subfr) |k| {
            ix.ltp_index[k] = @intCast(
                dec.decodeIcdf(
                    tables.ltp_gain_icdf_ptrs[@intCast(ix.per_index)],
                    8,
                ),
            );
        }
        ix.ltp_scale_index = if (cond_coding == .independently)
            @intCast(dec.decodeIcdf(&tables.ltpscale_icdf, 8))
        else
            0;
    }
    ch.ec_prev_signal_type = ix.signal_type;
    ix.seed = @intCast(dec.decodeIcdf(&tables.uniform4_icdf, 8));
}

/// The entropy tables and predictor for the first-stage vector `cb1_index`.
/// `silk_NLSF_unpack`.
pub fn nlsfUnpack(
    ec_ix: []i32,
    pred_q8: []u8,
    cb: *const tables.NlsfCodebook,
    cb1_index: usize,
) void {
    const order: usize = @intCast(cb.order);
    const sel = cb.ec_sel[cb1_index * order / 2..];
    var i: usize = 0;
    while (i < order) : (i += 2) {
        const entry: i32 = sel[i / 2];
        ec_ix[i] = ((entry >> 1) & 7) * (2 * nlsf_quant_max_amplitude + 1);
        pred_q8[i] = cb.pred_q8[
            i + @as(usize, @intCast(entry & 1)) * (order - 1)
        ];
        ec_ix[i + 1] = ((entry >> 5) & 7) * (2 * nlsf_quant_max_amplitude + 1);
        pred_q8[i + 1] = cb.pred_q8[
            i + @as(usize, @intCast((entry >> 4) & 1)) * (order - 1) + 1
        ];
    }
}

// Excitation.

/// The excitation's pulses, `frame_length` of them, rounded up to whole
/// shell blocks. `silk_decode_pulses`.
pub fn decodePulses(
    dec: *RangeDecoder,
    pulses: []i16,
    signal_type: i32,
    quant_offset_type: i32,
    frame_length: usize,
) void {
    const rate_level = dec.decodeIcdf(
        &tables.rate_levels_icdf[@intCast(signal_type >> 1)],
        8,
    );
    var iter = frame_length >> 4;
    // Only 10 ms at 12 kHz is not a whole number of blocks.
    if (iter * shell_codec_frame_length < frame_length) iter += 1;

    var sum_pulses: [max_nb_shell_blocks]i32 = undefined;
    var n_lshifts: [max_nb_shell_blocks]i32 = undefined;
    for (0..iter) |i| {
        n_lshifts[i] = 0;
        sum_pulses[i] = @intCast(
            dec.decodeIcdf(&tables.pulses_per_block_icdf[rate_level], 8),
        );
        // More than the table allows: another bit of each pulse, below.
        while (sum_pulses[i] == silk_max_pulses + 1) {
            n_lshifts[i] += 1;
            // After ten, the table is shifted so that it cannot happen again.
            const skip: usize = @intFromBool(n_lshifts[i] == 10);
            sum_pulses[i] = @intCast(
                dec.decodeIcdf(
                    tables.pulses_per_block_icdf[n_rate_levels - 1][skip..],
                    8,
                ),
            );
        }
    }

    for (0..iter) |i| {
        const block = pulses[i * shell_codec_frame_length..][
            0..shell_codec_frame_length
        ];
        if (sum_pulses[i] > 0) {
            shellDecoder(block, dec, sum_pulses[i]);
        } else {
            @memset(block, 0);
        }
    }

    for (0..iter) |i| {
        if (n_lshifts[i] > 0) {
            const n_ls = n_lshifts[i];
            const block = pulses[i * shell_codec_frame_length..][
                0..shell_codec_frame_length
            ];
            for (block) |*p| {
                var abs_q: i32 = p.*;
                for (0..@intCast(n_ls)) |_| {
                    abs_q = fix.lshift(abs_q, 1);
                    abs_q += @intCast(dec.decodeIcdf(&tables.lsb_icdf, 8));
                }
                p.* = @truncate(abs_q);
            }
            // So that the signs are decoded.
            sum_pulses[i] |= n_ls << 5;
        }
    }

    decodeSigns(
        dec,
        pulses,
        frame_length,
        signal_type,
        quant_offset_type,
        &sum_pulses,
    );
}

/// How `p` pulses split between two halves, one level of the tree.
fn decodeSplit(dec: *RangeDecoder, p: i32, table: []const u8) [2]i16 {
    if (p > 0) {
        const c1: i32 = @intCast(
            dec.decodeIcdf(
                table[tables.shell_code_table_offsets[@intCast(p)]..],
                8,
            ),
        );
        return .{ @intCast(c1), @intCast(p - c1) };
    }
    return .{ 0, 0 };
}

/// Sixteen pulse magnitudes adding up to `pulses4`, by halving.
/// `silk_shell_decoder`.
fn shellDecoder(pulses0: []i16, dec: *RangeDecoder, pulses4: i32) void {
    var pulses3: [2]i16 = undefined;
    var pulses2: [4]i16 = undefined;
    var pulses1: [8]i16 = undefined;
    const t0 = &tables.shell_code_table0;
    const t1 = &tables.shell_code_table1;
    const t2 = &tables.shell_code_table2;
    const t3 = &tables.shell_code_table3;
    pulses3 = decodeSplit(dec, pulses4, t3);
    pulses2[0..2].* = decodeSplit(dec, pulses3[0], t2);
    pulses1[0..2].* = decodeSplit(dec, pulses2[0], t1);
    pulses0[0..2].* = decodeSplit(dec, pulses1[0], t0);
    pulses0[2..4].* = decodeSplit(dec, pulses1[1], t0);
    pulses1[2..4].* = decodeSplit(dec, pulses2[1], t1);
    pulses0[4..6].* = decodeSplit(dec, pulses1[2], t0);
    pulses0[6..8].* = decodeSplit(dec, pulses1[3], t0);
    pulses2[2..4].* = decodeSplit(dec, pulses3[1], t2);
    pulses1[4..6].* = decodeSplit(dec, pulses2[2], t1);
    pulses0[8..10].* = decodeSplit(dec, pulses1[4], t0);
    pulses0[10..12].* = decodeSplit(dec, pulses1[5], t0);
    pulses1[6..8].* = decodeSplit(dec, pulses2[3], t1);
    pulses0[12..14].* = decodeSplit(dec, pulses1[6], t0);
    pulses0[14..16].* = decodeSplit(dec, pulses1[7], t0);
}

/// A sign for each pulse that is not zero. `silk_decode_signs`.
fn decodeSigns(
    dec: *RangeDecoder,
    pulses: []i16,
    length: usize,
    signal_type: i32,
    quant_offset_type: i32,
    sum_pulses: []const i32,
) void {
    var icdf = [2]u8{ 0, 0 };
    const base: usize = @intCast(7 * (quant_offset_type + (signal_type << 1)));
    const icdf_ptr = tables.sign_icdf[base..];
    const blocks = (length + shell_codec_frame_length / 2) >> 4;
    for (0..blocks) |i| {
        const p = sum_pulses[i];
        if (p > 0) {
            icdf[0] = icdf_ptr[@intCast(@min(p & 0x1f, 6))];
            for (
                pulses[i * shell_codec_frame_length..][
                    0..shell_codec_frame_length
                ],
            ) |*q| {
                if (q.* > 0) {
                    const bit: i16 = @intCast(dec.decodeIcdf(&icdf, 8));
                    q.* *= 2 * bit - 1;
                }
            }
        }
    }
}

// Parameters.

pub const gain_offset = (2 * 128) / 6 + 16 * 128;
pub const inv_scale_q16 = (65536 * (((88 - 2) * 128) / 6)) / (64 - 1);

/// The gains, from their indices. `silk_gains_dequant`.
pub fn gainsDequant(
    gain_q16: []i32,
    ind: []const i32,
    prev_ind: *i32,
    conditional: bool,
) void {
    for (gain_q16, ind[0..gain_q16.len], 0..) |*g, index, k| {
        if (k == 0 and !conditional) {
            // Down at most 16 steps, about 21.8 dB.
            prev_ind.* = @max(index, prev_ind.* - 16);
        } else {
            const ind_tmp = index - 4;
            const double_step_size_threshold = 2 * 36 - 64 + prev_ind.*;
            if (ind_tmp > double_step_size_threshold) {
                prev_ind.* += fix.lshift(ind_tmp, 1)
                    - double_step_size_threshold;
            } else {
                prev_ind.* += ind_tmp;
            }
        }
        prev_ind.* = fix.limit(prev_ind.*, 0, 63);
        g.* = fix.log2lin(
            @min(fix.smulwb(inv_scale_q16, prev_ind.*) + gain_offset, 3967),
        );
    }
}

/// The line spectral frequencies, from the codebook and the residuals.
/// `silk_NLSF_decode`.
pub fn nlsfDecode(
    nlsf_q15: []i16,
    indices: []const i32,
    cb: *const tables.NlsfCodebook,
) void {
    const order: usize = @intCast(cb.order);
    var ec_ix: [max_lpc_order]i32 = undefined;
    var pred_q8: [max_lpc_order]u8 = undefined;
    nlsfUnpack(&ec_ix, &pred_q8, cb, @intCast(indices[0]));

    // The residuals, each predicted from the one after.
    var res_q10: [max_lpc_order]i16 = undefined;
    var out_q10: i32 = 0;
    var i = order;
    while (i > 0) {
        i -= 1;
        const pred_q10 = fix.smulbb(out_q10, pred_q8[i]) >> 8;
        out_q10 = fix.lshift(indices[1 + i], 10);
        // NLSF_QUANT_LEVEL_ADJ, 0.1, in Q10.
        if (out_q10 > 0) {
            out_q10 = out_q10 - 102;
        } else if (out_q10 < 0) {
            out_q10 = out_q10 + 102;
        }
        out_q10 = fix.smlawb(pred_q10, out_q10, cb.quant_step_size_q16);
        res_q10[i] = @truncate(out_q10);
    }

    const idx0: usize = @intCast(indices[0]);
    const cb_element = cb.cb1_nlsf_q8[idx0 * order..];
    const cb_wght_q9 = cb.cb1_wght_q9[idx0 * order..];
    for (0..order) |k| {
        const tmp = @divTrunc(
            fix.lshift(@as(i32, res_q10[k]), 14),
            @as(i32, cb_wght_q9[k]),
        )
            +% fix.lshift(@as(i32, cb_element[k]), 7);
        nlsf_q15[k] = @intCast(fix.limit(tmp, 0, 32767));
    }
    lpc.nlsfStabilize(nlsf_q15[0..order], cb.delta_min_q15);
}

/// The pitch lags of each subframe. `silk_decode_pitch`.
fn decodePitch(
    lag_index: i32,
    contour_index: i32,
    pitch_lags: []i32,
    fs_khz: i32,
    nb_subfr: usize,
) void {
    const cbk: []const i8, const cbk_size: usize = if (fs_khz == 8)
        (if (nb_subfr == max_nb_subfr) .{
            @ptrCast(&tables.cb_lags_stage2),
            11,
        } else .{ @ptrCast(&tables.cb_lags_stage2_10_ms), 3 })
    else if (nb_subfr == max_nb_subfr) .{
        @ptrCast(&tables.cb_lags_stage3),
        34,
    } else .{ @ptrCast(&tables.cb_lags_stage3_10_ms), 12 };
    const min_lag = 2 * fs_khz;
    const max_lag = 18 * fs_khz;
    const lag = min_lag + lag_index;
    for (pitch_lags[0..nb_subfr], 0..) |*p, k| {
        p.* = fix.limit(
            lag + cbk[k * cbk_size + @as(usize, @intCast(contour_index))],
            min_lag,
            max_lag,
        );
    }
}

/// Gains, filters and pitch from the side information.
/// `silk_decode_parameters`.
fn decodeParameters(
    ch: *Channel,
    ctrl: *Control,
    cond_coding: CondCoding,
) void {
    const order = ch.lpc_order;
    gainsDequant(
        ctrl.gains_q16[0..ch.nb_subfr],
        &ch.indices.gains_indices,
        &ch.last_gain_index,
        cond_coding == .conditionally,
    );

    var nlsf_q15: [max_lpc_order]i16 = undefined;
    nlsfDecode(nlsf_q15[0..order], &ch.indices.nlsf_indices, ch.nlsf_cb);
    lpc.nlsf2a(ctrl.pred_coef_q12[1][0..order], nlsf_q15[0..order]);

    // No interpolation straight after a reset, which helps if the first
    // frame after a change of rate is lost.
    if (ch.first_frame_after_reset) ch.indices.nlsf_interp_coef_q2 = 4;
    if (ch.indices.nlsf_interp_coef_q2 < 4) {
        var nlsf0_q15: [max_lpc_order]i16 = undefined;
        for (0..order) |i| {
            nlsf0_q15[i] = @truncate(
                @as(i32, ch.prev_nlsf_q15[i])
                    + (fix.mul(
                        ch.indices.nlsf_interp_coef_q2,
                        @as(i32, nlsf_q15[i]) - ch.prev_nlsf_q15[i],
                    )
                        >> 2),
            );
        }
        lpc.nlsf2a(ctrl.pred_coef_q12[0][0..order], nlsf0_q15[0..order]);
    } else {
        @memcpy(
            ctrl.pred_coef_q12[0][0..order],
            ctrl.pred_coef_q12[1][0..order],
        );
    }
    @memcpy(ch.prev_nlsf_q15[0..order], nlsf_q15[0..order]);

    // Bandwidth expansion after a loss.
    if (ch.loss_cnt != 0) {
        lpc.bwexpander(ctrl.pred_coef_q12[0][0..order], 63570);
        lpc.bwexpander(ctrl.pred_coef_q12[1][0..order], 63570);
    }

    if (ch.indices.signal_type == SignalType.voiced) {
        decodePitch(
            ch.indices.lag_index,
            ch.indices.contour_index,
            &ctrl.pitch_l,
            ch.fs_khz,
            ch.nb_subfr,
        );
        const cbk = tables.ltp_vq_ptrs_q7[@intCast(ch.indices.per_index)];
        for (0..ch.nb_subfr) |k| {
            const v = cbk[@intCast(ch.indices.ltp_index[k])];
            for (0..ltp_order) |i|
                ctrl.ltp_coef_q14[k * ltp_order + i] = @as(i16, v[i]) << 7;
        }
        ctrl.ltp_scale_q14 = tables.ltpscales_table_q14[
            @intCast(ch.indices.ltp_scale_index)
        ];
    } else {
        @memset(ctrl.pitch_l[0..ch.nb_subfr], 0);
        @memset(ctrl.ltp_coef_q14[0..ltp_order * ch.nb_subfr], 0);
        ch.indices.per_index = 0;
        ctrl.ltp_scale_q14 = 0;
    }
}

// Synthesis.

/// The short-term prediction for `s[16 + i]` from the 16 before it.
inline fn lpcPredict(s: []const i32, i: usize, a: []const i16) i32 {
    var pred: i32 = @intCast(a.len >> 1);
    for (a, 0..) |c, j|
        pred = fix.smlawb(pred, s[max_lpc_order + i - j - 1], c);
    return pred;
}

/// The excitation through the long-term and short-term predictors, into
/// `xq`. `silk_decode_core`.
fn decodeCore(
    ch: *Channel,
    ctrl: *Control,
    xq: []i16,
    pulses: []const i16,
) void {
    const order = ch.lpc_order;
    var s_ltp: [max_frame_length]i16 = undefined;
    var s_ltp_q15: [2 * max_frame_length]i32 = undefined;
    var res_q14: [max_sub_frame_length]i32 = undefined;
    var s_lpc_q14: [max_sub_frame_length + max_lpc_order]i32 = undefined;

    const offset_q10: i32 = tables.quantization_offsets_q10[
        @intCast(ch.indices.signal_type >> 1)
    ][@intCast(ch.indices.quant_offset_type)];
    const nlsf_interpolation = ch.indices.nlsf_interp_coef_q2 < 4;

    // The excitation: the pulses, offset, with pseudo-random signs.
    var rand_seed: i32 = ch.indices.seed;
    for (0..ch.frame_length) |i| {
        rand_seed = fix.rand(rand_seed);
        var e = fix.lshift(@as(i32, pulses[i]), 14);
        if (e > 0) {
            e -= 80 << 4;
        } else if (e < 0) {
            e += 80 << 4;
        }
        e += offset_q10 << 4;
        if (rand_seed < 0) e = -e;
        ch.exc_q14[i] = e;
        rand_seed +%= pulses[i];
    }

    @memcpy(s_lpc_q14[0..max_lpc_order], &ch.slpc_q14_buf);
    var pexc: usize = 0;
    var pxq: usize = 0;
    var s_ltp_buf_idx = ch.ltp_mem_length;
    var lag: i32 = 0;
    for (0..ch.nb_subfr) |k| {
        var a_q12: [max_lpc_order]i16 = undefined;
        @memcpy(a_q12[0..order], ctrl.pred_coef_q12[k >> 1][0..order]);
        const b_q14 = ctrl.ltp_coef_q14[k * ltp_order..][0..ltp_order];
        var signal_type = ch.indices.signal_type;
        const gain_q10 = ctrl.gains_q16[k] >> 6;
        var inv_gain_q31 = fix.inverse32VarQ(ctrl.gains_q16[k], 47);

        // A change of gain rescales the short-term state.
        var gain_adj_q16: i32 = 1 << 16;
        if (ctrl.gains_q16[k] != ch.prev_gain_q16) {
            gain_adj_q16 = fix.div32VarQ(
                ch.prev_gain_q16,
                ctrl.gains_q16[k],
                16,
            );
            for (s_lpc_q14[0..max_lpc_order]) |*s|
                s.* = fix.smulww(gain_adj_q16, s.*);
        }
        ch.prev_gain_q16 = ctrl.gains_q16[k];

        // No abrupt change from voiced concealment to unvoiced decoding.
        if (
            ch.loss_cnt != 0
                and ch.prev_signal_type == SignalType.voiced
                and ch.indices.signal_type != SignalType.voiced
                and k < max_nb_subfr / 2
        ) {
            @memset(b_q14, 0);
            b_q14[ltp_order / 2] = 4096; // 0.25 in Q14
            signal_type = SignalType.voiced;
            ctrl.pitch_l[k] = ch.lag_prev;
        }

        if (signal_type == SignalType.voiced) {
            lag = ctrl.pitch_l[k];
            // Re-whitening, with new coefficients.
            if (k == 0 or (k == 2 and nlsf_interpolation)) {
                const lag_u: usize = @intCast(lag);
                const start_idx = ch.ltp_mem_length
                    - lag_u
                    - order
                    - ltp_order / 2;
                if (k == 2)
                    @memcpy(
                        ch.out_buf[ch.ltp_mem_length..][0..2 * ch.subfr_length],
                        xq[0..2 * ch.subfr_length],
                    );
                lpc.analysisFilter(
                    s_ltp[start_idx..ch.ltp_mem_length],
                    ch.out_buf[start_idx + k * ch.subfr_length..][
                        0..ch.ltp_mem_length - start_idx
                    ],
                    a_q12[0..order],
                );
                // Downscaled, on the first subframe, to limit how far one
                // packet depends on the last.
                if (k == 0)
                    inv_gain_q31 = fix.lshift(
                        fix.smulwb(inv_gain_q31, ctrl.ltp_scale_q14),
                        2,
                    );
                for (0..lag_u + ltp_order / 2) |i| {
                    s_ltp_q15[s_ltp_buf_idx - i - 1] = fix.smulwb(
                        inv_gain_q31,
                        s_ltp[ch.ltp_mem_length - i - 1],
                    );
                }
            } else if (gain_adj_q16 != 1 << 16) {
                for (0..@as(usize, @intCast(lag)) + ltp_order / 2) |i| {
                    const p = &s_ltp_q15[s_ltp_buf_idx - i - 1];
                    p.* = fix.smulww(gain_adj_q16, p.*);
                }
            }
        }

        // Long-term prediction.
        const pres: []const i32 = if (signal_type == SignalType.voiced) blk: {
            var pred_lag = s_ltp_buf_idx
                - @as(usize, @intCast(lag))
                + ltp_order / 2;
            for (0..ch.subfr_length) |i| {
                // Starting at 2 avoids the bias of smlawb's rounding down.
                var ltp_pred_q13: i32 = 2;
                for (0..ltp_order) |j|
                    ltp_pred_q13 = fix.smlawb(
                        ltp_pred_q13,
                        s_ltp_q15[pred_lag - j],
                        b_q14[j],
                    );
                pred_lag += 1;
                res_q14[i] = ch.exc_q14[pexc + i]
                    +% fix.lshift(ltp_pred_q13, 1);
                s_ltp_q15[s_ltp_buf_idx] = fix.lshift(res_q14[i], 1);
                s_ltp_buf_idx += 1;
            }
            break :blk res_q14[0..ch.subfr_length];
        } else ch.exc_q14[pexc..][0..ch.subfr_length];

        // Short-term prediction.
        for (0..ch.subfr_length) |i| {
            const pred = lpcPredict(&s_lpc_q14, i, a_q12[0..order]);
            s_lpc_q14[max_lpc_order + i] = fix.addSat32(
                pres[i],
                fix.lshiftSat32(pred, 4),
            );
            xq[pxq + i] = @intCast(
                fix.sat16(
                    fix.rshiftRound(
                        fix.smulww(s_lpc_q14[max_lpc_order + i], gain_q10),
                        8,
                    ),
                ),
            );
        }
        std.mem.copyForwards(
            i32,
            s_lpc_q14[0..max_lpc_order],
            s_lpc_q14[ch.subfr_length..][0..max_lpc_order],
        );
        pexc += ch.subfr_length;
        pxq += ch.subfr_length;
    }
    @memcpy(&ch.slpc_q14_buf, s_lpc_q14[0..max_lpc_order]);
}

// Packet loss concealment.

const harm_att_q15 = [2]i32{ 32440, 31130 };
const plc_rand_attenuate_v_q15 = [2]i32{ 31130, 26214 };
const plc_rand_attenuate_uv_q15 = [2]i32{ 32440, 29491 };
const bwe_coef_q16: i32 = 64881; // 0.99
const v_pitch_gain_start_min_q14 = 11469;
const v_pitch_gain_start_max_q14 = 15565;
const max_pitch_lag_ms = 18;
const rand_buf_size = 128;
const log2_inv_lpc_gain_high_thres = 3;
const log2_inv_lpc_gain_low_thres = 8;
const pitch_drift_fac_q16 = 655;

fn plcReset(ch: *Channel) void {
    ch.plc.pitch_l_q8 = fix.lshift(@as(i32, @intCast(ch.frame_length)), 8 - 1);
    ch.plc.prev_gain_q16 = .{ 1 << 16, 1 << 16 };
    ch.plc.subfr_length = 20;
    ch.plc.nb_subfr = 2;
}

/// `silk_PLC`'s check, before updating or concealing.
fn plcUpdateIfRateChanged(ch: *Channel) void {
    if (ch.fs_khz != ch.plc.fs_khz) {
        ch.plcReset();
        ch.plc.fs_khz = ch.fs_khz;
    }
}

/// What concealment will need, from a frame that arrived.
/// `silk_PLC_update`.
fn plcUpdate(ch: *Channel, ctrl: *const Control) void {
    const p = &ch.plc;
    ch.prev_signal_type = ch.indices.signal_type;
    var ltp_gain_q14: i32 = 0;
    if (ch.indices.signal_type == SignalType.voiced) {
        // The last subframe with a pitch pulse in it.
        var j: usize = 0;
        while (
            @as(i32, @intCast(j * ch.subfr_length))
                < ctrl.pitch_l[ch.nb_subfr - 1]
        )
        : (j += 1) {
            if (j == ch.nb_subfr) break;
            var temp: i32 = 0;
            const sub = ch.nb_subfr - 1 - j;
            for (0..ltp_order) |i|
                temp += ctrl.ltp_coef_q14[sub * ltp_order + i];
            if (temp > ltp_gain_q14) {
                ltp_gain_q14 = temp;
                @memcpy(
                    &p.ltp_coef_q14,
                    ctrl.ltp_coef_q14[sub * ltp_order..][0..ltp_order],
                );
                p.pitch_l_q8 = fix.lshift(ctrl.pitch_l[sub], 8);
            }
        }
        @memset(&p.ltp_coef_q14, 0);
        p.ltp_coef_q14[ltp_order / 2] = @truncate(ltp_gain_q14);

        // Limited.
        if (ltp_gain_q14 < v_pitch_gain_start_min_q14) {
            const tmp = fix.lshift(v_pitch_gain_start_min_q14, 10);
            const scale_q10 = @divTrunc(tmp, @max(ltp_gain_q14, 1));
            for (&p.ltp_coef_q14) |*c|
                c.* = @truncate(fix.smulbb(c.*, scale_q10) >> 10);
        } else if (ltp_gain_q14 > v_pitch_gain_start_max_q14) {
            const tmp = fix.lshift(v_pitch_gain_start_max_q14, 14);
            const scale_q14 = @divTrunc(tmp, @max(ltp_gain_q14, 1));
            for (&p.ltp_coef_q14) |*c|
                c.* = @truncate(fix.smulbb(c.*, scale_q14) >> 14);
        }
    } else {
        p.pitch_l_q8 = fix.lshift(fix.smulbb(ch.fs_khz, 18), 8);
        @memset(&p.ltp_coef_q14, 0);
    }
    @memcpy(
        p.prev_lpc_q12[0..ch.lpc_order],
        ctrl.pred_coef_q12[1][0..ch.lpc_order],
    );
    p.prev_ltp_scale_q14 = ctrl.ltp_scale_q14;
    p.prev_gain_q16 = ctrl.gains_q16[ch.nb_subfr - 2..][0..2].*;
    p.subfr_length = ch.subfr_length;
    p.nb_subfr = ch.nb_subfr;
}

/// A lost frame, made from the last: the pitch repeated through the last
/// filters, with noise, both fading. `silk_PLC_conceal`.
fn plcConceal(ch: *Channel, ctrl: *Control, frame: []i16) void {
    const p = &ch.plc;
    const order = ch.lpc_order;
    var s_ltp: [max_frame_length]i16 = undefined;
    var s_ltp_q14: [2 * max_frame_length]i32 = undefined;

    const prev_gain_q10 = [2]i32{
        p.prev_gain_q16[0] >> 6,
        p.prev_gain_q16[1] >> 6,
    };
    if (ch.first_frame_after_reset) @memset(&p.prev_lpc_q12, 0);

    // The quieter of the last two subframes is the noise to use.
    var exc_buf: [2 * max_sub_frame_length]i16 = undefined;
    for (0..2) |k| {
        for (0..ch.subfr_length) |i| {
            exc_buf[k * ch.subfr_length + i] = @intCast(
                fix.sat16(
                    fix.smulww(
                        ch.exc_q14[i + (k + ch.nb_subfr - 2) * ch.subfr_length],
                        prev_gain_q10[k],
                    )
                        >> 8,
                ),
            );
        }
    }
    const e1 = fix.sumSqrShift(exc_buf[0..ch.subfr_length]);
    const e2 = fix.sumSqrShift(exc_buf[ch.subfr_length..][0..ch.subfr_length]);
    const rand_off: usize = if (
        (e1.energy >> @intCast(e2.shift))
            < (e2.energy >> @intCast(e1.shift))
    )
        @intCast(
            @max(
                0,
                @as(i32, @intCast((p.nb_subfr - 1) * p.subfr_length))
                    - rand_buf_size,
            ),
        )
    else
        @intCast(
            @max(
                0,
                @as(i32, @intCast(p.nb_subfr * p.subfr_length)) - rand_buf_size,
            ),
        );
    const rand_ptr = ch.exc_q14[rand_off..];

    const b_q14 = &p.ltp_coef_q14;
    var rand_scale_q14: i32 = p.rand_scale_q14;
    const att_ix: usize = @intCast(@min(1, ch.loss_cnt));
    const harm_gain_q15 = harm_att_q15[att_ix];
    var rand_gain_q15: i32 = if (ch.prev_signal_type == SignalType.voiced)
        plc_rand_attenuate_v_q15[att_ix]
    else
        plc_rand_attenuate_uv_q15[att_ix];

    lpc.bwexpander(p.prev_lpc_q12[0..order], bwe_coef_q16);
    var a_q12: [max_lpc_order]i16 = undefined;
    @memcpy(a_q12[0..order], p.prev_lpc_q12[0..order]);

    if (ch.loss_cnt == 0) {
        rand_scale_q14 = 1 << 14;
        if (ch.prev_signal_type == SignalType.voiced) {
            // Less noise in a voiced frame.
            for (b_q14) |b|
                rand_scale_q14 = @as(i16, @truncate(rand_scale_q14 - b));
            rand_scale_q14 = @max(3277, rand_scale_q14);
            rand_scale_q14 = @as(
                i16,
                @truncate(
                    fix.smulbb(rand_scale_q14, p.prev_ltp_scale_q14) >> 14,
                ),
            );
        } else {
            // Less in an unvoiced one with a high LPC gain.
            const inv_gain_q30 = lpc.inversePredGain(p.prev_lpc_q12[0..order]);
            var down_scale_q30 = @min(
                @as(i32, 1 << 30) >> log2_inv_lpc_gain_high_thres,
                inv_gain_q30,
            );
            down_scale_q30 = @max(
                @as(i32, 1 << 30) >> log2_inv_lpc_gain_low_thres,
                down_scale_q30,
            );
            down_scale_q30 = fix.lshift(
                down_scale_q30,
                log2_inv_lpc_gain_high_thres,
            );
            rand_gain_q15 = fix.smulwb(down_scale_q30, rand_gain_q15) >> 14;
        }
    }

    var rand_seed = p.rand_seed;
    var lag = fix.rshiftRound(p.pitch_l_q8, 8);
    var s_ltp_buf_idx = ch.ltp_mem_length;

    // The LTP state, rewhitened and scaled.
    const idx = ch.ltp_mem_length
        - @as(usize, @intCast(lag))
        - order
        - ltp_order / 2;
    lpc.analysisFilter(
        s_ltp[idx..ch.ltp_mem_length],
        ch.out_buf[idx..ch.ltp_mem_length],
        a_q12[0..order],
    );
    var inv_gain_q30 = fix.inverse32VarQ(p.prev_gain_q16[1], 46);
    inv_gain_q30 = @min(inv_gain_q30, fix.i32_max >> 1);
    for (idx + order..ch.ltp_mem_length) |i|
        s_ltp_q14[i] = fix.smulwb(inv_gain_q30, s_ltp[i]);

    // Long-term synthesis, with noise.
    for (0..ch.nb_subfr) |_| {
        var pred_lag = s_ltp_buf_idx
            - @as(usize, @intCast(lag))
            + ltp_order / 2;
        for (0..ch.subfr_length) |_| {
            var ltp_pred_q12: i32 = 2;
            for (0..ltp_order) |j|
                ltp_pred_q12 = fix.smlawb(
                    ltp_pred_q12,
                    s_ltp_q14[pred_lag - j],
                    b_q14[j],
                );
            pred_lag += 1;
            rand_seed = fix.rand(rand_seed);
            const ri: usize = @intCast((rand_seed >> 25) & (rand_buf_size - 1));
            s_ltp_q14[s_ltp_buf_idx] = fix.lshift(
                fix.smlawb(ltp_pred_q12, rand_ptr[ri], rand_scale_q14),
                2,
            );
            s_ltp_buf_idx += 1;
        }
        // Both gains fade, and the pitch drifts down a little.
        for (b_q14) |*b| b.* = @truncate(fix.smulbb(harm_gain_q15, b.*) >> 15);
        rand_scale_q14 = @as(
            i16,
            @truncate(fix.smulbb(rand_scale_q14, rand_gain_q15) >> 15),
        );
        p.pitch_l_q8 = fix.smlawb(
            p.pitch_l_q8,
            p.pitch_l_q8,
            pitch_drift_fac_q16,
        );
        p.pitch_l_q8 = @min(
            p.pitch_l_q8,
            fix.lshift(fix.smulbb(max_pitch_lag_ms, ch.fs_khz), 8),
        );
        lag = fix.rshiftRound(p.pitch_l_q8, 8);
    }

    // Short-term synthesis.
    const s_lpc = s_ltp_q14[ch.ltp_mem_length - max_lpc_order..];
    @memcpy(s_lpc[0..max_lpc_order], &ch.slpc_q14_buf);
    for (0..ch.frame_length) |i| {
        const pred = lpcPredict(s_lpc, i, a_q12[0..order]);
        s_lpc[max_lpc_order + i] = fix.addSat32(
            s_lpc[max_lpc_order + i],
            fix.lshiftSat32(pred, 4),
        );
        frame[i] = @intCast(
            fix.sat16(
                fix.rshiftRound(
                    fix.smulww(s_lpc[max_lpc_order + i], prev_gain_q10[1]),
                    8,
                ),
            ),
        );
    }

    @memcpy(&ch.slpc_q14_buf, s_lpc[ch.frame_length..][0..max_lpc_order]);

    p.rand_seed = rand_seed;
    p.rand_scale_q14 = rand_scale_q14;
    @memset(&ctrl.pitch_l, lag);
}

/// A good frame after concealed ones faded in from the concealment's
/// energy. `silk_PLC_glue_frames`.
fn plcGlueFrames(ch: *Channel, frame: []i16) void {
    const p = &ch.plc;
    if (ch.loss_cnt != 0) {
        const e = fix.sumSqrShift(frame);
        p.conc_energy = e.energy;
        p.conc_energy_shift = e.shift;
        p.last_frame_lost = true;
        return;
    }
    if (p.last_frame_lost) {
        const e = fix.sumSqrShift(frame);
        var energy = e.energy;
        if (e.shift > p.conc_energy_shift) {
            p.conc_energy >>= @intCast(e.shift - p.conc_energy_shift);
        } else if (e.shift < p.conc_energy_shift) {
            energy >>= @intCast(p.conc_energy_shift - e.shift);
        }
        if (energy > p.conc_energy) {
            const lz = fix.clz32(p.conc_energy) - 1;
            p.conc_energy = fix.lshift(p.conc_energy, lz);
            energy >>= @intCast(@max(24 - lz, 0));
            const frac_q24 = @divTrunc(p.conc_energy, @max(energy, 1));
            var gain_q16 = fix.lshift(fix.sqrtApprox(frac_q24), 4);
            var slope_q16 = @divTrunc(
                (@as(i32, 1) << 16) - gain_q16,
                @as(i32, @intCast(frame.len)),
            );
            // Four times steeper, not to miss onsets after DTX.
            slope_q16 = fix.lshift(slope_q16, 2);
            for (frame) |*f| {
                f.* = @truncate(fix.smulwb(gain_q16, f.*));
                gain_q16 += slope_q16;
                if (gain_q16 > 1 << 16) break;
            }
        }
    }
    p.last_frame_lost = false;
}

// Comfort noise.

fn cngReset(ch: *Channel) void {
    const step = @divTrunc(fix.i16_max, @as(i32, @intCast(ch.lpc_order + 1)));
    var acc: i32 = 0;
    for (ch.cng.smth_nlsf_q15[0..ch.lpc_order]) |*n| {
        acc += step;
        n.* = @intCast(acc);
    }
    ch.cng.smth_gain_q16 = 0;
    ch.cng.rand_seed = 3176576;
}

/// Tracks the background from frames without speech, and adds it to
/// concealed ones. `silk_CNG`.
fn cngApply(ch: *Channel, ctrl: *const Control, frame: []i16) void {
    const c = &ch.cng;
    const order = ch.lpc_order;
    if (ch.fs_khz != c.fs_khz) {
        ch.cngReset();
        c.fs_khz = ch.fs_khz;
    }
    if (
        ch.loss_cnt == 0
            and ch.prev_signal_type == SignalType.no_voice_activity
    ) {
        // Smoothed line spectral frequencies.
        for (0..order) |i| {
            c.smth_nlsf_q15[i] = @truncate(
                @as(i32, c.smth_nlsf_q15[i])
                    + fix.smulwb(
                        @as(i32, ch.prev_nlsf_q15[i]) - c.smth_nlsf_q15[i],
                        16348,
                    ),
            );
        }
        // The excitation of the loudest subframe.
        var max_gain_q16: i32 = 0;
        var subfr: usize = 0;
        for (0..ch.nb_subfr) |i| {
            if (ctrl.gains_q16[i] > max_gain_q16) {
                max_gain_q16 = ctrl.gains_q16[i];
                subfr = i;
            }
        }
        const sl = ch.subfr_length;
        std.mem.copyBackwards(
            i32,
            c.exc_buf_q14[sl..][0..(ch.nb_subfr - 1) * sl],
            c.exc_buf_q14[0..(ch.nb_subfr - 1) * sl],
        );
        @memcpy(c.exc_buf_q14[0..sl], ch.exc_q14[subfr * sl..][0..sl]);
        // Smoothed gains.
        for (0..ch.nb_subfr) |i| {
            c.smth_gain_q16 +%= fix.smulwb(
                ctrl.gains_q16[i] -% c.smth_gain_q16,
                4634,
            );
            // 3 dB above this subframe's: adapt faster.
            if (fix.smulww(c.smth_gain_q16, 46396) > ctrl.gains_q16[i])
                c.smth_gain_q16 = ctrl.gains_q16[i];
        }
    }

    if (ch.loss_cnt != 0) {
        const length = frame.len;
        var sig: [max_frame_length + max_lpc_order]i32 = undefined;
        var gain_q16 = fix.smulww(
            ch.plc.rand_scale_q14,
            ch.plc.prev_gain_q16[1],
        );
        if (gain_q16 >= (1 << 21) or c.smth_gain_q16 > (1 << 23)) {
            gain_q16 = fix.smultt(gain_q16, gain_q16);
            gain_q16 = fix.smultt(c.smth_gain_q16, c.smth_gain_q16)
                -% fix.lshift(gain_q16, 5);
            gain_q16 = fix.lshift(fix.sqrtApprox(gain_q16), 16);
        } else {
            gain_q16 = fix.smulww(gain_q16, gain_q16);
            gain_q16 = fix.smulww(c.smth_gain_q16, c.smth_gain_q16)
                -% fix.lshift(gain_q16, 5);
            gain_q16 = fix.lshift(fix.sqrtApprox(gain_q16), 8);
        }
        const gain_q10 = gain_q16 >> 6;

        // The excitation, from the buffer at random.
        var exc_mask: i32 = 255;
        while (exc_mask > @as(i32, @intCast(length))) exc_mask >>= 1;
        var seed = c.rand_seed;
        for (0..length) |i| {
            seed = fix.rand(seed);
            sig[max_lpc_order + i] = c.exc_buf_q14[
                @intCast((seed >> 24) & exc_mask)
            ];
        }
        c.rand_seed = seed;

        var a_q12: [max_lpc_order]i16 = undefined;
        lpc.nlsf2a(a_q12[0..order], c.smth_nlsf_q15[0..order]);
        @memcpy(sig[0..max_lpc_order], &c.synth_state);
        for (0..length) |i| {
            const pred = lpcPredict(&sig, i, a_q12[0..order]);
            sig[max_lpc_order + i] = fix.addSat32(
                sig[max_lpc_order + i],
                fix.lshiftSat32(pred, 4),
            );
            const noise = fix.sat16(
                fix.rshiftRound(
                    fix.smulww(sig[max_lpc_order + i], gain_q10),
                    8,
                ),
            );
            frame[i] = @intCast(fix.sat16(@as(i32, frame[i]) + noise));
        }
        @memcpy(&c.synth_state, sig[length..][0..max_lpc_order]);
    } else {
        @memset(c.synth_state[0..order], 0);
    }
}
