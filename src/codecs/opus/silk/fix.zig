// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2006-2011 Skype Limited
// SPDX-License-Identifier: MIT AND BSD-3-Clause

//! SILK's fixed-point arithmetic: the macros of libopus's `macros.h`,
//! `SigProc_FIX.h` and `Inlines.h`, and the small functions built on them.
//!
//! SILK decodes in integers even in libopus's floating point build, so
//! matching it is a matter of doing exactly its arithmetic. Where a macro
//! has a 64-bit form, this is that form, which is the one libopus uses on a
//! 64-bit machine. The 32-bit forms round differently. A plain 32-bit add,
//! subtract or multiply wraps, as the compiled C does: a valid stream never
//! overflows, and an invalid one decodes to the same garbage libopus makes
//! rather than stopping the program.

const std = @import("std");

pub const i16_max: i32 = std.math.maxInt(i16);
pub const i16_min: i32 = std.math.minInt(i16);
pub const i32_max: i32 = std.math.maxInt(i32);
pub const i32_min: i32 = std.math.minInt(i32);

/// `(opus_int16)x`: the low 16 bits, as a signed value.
pub inline fn lo16(x: i32) i32 {
    return @as(i16, @truncate(x));
}

pub inline fn add(a: i32, b: i32) i32 {
    return a +% b;
}

pub inline fn sub(a: i32, b: i32) i32 {
    return a -% b;
}

pub inline fn mul(a: i32, b: i32) i32 {
    return a *% b;
}

/// `a + b * c`.
pub inline fn mla(a: i32, b: i32, c: i32) i32 {
    return a +% (b *% c);
}

/// `(a * (opus_int16)b) >> 16`.
pub inline fn smulwb(a: i32, b: i32) i32 {
    return @truncate((@as(i64, a) * lo16(b)) >> 16);
}

/// `a + ((b * (opus_int16)c) >> 16)`.
pub inline fn smlawb(a: i32, b: i32, c: i32) i32 {
    return @truncate(@as(i64, a) + ((@as(i64, b) * lo16(c)) >> 16));
}

/// `(a * (b >> 16)) >> 16`.
pub inline fn smulwt(a: i32, b: i32) i32 {
    return @truncate((@as(i64, a) * (b >> 16)) >> 16);
}

/// `a + ((b * (c >> 16)) >> 16)`.
pub inline fn smlawt(a: i32, b: i32, c: i32) i32 {
    return @truncate(@as(i64, a) + ((@as(i64, b) * (c >> 16)) >> 16));
}

/// `(opus_int16)a * (opus_int16)b`.
pub inline fn smulbb(a: i32, b: i32) i32 {
    return lo16(a) * lo16(b);
}

/// `a + (opus_int16)b * (opus_int16)c`.
pub inline fn smlabb(a: i32, b: i32, c: i32) i32 {
    return a +% (lo16(b) * lo16(c));
}

pub inline fn smlabbOvflw(a: i32, b: i32, c: i32) i32 {
    return a +% (lo16(b) * lo16(c));
}

/// `(opus_int16)a * (b >> 16)`.
pub inline fn smulbt(a: i32, b: i32) i32 {
    return lo16(a) *% (b >> 16);
}

/// `a + (opus_int16)b * (c >> 16)`.
pub inline fn smlabt(a: i32, b: i32, c: i32) i32 {
    return a +% (lo16(b) *% (c >> 16));
}

/// `(a >> 16) * (b >> 16)`.
pub inline fn smultt(a: i32, b: i32) i32 {
    return (a >> 16) *% (b >> 16);
}

/// `(a * b) >> 16`, in 64 bits.
pub inline fn smulww(a: i32, b: i32) i32 {
    return @truncate((@as(i64, a) * b) >> 16);
}

/// `a + ((b * c) >> 16)`, in 64 bits.
pub inline fn smlaww(a: i32, b: i32, c: i32) i32 {
    return @truncate(@as(i64, a) + ((@as(i64, b) * c) >> 16));
}

/// `(a * b) >> 32`: the top word.
pub inline fn smmul(a: i32, b: i32) i32 {
    return @truncate((@as(i64, a) * b) >> 32);
}

pub inline fn smull(a: i32, b: i32) i64 {
    return @as(i64, a) * b;
}

/// `a << shift`, through an unsigned value, so wrapping.
pub inline fn lshift(a: i32, shift: anytype) i32 {
    return @bitCast(@as(u32, @bitCast(a)) << @intCast(shift));
}

pub inline fn rshift(a: i32, shift: anytype) i32 {
    return a >> @intCast(shift);
}

/// Shifted right by `shift`, rounded: `silk_RSHIFT_ROUND`.
pub inline fn rshiftRound(a: i32, shift: anytype) i32 {
    if (shift == 1) return (a >> 1) +% (a & 1);
    return ((a >> @intCast(shift - 1)) +% 1) >> 1;
}

pub inline fn rshiftRound64(a: i64, shift: anytype) i64 {
    if (shift == 1) return (a >> 1) +% (a & 1);
    return ((a >> @intCast(shift - 1)) +% 1) >> 1;
}

pub inline fn sat16(a: i32) i32 {
    return if (a > i16_max)
        i16_max
    else if (a < i16_min)
        i16_min
    else
        a;
}

/// `silk_LIMIT`: `a` between the two limits, whichever order they are in.
pub inline fn limit(
    a: anytype,
    limit1: @TypeOf(a),
    limit2: @TypeOf(a),
) @TypeOf(a) {
    if (limit1 > limit2) {
        return if (a > limit1)
            limit1
        else if (a < limit2)
            limit2
        else
            a;
    }
    return if (a > limit2)
        limit2
    else if (a < limit1)
        limit1
    else
        a;
}

/// `a << shift`, saturated first: `silk_LSHIFT_SAT32`.
pub inline fn lshiftSat32(a: i32, shift: anytype) i32 {
    return lshift(
        limit(a, i32_min >> @intCast(shift), i32_max >> @intCast(shift)),
        shift,
    );
}

pub fn addSat32(a: i32, b: i32) i32 {
    const s = a +% b;
    if (@as(u32, @bitCast(s)) & 0x80000000 == 0) {
        return if (@as(u32, @bitCast(a & b)) & 0x80000000 != 0) i32_min else s;
    }
    return if (@as(u32, @bitCast(a | b)) & 0x80000000 == 0) i32_max else s;
}

pub fn subSat32(a: i32, b: i32) i32 {
    const d = a -% b;
    const ua: u32 = @bitCast(a);
    const ub: u32 = @bitCast(b);
    if (@as(u32, @bitCast(d)) & 0x80000000 == 0) {
        return if (ua & (ub ^ 0x80000000) & 0x80000000 != 0) i32_min else d;
    }
    return if ((ua ^ 0x80000000) & ub & 0x80000000 != 0) i32_max else d;
}

/// `silk_ADD_POS_SAT32`.
pub inline fn addPosSat32(a: i32, b: i32) i32 {
    const s = a +% b;
    return if (@as(u32, @bitCast(s)) & 0x80000000 != 0) i32_max else s;
}

/// `silk_abs`, which is wrong for the most negative value, as libopus's is.
pub inline fn abs(a: i32) i32 {
    return if (a > 0) a else 0 -% a;
}

pub inline fn clz32(a: i32) i32 {
    return @clz(@as(u32, @bitCast(a)));
}

pub inline fn clz16(a: i32) i32 {
    return @clz(@as(u16, @bitCast(@as(i16, @truncate(a)))));
}

/// Rotated right by `rot`, or left by `-rot`.
pub fn ror32(a: i32, rot: i32) i32 {
    const x: u32 = @bitCast(a);
    if (rot == 0) return a;
    if (rot < 0) {
        const m: u5 = @intCast(-rot);
        return @bitCast(std.math.rotl(u32, x, m));
    }
    return @bitCast(std.math.rotr(u32, x, @as(u5, @intCast(rot))));
}

/// The leading zeros of `a`, and the seven bits after the leading one.
pub fn clzFrac(a: i32) struct { lz: i32, frac_q7: i32 } {
    const lz = clz32(a);
    return .{ .lz = lz, .frac_q7 = ror32(a, 24 - lz) & 0x7f };
}

/// A square root to within 10%, and to 2.5% above 120.
pub fn sqrtApprox(x: i32) i32 {
    if (x <= 0) return 0;
    const cf = clzFrac(x);
    var y: i32 = if (cf.lz & 1 != 0) 32768 else 46214;
    y >>= @intCast(cf.lz >> 1);
    return smlawb(y, y, smulbb(213, cf.frac_q7));
}

/// Nearly `(a << q_res) / b`. `silk_DIV32_varQ`.
pub fn div32VarQ(a: i32, b: i32, q_res: i32) i32 {
    const a_headrm = clz32(abs(a)) - 1;
    var a_nrm = lshift(a, a_headrm);
    const b_headrm = clz32(abs(b)) - 1;
    const b_nrm = lshift(b, b_headrm);
    // The inverse of b, to 14 bits.
    const b_inv = @divTrunc(i32_max >> 2, b_nrm >> 16);
    var result = smulwb(a_nrm, b_inv);
    // The residual, which may overflow: it ends up small.
    a_nrm = a_nrm -% lshift(smmul(b_nrm, result), 3);
    result = smlawb(result, a_nrm, b_inv);
    const shift = 29 + a_headrm - b_headrm - q_res;
    if (shift < 0) return lshiftSat32(result, -shift);
    if (shift < 32) return result >> @intCast(shift);
    return 0;
}

/// Nearly `(1 << q_res) / b`. `silk_INVERSE32_varQ`.
pub fn inverse32VarQ(b: i32, q_res: i32) i32 {
    const b_headrm = clz32(abs(b)) - 1;
    const b_nrm = lshift(b, b_headrm);
    const b_inv = @divTrunc(i32_max >> 2, b_nrm >> 16);
    var result = lshift(b_inv, 16);
    const err_q32 = lshift((@as(i32, 1) << 29) -% smulwb(b_nrm, b_inv), 3);
    result = smlaww(result, err_q32, b_inv);
    const shift = 61 - b_headrm - q_res;
    if (shift <= 0) return lshiftSat32(result, -shift);
    if (shift < 32) return result >> @intCast(shift);
    return 0;
}

/// SILK's pseudo-random generator. Keep what it returns as the next seed.
pub inline fn rand(seed: i32) i32 {
    return 907633515 +% (seed *% 196314165);
}

/// About `128 * log2(x)`. `silk_lin2log`.
pub fn lin2log(x: i32) i32 {
    const cf = clzFrac(x);
    return smlawb(cf.frac_q7, cf.frac_q7 *% (128 - cf.frac_q7), 179)
        +% lshift(31 - cf.lz, 7);
}

/// About `2^(x / 128)`, the inverse of `lin2log`. `silk_log2lin`.
pub fn log2lin(x: i32) i32 {
    if (x < 0) return 0;
    if (x >= 3967) return i32_max;
    var out = lshift(1, x >> 7);
    const frac_q7 = x & 0x7f;
    const t = smlawb(frac_q7, smulbb(frac_q7, 128 - frac_q7), -174);
    if (x < 2048) {
        out = out +% ((out *% t) >> 7);
    } else {
        out = mla(out, out >> 7, t);
    }
    return out;
}

/// The energy of `x`, and the right shift that made it fit in 31 bits
/// with two to spare. `silk_sum_sqr_shift`.
pub fn sumSqrShift(x: []const i16) struct { energy: i32, shift: i32 } {
    const len: i32 = @intCast(x.len);
    var shift: i32 = 31 - clz32(len);
    var nrg: i32 = sumShifted(x, len, shift);
    shift = @max(0, shift + 3 - clz32(nrg));
    nrg = sumShifted(x, 0, shift);
    return .{ .energy = nrg, .shift = shift };
}

fn sumShifted(x: []const i16, start: i32, shift: i32) i32 {
    var nrg: u32 = @bitCast(start);
    var i: usize = 0;
    while (i + 1 < x.len) : (i += 2) {
        var t: u32 = @bitCast(smulbb(x[i], x[i]));
        t = @bitCast(smlabbOvflw(@bitCast(t), x[i + 1], x[i + 1]));
        nrg +%= t >> @intCast(shift);
    }
    if (i < x.len) {
        const t: u32 = @bitCast(smulbb(x[i], x[i]));
        nrg +%= t >> @intCast(shift);
    }
    return @bitCast(nrg);
}
