// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2001-2011 Timothy B. Terriberry
// SPDX-FileCopyrightText: © 2008-2009 Xiph.Org Foundation
// SPDX-License-Identifier: MIT AND BSD-2-Clause

//! The range decoder of RFC 6716, Section 4.1: the entropy coder every symbol
//! in an Opus frame goes through, SILK's and CELT's alike.
//!
//! A frame is read from both ends. Range-coded symbols come from the front,
//! a byte at a time into a 32-bit window. Raw bits, which CELT uses for
//! values whose distribution is flat enough that coding them would gain
//! nothing, come from the back, least significant first (Section 4.1.4).
//! Reading past either end reads zeros, as the specification requires, and
//! is not an error: a frame that has run out of bits decodes as silence
//! rather than failing, and `tell` says how far past the end it went.
//!
//! This follows the reference implementation's `entdec.c` step for step,
//! since the decoder is only correct if it makes exactly the same arithmetic
//! decisions the encoder did.

const RangeDecoder = @This();

const std = @import("std");

/// Bits in a symbol the coder outputs: bytes.
const sym_bits = 8;
/// Bits in the state registers.
const code_bits = 32;
const sym_max = (1 << sym_bits) - 1;
/// The top of the range, where normalization stops.
const code_top: u32 = 1 << (code_bits - 1);
/// The bottom of the range, below which a byte is shifted in.
const code_bot: u32 = code_top >> sym_bits;
/// Bits of the first byte that go into the initial value. The rest carry
/// over into the next.
const code_extra = (code_bits - 2) % sym_bits + 1;
/// `decodeUint` splits values wider than this into a coded top and raw
/// bits.
const uint_bits = 8;
/// The raw-bit window.
const window_size = 32;
/// Fractional bits in `tellFrac`'s result: eighths of a bit.
pub const bitres = 3;

buf: []const u8,
/// Where the next range-coded byte comes from.
offs: u32 = 0,
/// How many bytes of raw bits have been read from the back.
end_offs: u32 = 0,
/// Raw bits read from the back and not yet used.
end_window: u32 = 0,
nend_bits: u32 = 0,
/// Bits consumed so far, as `tell` counts them.
nbits_total: i32,
/// The size of the current range.
rng: u32,
/// The difference between the top of the range and the value read, less
/// one.
val: u32,
/// The previous byte read, of which only part has gone into `val`.
rem: u32,
/// The scale `decode` or `decodeBin` worked out, which `update` needs.
ext: u32 = 0,
/// Set when a value outside its range was decoded: the frame is corrupt,
/// and what was decoded from it is not to be trusted.
err: bool = false,

pub fn init(buf: []const u8) RangeDecoder {
    var d: RangeDecoder = .{
        .buf = buf,
        .nbits_total = code_bits
            + 1
            - ((code_bits - code_extra) / sym_bits) * sym_bits,
        .rng = 1 << code_extra,
        .val = undefined,
        .rem = undefined,
    };
    d.rem = d.readByte();
    d.val = d.rng - 1 - (d.rem >> (sym_bits - code_extra));
    d.normalize();
    return d;
}

fn readByte(d: *RangeDecoder) u32 {
    if (d.offs < d.buf.len) {
        const b = d.buf[d.offs];
        d.offs += 1;
        return b;
    }
    return 0;
}

fn readByteFromEnd(d: *RangeDecoder) u32 {
    if (d.end_offs < d.buf.len) {
        d.end_offs += 1;
        return d.buf[d.buf.len - d.end_offs];
    }
    return 0;
}

/// Brings the range back above `code_bot`, a byte at a time.
fn normalize(d: *RangeDecoder) void {
    while (d.rng <= code_bot) {
        d.nbits_total += sym_bits;
        d.rng <<= sym_bits;
        var sym = d.rem;
        d.rem = d.readByte();
        sym = (sym << sym_bits | d.rem) >> (sym_bits - code_extra);
        d.val = ((d.val << sym_bits) +% (sym_max & ~sym)) & (code_top - 1);
    }
}

/// The first half of decoding a symbol with a cumulative frequency table of
/// total `ft`: where in `[0, ft)` the value falls. `update` must follow,
/// with the bounds of the symbol that covers it.
pub fn decode(d: *RangeDecoder, ft: u32) u32 {
    d.ext = d.rng / ft;
    const s = d.val / d.ext;
    return ft - @min(s + 1, ft);
}

/// `decode` for a total of `1 << bits`, which needs no division.
pub fn decodeBin(d: *RangeDecoder, bits: u5) u32 {
    d.ext = d.rng >> bits;
    const s = d.val / d.ext;
    const ft = @as(u32, 1) << bits;
    return ft - @min(s + 1, ft);
}

/// The second half: the symbol decoded covers `[fl, fh)` of `ft`.
pub fn update(d: *RangeDecoder, fl: u32, fh: u32, ft: u32) void {
    const s = d.ext * (ft - fh);
    d.val -= s;
    d.rng = if (fl > 0) d.ext * (fh - fl) else d.rng - s;
    d.normalize();
}

/// A bit that is 1 with probability `1 / (1 << logp)`.
pub fn decodeBitLogp(d: *RangeDecoder, logp: u5) bool {
    const r = d.rng;
    const v = d.val;
    const s = r >> logp;
    const bit = v < s;
    if (!bit) d.val = v - s;
    d.rng = if (bit) s else r - s;
    d.normalize();
    return bit;
}

/// A symbol from an "inverse" cumulative distribution: `icdf[k]` is
/// `(1 << ftb)` less the cumulative frequency of symbols up to and
/// including `k`, so the table falls to zero. Section 4.1.3.3.
pub fn decodeIcdf(d: *RangeDecoder, icdf: []const u8, ftb: u5) u32 {
    var s = d.rng;
    const v = d.val;
    const r = s >> ftb;
    var ret: u32 = 0;
    var t: u32 = undefined;
    while (true) {
        t = s;
        s = r * icdf[ret];
        if (v >= s) break;
        ret += 1;
    }
    d.val = v - s;
    d.rng = t - s;
    d.normalize();
    return ret;
}

/// A value uniformly distributed in `[0, ft)`, `ft` at least 2. Wide values
/// are split: the top `uint_bits` range-coded, the rest raw bits from the
/// back. Section 4.1.5.
pub fn decodeUint(d: *RangeDecoder, ft_: u32) u32 {
    std.debug.assert(ft_ > 1);
    const ft = ft_ - 1;
    var ftb: u6 = ilog(ft);
    if (ftb > uint_bits) {
        ftb -= uint_bits;
        const ft1 = (ft >> @intCast(ftb)) + 1;
        const s = d.decode(ft1);
        d.update(s, s + 1, ft1);
        const t = s << @intCast(ftb) | d.decodeBits(@intCast(ftb));
        if (t <= ft) return t;
        d.err = true;
        return ft;
    }
    const s = d.decode(ft + 1);
    d.update(s, s + 1, ft + 1);
    return s;
}

/// `bits` raw bits from the back of the frame, the first read in the least
/// significant place. Section 4.1.4.
pub fn decodeBits(d: *RangeDecoder, bits: u5) u32 {
    var window = d.end_window;
    var available = d.nend_bits;
    if (available < bits) {
        while (true) {
            window |= d.readByteFromEnd() << @intCast(available);
            available += sym_bits;
            if (available > window_size - sym_bits) break;
        }
    }
    const ret = window & ((@as(u32, 1) << bits) - 1);
    window >>= bits;
    available -= bits;
    d.end_window = window;
    d.nend_bits = available;
    d.nbits_total += bits;
    return ret;
}

/// The last `n` bytes of the frame taken away from it, as belonging to
/// something else: in Opus, a redundant CELT frame after a SILK one.
/// Before any raw bits are read from the end.
pub fn shrink(d: *RangeDecoder, n: usize) void {
    std.debug.assert(d.end_offs == 0);
    d.buf = d.buf[0..d.buf.len - n];
}

/// Whole bits used so far, rounded up: what the encoder must have spent to
/// have written this much. Section 4.1.6.1.
pub fn tell(d: *const RangeDecoder) i32 {
    return d.nbits_total - @as(i32, ilog(d.rng));
}

/// Bits used so far, in eighths of a bit. Section 4.1.6.2.
pub fn tellFrac(d: *const RangeDecoder) u32 {
    const correction = [8]u32{
        35733,
        38967,
        42495,
        46340,
        50535,
        55109,
        60097,
        65535,
    };
    const nbits: u32 = @as(u32, @intCast(d.nbits_total)) << bitres;
    var l: u32 = ilog(d.rng);
    const r = d.rng >> @intCast(l - 16);
    var b = (r >> 12) - 8;
    if (r > correction[b]) b += 1;
    l = (l << 3) + b;
    return nbits - l;
}

/// The number of bits needed to write `x`: 0 for 0, 1 for 1, 2 for 2 and 3,
/// and so on. The specification's `ilog`.
pub fn ilog(x: u32) u6 {
    return @intCast(32 - @clz(x));
}
