// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-License-Identifier: MIT

//! An Opus packet: the table-of-contents byte that says what is in it, and
//! the frames it carries. RFC 6716, Section 3.
//!
//! A packet is what a transport delivers as one unit -- an RTP payload, an
//! Ogg packet, a Sendspin audio message -- and holds between one and 48
//! frames of the same mode, bandwidth and duration, 120 ms at most. Its
//! first byte says which of those, and how the frames after it are sized.
//! `parse` checks every requirement Section 3.4 places on a well-formed
//! packet, and a packet that fails one is refused rather than guessed at.

const std = @import("std");

/// Which coding layers a frame uses. Section 3.1.
pub const Mode = enum {
    /// Linear prediction: speech, up to wideband.
    silk,
    /// SILK below 8 kHz and CELT above it.
    hybrid,
    /// The transform codec: music, and low delay.
    celt,
};

/// The audio bandwidth a frame codes. Section 2.
pub const Bandwidth = enum(u3) {
    /// 4 kHz, sampled at 8 kHz.
    narrow,
    /// 6 kHz, sampled at 12 kHz.
    medium,
    /// 8 kHz, sampled at 16 kHz.
    wide,
    /// 12 kHz, sampled at 24 kHz.
    superwide,
    /// 20 kHz, sampled at 48 kHz.
    full,
};

/// The table-of-contents byte, which begins every packet. Section 3.1.
pub const Toc = struct {
    /// The configuration number, 0 to 31: mode, bandwidth and frame size
    /// together.
    config: u5,
    stereo: bool,
    /// How the frames are packed: Section 3.2.
    code: u2,

    pub fn fromByte(byte: u8) Toc {
        return .{
            .config = @intCast(byte >> 3),
            .stereo = byte & 0x04 != 0,
            .code = @intCast(byte & 0x03),
        };
    }

    /// Table 2.
    pub fn mode(t: Toc) Mode {
        return switch (t.config) {
            0...11 => .silk,
            12...15 => .hybrid,
            16...31 => .celt,
        };
    }

    /// Table 2.
    pub fn bandwidth(t: Toc) Bandwidth {
        return switch (t.config) {
            0...3 => .narrow,
            4...7 => .medium,
            8...11 => .wide,
            12...13 => .superwide,
            14...15 => .full,
            16...19 => .narrow,
            20...23 => .wide,
            24...27 => .superwide,
            28...31 => .full,
        };
    }

    /// How long each frame is, in samples at 48 kHz, which is how Opus counts
    /// time whatever the bandwidth. Table 2.
    pub fn frameSamples(t: Toc) u32 {
        const i: u2 = @truncate(t.config);
        return switch (t.mode()) {
            // 10, 20, 40, 60 ms.
            .silk => ([4]u32{ 480, 960, 1920, 2880 })[i],
            // 10, 20 ms.
            .hybrid => ([2]u32{ 480, 960 })[i & 1],
            // 2.5, 5, 10, 20 ms.
            .celt => ([4]u32{ 120, 240, 480, 960 })[i],
        };
    }
};

/// The largest frame the specification allows, in bytes. Section 3.4, R2.
pub const max_frame_bytes = 1275;
/// The most frames a packet can hold: 48 of 2.5 ms is 120 ms.
pub const max_frames = 48;
/// The longest a packet can last, in samples at 48 kHz.
pub const max_packet_samples = 5760;

pub const Packet = struct {
    toc: Toc,
    /// The frames, each a slice of the packet. A frame may be empty: a
    /// frame the encoder left out, which the decoder fills in as if it had
    /// been lost.
    frames: [max_frames][]const u8,
    count: u8,
    /// Bytes after the frames, which a decoder ignores. Section 3.2.5.
    padding: []const u8,

    pub fn frameSlice(p: *const Packet) []const []const u8 {
        return p.frames[0..p.count];
    }

    /// The packet's length, in samples at 48 kHz.
    pub fn samples(p: *const Packet) u32 {
        return p.toc.frameSamples() * p.count;
    }
};

pub const ParseError = error{
    /// Empty, or ending where the table of contents says there is more:
    /// R1, and the lengths of codes 2 and 3.
    Truncated,
    /// A frame longer than 1275 bytes: R2.
    FrameTooLong,
    /// Code 1 with an odd number of bytes to split: R3.
    UnevenFrames,
    /// Code 2 or 3 with a frame longer than what is left: R4 and R7.
    BadFrameLength,
    /// Code 3 with no frames, or more than 120 ms of them: R5.
    BadFrameCount,
    /// Code 3, constant bitrate, with bytes that do not divide evenly
    /// between the frames: R6.
    UnevenCbr,
};

pub fn parse(data: []const u8) ParseError!Packet {
    if (data.len < 1) return error.Truncated;
    var p: Packet = .{
        .toc = .fromByte(data[0]),
        .frames = undefined,
        .count = 0,
        .padding = &.{},
    };
    var rest = data[1..];

    switch (p.toc.code) {
        // One frame.
        0 => {
            try frameLength(rest.len);
            p.frames[0] = rest;
            p.count = 1;
        },
        // Two frames of the same size.
        1 => {
            if (rest.len % 2 != 0) return error.UnevenFrames;
            const n = rest.len / 2;
            try frameLength(n);
            p.frames[0] = rest[0..n];
            p.frames[1] = rest[n..];
            p.count = 2;
        },
        // Two frames, the first's length given.
        2 => {
            const n = try readLength(&rest);
            if (n > rest.len) return error.BadFrameLength;
            try frameLength(n);
            try frameLength(rest.len - n);
            p.frames[0] = rest[0..n];
            p.frames[1] = rest[n..];
            p.count = 2;
        },
        // Any number of frames.
        3 => {
            if (rest.len < 1) return error.Truncated;
            const count_byte = rest[0];
            rest = rest[1..];
            const vbr = count_byte & 0x80 != 0;
            const has_padding = count_byte & 0x40 != 0;
            const m: u8 = count_byte & 0x3f;
            if (
                m == 0
                    or @as(u32, m) * p.toc.frameSamples() > max_packet_samples
            )
                return error.BadFrameCount;
            p.count = m;

            // Padding: a length in bytes of 255 that each add 254 and go on,
            // ending with one below 255 that adds itself. The padding is
            // the last bytes of the packet.
            var padding: usize = 0;
            if (has_padding) {
                while (true) {
                    if (rest.len < 1) return error.Truncated;
                    const b = rest[0];
                    rest = rest[1..];
                    padding += if (b == 255) 254 else b;
                    if (b != 255) break;
                }
            }
            if (padding > rest.len) return error.Truncated;
            p.padding = rest[rest.len - padding..];
            rest = rest[0..rest.len - padding];

            if (vbr) {
                // Every length but the last is given. The last frame is what
                // remains.
                var lengths: [max_frames]usize = undefined;
                var total: usize = 0;
                for (lengths[0..m - 1]) |*len| {
                    len.* = try readLength(&rest);
                    try frameLength(len.*);
                    total += len.*;
                }
                if (total > rest.len) return error.BadFrameLength;
                lengths[m - 1] = rest.len - total;
                try frameLength(lengths[m - 1]);
                var at: usize = 0;
                for (lengths[0..m], p.frames[0..m]) |len, *frame| {
                    frame.* = rest[at..at + len];
                    at += len;
                }
            } else {
                if (rest.len % m != 0) return error.UnevenCbr;
                const n = rest.len / m;
                try frameLength(n);
                for (p.frames[0..m], 0..) |*frame, i|
                    frame.* = rest[i * n..(i + 1) * n];
            }
        },
    }
    return p;
}

/// A self-delimited packet and how many bytes it took: the framing of
/// every stream but the last in a multistream packet, where the last
/// frame's length is given too, since the packet does not end where the
/// data does. RFC 6716, Appendix B, `opus_packet_parse_impl` with
/// `self_delimited` set.
pub const SelfDelimited = struct {
    packet: Packet,
    /// The bytes it took, padding included: where the next one starts.
    len: usize,
};

pub fn parseSelfDelimited(data: []const u8) ParseError!SelfDelimited {
    if (data.len < 1) return error.Truncated;
    var p: Packet = .{
        .toc = .fromByte(data[0]),
        .frames = undefined,
        .count = 0,
        .padding = &.{},
    };
    var at: usize = 1;
    // What is left for the frames, less any padding, which comes after
    // them. `at` moves only over what precedes them.
    var avail: isize = @intCast(data.len - 1);
    var last_size: isize = avail;
    var sizes: [max_frames]usize = undefined;
    var count: usize = undefined;
    var cbr = false;
    var pad: usize = 0;
    switch (p.toc.code) {
        0 => count = 1,
        1 => {
            count = 2;
            cbr = true;
        },
        2 => {
            count = 2;
            const n = try sizeAt(data, &at, &avail);
            if (n > avail) return error.BadFrameLength;
            sizes[0] = @intCast(n);
            last_size = avail - n;
        },
        3 => {
            if (avail < 1) return error.Truncated;
            const ch = data[at];
            at += 1;
            avail -= 1;
            count = ch & 0x3f;
            if (count == 0 or count * p.toc.frameSamples() > max_packet_samples)
                return error.BadFrameCount;
            if (ch & 0x40 != 0) {
                while (true) {
                    if (avail <= 0) return error.Truncated;
                    const b = data[at];
                    at += 1;
                    avail -= 1;
                    const tmp: usize = if (b == 255) 254 else b;
                    avail -= @intCast(tmp);
                    pad += tmp;
                    if (b != 255) break;
                }
            }
            if (avail < 0) return error.Truncated;
            cbr = ch & 0x80 == 0;
            if (!cbr) {
                last_size = avail;
                for (0..count - 1) |i| {
                    const before = at;
                    const n = try sizeAt(data, &at, &avail);
                    if (n > avail) return error.BadFrameLength;
                    sizes[i] = @intCast(n);
                    last_size -= @as(isize, @intCast(at - before)) + n;
                }
                if (last_size < 0) return error.BadFrameLength;
            }
        },
    }
    // The last frame's length, given.
    const before = at;
    const n = try sizeAt(data, &at, &avail);
    if (n > avail) return error.BadFrameLength;
    sizes[count - 1] = @intCast(n);
    if (cbr) {
        if (n * @as(isize, @intCast(count)) > avail)
            return error.BadFrameLength;
        for (sizes[0..count - 1]) |*sz| sz.* = @intCast(n);
    } else if (@as(isize, @intCast(at - before)) + n > last_size) {
        return error.BadFrameLength;
    }
    p.count = @intCast(count);
    for (sizes[0..count], p.frames[0..count]) |sz, *frame| {
        frame.* = data[at..][0..sz];
        at += sz;
    }
    p.padding = data[at..][0..pad];
    return .{ .packet = p, .len = at + pad };
}

/// A length, as `readLength`, but within what `avail` says is left.
/// `parse_size`.
fn sizeAt(data: []const u8, at: *usize, avail: *isize) ParseError!isize {
    if (avail.* < 1) return error.Truncated;
    const first = data[at.*];
    if (first < 252) {
        at.* += 1;
        avail.* -= 1;
        return first;
    }
    if (avail.* < 2) return error.Truncated;
    const n = @as(isize, first) + 4 * @as(isize, data[at.* + 1]);
    at.* += 2;
    avail.* -= 2;
    return n;
}

fn frameLength(n: usize) ParseError!void {
    if (n > max_frame_bytes) return error.FrameTooLong;
}

/// A frame length, in one byte below 252 or two for longer: the first plus
/// four times the second. Section 3.2.1.
fn readLength(rest: *[]const u8) ParseError!usize {
    if (rest.len < 1) return error.Truncated;
    const first = rest.*[0];
    if (first < 252) {
        rest.* = rest.*[1..];
        return first;
    }
    if (rest.len < 2) return error.Truncated;
    const n = @as(usize, first) + 4 * @as(usize, rest.*[1]);
    rest.* = rest.*[2..];
    return n;
}
