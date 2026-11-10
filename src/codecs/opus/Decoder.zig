// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2010 Xiph.Org Foundation, Skype Limited
// SPDX-License-Identifier: MIT AND BSD-2-Clause

//! An Opus decoder: packets in, PCM out. RFC 6716, Section 4, following
//! libopus's `opus_decoder.c`.
//!
//! A packet's table of contents says which codec its frames use. Each frame
//! goes to SILK, to CELT, or in hybrid mode to both, and the results are
//! added together. Where the mode changes, the two are cross-faded, using
//! either a short redundant CELT frame the encoder put in the packet for
//! the purpose or concealment of what the old mode would have made next.
//! Output is floating point in [-1, 1], interleaved, at the rate the
//! decoder uses (48 kHz), whatever rate the packet was coded at.
//!
//! Concealment remains internal for empty coded frames and mode transitions.

const Decoder = @This();
const sample_rate = 48000;

const std = @import("std");
const packet = @import("packet.zig");
const RangeDecoder = @import("RangeDecoder.zig");
const CeltDecoder = @import("celt/Decoder.zig");
const celt_mode = @import("celt/mode.zig");
const SilkDecoder = @import("silk/Decoder.zig");
const SilkChannel = @import("silk/Channel.zig");

/// The most samples a packet can hold, a channel, at 48 kHz: 120 ms.
pub const max_frame_samples = packet.max_packet_samples;

/// Output channels.
channels: usize,
celt: CeltDecoder,
silk: SilkDecoder,
/// What SILK is told of each packet, kept between them as libopus keeps it.
silk_ctrl: SilkDecoder.Control,

/// The last packet's mode, bandwidth, frame length and channels.
mode: ?packet.Mode = null,
bandwidth: ?packet.Bandwidth = null,
/// The last frame's length, in samples at the output rate.
frame_size: usize,
stream_channels: usize,

/// What the last frame was, which a lost one is concealed as.
prev_mode: ?packet.Mode = null,
/// Whether the last frame ended with a redundant CELT frame, which makes
/// the next one CELT's to conceal.
prev_redundancy: bool = false,

pub fn init(channels: usize) error{ BadArg }!Decoder {
    return .{
        .channels = channels,
        .celt = try .init(channels),
        .silk = .init(),
        .silk_ctrl = .{
            .n_channels_api = channels,
            .n_channels_internal = 0,
            .api_sample_rate = @intCast(sample_rate),
            .internal_sample_rate = 0,
            .payload_size_ms = 0,
        },
        .frame_size = sample_rate / 400,
        .stream_channels = channels,
    };
}

/// Back to how `init` left it: after a seek, or a new stream.
pub fn reset(d: *Decoder) void {
    d.celt.reset();
    d.silk.reset();
    d.mode = null;
    d.bandwidth = null;
    d.prev_mode = null;
    d.prev_redundancy = false;
    d.frame_size = sample_rate / 400;
    d.stream_channels = d.channels;
}

pub const Error = packet.ParseError
    || error{
        /// `pcm` has less room than the packet has samples.
        BufferTooSmall,
        /// A frame read more bits than it had.
        InvalidPacket,
    };

/// A packet already parsed: its frames, one after another.
fn decodeParsed(d: *Decoder, p: packet.Packet, pcm: []f32) Error!usize {
    const room = pcm.len / d.channels;
    const frame_size = p.toc.frameSamples();
    if (@as(usize, p.count) * frame_size > room) return error.BufferTooSmall;

    // Only now, so that an invalid packet changes nothing.
    d.mode = p.toc.mode();
    d.bandwidth = p.toc.bandwidth();
    d.frame_size = frame_size;
    d.stream_channels = if (p.toc.stereo) 2 else 1;

    var at: usize = 0;
    for (p.frameSlice()) |frame| {
        at += try d.decodeFrame(frame, pcm[at * d.channels..], room - at);
    }
    return at;
}

/// Decode one stream, self-delimited unless it is the last in the packet.
/// `consumed` reports the stream's encoded length.
pub fn decodeStream(
    d: *Decoder,
    data: []const u8,
    pcm: []f32,
    self_delimited: bool,
    consumed: *usize,
) Error!usize {
    consumed.* = 0;
    if (data.len == 0) return error.InvalidPacket;
    const p = if (self_delimited) blk: {
        const stream = try packet.parseSelfDelimited(data);
        consumed.* = stream.len;
        break :blk stream.packet;
    } else blk: {
        consumed.* = data.len;
        break :blk try packet.parse(data);
    };
    return d.decodeParsed(p, pcm);
}

/// One frame, or a lost one, into `pcm`, at most `frame_size_` samples a
/// channel. Returns how many. `opus_decode_frame`.
fn decodeFrame(
    d: *Decoder,
    data_: ?[]const u8,
    pcm: []f32,
    frame_size_: usize,
) Error!usize {
    const f20: usize = sample_rate / 50;
    const f10 = f20 >> 1;
    const f5 = f10 >> 1;
    const f2_5 = f5 >> 1;
    const ch = d.channels;
    if (frame_size_ < f2_5) return error.BufferTooSmall;
    var frame_size: usize = @min(frame_size_, sample_rate / 25 * 3);

    // A frame of a byte or none is concealed, no longer than the table of
    // contents says.
    var data = data_;
    if (data == null or data.?.len <= 1) {
        data = null;
        frame_size = @min(frame_size, d.frame_size);
    }

    var audiosize: usize = undefined;
    var mode: packet.Mode = undefined;
    var bandwidth: ?packet.Bandwidth = null;
    var len: usize = 0;
    var dec: RangeDecoder = undefined;
    if (data) |bytes| {
        audiosize = d.frame_size;
        mode = d.mode.?;
        bandwidth = d.bandwidth;
        len = bytes.len;
        dec = .init(bytes);
    } else {
        audiosize = frame_size;
        // Concealed in the last mode, or CELT's after a redundant frame.
        mode = if (d.prev_redundancy)
            .celt
        else
            d.prev_mode orelse {
                // Nothing to conceal from yet.
                @memset(pcm[0..audiosize * ch], 0);
                return audiosize;
            };
        // Only in the sizes concealment can make.
        if (audiosize > f20) {
            var at: usize = 0;
            while (at < audiosize)
                at += try d.decodeFrame(
                    null,
                    pcm[at * ch..],
                    @min(audiosize - at, f20),
                );
            return frame_size;
        } else if (audiosize < f20) {
            if (audiosize > f10) {
                audiosize = f10;
            } else if (mode != .silk and audiosize > f5 and audiosize < f10) {
                audiosize = f5;
            }
        }
    }

    // A change between CELT and the others: the old mode's concealment,
    // cross-faded into the new, unless the packet has a redundant frame.
    var transition = data != null
        and d.prev_mode != null
        and ((mode == .celt and d.prev_mode.? != .celt and !d.prev_redundancy)
            or (mode != .celt and d.prev_mode.? == .celt));
    var pcm_transition: [2 * 240]f32 = undefined;
    if (transition and mode == .celt) {
        _ = try d.decodeFrame(null, &pcm_transition, @min(f5, audiosize));
    }
    if (audiosize > frame_size) return error.BufferTooSmall;
    frame_size = audiosize;

    // SILK, into its own buffer: its concealment makes 10 ms however
    // little is asked for.
    var pcm_silk: [2 * 2880]i16 = undefined;
    if (mode != .celt) {
        if (d.prev_mode == .celt) d.silk.reset();
        d.silk_ctrl.payload_size_ms = @intCast(
            @max(10, 1000 * audiosize / sample_rate),
        );
        if (data != null) {
            d.silk_ctrl.n_channels_internal = d.stream_channels;
            d.silk_ctrl.internal_sample_rate = if (mode == .silk)
                switch (bandwidth.?) {
                    .narrow => 8000,
                    .medium => 12000,
                    else => 16000,
                }
            else
                16000;
        }
        const lost: SilkChannel.LostFlag = if (data == null)
            .packet_lost
        else
            .decode_normal;
        var decoded: usize = 0;
        while (decoded < frame_size) {
            decoded += d.silk.decode(
                &d.silk_ctrl,
                lost,
                decoded == 0,
                &dec,
                pcm_silk[decoded * ch..],
            )
                catch |err| switch (err) {
                    error.InvalidFrameSize => if (lost == .packet_lost) blk: {
                        // Concealment failing is not fatal.
                        @memset(
                            pcm_silk[decoded * ch..][0..frame_size * ch],
                            0,
                        );
                        break :blk frame_size;
                    } else return error.InvalidPacket,
                };
        }
    }

    // A redundant CELT frame at the end of a SILK or hybrid one.
    var redundancy = false;
    var celt_to_silk = false;
    var redundancy_bytes: usize = 0;
    if (
        mode != .celt
            and data != null
            and dec.tell()
                + 17
                + 20 * @as(i32, @intFromBool(mode == .hybrid)) <= 8
                    * @as(i32, @intCast(len))
    ) {
        redundancy = if (mode == .hybrid) dec.decodeBitLogp(12) else true;
        if (redundancy) {
            celt_to_silk = dec.decodeBitLogp(1);
            const rb: i32 = if (mode == .hybrid)
                @as(i32, @intCast(dec.decodeUint(256))) + 2
            else
                @as(i32, @intCast(len)) - ((dec.tell() + 7) >> 3);
            const new_len = @as(i32, @intCast(len)) - rb;
            // A sanity check, which no valid packet fails.
            if (new_len * 8 < dec.tell()) {
                len = 0;
                redundancy_bytes = 0;
                redundancy = false;
            } else {
                len = @intCast(new_len);
                redundancy_bytes = @intCast(rb);
            }
            dec.shrink(redundancy_bytes);
        }
    }
    const start_band: usize = if (mode != .celt) 17 else 0;
    if (redundancy) transition = false;
    if (transition and mode != .celt) {
        _ = try d.decodeFrame(null, &pcm_transition, @min(f5, audiosize));
    }

    if (bandwidth) |bw| {
        d.celt.end = switch (bw) {
            .narrow => 13,
            .medium, .wide => 17,
            .superwide => 19,
            .full => 21,
        };
    }
    d.celt.stream_channels = d.stream_channels;

    // CELT to SILK: the redundant frame first, while CELT's state is still
    // the last frame's.
    var redundant_audio: [2 * 240]f32 = undefined;
    const bytes = data orelse &.{};
    if (redundancy and celt_to_silk) {
        d.celt.start = 0;
        _ = d.celt.decode(
            bytes[len..][0..redundancy_bytes],
            &redundant_audio,
            f5,
            null,
        )
            catch return error.InvalidPacket;
    }

    // After any concealment.
    d.celt.start = start_band;

    if (mode != .silk) {
        const celt_frame_size = @min(f20, frame_size);
        // A change of mode discards CELT's state.
        if (
            d.prev_mode != null
                and mode != d.prev_mode.?
                and !d.prev_redundancy
        )
            d.celt.reset();
        const celt_data: ?[]const u8 = if (data == null)
            null
        else
            bytes[0..len];
        _ = d.celt.decode(
            celt_data,
            pcm,
            celt_frame_size,
            if (celt_data != null) &dec else null,
        )
            catch return error.InvalidPacket;
    } else {
        @memset(pcm[0..frame_size * ch], 0);
        // From hybrid to SILK, CELT fades out by decoding silence.
        if (
            d.prev_mode == .hybrid
                and !(redundancy and celt_to_silk and d.prev_redundancy)
        ) {
            d.celt.start = 0;
            const silence = [2]u8{ 0xff, 0xff };
            _ = d.celt.decode(&silence, pcm, f2_5, null)
                catch return error.InvalidPacket;
        }
    }

    if (mode != .celt) {
        for (pcm[0..frame_size * ch], pcm_silk[0..frame_size * ch]) |*o, s| {
            o.* = o.* + (1.0 / 32768.0) * @as(f32, @floatFromInt(s));
        }
    }

    // SILK to CELT: the redundant frame after, cross-faded into the end.
    if (redundancy and !celt_to_silk) {
        d.celt.reset();
        d.celt.start = 0;
        _ = d.celt.decode(
            bytes[len..][0..redundancy_bytes],
            &redundant_audio,
            f5,
            null,
        )
            catch return error.InvalidPacket;
        smoothFade(
            pcm[ch * (frame_size - f2_5)..],
            redundant_audio[ch * f2_5..],
            pcm[ch * (frame_size - f2_5)..],
            f2_5,
            ch,
            sample_rate,
        );
    }
    // CELT to SILK, unless the last frame was not CELT's: the redundant
    // frame's start, and a cross-fade from it.
    if (
        redundancy
            and celt_to_silk
            and (d.prev_mode != .silk or d.prev_redundancy)
    ) {
        @memcpy(pcm[0..f2_5 * ch], redundant_audio[0..f2_5 * ch]);
        smoothFade(
            redundant_audio[ch * f2_5..],
            pcm[ch * f2_5..],
            pcm[ch * f2_5..],
            f2_5,
            ch,
            sample_rate,
        );
    }
    if (transition) {
        if (audiosize >= f5) {
            @memcpy(pcm[0..ch * f2_5], pcm_transition[0..ch * f2_5]);
            smoothFade(
                pcm_transition[ch * f2_5..],
                pcm[ch * f2_5..],
                pcm[ch * f2_5..],
                f2_5,
                ch,
                sample_rate,
            );
        } else {
            // Too short for a clean transition, the best there is.
            smoothFade(&pcm_transition, pcm, pcm, f2_5, ch, sample_rate);
        }
    }

    d.prev_mode = mode;
    d.prev_redundancy = redundancy and !celt_to_silk;
    return audiosize;
}

/// `in1` into `in2` over `overlap` samples a channel, along the square of
/// CELT's window. `smooth_fade`.
fn smoothFade(
    in1: []const f32,
    in2: []const f32,
    out: []f32,
    overlap: usize,
    channels: usize,
    rate: u32,
) void {
    const inc = 48000 / rate;
    for (0..channels) |c| {
        for (0..overlap) |i| {
            const w = celt_mode.window[i * inc] * celt_mode.window[i * inc];
            out[i * channels + c] = w * in2[i * channels + c]
                + (1.0 - w) * in1[i * channels + c];
        }
    }
}

