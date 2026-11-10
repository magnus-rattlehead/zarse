// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2011 Xiph.Org Foundation
// SPDX-License-Identifier: MIT AND BSD-2-Clause

//! Decode complete Opus packets to interleaved float samples at 48 kHz.
//! Supports the Ogg pipeline's mono/stereo and surround channel mappings.
//! All streams but the last use self-delimited packet framing (RFC 7845).

const MultistreamDecoder = @This();
const std = @import("std");
const Decoder = @import("Decoder.zig");
const packet = @import("packet.zig");

channels: usize,
coupled_streams: usize,
mapping: [8]u8,
decoders: []Decoder,

pub const Error = Decoder.Error;

pub fn init(
    allocator: std.mem.Allocator,
    channels: usize,
    streams: usize,
    coupled_streams: usize,
    mapping: []const u8,
) (std.mem.Allocator.Error || error{ BadArg })!MultistreamDecoder {
    if (
        channels == 0
            or channels > 8
            or mapping.len < channels
            or streams == 0
            or coupled_streams > streams
            or streams > 255 - coupled_streams
    )
        return error.BadArg;
    for (mapping[0..channels]) |m| {
        if (m != 255 and m >= streams + coupled_streams) return error.BadArg;
    }
    const decoders = try allocator.alloc(Decoder, streams);
    errdefer allocator.free(decoders);
    for (decoders, 0..) |*d, s| {
        d.* = try .init(if (s < coupled_streams) 2 else 1);
    }
    var map: [8]u8 = @splat(255);
    @memcpy(map[0..channels], mapping[0..channels]);
    return .{
        .channels = channels,
        .coupled_streams = coupled_streams,
        .mapping = map,
        .decoders = decoders,
    };
}

pub fn deinit(self: *MultistreamDecoder, allocator: std.mem.Allocator) void {
    allocator.free(self.decoders);
    self.* = undefined;
}

pub fn reset(self: *MultistreamDecoder) void {
    for (self.decoders) |*decoder| decoder.reset();
}

/// Validate every stream's framing, duration, and output capacity before
/// changing any decoder state.
fn validate(self: *const MultistreamDecoder, data: []const u8) Error!usize {
    var rest = data;
    var samples: usize = 0;
    for (0..self.decoders.len) |s| {
        if (rest.len == 0) return error.InvalidPacket;
        const parsed = if (s + 1 < self.decoders.len) blk: {
            const stream = try packet.parseSelfDelimited(rest);
            rest = rest[stream.len..];
            break :blk stream.packet;
        } else try packet.parse(rest);
        const n = parsed.samples();
        if (s != 0 and n != samples) return error.InvalidPacket;
        samples = n;
    }
    return samples;
}

/// Decode a complete packet. Empty input is invalid, not packet loss.
/// Returns the number of frames per output channel.
pub fn decode(
    self: *MultistreamDecoder,
    data: []const u8,
    pcm: []f32,
) Error!usize {
    if (data.len < 2 * self.decoders.len - 1) return error.InvalidPacket;
    const frames = try self.validate(data);
    if (pcm.len / self.channels < frames) return error.BufferTooSmall;
    var buf: [2 * Decoder.max_frame_samples]f32 = undefined;
    var rest = data;
    for (self.decoders, 0..) |*decoder, s| {
        const channels: usize = if (s < self.coupled_streams) 2 else 1;
        var consumed: usize = 0;
        const got = try decoder.decodeStream(
            rest,
            buf[0..frames * channels],
            s + 1 < self.decoders.len,
            &consumed,
        );
        if (got != frames) return error.InvalidPacket;
        rest = rest[consumed..];
        const first_channel = if (channels == 2)
            2 * s
        else
            s + self.coupled_streams;
        for (self.mapping[0..self.channels], 0..) |mapped, output_channel| {
            if (
                mapped == 255
                    or mapped < first_channel
                    or mapped >= first_channel + channels
            )
                continue;
            const side = mapped - first_channel;
            for (0..frames) |i| {
                pcm[i * self.channels + output_channel] = buf[
                    i * channels + side
                ];
            }
        }
    }
    for (self.mapping[0..self.channels], 0..) |mapped, channel| {
        if (mapped == 255) {
            for (0..frames) |i| pcm[i * self.channels + channel] = 0;
        }
    }
    return frames;
}
