//! read container bytes as encoded packets and timing events
const std = @import("std");
const types = @import("types.zig");
const Source = @import("Source.zig");
const features = @import("build_options");
const Wave = if (features.wav)
    @import("containers/wave/root.zig").Reader
else
    void;
const Ogg = if (features.ogg)
    @import("containers/ogg/root.zig").Reader
else
    void;
const Mp3 = if (features.mp3)
    @import("containers/mp3/root.zig").Reader
else
    void;
const PacketSource = @This();
const Flac = if (features.flac)
    @import("containers/flac/root.zig").Reader
else
    void;

pub const Error = types.Error;
const VorbisSetup = if (features.vorbis)
    @import("codecs/vorbis/root.zig").Setup
else
    void;

/// owned container state selected during detection
const Backend = union(enum) {
    empty: void,
    wave: if (features.wav) *Wave else void,
    ogg: if (features.ogg) *Ogg else void,
    mp3: if (features.mp3) *Mp3 else void,
    flac: if (features.flac) *Flac else void,
};
const containers = (if (features.wav) .{Wave} else .{})
    ++ (if (features.ogg) .{Ogg} else .{})
    ++ (if (features.mp3) .{Mp3} else .{})
    ++ (if (features.flac) .{Flac} else .{});

backend: Backend = .empty,
encoding: types.DecodingScheme = .unsupported,
format: types.AudioFormat = .{ .sample_rate = 0, .channels = 0 },
presentation: types.Presentation = .{},
/// first decoded sample position before trimming
decoded_origin: i64 = 0,
seekable: bool = false,
max_payload_bytes: usize = 64 * 1024 * 1024,

/// encoded packet or playback control event
pub const Event = union(enum) {
    packet: types.Packet,
    /// send final trim bounds before the affected packets
    presentation: types.Presentation,
    /// emit held silence by repeating the last decoded frame
    silence: u64,
    end,
};

/// untrimmed pcm origin to use after resetting codec
pub const SeekResult = struct { frame: i64 };

/// detect and open a container; allocate buffers and any seek index
/// seekable input may be indexed; sequential input is parsed incrementally
/// borrow exclusive reader access until deinit; never close the caller's input
pub fn init(
    allocator: std.mem.Allocator,
    source: Source,
    options: types.Options,
) Error!PacketSource {
    const seekable = source.seek_to != null and try source.getSize() != null;
    // rewind only when size and seeking are available
    if (seekable) try source.seekTo(0);
    // consume shared detection prefix, selected container receives a copy
    var prefix: [4]u8 = undefined;
    if (try source.reader.readSliceShort(&prefix) != prefix.len) {
        return error.UnsupportedFormat;
    }
    // first matching probe owns full validation and remaining input
    inline for (containers) |Container| {
        if (Container.probe(&prefix)) {
            return Container.init(allocator, source, prefix, options);
        }
    }
    return error.UnsupportedFormat;
}

/// open an unchanged parsed wav layout; requires seeking and size
pub fn initParsed(
    allocator: std.mem.Allocator,
    source: Source,
    parsed: types.ParsedAudio,
    options: types.Options,
) Error!PacketSource {
    if (!features.wav) return error.UnsupportedFormat;
    return Wave.initParsed(allocator, source, parsed, options);
}

/// transfer container ownership; leave self empty and safe to deinit
/// finish external packet/header borrows before transferring
pub fn take(self: *PacketSource) PacketSource {
    const owned = self.*;
    self.* = .{};
    return owned;
}

/// transfer parsed setup ownership, leaving the container's slot empty
pub fn takeVorbisSetup(self: *PacketSource) ?VorbisSetup {
    if (!features.vorbis or !features.ogg) return null;
    if (self.backend != .ogg) return null;
    const headers = if (self.backend.ogg.vorbis_headers) |*headers|
        headers
    else
        return null;
    return headers.takeSetup();
}

/// set framing before requesting packets
pub fn configure(self: *PacketSource, scheme: types.InputScheme) Error!void {
    switch (self.backend) {
        inline else => |container| if (@TypeOf(container) == void)
                return error.InvalidState
            else {
                try container.configure(scheme);
                self.seekable = container.canSeek();
            },
    }
}

/// borrow a packet until the next advancing source operation
/// data and input failures require successful seeking or reopening before reuse
pub fn next(self: *PacketSource) Error!Event {
    const event = switch (self.backend) {
        inline else => |container| if (@TypeOf(container) == void)
                return error.InvalidState
            else
                try container.next(),
    };
    // enforce shared packet limits before codec sees borrowed bytes
    if (event == .packet) {
        if (event.packet.bytes.len == 0) return error.InvalidData;
        if (event.packet.bytes.len > self.max_payload_bytes) {
            return error.LimitExceeded;
        }
    }
    return event;
}

/// return metadata with untrimmed frame counts
pub fn info(self: *const PacketSource) types.Info {
    return switch (self.backend) {
        inline else => |container| if (@TypeOf(container) == void)
                unreachable
            else
                container.info(),
    };
}

/// reject predictable seek failures before changing input or parser state
pub fn validateSeek(
    configured: bool,
    seekable: bool,
    frame: u64,
    origin: u64,
    end: ?u64,
) Error!void {
    if (!configured) return error.InvalidState;
    if (!seekable) return error.Unseekable;
    if (frame < origin or frame > std.math.maxInt(i64)) return error.OutOfRange;
    if (end) |limit| {
        if (frame > limit) return error.OutOfRange;
    }
}

/// seek at or before an untrimmed frame, preserving prior data needed for held silence
/// config, capability, and known range rejections leave state unchanged
/// input failures may advance state and require successful seeking or reopening
pub fn seek(self: *PacketSource, frame: u64) Error!SeekResult {
    return switch (self.backend) {
        inline else => |container| if (@TypeOf(container) == void)
                error.InvalidState
            else
                container.seek(frame),
    };
}

/// free packet storage and parser state; leave input open
pub fn deinit(self: *PacketSource) void {
    switch (self.backend) {
        inline else => |container| if (@TypeOf(container) != void) {
                container.deinit();
            },
    }
    self.* = .{};
}
