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
presentation: types.Presentation = .{},
/// first decoded sample position before trimming
decoded_origin: u64 = 0,
seekable: bool = false,

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
pub const SeekResult = struct { frame: u64 };

/// detect and open a container
/// seekable input may be indexed; sequential input is parsed incrementally
/// requires exclusive reader access until deinit; leaves input open
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
    // the first matching probe validates the remaining input
    inline for (containers) |Container| {
        if (Container.probe(&prefix)) {
            return Container.init(allocator, source, prefix, options);
        }
    }
    return error.UnsupportedFormat;
}

/// leave self empty and safe to deinit
/// finish using returned packets before calling
pub fn take(self: *PacketSource) PacketSource {
    const owned = self.*;
    self.* = .{};
    return owned;
}

/// remove and return the cached vorbis setup
pub fn takeVorbisSetup(self: *PacketSource) ?VorbisSetup {
    if (!features.vorbis or !features.ogg) return null;
    if (self.backend != .ogg) return null;
    const headers = if (self.backend.ogg.vorbis_headers) |*headers|
        headers
    else
        return null;
    return headers.takeSetup();
}

/// packet bytes remain valid until the next advancing source operation
/// data and input failures require successful seeking or reopening before reuse
pub fn next(self: *PacketSource) Error!Event {
    return switch (self.backend) {
        inline else => |container| if (@TypeOf(container) == void)
                error.InvalidState
            else
                container.next(),
    };
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
    seekable: bool,
    frame: u64,
    origin: u64,
    end: ?u64,
) Error!void {
    if (!seekable) return error.Unseekable;
    if (frame < origin) return error.OutOfRange;
    if (end) |limit| {
        if (frame > limit) return error.OutOfRange;
    }
}

/// seek at or before an untrimmed frame, preserving prior data needed for held silence
/// capability and known range rejections leave state unchanged
/// input failures may advance state and require successful seeking or reopening
pub fn seek(self: *PacketSource, frame: u64) Error!SeekResult {
    return switch (self.backend) {
        inline else => |container| if (@TypeOf(container) == void)
                error.InvalidState
            else
                container.seek(frame),
    };
}

/// leave input open
pub fn deinit(self: *PacketSource) void {
    switch (self.backend) {
        inline else => |container| if (@TypeOf(container) != void) {
                container.deinit();
            },
    }
    self.* = .{};
}
