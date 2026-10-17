//! read container bytes as encoded packets and timing events
const std = @import("std");
const types = @import("types.zig");
const Source = @import("Source.zig");
const Wave = @import("containers/wave/root.zig").Reader;
const Ogg = @import("containers/ogg/root.zig").Reader;
const Mp3 = @import("containers/mp3/root.zig").Reader;
const PacketSource = @This();

pub const Error = types.Error;

/// owned container state selected during detection
const Backend = union(enum) { wave: *Wave, ogg: *Ogg, mp3: *Mp3 };
const containers = .{ Wave, Ogg, Mp3 };

backend: Backend,
encoding: types.DecodingScheme,
format: types.AudioFormat,
presentation: types.Presentation = .{},
/// first decoded sample position before trimming
decoded_origin: i64 = 0,
seekable: bool,
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
    return Wave.initParsed(allocator, source, parsed, options);
}

/// set framing before requesting packets
pub fn configure(self: *PacketSource, scheme: types.InputScheme) Error!void {
    switch (self.backend) {
        inline else => |container| try container.configure(scheme),
    }
}

/// borrow a packet until the next advancing source operation
pub fn next(self: *PacketSource) Error!Event {
    const event = switch (self.backend) {
        inline else => |container| try container.next(),
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
        inline else => |container| container.info(),
    };
}

/// seek at or before a frame, preserving prior data needed for held silence
pub fn seek(self: *PacketSource, frame: u64) Error!SeekResult {
    if (!self.seekable) return error.Unseekable;
    return switch (self.backend) {
        inline else => |container| container.seek(frame),
    };
}

/// free packet storage and parser state; leave input open
pub fn deinit(self: *PacketSource) void {
    switch (self.backend) {
        inline else => |container| container.deinit(),
    }
    self.* = undefined;
}
