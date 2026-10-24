//! decode encoded units into pcm; select codecs and manage sample storage
const std = @import("std");
const types = @import("types.zig");
const pcm = @import("codecs/pcm.zig");
const wave = @import("codecs/wave/root.zig");
const features = @import("build_options");
const Codec = @This();

pub const Error = types.Error;

backend: Backend = .empty,
/// decoding history needed before a seek target, in frames
preroll_frames: u64 = 0,

const Backend = union(enum) {
    empty: void,
    pcm: if (features.pcm) *UnitAdapter(pcm) else void,
    alaw: if (features.wave_codecs) *UnitAdapter(wave.alaw) else void,
    mulaw: if (features.wave_codecs) *UnitAdapter(wave.mulaw) else void,
    ima_adpcm: if (features.wave_codecs) *UnitAdapter(wave.ima_adpcm) else void,
    ms_adpcm: if (features.wave_codecs) *UnitAdapter(wave.ms_adpcm) else void,
    mp3: if (features.mp3) *UnitAdapter(@import("codecs/mp3.zig")) else void,
    flac: if (features.flac) *UnitAdapter(@import("codecs/flac.zig")) else void,
    opus: if (features.opus) *UnitAdapter(@import("codecs/opus.zig")) else void,
    vorbis: if (features.vorbis)
        *UnitAdapter(@import("codecs/vorbis.zig"))
    else
        void,
};

pub fn initSource(
    allocator: std.mem.Allocator,
    source: *@import("PacketSource.zig"),
    options: types.Options,
) Error!Codec {
    const info = source.info();
    const format = types.AudioFormat{
        .sample_rate = info.sample_rate,
        .channels = info.channels,
        .channel_mask = info.channel_mask,
    };
    const scheme = source.encoding;
    return switch (scheme) {
        .pcm => |config| {
            if (!features.pcm) return error.UnsupportedEncoding;
            return pcm.init(allocator, config, format, options);
        },
        .mp3 => {
            if (!features.mp3) return error.UnsupportedEncoding;
            return @import("codecs/mp3.zig").init(allocator, format, options);
        },
        .alaw => {
            if (!features.wave_codecs) return error.UnsupportedEncoding;
            return wave.alaw.init(allocator, format, options);
        },
        .mulaw => {
            if (!features.wave_codecs) return error.UnsupportedEncoding;
            return wave.mulaw.init(allocator, format, options);
        },
        .ima_adpcm => |config| {
            if (!features.wave_codecs) return error.UnsupportedEncoding;
            return wave.ima_adpcm.init(allocator, config, format, options);
        },
        .ms_adpcm => |config| {
            if (!features.wave_codecs) return error.UnsupportedEncoding;
            return wave.ms_adpcm.init(allocator, config, format, options);
        },
        .flac => |config| {
            if (!features.flac) return error.UnsupportedEncoding;
            return @import("codecs/flac.zig")
                .init(allocator, config, format, options);
        },
        .opus => |config| {
            if (!features.opus) return error.UnsupportedEncoding;
            return @import("codecs/opus.zig")
                .init(allocator, config, format, options);
        },
        .vorbis => {
            if (!features.vorbis) return error.UnsupportedEncoding;
            var setup = source.takeVorbisSetup()
                orelse return error.InvalidState;
            defer setup.deinit();
            return @import("codecs/vorbis.zig")
                .initTakeSetup(allocator, &setup, format, options);
        },
        else => error.UnsupportedEncoding,
    };
}

/// decode complete units to interleaved f32 samples, possibly empty while priming history
/// output remains valid until the next codec operation
/// after a data or backend failure, reset successfully or reopen
pub fn decode(self: *Codec, bytes: []const u8) Error![]const f32 {
    return switch (self.backend) {
        inline else => |backend| if (@TypeOf(backend) == void)
                error.InvalidState
            else
                backend.decode(bytes),
    };
}

/// clear history; keep config
pub fn reset(self: *Codec) Error!void {
    switch (self.backend) {
        inline else => |backend| if (@TypeOf(backend) == void)
                return error.InvalidState
            else
                try backend.reset(),
    }
}

/// leave self empty and safe to deinit
pub fn take(self: *Codec) Codec {
    const owned = self.*;
    self.* = .{};
    return owned;
}

pub fn deinit(self: *Codec) void {
    switch (self.backend) {
        inline else => |backend| if (@TypeOf(backend) != void) {
                backend.deinit();
            },
    }
    self.* = .{};
}

/// manage codec state and sample storage
pub fn UnitAdapter(comptime Unit: type) type {
    return struct {
        const Self = @This();
        allocator: std.mem.Allocator,
        unit: Unit,
        scheme: types.InputScheme,
        channels: u16,
        max_frames_per_unit: u32,
        output: []f32 = &.{},

        /// consume unit resources even on failure
        pub fn init(
            allocator: std.mem.Allocator,
            unit: Unit,
            comptime backend: std.meta.Tag(Backend),
            scheme: types.InputScheme,
            max_frames_per_unit: u32,
            format: types.AudioFormat,
            options: types.Options,
        ) Error!Codec {
            var owned = unit;
            errdefer owned.deinit();
            if (
                max_frames_per_unit == 0
                    or format.channels == 0
                    or format.sample_rate == 0
            ) {
                return error.InvalidData;
            }
            // packet codecs use one unit, fixed units may share one buffer
            var units: usize = 1;
            if (scheme.unitBytes()) |size| {
                if (size == 0) return error.InvalidData;
                if (size > options.bufferCapacity()) return error.LimitExceeded;
                units = options.bufferCapacity() / size;
            }
            // reserve max pcm output for all buffered units
            const frames = std.math.mul(usize, units, max_frames_per_unit)
                catch return error.LimitExceeded;
            const capacity = std.math.mul(usize, frames, format.channels)
                catch return error.LimitExceeded;
            const self = try allocator.create(Self);
            errdefer allocator.destroy(self);
            const output = try allocator.alloc(f32, capacity);
            self.* = .{
                .allocator = allocator,
                .unit = owned,
                .scheme = scheme,
                .channels = format.channels,
                .max_frames_per_unit = max_frames_per_unit,
                .output = output,
            };
            return .{
                .backend = @unionInit(Backend, @tagName(backend), self),
                .preroll_frames = Unit.preroll_frames,
            };
        }

        fn decode(self: *Self, bytes: []const u8) Error![]const f32 {
            if (bytes.len == 0) return error.InvalidData;
            var units: usize = 1;
            if (self.scheme.unitBytes()) |size| {
                if (bytes.len % size != 0) return error.InvalidData;
                units = bytes.len / size;
            }
            const max_frames = std.math.mul(
                usize,
                units,
                self.max_frames_per_unit,
            )
                catch return error.LimitExceeded;
            const capacity = std.math.mul(usize, max_frames, self.channels)
                catch return error.LimitExceeded;
            if (capacity > self.output.len) return error.LimitExceeded;
            const frames = try self.unit.decodeUnit(
                bytes,
                self.output[0..capacity],
            );
            if (frames > capacity / self.channels) {
                return error.InvalidState;
            }
            return self.output[0..frames * self.channels];
        }

        fn reset(self: *Self) Error!void {
            try self.unit.reset();
        }

        fn deinit(self: *Self) void {
            const allocator = self.allocator;
            self.unit.deinit();
            allocator.free(self.output);
            allocator.destroy(self);
        }
    };
}
