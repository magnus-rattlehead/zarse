//! decode packets into pcm; select codecs and adapt byte streams
const std = @import("std");
const types = @import("types.zig");
const pcm = @import("codecs/pcm.zig");
const opus_enabled = @import("codec_options").opus;
const vorbis_enabled = @import("codec_options").vorbis;
const Codec = @This();

pub const Error = types.Error;

backend: Backend,
scheme: types.InputScheme,
/// decoding history needed before a seek target, in frames
preroll_frames: u64 = 0,

/// owned adapters selected by enabled codec backends
const Backend = union(enum) {
    pcm: *UnitAdapter(pcm),
    mp3: *UnitAdapter(@import("codecs/mp3.zig")),
    opus: if (opus_enabled) *UnitAdapter(@import("codecs/opus.zig")) else void,
    vorbis: if (vorbis_enabled)
        *UnitAdapter(@import("codecs/vorbis.zig"))
    else
        void,
};

/// open a codec with owned state and sample storage
pub fn init(
    allocator: std.mem.Allocator,
    scheme: types.DecodingScheme,
    format: types.AudioFormat,
    options: types.Options,
) Error!Codec {
    if (format.channels == 0 or format.sample_rate == 0) {
        return error.InvalidData;
    }
    return switch (scheme) {
        .pcm => |config| pcm.init(allocator, config, format, options),
        .mp3 => @import("codecs/mp3.zig").init(allocator, format, options),
        .opus => |config| {
            if (!opus_enabled) return error.UnsupportedEncoding;
            return @import("codecs/opus.zig")
                .init(allocator, config, format, options);
        },
        .vorbis => |config| {
            if (!vorbis_enabled) return error.UnsupportedEncoding;
            return @import("codecs/vorbis.zig")
                .init(allocator, config, format, options);
        },
        else => error.UnsupportedEncoding,
    };
}

/// consume or copy packet bytes before accepting them
pub fn submit(self: *Codec, packet: types.Packet) Error!types.Submit {
    // inline dispatch preserves each adapter type
    return switch (self.backend) {
        inline else => |backend| if (@TypeOf(backend) == void)
                error.InvalidState
            else
                backend.submit(packet),
    };
}

/// borrow output until the next codec operation
pub fn receive(self: *Codec) Error!types.Receive {
    return switch (self.backend) {
        inline else => |backend| if (@TypeOf(backend) == void)
                error.InvalidState
            else
                backend.receive(),
    };
}

/// end input and drain delayed output
pub fn finish(self: *Codec) Error!void {
    switch (self.backend) {
        inline else => |backend| if (@TypeOf(backend) != void) {
                try backend.finish();
            },
    }
}

/// clear output and history; keep configuration
pub fn reset(self: *Codec) void {
    switch (self.backend) {
        inline else => |backend| if (@TypeOf(backend) != void) {
                backend.reset();
            },
    }
}

/// free codec state and sample storage
pub fn deinit(self: *Codec) void {
    switch (self.backend) {
        inline else => |backend| if (@TypeOf(backend) != void) {
                backend.deinit();
            },
    }
    self.* = undefined;
}

/// adapt a synchronous codec to packet submission and sample output
pub fn UnitAdapter(comptime Unit: type) type {
    return struct {
        const Self = @This();
        allocator: std.mem.Allocator,
        unit: Unit,
        scheme: types.InputScheme,
        format: types.AudioFormat,
        max_frames_per_unit: u32,
        options: types.Options,
        output: []f32 = &.{},
        // borrowed view into owned output, blocks next submit until received
        pending: ?types.AudioBlock = null,
        draining: bool = false,

        /// allocate adapter and bounded output, take unit ownership on success
        pub fn init(
            allocator: std.mem.Allocator,
            unit: Unit,
            scheme: types.InputScheme,
            max_frames_per_unit: u32,
            format: types.AudioFormat,
            options: types.Options,
        ) Error!*Self {
            if (
                max_frames_per_unit == 0
                    or format.channels == 0
                    or format.sample_rate == 0
            ) {
                return error.InvalidData;
            }
            // packet codecs use one unit, fixed units may share one submission
            var units: usize = 1;
            if (scheme.unitBytes()) |size| {
                if (size == 0) return error.InvalidData;
                if (size > options.bufferCapacity()) return error.LimitExceeded;
                units = options.bufferCapacity() / size;
            }
            // reserve maximum pcm output for all buffered units
            const frames = std.math.mul(usize, units, max_frames_per_unit)
                catch return error.LimitExceeded;
            const capacity = std.math.mul(usize, frames, format.channels)
                catch return error.LimitExceeded;
            const self = try allocator.create(Self);
            errdefer allocator.destroy(self);
            const output = try allocator.alloc(f32, capacity);
            self.* = .{
                .allocator = allocator,
                .unit = unit,
                .scheme = scheme,
                .format = format,
                .max_frames_per_unit = max_frames_per_unit,
                .options = options,
                .output = output,
            };
            return self;
        }

        /// transfer adapter ownership; the backend tag must match its type
        pub fn interface(
            self: *Self,
            comptime backend: std.meta.Tag(Backend),
        ) Codec {
            return .{
                .backend = @unionInit(Backend, @tagName(backend), self),
                .scheme = self.scheme,
                .preroll_frames = Unit.preroll_frames,
            };
        }

        /// decode synchronously when pending output cleared, borrow packet bytes during call
        fn submit(self: *Self, packet: types.Packet) Error!types.Submit {
            if (self.draining) return error.InvalidState;
            // caller must receive output before next decode can overwrite storage
            if (self.pending != null) return .receive_output_first;
            if (packet.bytes.len > self.options.max_payload_bytes) {
                return error.LimitExceeded;
            }
            if (packet.bytes.len == 0) return error.InvalidData;
            // packet codecs use one unit, fixed units may share one submission
            var units: usize = 1;
            if (self.scheme.unitBytes()) |size| {
                if (packet.bytes.len % size != 0) return error.InvalidData;
                units = packet.bytes.len / size;
            }
            // check output bound before decoding untrusted input
            const max_frames = std.math.mul(
                usize,
                units,
                self.max_frames_per_unit,
            )
                catch return error.LimitExceeded;
            const capacity = std.math.mul(
                usize,
                max_frames,
                self.format.channels,
            )
                catch return error.LimitExceeded;
            if (capacity > self.output.len) return error.LimitExceeded;
            const frames = try self.unit.decodeUnit(
                packet.bytes,
                self.output[0..capacity],
            );
            if (frames > capacity / self.format.channels) {
                return error.InvalidState;
            }
            // retain output view and timestamp until receive
            self.pending = .{
                .samples = self.output[0..frames * self.format.channels],
                .format = self.format,
                .timestamp = packet.timestamp,
            };
            return .accepted;
        }

        /// take pending output once, otherwise request input or report drained
        fn receive(self: *Self) Error!types.Receive {
            if (self.pending) |block| {
                self.pending = null;
                return .{ .audio = block };
            }
            return if (self.draining) .finished else .need_input;
        }

        /// stop accepting input, leave pending output available
        fn finish(self: *Self) Error!void {
            self.draining = true;
        }

        /// discard pending output and decoding history
        fn reset(self: *Self) void {
            self.pending = null;
            self.draining = false;
            self.unit.reset();
        }

        /// free unit state, output storage and adapter
        fn deinit(self: *Self) void {
            const allocator = self.allocator;
            if (@hasDecl(Unit, "deinit")) self.unit.deinit();
            allocator.free(self.output);
            allocator.destroy(self);
        }
    };
}

/// gather complete units from a byte stream; packet sources bypass this
pub const Framer = struct {
    allocator: std.mem.Allocator,
    scheme: types.InputScheme,
    buffer: []u8 = &.{},
    length: usize = 0,
    // bytes already returned, discarded during next compact
    consumed: usize = 0,
    eof: bool = false,
    finished: bool = false,

    /// allocate the configured buffer; reject packet framing
    pub fn init(
        allocator: std.mem.Allocator,
        scheme: types.InputScheme,
        options: types.Options,
    ) Error!Framer {
        const unit = scheme.unitBytes() orelse return error.InvalidData;
        if (unit == 0) return error.InvalidData;
        if (unit > options.bufferCapacity()) return error.LimitExceeded;
        const buffer = try allocator.alloc(u8, options.bufferCapacity());
        return .{ .allocator = allocator, .scheme = scheme, .buffer = buffer };
    }

    /// borrow complete units until next call, null means end of input
    /// the reader must stop before any trailing container metadata
    pub fn next(self: *Framer, reader: *std.Io.Reader) Error!?[]const u8 {
        if (self.finished) return null;
        self.compact();
        const unit = self.scheme.unitBytes().?;
        while (true) {
            // emit complete units together, retain trailing partial bytes
            const complete_bytes = self.length - self.length % unit;
            if (complete_bytes != 0) {
                self.consumed = complete_bytes;
                return self.buffer[0..complete_bytes];
            }
            // reject a partial final unit at eof
            if (self.eof) {
                if (self.length != 0) return error.TruncatedData;
                self.finished = true;
                return null;
            }
            // remaining partial unit is smaller than the buffer validated during init
            const count = try reader.readSliceShort(self.buffer[self.length..]);
            if (count == 0) self.eof = true;
            self.length += count;
        }
    }

    /// discard emitted bytes while retaining incomplete final unit
    fn compact(self: *Framer) void {
        if (self.consumed != 0) {
            std.mem.copyForwards(
                u8,
                self.buffer[0..self.length - self.consumed],
                self.buffer[self.consumed..self.length],
            );
            self.length -= self.consumed;
            self.consumed = 0;
        }
    }

    /// discard buffered bytes after seeking
    pub fn reset(self: *Framer) void {
        self.length = 0;
        self.consumed = 0;
        self.eof = false;
        self.finished = false;
    }

    /// free the input buffer
    pub fn deinit(self: *Framer) void {
        self.allocator.free(self.buffer);
        self.* = undefined;
    }
};
