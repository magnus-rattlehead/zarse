//! decode vorbis packets
//! ref: spec section 4, stb_vorbis.c, libvorbis block.c
const std = @import("std");
const types = @import("../types.zig");
const Codec = @import("../Codec.zig");
const vorbis = @import("vorbis/root.zig");
const Self = @This();

pub const Error = types.Error;
pub const Config = types.VorbisConfig;
pub const preroll_frames: u64 = 0;

decoder: vorbis.Decoder,
channels: u16,
max_frames: u32,

/// parse headers and allocate decoder state; the source supplies seek overlap
pub fn init(
    allocator: std.mem.Allocator,
    config: Config,
    format: types.AudioFormat,
    options: types.Options,
) Error!Codec {
    for (config.headers) |bytes| {
        if (bytes.len > options.bufferCapacity()) return error.LimitExceeded;
        if (bytes.len == 0) return error.InvalidData;
    }
    const info = vorbis.Info.readIdentification(config.headers[0]) catch |err| {
        return mapError(err);
    };
    if (
        info.sample_rate != format.sample_rate
            or info.channels != format.channels
    )
        return error.InvalidData;
    vorbis.validateHeaderPrefix(config.headers[1], vorbis.packet_comment)
        catch |err| return mapError(err);
    var setup = vorbis.Setup.init(allocator, info, config.headers[2])
        catch |err| return mapError(err);
    defer setup.deinit();
    return initTakeSetup(allocator, &setup, format, options);
}

/// empty source before validation; consume its parsed setup even on failure
pub fn initTakeSetup(
    allocator: std.mem.Allocator,
    source: *vorbis.Setup,
    format: types.AudioFormat,
    options: types.Options,
) Error!Codec {
    var owned_setup = source.take();
    defer owned_setup.deinit();
    const info = owned_setup.info;
    if (
        info.sample_rate != format.sample_rate
            or info.channels != format.channels
    ) {
        return error.InvalidData;
    }
    var decoder = vorbis.Decoder.initTakeSetup(allocator, &owned_setup)
        catch |err| return mapError(err);
    defer decoder.deinit();
    // two long blocks produce at most half a block of samples per channel
    const max_frames: u32 = @intCast(info.blocksize1() / 2);
    var unit = Self{
        .decoder = decoder.take(),
        .channels = format.channels,
        .max_frames = max_frames,
    };
    defer unit.deinit();
    return Codec.UnitAdapter(Self)
        .initTake(
            allocator,
            &unit,
            .vorbis,
            .packets,
            max_frames,
            format,
            options,
        );
}

/// transfer decoder ownership; leave self safe to deinit
pub fn take(self: *Self) Self {
    return .{
        .decoder = self.decoder.take(),
        .channels = self.channels,
        .max_frames = self.max_frames,
    };
}

/// decode packet to interleaved pcm, return frames per channel
/// output must be channel aligned and hold configured max
/// first packet may return zero, input failures require reset
pub fn decodeUnit(self: *Self, bytes: []const u8, output: []f32) Error!usize {
    try Codec.validateOutput(output, self.channels, self.max_frames);
    if (bytes.len == 0) return error.InvalidData;
    return self.decoder.decodePacket(bytes, output) catch |err| {
        return mapError(err);
    };
}

/// discard synthesis overlap before decoding from a new position
pub fn reset(self: *Self) Error!void {
    self.decoder.reset();
}

/// release the decoder state
pub fn deinit(self: *Self) void {
    self.decoder.deinit();
}

/// share parsed headers with the container for block size checks
pub const Headers = struct {
    allocator: std.mem.Allocator,
    /// stream params of the identification header
    info: vorbis.Info = .{},
    block_flags: [64]bool = @splat(false),
    mode_count: u8 = 0,
    setup: ?vorbis.Setup = null,
    /// number of headers seen so far
    count: u8 = 0,

    /// initialize header state; the caller must call deinit
    pub fn init(self: *Headers, allocator: std.mem.Allocator) void {
        self.* = .{ .allocator = allocator };
    }

    pub fn add(self: *Headers, bytes: []const u8) Error!void {
        if (self.count >= 3) return error.InvalidState;
        switch (self.count) {
            0 => {
                self.info = vorbis.Info.readIdentification(bytes) catch |err| {
                    return mapError(err);
                };
            },
            1 => vorbis.validateHeaderPrefix(bytes, vorbis.packet_comment)
                catch |err| return mapError(err),
            2 => {
                var setup = vorbis.Setup.init(self.allocator, self.info, bytes)
                    catch |err| return mapError(err);
                defer setup.deinit();
                for (setup.modes, 0..) |mode, index|
                    self.block_flags[index] = mode.blockflag;
                self.mode_count = @intCast(setup.modes.len);
                self.setup = setup.take();
            },
            else => unreachable,
        }
        self.count += 1;
    }

    /// return transform size selected by packet mode, headers must be complete
    pub fn blockSize(self: *const Headers, bytes: []const u8) Error!u16 {
        if (self.count != 3 or self.mode_count == 0) return error.InvalidState;
        var reader: vorbis.Reader = .init(bytes);
        if ((reader.read(1) catch |err| return mapError(err)) != 0)
            return error.InvalidData;
        const mode = reader.read(vorbis.ilog(self.mode_count - 1))
            catch |err| return mapError(err);
        if (mode >= self.mode_count) return error.InvalidData;
        return @intCast(
            if (self.block_flags[mode])
                self.info.blocksize1()
            else
                self.info.blocksize0(),
        );
    }

    pub fn deinit(self: *Headers) void {
        if (self.setup) |*setup| setup.deinit();
        self.* = undefined;
    }

    /// transfer setup tables; retain only the container's block-size metadata
    pub fn takeSetup(self: *Headers) ?vorbis.Setup {
        const setup = if (self.setup) |*setup| setup.take() else return null;
        self.setup = null;
        return setup;
    }
};

/// preserve backend failures instead of reporting every failure as corrupt input
pub fn mapError(err: vorbis.Error) Error {
    return switch (err) {
        error.EndOfStream => error.TruncatedData,
        error.InvalidBitCount => error.InvalidData,
        else => |other| other,
    };
}
