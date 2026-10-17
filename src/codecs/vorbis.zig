//! decode vorbis packets using libvorbis, which owns internal allocations
const std = @import("std");
const types = @import("../types.zig");
const Codec = @import("../Codec.zig");
const libvorbis = @cImport({
        @cInclude("vorbis/codec.h");
    });
comptime {
    _ = @import("ogg_compat.zig");
}
const Self = @This();

pub const Error = types.Error;
pub const Config = types.VorbisConfig;
pub const preroll_frames: u64 = 0;

allocator: std.mem.Allocator,
headers: *Headers,
dsp: *libvorbis.vorbis_dsp_state,
block: *libvorbis.vorbis_block,
channels: u16,
sequence: i64 = 0,

/// parse headers and allocate decoder state; the source supplies seek overlap
pub fn init(
    allocator: std.mem.Allocator,
    config: Config,
    format: types.AudioFormat,
    options: types.Options,
) Error!Codec {
    const headers = try allocator.create(Headers);
    headers.init();
    errdefer {
        headers.deinit();
        allocator.destroy(headers);
    }
    // validate all three headers before constructing synthesis state
    for (config.headers) |bytes| {
        if (bytes.len > options.bufferCapacity()) return error.LimitExceeded;
        try headers.add(bytes);
    }
    if (
        headers.info.rate != format.sample_rate
            or headers.info.channels != format.channels
            or format.channels == 0
            or format.channels > 255
    )
        return error.InvalidData;
    // allocate stable dsp and block addresses for pointers retained by libvorbis
    const dsp = try allocator.create(libvorbis.vorbis_dsp_state);
    errdefer allocator.destroy(dsp);
    dsp.* = std.mem.zeroes(libvorbis.vorbis_dsp_state);
    if (libvorbis.vorbis_synthesis_init(dsp, &headers.info) != 0)
        return error.InvalidData;
    errdefer libvorbis.vorbis_dsp_clear(dsp);
    const block = try allocator.create(libvorbis.vorbis_block);
    errdefer allocator.destroy(block);
    block.* = std.mem.zeroes(libvorbis.vorbis_block);
    if (libvorbis.vorbis_block_init(dsp, block) != 0) return error.InvalidData;
    errdefer _ = libvorbis.vorbis_block_clear(block);
    // two long blocks produce at most half a block of samples per channel
    const max_frames: u32 = @intCast(
        @divTrunc(libvorbis.vorbis_info_blocksize(&headers.info, 1), 2),
    );
    const unit = Self{
        .allocator = allocator,
        .headers = headers,
        .dsp = dsp,
        .block = block,
        .channels = format.channels,
    };
    const adapter = try Codec.UnitAdapter(Self)
        .init(allocator, unit, .packets, max_frames, format, options);
    return adapter.interface(.vorbis);
}

/// decode packet into interleaved output, first overlap packet may produce zero frames
pub fn decodeUnit(self: *Self, bytes: []const u8, output: []f32) Error!usize {
    var packet = try makePacket(bytes, self.sequence);
    if (libvorbis.vorbis_synthesis(self.block, &packet) != 0)
        return error.InvalidData;
    // combine adjacent windows; the first packet only saves overlap
    if (libvorbis.vorbis_synthesis_blockin(self.dsp, self.block) != 0)
        return error.InvalidData;
    self.sequence = std.math.add(i64, self.sequence, 1)
        catch return error.LimitExceeded;
    var pcm: [*c][*c]f32 = null;
    // borrow planar output until synthesis read releases it
    const count = libvorbis.vorbis_synthesis_pcmout(self.dsp, &pcm);
    if (count < 0) return error.InvalidData;
    const frames: usize = @intCast(count);
    if (frames > output.len / self.channels) return error.LimitExceeded;
    // interleave channel planes before releasing samples
    for (0..frames) |frame| {
        for (0..self.channels) |channel| {
            output[frame * self.channels + channel] = pcm[channel][frame];
        }
    }
    // release samples only after copying them into adapter output
    if (libvorbis.vorbis_synthesis_read(self.dsp, count) != 0)
        return error.InvalidData;
    return frames;
}

/// clear synthesis overlap and restart packet numbering
pub fn reset(self: *Self) void {
    const result = libvorbis.vorbis_synthesis_restart(self.dsp);
    std.debug.assert(result == 0);
    self.sequence = 0;
}

/// clear libvorbis state before freeing owned backing allocations
pub fn deinit(self: *Self) void {
    _ = libvorbis.vorbis_block_clear(self.block);
    libvorbis.vorbis_dsp_clear(self.dsp);
    self.headers.deinit();
    self.allocator.destroy(self.block);
    self.allocator.destroy(self.dsp);
    self.allocator.destroy(self.headers);
}

/// wrap borrowed bytes for synchronous libvorbis calls
fn makePacket(bytes: []const u8, sequence: i64) Error!libvorbis.ogg_packet {
    // leave granules unknown; the pipeline handles timing and trimming
    return .{
        .packet = @constCast(bytes.ptr),
        .bytes = std.math.cast(c_int, bytes.len)
            orelse return error.LimitExceeded,
        .b_o_s = 0,
        .e_o_s = 0,
        .granulepos = -1,
        .packetno = sequence,
    };
}

/// share parsed headers with the container for block size checks
/// keep this state at a stable address for libvorbis
pub const Headers = struct {
    info: libvorbis.vorbis_info = std.mem.zeroes(libvorbis.vorbis_info),
    comments: libvorbis.vorbis_comment = std.mem.zeroes(
        libvorbis.vorbis_comment,
    ),
    count: u8 = 0,
    /// initialize header state at stable address, caller must deinit
    pub fn init(self: *Headers) void {
        self.* = .{};
        libvorbis.vorbis_info_init(&self.info);
        libvorbis.vorbis_comment_init(&self.comments);
    }

    /// validate next header in order, retain codec setup through cleanup
    pub fn add(self: *Headers, bytes: []const u8) Error!void {
        // libvorbis header initialization has no error result, detect failed setup allocation
        if (self.info.codec_setup == null) return error.OutOfMemory;
        var packet = try makePacket(bytes, self.count);
        // only identification starts the stream; libvorbis checks header order
        packet.b_o_s = @intFromBool(self.count == 0);
        if (
            self.count >= 3
                or libvorbis.vorbis_synthesis_headerin(
                    &self.info,
                    &self.comments,
                    &packet,
                ) != 0
        )
            return error.InvalidData;
        self.count += 1;
        if (self.count == 3) {
            // discard validated comments after setup
            libvorbis.vorbis_comment_clear(&self.comments);
        }
    }

    /// return transform size selected by packet mode, headers must be complete
    pub fn blockSize(self: *Headers, bytes: []const u8) Error!u16 {
        var packet = try makePacket(bytes, 0);
        // setup modes select the short or long transform size
        const block_size = libvorbis.vorbis_packet_blocksize(
            &self.info,
            &packet,
        );
        if (block_size < 64 or block_size > 8192) return error.InvalidData;
        return @intCast(block_size);
    }

    /// release parsed setup and comment allocations
    pub fn deinit(self: *Headers) void {
        libvorbis.vorbis_comment_clear(&self.comments);
        libvorbis.vorbis_info_clear(&self.info);
    }
};
