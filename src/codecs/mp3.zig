//! decode complete mpeg 1 layer iii frames into f32 pcm samples, alternating channels
//! source: https://github.com/lieff/minimp3
const std = @import("std");
const bit_reader = @import("bit_reader.zig");
const types = @import("../types.zig");
const Codec = @import("../Codec.zig");
const framing = @import("../containers/mp3/framing.zig");
const Header = framing.Header;
const tables = @import("mp3/tables.zig");
const Self = @This();

pub const Error = types.Error;
// container seek already goes back far enough to rebuild saved bytes and samples
pub const preroll_frames: u64 = 0;
// 1152 pcm frames with room for two channels
pub const max_samples_per_frame = 1152 * 2;
// main data can start up to 511 bytes before current frame payload
const max_bitreservoir_bytes = 511;
// encoded block type for three short windows
const short_block_type = 2;
// encoded block type for switching back to long windows
const stop_block_type = 3;
// adjust values by a power of two before producing pcm
const bits_dequantizer_out = -1;
// largest quarter step gain exponent after subtracting fixed offsets
const max_scf = 255 + bits_dequantizer_out * 4 - 210;
// round gain exponent limit up to a multiple of four
const max_scfi = (max_scf + 3) & ~@as(i32, 3);

format: types.AudioFormat,
state: State = .{},
scratch: Scratch = .{},

/// saved bytes and samples needed by following frames
const State = struct {
    // saved end of previous transform, combined with next granule
    mdct_overlap: [2][9 * 32]f32 = @splat(@splat(0)),
    // saved samples for final pcm output across blocks
    qmf_state: [15 * 2 * 32]f32 = @splat(0),
    // number of saved bytes in reservoir
    reservoir_len: usize = 0,
    // previous header used to check whether stream settings still match
    header: [4]u8 = @splat(0),
    reservoir: [max_bitreservoir_bytes]u8 = @splat(0),
};

/// decoding settings for one channel granule, each granule produces 576 pcm frames
const Granule = struct {
    sfbtab: []const u8 = &.{},
    part_23_length: u16 = 0,
    big_values: u16 = 0,
    scalefac_compress: u8 = 0,
    global_gain: u8 = 0,
    block_type: u8 = 0,
    mixed_block_flag: u8 = 0,
    n_long_sfb: u8 = 0,
    n_short_sfb: u8 = 0,
    table_select: [3]u8 = @splat(0),
    region_count: [3]u8 = @splat(0),
    subblock_gain: [3]u8 = @splat(0),
    preflag: u8 = 0,
    scalefac_scale: u8 = 0,
    count1_table: u8 = 0,
    scfsi: u8 = 0,
};

/// temporary buffers reused while decoding frames
const Scratch = struct {
    main_bits: Bits = .{},
    maindata: [max_bitreservoir_bytes + framing.max_frame_bytes]u8 = @splat(0),
    // up to two granules for each of two channels
    granules: [4]Granule = @splat(.{}),
    /// both channels share one array for stereo processing
    grbuf: [2 * 576]f32 = @splat(0),
    scf: [40]f32 = @splat(0),
    syn: [(18 + 15) * 2 * 32]f32 = @splat(0),
    ist_pos: [2][39]u8 = @splat(@splat(0)),
};

/// borrow encoded bytes with an msb first cursor; short fields return errors
/// failed reads, peeks and skips leave the cursor unchanged
const Bits = bit_reader.SliceReader(.big);

/// select huffman entry from available prefix, short codes need fewer bits
/// read and advance still check codeword and sign bits against granule end
fn huffmanIndex(reader: *const Bits, width: u6) bit_reader.Error!u32 {
    const count: u6 = @intCast(@min(width, reader.remaining()));
    const prefix = try reader.peek(count);
    if (count == 0) return 0;
    return prefix << @as(u5, @intCast(width - count));
}

/// check mono or stereo format, allocate packet adapter and pcm output
pub fn init(
    allocator: std.mem.Allocator,
    format: types.AudioFormat,
    options: types.Options,
) Error!Codec {
    if (format.channels == 0 or format.channels > 2 or format.sample_rate == 0)
        return error.InvalidData;
    // mpeg 1 layer iii has three sample rates
    switch (format.sample_rate) {
        32000, 44100, 48000 => {},
        else => return error.UnsupportedEncoding,
    }
    // field defaults clear saved bytes, samples and temporary buffers
    const unit = Self{ .format = format };
    // mpeg 1 produces at most 1152 pcm frames per channel per encoded packet
    return Codec.UnitAdapter(Self)
        .init(allocator, unit, .mp3, .packets, 1152, format, options);
}

/// decode one complete mpeg 1 layer iii frame, return pcm frames per channel
/// save available main data and return zero while earlier bytes are missing
/// output must be channel aligned and hold 1152 frames; input failures require reset before reuse
pub fn decodeUnit(self: *Self, bytes: []const u8, output: []f32) Error!usize {
    try Codec.validateOutput(output, self.format.channels, 1152);
    if (bytes.len == 0) return error.InvalidData;
    return self.decodeFrame(bytes, output)
        catch |err| switch (err) {
            error.EndOfStream => error.TruncatedData,
            error.InvalidBitCount => error.InvalidData,
            else => |other| return other,
        };
}

fn decodeFrame(
    self: *Self,
    bytes: []const u8,
    output: []f32,
) (Error || bit_reader.Error)!usize {
    const header = try Header.parse(bytes);
    // limit decoding to mpeg 1 before reading version specific fields
    if (header.version() != .mpeg1) return error.UnsupportedEncoding;
    // free format uses packet length, other frames give bitrate for calculating length
    if (bytes.len < header.minimumFrameBytes()) return error.TruncatedData;
    if (bytes.len > framing.max_frame_bytes) return error.InvalidData;
    if (header.frameBytes(null)) |length| {
        if (bytes.len < length) return error.TruncatedData;
        if (bytes.len > length) return error.InvalidData;
    }
    // output rate and channel count must match configured format
    const channels = header.channels();
    if (
        header.sampleRate() != self.format.sample_rate
            or channels != self.format.channels
    )
        return error.FormatChanged;
    const frames: usize = 1152;
    if (output.len < frames * channels) return error.InvalidBufferSize;

    // clear saved bytes and samples for first frame or changed stream settings
    const previous = Header{ .bytes = self.state.header };
    if (self.state.header[0] != 0xff or !previous.compatible(header))
        self.state = .{};
    self.state.header = header.bytes;
    // invalid side information clears saved bytes and samples before next packet
    errdefer self.state = .{};

    var bits = Bits.init(bytes[4..]);
    // crc takes two bytes before side information
    if (header.hasCrc()) _ = try bits.read(16);
    // two granules per channel, each produces 576 frames
    const granules = self.scratch.granules[0..2 * channels];
    const main_data_begin = try readSideInfo(&bits, granules, header);

    // combine required saved bytes with current frame main data
    const main_data = bytes[header.minimumFrameBytes()..];
    const ready = restoreReservoir(
        &self.state,
        main_data,
        &self.scratch,
        main_data_begin,
    );
    // keep unused main data even when earlier reservoir bytes were unavailable
    defer saveReservoir(&self.state, &self.scratch.main_bits);
    if (!ready) return 0;

    for (0..2) |index| {
        // unused frequency values stay zero in active channels
        @memset(self.scratch.grbuf[0..576 * @as(usize, channels)], 0);
        const first = index * channels;
        try decodeGranule(
            &self.state,
            &self.scratch,
            granules[first..][0..channels],
            channels,
        );
        // produce 18 blocks of 32 pcm frames, alternate channels in output
        const samples = 576 * @as(usize, channels);
        synthGranule(
            &self.state.qmf_state,
            &self.scratch.grbuf,
            channels,
            output[index * samples..][0..samples],
            &self.scratch.syn,
        );
    }
    return frames;
}

/// clear saved bytes, transform overlap and synthesis history after a seek
pub fn reset(self: *Self) Error!void {
    self.state = .{};
}

/// adapter owns codec memory and frees it during cleanup
pub fn deinit(_: *Self) void {}

/// read granule settings and number of earlier bytes needed, reject invalid side information
fn readSideInfo(
    bits: *Bits,
    granules: []Granule,
    header: Header,
) (Error || bit_reader.Error)!usize {
    const channels: usize = header.channels();
    const count = channels * 2;
    const band_table = header.bandTableIndex();
    // nine bit count of how far back main data starts
    const main_data_begin: usize = try bits.read(9);
    // pack private bits and reuse flags as minimp3 does, mono uses nine bits, stereo eleven
    var scfsi = try bits.read(@intCast(7 + count));

    var part_23_sum: usize = 0;
    for (granules[0..count]) |*granule| {
        // mono skips four extra bits to follow same flag layout as stereo
        if (channels == 1) scfsi <<= 4;
        // buffers are reused across frames, clear fields this block type leaves out
        granule.* = .{};
        // part 2 holds scalefactors, part 3 holds huffman coded frequency values
        granule.part_23_length = @intCast(try bits.read(12));
        part_23_sum += granule.part_23_length;
        granule.big_values = @intCast(try bits.read(9));
        // each big value entry covers two of the 576 frequency values
        if (granule.big_values > 288) return error.InvalidData;
        granule.global_gain = @intCast(try bits.read(8));
        granule.scalefac_compress = @intCast(try bits.read(4));
        // normal long blocks have 22 scalefactor bands
        granule.sfbtab = &tables.g_scf_long[band_table];
        granule.n_long_sfb = 22;

        if ((try bits.read(1)) != 0) {
            // switched blocks choose start, short or stop windows, zero is invalid here
            granule.block_type = @intCast(try bits.read(2));
            if (granule.block_type == 0) return error.InvalidData;
            granule.mixed_block_flag = @intCast(try bits.read(1));
            // switched blocks have two huffman regions, 255 covers all remaining bands
            granule.region_count[0] = 7;
            granule.region_count[1] = 255;
            if (granule.block_type == short_block_type) {
                // short windows read new scalefactors, clear reuse flags for this channel
                scfsi &= 0x0f0f;
                if (granule.mixed_block_flag == 0) {
                    granule.region_count[0] = 8;
                    granule.sfbtab = &tables.g_scf_short[band_table];
                    granule.n_long_sfb = 0;
                    // 13 bands for each of three short windows
                    granule.n_short_sfb = 39;
                } else {
                    // mixed blocks use long windows for lower bands, short windows for higher bands
                    granule.sfbtab = &tables.g_scf_mixed[band_table];
                    granule.n_long_sfb = 8;
                    granule.n_short_sfb = 30;
                }
            }
            for (granule.table_select[0..2]) |*table|
                table.* = @intCast(try bits.read(5));
            // separate gain adjustment for each of the three short windows
            for (&granule.subblock_gain) |*gain|
                gain.* = @intCast(try bits.read(3));
        } else {
            // long blocks give three huffman table choices and two region lengths
            for (&granule.table_select) |*table|
                table.* = @intCast(try bits.read(5));
            granule.region_count[0] = @intCast(try bits.read(4));
            granule.region_count[1] = @intCast(try bits.read(3));
            granule.region_count[2] = 255;
        }

        // preflag enables extra reduction for higher bands
        granule.preflag = @intCast(try bits.read(1));
        granule.scalefac_scale = @intCast(try bits.read(1));
        granule.count1_table = @intCast(try bits.read(1));
        // take next four reuse flags, one per scalefactor group
        granule.scfsi = @intCast((scfsi >> 12) & 15);
        scfsi <<= 4;
    }
    const limit = bits.bytes.len * 8;
    // scalefactor and huffman bits must fit current main data plus requested earlier bytes
    if (part_23_sum > limit - bits.position + main_data_begin * 8)
        return error.InvalidData;
    return main_data_begin;
}

/// read or reuse scalefactor groups, save values for later stereo processing
fn readScalefactors(
    scf: []u8,
    ist_pos: []u8,
    sizes: []const u8,
    counts: []const u8,
    bits: *Bits,
    scfsi: u8,
) (Error || bit_reader.Error)!void {
    var offset: usize = 0;
    var reuse = scfsi;
    for (0..4) |group| {
        const count = counts[group];
        if (count == 0) break;
        const values = scf[offset..][0..count];
        const positions = ist_pos[offset..][0..count];
        if (reuse & 8 != 0) {
            // copy same group from previous granule, use no new bits
            @memcpy(values, positions);
        } else if (sizes[group] == 0) {
            // zero width encodes zero for every band in this group
            @memset(values, 0);
            @memset(positions, 0);
        } else {
            const width = sizes[group];
            for (values, positions) |*value, *position| {
                value.* = @intCast(try bits.read(@intCast(width)));
                position.* = value.*;
            }
        }
        offset += count;
        // move next reuse flag into bit three
        reuse *= 2;
    }
    // final bands use zero scalefactors, short blocks need one for each window
    @memset(scf[offset..][0..3], 0);
}

/// reduce value by factor of two for every four exponent steps
/// exponent must be zero or greater
fn scaleQuarterExponent(value: f32, exponent: i32) f32 {
    var scaled = value;
    var remaining = exponent;
    while (true) {
        // handle at most 30 whole steps at once, keeps integer shift within 32 bits
        const step = @min(120, remaining);
        const whole: f32 = @floatFromInt(
            @as(u32, 1 << 30) >> @intCast(step >> 2),
        );
        // table includes division by two to power thirty, whole factor balances it
        scaled *= tables.g_expfrac[@intCast(step & 3)] * whole;
        remaining -= step;
        if (remaining <= 0) return scaled;
    }
}

/// turn encoded scalefactors into multipliers for each frequency band
fn decodeScalefactors(
    header: Header,
    ist_pos: []u8,
    bits: *Bits,
    granule: *const Granule,
    scf: []f32,
) (Error || bit_reader.Error)!void {
    // table rows give group sizes for long, mixed and short blocks
    const layout: usize = @as(usize, @intFromBool(granule.n_short_sfb != 0))
        + @intFromBool(granule.n_long_sfb == 0);
    const partition = tables.g_scf_partitions[layout][0..4];
    // compression field packs two widths, each shared by two scalefactor groups
    const part = tables.g_scfc_decode[granule.scalefac_compress];
    const sizes = [_]u8{ part >> 2, part >> 2, part & 3, part & 3 };
    var encoded: [40]u8 = undefined;
    try readScalefactors(
        &encoded,
        ist_pos,
        &sizes,
        partition,
        bits,
        granule.scfsi,
    );

    const shift: u3 = @intCast(granule.scalefac_scale + 1);
    if (granule.n_short_sfb != 0) {
        // convert each short window gain to scalefactor steps, add its reduction
        const gain_shift = 3 - shift;
        var band: usize = granule.n_long_sfb;
        const end = band + granule.n_short_sfb;
        while (band < end) : (band += 3) {
            for (granule.subblock_gain, 0..) |gain, window|
                encoded[band + window] +%= gain << gain_shift;
        }
    } else if (granule.preflag != 0) {
        // long blocks can reduce highest ten coded frequency bands further
        for (tables.g_preamp, 0..) |extra, band| encoded[11 + band] +%= extra;
    }

    // subtract 210 from global gain, mid side stereo needs another half power of two reduction
    const gain_exponent = @as(i32, granule.global_gain)
        + bits_dequantizer_out * 4
        - 210
        - @as(i32, if (header.isMidSideStereo()) 2 else 0);
    const gain = scaleQuarterExponent(
        @floatFromInt(@as(u32, 1) << @intCast(@divTrunc(max_scfi, 4))),
        max_scfi - gain_exponent,
    );
    const band_count = @as(usize, granule.n_long_sfb) + granule.n_short_sfb;
    // scalefac scale chooses half or whole powers of two for reducing each band
    for (scf[0..band_count], encoded[0..band_count]) |*scale, value|
        scale.* = scaleQuarterExponent(gain, @as(i32, value) << shift);
}

/// raise a nonnegative value to power four over three
fn pow43(value: i32) f32 {
    if (value < 129) return tables.g_pow43[@intCast(16 + value)];
    var x = value;
    var multiplier: f32 = 256;
    if (x < 1024) {
        // scale smaller values up to reuse same table range
        x <<= 3;
        multiplier = 16;
    }
    // round to nearest table entry, measure distance from that entry
    const adjustment = 2 * x & 64;
    const fraction = @as(f32, @floatFromInt((x & 63) - adjustment))
        / @as(f32, @floatFromInt((x & ~@as(i32, 63)) + adjustment));
    const base = tables.g_pow43[@intCast(16 + ((x + adjustment) >> 6))];
    // keep reference calculation order so f32 rounding stays the same
    return base
        * (1
            + fraction * (@as(f32, 4.0 / 3.0) + fraction * @as(f32, 2.0 / 9.0)))
        * multiplier;
}

/// decode frequency values and apply band scales, caller clears output first
fn decodeHuffman(
    output: []f32,
    bits: *Bits,
    granule: *const Granule,
    scf: []const f32,
    bit_limit: usize,
) (Error || bit_reader.Error)!void {
    var reader = bits.*;
    // next channel starts at declared granule end, including any unused padding bits
    defer bits.position = bit_limit;
    var region: usize = 0;
    var band: usize = 0;
    var sample: usize = 0;
    var pairs_left: i32 = granule.big_values;
    var scale: f32 = 0;
    while (pairs_left > 0) {
        const table = granule.table_select[region];
        const codebook = tables.tabs[
            @as(usize, @intCast(tables.tabindex[table]))..
        ];
        const extra_bits = tables.g_linbits[table];
        // region counts encode number of bands minus one
        var bands_left = @as(usize, granule.region_count[region]) + 1;
        region += 1;
        while (true) {
            const band_pairs = @as(i32, granule.sfbtab[band] / 2);
            scale = scf[band];
            band += 1;
            const count: usize = @intCast(@min(pairs_left, band_pairs));
            for (0..count) |_| {
                // first five bits choose a value or a smaller lookup table
                var width: u6 = 5;
                var leaf = codebook[try huffmanIndex(&reader, width)];
                while (leaf < 0) {
                    try reader.advance(width);
                    width = @intCast(leaf & 7);
                    const next = @as(
                        i32,
                        @intCast(try huffmanIndex(&reader, width)),
                    )
                        - (@as(i32, leaf) >> 3);
                    leaf = codebook[@intCast(next)];
                }
                // high bits hold code length, low two groups of four hold pair values
                try reader.advance(@as(u16, @intCast(leaf)) >> 8);
                for (0..2) |_| {
                    var value: i32 = leaf & 15;
                    if (value == 15 and extra_bits != 0) {
                        // extra bits extend largest table value before its sign bit
                        value += @intCast(
                            try reader.read(@intCast(extra_bits)),
                        );
                        const sign: f32 = if ((try reader.peek(1)) != 0)
                            -1
                        else
                            1;
                        output[sample] = scale * pow43(value) * sign;
                    } else {
                        // table stores positive and negative small values
                        const negative: i32 = if (value == 0)
                            0
                        else
                            @intCast(try reader.peek(1));
                        output[sample] = tables.g_pow43[
                            @intCast(16 + value - 16 * negative)
                        ]
                            * scale;
                    }
                    if (value != 0) try reader.advance(1);
                    sample += 1;
                    leaf >>= 4;
                }
            }
            // subtract whole band so count1 can pick up partway through same band
            pairs_left -= band_pairs;
            bands_left -= 1;
            if (pairs_left <= 0 or bands_left == 0) break;
        }
    }

    // remaining values are zero or signed one, decoded four at a time
    var pairs_until_scale = 1 - pairs_left;
    const count1_book: []const u8 = if (granule.count1_table != 0)
        &tables.tab33
    else
        &tables.tab32;
    while (sample < 576 and reader.remaining() != 0) {
        var leaf = count1_book[try huffmanIndex(&reader, 4)];
        if (leaf & 8 == 0) {
            // some four bit entries need up to three more bits to choose a code
            const extra: u6 = @intCast(leaf & 3);
            const mask = (@as(u32, 1) << @as(u5, @intCast(extra))) - 1;
            leaf = count1_book[
                (leaf >> 3) + (try huffmanIndex(&reader, 4 + extra) & mask)
            ];
        }
        // minimp3 checks codeword end before reading its sign bits
        // an incomplete final count1 group belongs to granule stuffing
        const code_bits = leaf & 7;
        if (code_bits > reader.remaining()) break;
        const sign_bits = @popCount(leaf & 0xf0);
        if (@as(usize, code_bits) + sign_bits > reader.remaining()) break;
        try reader.advance(code_bits);
        for (0..4) |component| {
            if (component % 2 == 0) {
                // each band length counts pairs, a group can cross into next band
                pairs_until_scale -= 1;
                if (pairs_until_scale == 0) {
                    pairs_until_scale = granule.sfbtab[band] / 2;
                    if (pairs_until_scale == 0) return;
                    scale = scf[band];
                    band += 1;
                }
            }
            if (leaf & (@as(u8, 128) >> @as(u3, @intCast(component))) != 0) {
                output[sample] = if ((try reader.peek(1)) != 0)
                    -scale
                else
                    scale;
                try reader.advance(1);
            }
            sample += 1;
        }
    }
}

/// convert mid and side values into left and right channels
fn midSideStereo(samples: []f32, count: usize) void {
    // each channel has 576 values, keep that spacing when processing part of a band
    for (samples[0..count], samples[576..][0..count]) |*left, *right| {
        const mid = left.*;
        const side = right.*;
        // scalefactor stage already divided values by square root of two
        left.* = mid + side;
        right.* = mid - side;
    }
}

/// split one coded band between left and right channels using separate scales
fn intensityStereoBand(
    samples: []f32,
    count: usize,
    left_scale: f32,
    right_scale: f32,
) void {
    for (samples[0..count], samples[576..][0..count]) |*left, *right| {
        const value = left.*;
        right.* = value * right_scale;
        left.* = value * left_scale;
    }
}

/// find last right channel band containing nonzero values in each window
fn stereoTopBand(
    right: []const f32,
    bands: []const u8,
    band_count: usize,
    max_band: *[3]i32,
) void {
    // negative one means every checked value in that window was zero
    max_band.* = @splat(-1);
    var first: usize = 0;
    for (bands[0..band_count], 0..) |count, band| {
        for (right[first..][0..count]) |value| {
            if (value != 0) {
                // short window bands repeat in groups of three
                max_band[band % 3] = @intCast(band);
                break;
            }
        }
        first += count;
    }
}

/// split higher bands using intensity stereo, apply mid side to remaining bands
fn processStereo(
    samples: []f32,
    ist_pos: []const u8,
    bands: []const u8,
    header: Header,
    max_band: *const [3]i32,
) void {
    const mid_side = header.isMidSideStereo();
    var first: usize = 0;
    for (bands, 0..) |count, band| {
        if (count == 0) break;
        const position = ist_pos[band];
        if (@as(i32, @intCast(band)) > max_band[band % 3] and position < 7) {
            // only split bands above last coded right band with valid stereo settings
            // seven fixed ways to divide a band between channels
            const left_scale = tables.g_pan[2 * @as(usize, position)];
            const right_scale = tables.g_pan[2 * @as(usize, position) + 1];
            // mid side bands were reduced earlier, undo that reduction before splitting
            const scale: f32 = if (mid_side) 1.41421356 else 1;
            intensityStereoBand(
                samples[first..],
                count,
                left_scale * scale,
                right_scale * scale,
            );
        } else if (mid_side) {
            midSideStereo(samples[first..], count);
        }
        first += count;
    }
}

/// find where intensity stereo starts, fill final band settings and rebuild both channels
fn intensityStereo(
    samples: []f32,
    ist_pos: []u8,
    granule: *const Granule,
    header: Header,
) void {
    const band_count = @as(usize, granule.n_long_sfb) + granule.n_short_sfb;
    var max_band: [3]i32 = undefined;
    stereoTopBand(samples[576..], granule.sfbtab, band_count, &max_band);
    if (granule.n_long_sfb != 0) {
        // blocks with long windows share one last coded band across all windows
        const last = @max(@max(max_band[0], max_band[1]), max_band[2]);
        max_band = @splat(last);
    }
    const windows: usize = if (granule.n_short_sfb != 0) 3 else 1;
    for (0..windows) |window| {
        const top = band_count - windows + window;
        const previous = top - windows;
        // final bands have no encoded setting, reuse previous band or a fixed default
        ist_pos[top] = if (max_band[window] >= @as(i32, @intCast(previous)))
            3
        else
            ist_pos[previous];
    }
    processStereo(samples, ist_pos, granule.sfbtab, header, &max_band);
}

/// put values from three short windows into frequency order within each band
/// scratch must be separate from samples, with room for all reordered values
fn reorder(samples: []f32, scratch: []f32, bands: []const u8) void {
    var first: usize = 0;
    var band: usize = 0;
    // band widths repeat for each of three windows, zero marks end of table
    while (bands[band] != 0) : (band += 3) {
        const width: usize = bands[band];
        const count = 3 * width;
        const source = samples[first..][0..count];
        const output = scratch[first..][0..count];
        for (0..width) |i| {
            // a0 a1 b0 b1 c0 c1 becomes a0 b0 c0 a1 b1 c1
            output[3 * i] = source[i];
            output[3 * i + 1] = source[width + i];
            output[3 * i + 2] = source[2 * width + i];
        }
        first += count;
    }
    // copy back after all source values have been read
    @memcpy(samples[0..first], scratch[0..first]);
}

/// reduce unwanted mixing at boundaries between long window subbands
fn antialias(samples: []f32, boundary_count: usize) void {
    // zero boundaries for short blocks, one for mixed blocks, 31 for long blocks
    for (0..boundary_count) |band| {
        // each subband has 18 values, start at first value of next subband
        const boundary = (band + 1) * 18;
        for (0..8) |i| {
            // pair high frequency end of lower subband with low frequency end of upper subband
            const upper = samples[boundary + i];
            const lower = samples[boundary - 1 - i];
            const keep = tables.g_aa[0][i];
            const mix = tables.g_aa[1][i];
            samples[boundary + i] = upper * keep - lower * mix;
            samples[boundary - 1 - i] = upper * mix + lower * keep;
        }
    }
}

/// run nine point dct used by long block imdct, overwrite input
fn dct3Nine(values: *[9]f32) void {
    // combine even inputs first, keep reference calculation order for f32 rounding
    var s0 = values[0];
    var s2 = values[2];
    var s4 = values[4];
    var s6 = values[6];
    var s8 = values[8];
    var t0 = s0 + s6 * 0.5;
    s0 -= s6;
    var t4 = (s4 + s2) * 0.93969262;
    var t2 = (s8 + s2) * 0.76604444;
    s6 = (s4 - s8) * 0.17364818;
    s4 += s8 - s2;

    s2 = s0 - s4 * 0.5;
    values[4] = s4 + s0;
    s8 = t0 - t2 + s6;
    s0 = t0 - t4 + t2;
    s4 = t0 + t4 - s6;

    // odd inputs supply remaining parts of each output pair
    var s1 = values[1];
    var s3 = values[3];
    var s5 = values[5];
    var s7 = values[7];
    s3 *= 0.86602540;
    t0 = (s5 + s1) * 0.98480775;
    t4 = (s5 - s7) * 0.34202014;
    t2 = (s1 + s7) * 0.64278761;
    s1 = (s1 - s5 - s7) * 0.86602540;

    s5 = t0 - s3 - t2;
    s7 = t4 - s3 - t0;
    s3 = t4 + s3 - t2;

    // mirrored pairs share sums, middle output was written above
    values[0] = s4 - s7;
    values[1] = s2 + s1;
    values[2] = s0 - s3;
    values[3] = s8 + s5;
    values[5] = s8 - s5;
    values[6] = s0 + s3;
    values[7] = s2 - s1;
    values[8] = s4 + s7;
}

/// run long block imdct, combine saved tail and keep new tail for next block
fn imdct36(
    samples: []f32,
    overlap: []f32,
    window: []const f32,
    band_count: usize,
) void {
    for (0..band_count) |band| {
        const block = samples[band * 18..][0..18];
        // symmetry lets nine saved values represent an 18 sample tail
        const tail = overlap[band * 9..][0..9];
        var cosine: [9]f32 = undefined;
        var sine: [9]f32 = undefined;
        cosine[0] = -block[0];
        sine[0] = block[17];
        // split 18 frequency values into two nine point transforms
        for (0..4) |i| {
            sine[8 - 2 * i] = block[4 * i + 1] - block[4 * i + 2];
            cosine[1 + 2 * i] = block[4 * i + 1] + block[4 * i + 2];
            sine[7 - 2 * i] = block[4 * i + 4] - block[4 * i + 3];
            cosine[2 + 2 * i] = -(block[4 * i + 3] + block[4 * i + 4]);
        }
        dct3Nine(&cosine);
        dct3Nine(&sine);
        for (0..4) |i| sine[2 * i + 1] = -sine[2 * i + 1];

        for (0..9) |i| {
            const previous = tail[i];
            // rotate transformed pairs, save one part for next block
            const sum = cosine[i] * tables.g_twid9[9 + i]
                + sine[i] * tables.g_twid9[i];
            tail[i] = cosine[i] * tables.g_twid9[i]
                - sine[i] * tables.g_twid9[9 + i];
            // apply window while combining previous tail with current block
            block[i] = previous * window[i] - sum * window[9 + i];
            block[17 - i] = previous * window[9 + i] + sum * window[i];
        }
    }
}

/// run three point inverse dct for short blocks
fn idct3(x0: f32, x1: f32, x2: f32, output: []f32) void {
    const middle = x1 * 0.86602540;
    const base = x0 - x2 * 0.5;
    output[1] = x0 + x2;
    output[0] = base + middle;
    output[2] = base - middle;
}

/// run one short window imdct, combine saved tail and keep new tail
fn imdct12(samples: []const f32, output: []f32, overlap: []f32) void {
    var cosine: [3]f32 = undefined;
    var sine: [3]f32 = undefined;
    // reordered windows are interleaved, take every third frequency value
    idct3(
        -samples[0],
        samples[6] + samples[3],
        samples[12] + samples[9],
        &cosine,
    );
    idct3(
        samples[15],
        samples[12] - samples[9],
        samples[6] - samples[3],
        &sine,
    );
    sine[1] = -sine[1];

    for (0..3) |i| {
        const previous = overlap[i];
        const sum = cosine[i] * tables.g_twid3[3 + i]
            + sine[i] * tables.g_twid3[i];
        overlap[i] = cosine[i] * tables.g_twid3[i]
            - sine[i] * tables.g_twid3[3 + i];
        // same constants also supply short window weights in reverse order
        output[i] = previous * tables.g_twid3[2 - i]
            - sum * tables.g_twid3[5 - i];
        output[5 - i] = previous * tables.g_twid3[5 - i]
            + sum * tables.g_twid3[2 - i];
    }
}

/// combine three short windows with different starting positions in each subband
fn imdctShort(samples: []f32, overlap: []f32, band_count: usize) void {
    for (0..band_count) |band| {
        const block = samples[band * 18..][0..18];
        const tail = overlap[band * 9..][0..9];
        // preserve frequency values while output replaces same block
        const input: [18]f32 = block[0..18].*;
        // saved tail supplies first six samples before first short window starts
        @memcpy(block[0..6], tail[0..6]);
        // windows start six samples apart, last window becomes next block tail
        imdct12(&input, block[6..12], tail[6..9]);
        imdct12(input[1..], block[12..18], tail[6..9]);
        imdct12(input[2..], tail[0..6], tail[6..9]);
    }
}

/// flip sign of every other sample in odd numbered subbands before pcm output
fn changeSign(samples: []f32) void {
    // correct frequency reversal introduced by filterbank in alternating subbands
    for (0..16) |band| {
        const block = samples[(2 * band + 1) * 18..][0..18];
        for (0..9) |i| block[2 * i + 1] = -block[2 * i + 1];
    }
}

/// choose long, short or mixed windows, handle switch back to long windows
fn imdctGranule(
    samples: []f32,
    overlap: []f32,
    block_type: u8,
    long_bands: usize,
) void {
    // mixed blocks begin with two subbands using normal long windows
    if (long_bands != 0)
        imdct36(samples, overlap, &tables.g_mdct_window[0], long_bands);

    const rest = samples[long_bands * 18..];
    const tail = overlap[long_bands * 9..];
    if (block_type == short_block_type) {
        imdctShort(rest, tail, 32 - long_bands);
    } else {
        // stop blocks use transition window, normal and start blocks share first half
        const window = &tables.g_mdct_window[
            @intFromBool(block_type == stop_block_type)
        ];
        imdct36(rest, tail, window, 32 - long_bands);
    }
}

/// keep up to 511 unused main data bytes for following frames
fn saveReservoir(state: *State, bits: *const Bits) void {
    // partly used byte belongs to current frame, round past it
    const consumed = (bits.position + 7) / 8;
    const remaining = bits.bytes.len -| consumed;
    // back pointer can reach at most 511 bytes, keep newest unused bytes
    state.reservoir_len = @min(remaining, state.reservoir.len);
    const first = bits.bytes.len - state.reservoir_len;
    @memcpy(state.reservoir[0..state.reservoir_len], bits.bytes[first..]);
}

/// add saved bytes before current main data, report whether enough saved bytes were available
fn restoreReservoir(
    state: *const State,
    main_data: []const u8,
    scratch: *Scratch,
    main_data_begin: usize,
) bool {
    // back pointer counts backwards from end of saved bytes, use what is available during warm up
    const available = @min(state.reservoir_len, main_data_begin);
    const first = state.reservoir_len - available;
    @memcpy(
        scratch.maindata[0..available],
        state.reservoir[first..state.reservoir_len],
    );
    @memcpy(scratch.maindata[available..][0..main_data.len], main_data);
    // granule reader starts at first copied byte
    scratch.main_bits = Bits.init(
        scratch.maindata[0..available + main_data.len],
    );
    // caller skips decoding when history is short, still saves data for next frame
    return state.reservoir_len >= main_data_begin;
}

/// decode one granule per channel into subband samples, caller clears grbuf first
fn decodeGranule(
    state: *State,
    scratch: *Scratch,
    granules: []const Granule,
    channels: u16,
) (Error || bit_reader.Error)!void {
    const header = Header{ .bytes = state.header };
    for (granules[0..channels], 0..) |*granule, channel| {
        // declared length includes both scalefactor and huffman bits
        const bit_limit = scratch.main_bits.position + granule.part_23_length;
        var channel_bits = scratch.main_bits;
        channel_bits.limit = bit_limit;
        try decodeScalefactors(
            header,
            &scratch.ist_pos[channel],
            &channel_bits,
            granule,
            &scratch.scf,
        );
        const samples = scratch.grbuf[channel * 576..][0..576];
        // huffman decoder leaves reader at next channel boundary
        try decodeHuffman(
            samples,
            &channel_bits,
            granule,
            &scratch.scf,
            bit_limit,
        );
        scratch.main_bits.position = bit_limit;
    }

    // rebuild left and right frequency values before transforming either channel
    if (header.isIntensityStereo()) {
        intensityStereo(
            &scratch.grbuf,
            &scratch.ist_pos[1],
            &granules[0],
            header,
        );
    } else if (header.isMidSideStereo()) {
        midSideStereo(&scratch.grbuf, 576);
    }

    for (granules[0..channels], 0..) |*granule, channel| {
        const samples = scratch.grbuf[channel * 576..][0..576];
        // mixed blocks keep first two subbands as long windows
        const long_bands: usize = if (granule.mixed_block_flag != 0) 2 else 0;
        var boundaries: usize = 31;
        if (granule.n_short_sfb != 0) {
            // leave long prefix in place, reorder only short window values
            reorder(
                samples[long_bands * 18..],
                &scratch.syn,
                granule.sfbtab[granule.n_long_sfb..],
            );
            // pure short blocks skip antialias, mixed blocks process one boundary
            boundaries = long_bands -| 1;
        }
        antialias(samples, boundaries);
        // combine saved transform tail, save new tail for next granule
        imdctGranule(
            samples,
            &state.mdct_overlap[channel],
            granule.block_type,
            long_bands,
        );
        changeSign(samples);
    }
}

/// transform 32 subbands at each of the 18 time positions for synthesis
fn dctII(samples: []f32) void {
    const Vec = @Vector(2, f32);
    // each lane handles one of two neighboring time positions
    for (0..9) |pair| {
        const time = pair * 2;
        var parts: [4][8]Vec = undefined;
        // pair mirrored subbands, split 32 point transform into four 8 point parts
        for (0..8) |i| {
            const x0: Vec = samples[i * 18 + time..][0..2].*;
            const x1: Vec = samples[(15 - i) * 18 + time..][0..2].*;
            const x2: Vec = samples[(16 + i) * 18 + time..][0..2].*;
            const x3: Vec = samples[(31 - i) * 18 + time..][0..2].*;
            const sum0 = x0 + x3;
            const sum1 = x1 + x2;
            const diff0 = (x1 - x2) * @as(Vec, @splat(tables.g_sec[3 * i]));
            const diff1 = (x0 - x3) * @as(Vec, @splat(tables.g_sec[3 * i + 1]));
            parts[0][i] = sum0 + sum1;
            parts[1][i] = (sum0 - sum1)
                * @as(Vec, @splat(tables.g_sec[3 * i + 2]));
            parts[2][i] = diff1 + diff0;
            parts[3][i] = (diff1 - diff0)
                * @as(Vec, @splat(tables.g_sec[3 * i + 2]));
        }

        for (&parts) |*part| {
            var x0 = part[0];
            var x1 = part[1];
            var x2 = part[2];
            var x3 = part[3];
            var x4 = part[4];
            var x5 = part[5];
            var x6 = part[6];
            var x7 = part[7];
            // sums handle even positions, differences handle odd positions
            var temp = x0 - x7;
            x0 += x7;
            x7 = x1 - x6;
            x1 += x6;
            x6 = x2 - x5;
            x2 += x5;
            x5 = x3 - x4;
            x3 += x4;
            x4 = x0 - x3;
            x0 += x3;
            x3 = x1 - x2;
            x1 += x2;
            part[0] = x0 + x1;
            part[4] = (x0 - x1) * @as(Vec, @splat(0.70710677));
            x5 += x6;
            x6 = (x6 + x7) * @as(Vec, @splat(0.70710677));
            x7 += temp;
            x3 = (x3 + x4) * @as(Vec, @splat(0.70710677));
            // three steps rotate odd part by pi over eight
            x5 -= x7 * @as(Vec, @splat(0.198912367));
            x7 += x5 * @as(Vec, @splat(0.382683432));
            x5 -= x7 * @as(Vec, @splat(0.198912367));
            x0 = temp - x6;
            temp += x6;
            part[1] = (temp + x7) * @as(Vec, @splat(0.50979561));
            part[2] = (x4 + x3) * @as(Vec, @splat(0.54119611));
            part[3] = (x0 - x5) * @as(Vec, @splat(0.60134488));
            part[5] = (x0 + x5) * @as(Vec, @splat(0.89997619));
            part[6] = (x4 - x3) * @as(Vec, @splat(1.30656302));
            part[7] = (temp - x7) * @as(Vec, @splat(2.56291556));
        }

        // merge parts into original subband layout, leave other time positions in place
        for (0..7) |i| {
            const first = i * 4 * 18 + time;
            samples[first..][0..2].* = parts[0][i];
            samples[first + 18..][0..2].* = parts[2][i]
                + parts[3][i]
                + parts[3][i + 1];
            samples[first + 36..][0..2].* = parts[1][i] + parts[1][i + 1];
            samples[first + 54..][0..2].* = parts[2][i + 1]
                + parts[3][i]
                + parts[3][i + 1];
        }
        const last = 28 * 18 + time;
        samples[last..][0..2].* = parts[0][7];
        samples[last + 18..][0..2].* = parts[2][7] + parts[3][7];
        samples[last + 36..][0..2].* = parts[1][7];
        samples[last + 54..][0..2].* = parts[3][7];
    }
}

/// divide synthesized samples by 32768 for f32 output
fn scalePcm(sample: f32) f32 {
    return sample * (1.0 / 32768.0);
}

/// write pcm positions zero and sixteen for one channel
fn synthPair(output: []f32, channels: u16, values: []const f32) void {
    // symmetry leaves eight terms for these two positions
    var first = (values[14 * 64] - values[0]) * 29;
    first += (values[64] + values[13 * 64]) * 213;
    first += (values[12 * 64] - values[2 * 64]) * 459;
    first += (values[3 * 64] + values[11 * 64]) * 2037;
    first += (values[10 * 64] - values[4 * 64]) * 5153;
    first += (values[5 * 64] + values[9 * 64]) * 6574;
    first += (values[8 * 64] - values[6 * 64]) * 37489;
    first += values[7 * 64] * 75038;
    output[0] = scalePcm(first);

    // position sixteen uses other pair of history lanes
    const next = values[2..];
    var middle = next[14 * 64] * 104;
    middle += next[12 * 64] * 1567;
    middle += next[10 * 64] * 9727;
    middle += next[8 * 64] * 64019;
    middle += next[6 * 64] * -9975;
    middle += next[4 * 64] * -45;
    middle += next[2 * 64] * 146;
    middle += next[0] * -5;
    output[16 * channels] = scalePcm(middle);
}

/// combine two time positions with saved history, write 64 pcm frames
fn synth(
    samples: []const f32,
    output: []f32,
    channels: u16,
    history: []f32,
) void {
    const left = samples;
    // mono fills both channel lanes from same samples
    const right = samples[576 * (channels - 1)..];
    const current = 15 * 64;
    // place subbands zero and sixteen in slots used by paired outputs
    history[current + 60..][0..4].* = .{
        left[16 * 18],
        right[16 * 18],
        left[0],
        right[0],
    };
    history[current + 124..][0..4].* = .{
        left[16 * 18 + 1],
        right[16 * 18 + 1],
        left[1],
        right[1],
    };

    for (0..2) |time| {
        for (0..channels) |channel| {
            const first = time * 32 * channels + channel;
            synthPair(
                output[first..],
                channels,
                history[60 + time * 64 + channel..],
            );
        }
    }

    for (0..15) |step| {
        // process mirrored positions from outside in, each uses sixteen window weights
        const band = 14 - step;
        const slot = current + band * 4;
        const upper = (31 - band) * 18;
        const lower = (1 + band) * 18;
        // four lanes hold left and right from each of two time positions
        history[slot..][0..4].* = .{
            left[upper],
            right[upper],
            left[upper + 1],
            right[upper + 1],
        };
        history[slot + 64..][0..2].* = .{ left[lower + 1], right[lower + 1] };
        history[slot - 64 + 2..][0..2].* = .{ left[lower], right[lower] };

        var low: @Vector(4, f32) = undefined;
        var high: @Vector(4, f32) = undefined;
        // four lanes cover two channels at two time positions
        // unroll fixed taps so first term and alternating signs stay simple
        inline for (0..8) |tap| {
            const w0: @Vector(4, f32) = @splat(
                tables.g_win[step * 16 + tap * 2],
            );
            const w1: @Vector(4, f32) = @splat(
                tables.g_win[step * 16 + tap * 2 + 1],
            );
            const recent: @Vector(4, f32) = history[slot - tap * 64..][0..4].*;
            const older: @Vector(4, f32) = history[slot - (15 - tap) * 64..][
                0..4
            ].*;
            const sum = recent * w1 + older * w0;
            const diff = if (tap % 2 == 0)
                recent * w0 - older * w1
            else
                older * w1 - recent * w0;
            if (tap == 0) {
                low = diff;
                high = sum;
            } else {
                low += diff;
                high += sum;
            }
        }

        const low_values: [4]f32 = low;
        const high_values: [4]f32 = high;
        // mirrored pcm positions share each weighted history sum
        for (0..2) |time| {
            for (0..channels) |channel| {
                const lane = time * 2 + channel;
                output[(time * 32 + 15 - band) * channels + channel] = scalePcm(
                    low_values[lane],
                );
                output[(time * 32 + 17 + band) * channels + channel] = scalePcm(
                    high_values[lane],
                );
            }
        }
    }
}

/// convert one granule to pcm, save fifteen history blocks for next call
fn synthGranule(
    qmf_state: []f32,
    samples: []f32,
    channels: u16,
    output: []f32,
    history: []f32,
) void {
    for (0..channels) |channel| dctII(samples[channel * 576..][0..576]);

    // work buffer starts with fifteen saved blocks, each block has 64 values
    @memcpy(history[0..15 * 64], qmf_state[0..15 * 64]);
    for (0..9) |pair| {
        const time = pair * 2;
        synth(
            samples[time..],
            output[time * 32 * channels..],
            channels,
            history[time * 64..],
        );
    }

    const tail = history[18 * 64..][0..15 * 64];
    if (channels == 1) {
        // mono updates left history lanes, preserve right lanes as minimp3 does
        for (0..15 * 32) |i| qmf_state[i * 2] = tail[i * 2];
    } else {
        @memcpy(qmf_state[0..15 * 64], tail);
    }
}
