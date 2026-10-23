//! packet decode and lapped synthesis
//! ref: stb_vorbis.c vorbis_decode_packet_rest(), libvorbis block.c
const std = @import("std");
const root = @import("root.zig");
const headers = @import("headers.zig");
const mdct = @import("mdct.zig");
const Setup = @import("setup.zig").Setup;
const Error = root.Error;
const Reader = root.Reader;

/// overlap history retained between packets
const State = struct {
    /// lapped channel rows, each holding twice the long block size
    timeline: []f32 = &.{},
    /// time of the first sample of the previous block
    previous_start: isize = 0,
    /// number of samples of the previous block
    previous_size: usize = 0,
    /// time of the first sample of the timeline
    timeline_start: isize = 0,
    /// number of valid samples per channel in the timeline
    timeline_len: usize = 0,
};

const vector_lanes = std.simd.suggestVectorLength(f32) orelse 4;
const SampleVector = @Vector(vector_lanes, f32);

/// undo coupling lane by lane
fn undoCoupling(magnitude: []f32, angle: []f32) void {
    var index: usize = 0;
    while (magnitude.len - index >= vector_lanes) : (index += vector_lanes) {
        const mag: SampleVector = magnitude[index..][0..vector_lanes].*;
        const ang: SampleVector = angle[index..][0..vector_lanes].*;
        const positive_mag = mag > @as(SampleVector, @splat(0));
        const positive_ang = ang > @as(SampleVector, @splat(0));
        const sum = mag + ang;
        const difference = mag - ang;
        magnitude[index..][0..vector_lanes].* = @select(
            f32,
            positive_ang,
            mag,
            @select(f32, positive_mag, sum, difference),
        );
        angle[index..][0..vector_lanes].* = @select(
            f32,
            positive_ang,
            @select(f32, positive_mag, difference, sum),
            mag,
        );
    }
    for (magnitude[index..], angle[index..]) |*m, *a| {
        const mag = m.*;
        const ang = a.*;
        if (ang > 0) {
            a.* = if (mag > 0) mag - ang else mag + ang;
        } else {
            m.* = if (mag > 0) mag + ang else mag - ang;
            a.* = mag;
        }
    }
}

fn multiplySpectrum(spectrum: []f32, curve: []const f32) void {
    var index: usize = 0;
    while (spectrum.len - index >= vector_lanes) : (index += vector_lanes) {
        const values: SampleVector = spectrum[index..][0..vector_lanes].*;
        const scales: SampleVector = curve[index..][0..vector_lanes].*;
        spectrum[index..][0..vector_lanes].* = values * scales;
    }
    for (spectrum[index..], curve[index..]) |*value, scale| value.* *= scale;
}

fn overlapAdd(output: []f32, samples: []const f32, window: []const f32) void {
    var index: usize = 0;
    while (output.len - index >= vector_lanes) : (index += vector_lanes) {
        const previous: SampleVector = output[index..][0..vector_lanes].*;
        const current: SampleVector = samples[index..][0..vector_lanes].*;
        const weights: SampleVector = window[index..][0..vector_lanes].*;
        // preserve separate multiplication and addition, including f32 rounding
        output[index..][0..vector_lanes].* = previous + current * weights;
    }
    for (
        output[index..],
        samples[index..],
        window[index..],
    ) |*value, sample, weight|
        value.* += sample * weight;
}

/// packet buffers reused by all channels and residue submaps
const Scratch = struct {
    /// spectral rows with bins values per channel
    spectrum: []f32 = &.{},
    /// unwindowed sample rows with block values per channel
    samples: []f32 = &.{},
    /// floor gain rows with bins values per channel
    curves: []f32 = &.{},
    /// window of the current block
    window: []f32 = &.{},
    /// first sample of the window of the current block
    window_start: usize = 0,
    /// sample after the window of the current block
    window_end: usize = 0,
    /// scratch for the partition classifications
    classifications: []u8 = &.{},
    /// scratch for the vector quantization values
    vectors: []f32 = &.{},
    /// scratch for the transform
    transform: []f32 = &.{},
};

/// owns setup tables, transform tables, synthesis history and scratch buffers
pub const Decoder = struct {
    allocator: std.mem.Allocator,
    setup: Setup,
    transforms: [2]mdct.Transform = @splat(.{}),
    state: State = .{},
    scratch: Scratch = .{},
    channels: usize = 0,
    bins: usize = 0,
    block: usize = 0,

    /// parse headers and allocate reusable buffers; the caller must call deinit
    pub fn init(
        allocator: std.mem.Allocator,
        info: headers.Info,
        setup: []const u8,
    ) Error!Decoder {
        var parsed = try Setup.init(allocator, info, setup);
        defer parsed.deinit();
        return initTakeSetup(allocator, &parsed);
    }

    /// empty source immediately and consume its setup, including on failure
    pub fn initTakeSetup(
        allocator: std.mem.Allocator,
        source: *Setup,
    ) Error!Decoder {
        var self: Decoder = .{ .allocator = allocator, .setup = source.take() };
        errdefer self.deinit();
        const info = self.setup.info;
        // transform tables are built for both block sizes
        self.transforms[0] = try mdct.Transform.init(
            allocator,
            info.blocksize0(),
        );
        self.transforms[1] = try mdct.Transform.init(
            allocator,
            info.blocksize1(),
        );
        // sample buffers hold one row per channel
        const rows = info.channels;
        self.channels = rows;
        self.bins = info.blocksize1() / 2;
        self.block = info.blocksize1();
        self.scratch.spectrum = try allocator.alloc(f32, rows * self.bins);
        @memset(self.scratch.spectrum, 0);
        self.scratch.samples = try allocator.alloc(f32, rows * self.block);
        @memset(self.scratch.samples, 0);
        self.scratch.curves = try allocator.alloc(f32, rows * self.bins);
        @memset(self.scratch.curves, 0);
        self.state.timeline = try allocator.alloc(f32, rows * (self.block * 2));
        @memset(self.state.timeline, 0);
        self.scratch.window = try allocator.alloc(f32, self.block);
        @memset(self.scratch.window, 0);
        self.scratch.transform = try allocator.alloc(f32, self.bins);
        // every partition's class is reused by the later residue passes
        var classes: usize = 0;
        for (
            self.setup.residues,
            self.setup.residue_types,
        ) |vector, residue_type| {
            classes = @max(
                classes,
                vector.classificationCount(residue_type, self.bins, rows),
            );
        }
        self.scratch.classifications = try allocator.alloc(u8, classes);
        @memset(self.scratch.classifications, 0);
        // vector scratch holds the values of one codebook
        var dimensions: usize = 1;
        for (self.setup.codebooks) |book| {
            dimensions = @max(dimensions, book.dimensions);
        }
        self.scratch.vectors = try allocator.alloc(f32, dimensions);
        @memset(self.scratch.vectors, 0);
        return self;
    }

    /// transfer tables, buffers and history; leave self safe to deinit
    pub fn take(self: *Decoder) Decoder {
        const owned = self.*;
        self.* = .{
            .allocator = self.allocator,
            .setup = .{ .allocator = self.setup.allocator },
        };
        return owned;
    }

    /// release all tables and buffers with the allocator supplied to init
    pub fn deinit(self: *Decoder) void {
        self.setup.deinit();
        for (&self.transforms) |*transform| transform.deinit(self.allocator);
        self.allocator.free(self.scratch.spectrum);
        self.allocator.free(self.scratch.samples);
        self.allocator.free(self.scratch.curves);
        self.allocator.free(self.state.timeline);
        self.allocator.free(self.scratch.window);
        self.allocator.free(self.scratch.classifications);
        self.allocator.free(self.scratch.vectors);
        self.allocator.free(self.scratch.transform);
        self.* = .{
            .allocator = self.allocator,
            .setup = .{ .allocator = self.setup.allocator },
        };
    }

    /// decode packet to interleaved pcm, return frames per channel
    /// output needs channels * half the long block size
    /// first packet primes overlap and returns zero, input failures require reset
    pub fn decodePacket(
        self: *Decoder,
        packet: []const u8,
        output: []f32,
    ) Error!usize {
        std.debug.assert(output.len >= self.channels * self.bins);
        var reader: Reader = .init(packet);
        // packet type must be an audio packet
        if (try reader.read(1) != 0) return error.InvalidData;
        const mode_index = try self.setup.readMode(&reader);
        const mode = self.setup.modes[mode_index];
        const long = mode.blockflag;
        const size = if (long)
            self.setup.info.blocksize1()
        else
            self.setup.info.blocksize0();
        // a long block names the size of the neighboring blocks
        var previous_flag = false;
        var next_flag = false;
        if (long) {
            previous_flag = try reader.read(1) == 1;
            next_flag = try reader.read(1) == 1;
        }
        const bins = size / 2;
        self.buildWindow(size, long, previous_flag, next_flag);
        const map = &self.setup.mappings[mode.mapping];

        // floor curves come first, they mark the channels with energy
        var nonzero: [root.max_channels]bool = @splat(false);
        for (0..self.channels) |channel| {
            const submap = map.mux[channel];
            const floor_index = map.submap_floor[submap];
            const curve = self.scratch.curves[channel * self.bins..][0..bins];
            nonzero[channel] = try self.setup.floors[floor_index]
                .decode(&reader, self.setup.codebooks, curve);
            @memset(self.scratch.spectrum[channel * self.bins..][0..bins], 0);
        }
        // coupling can make an empty channel carry energy
        for (0..map.coupling_steps) |step| {
            const magnitude = map.coupling_magnitude[step];
            const angle = map.coupling_angle[step];
            if (nonzero[magnitude] or nonzero[angle]) {
                nonzero[magnitude] = true;
                nonzero[angle] = true;
            }
        }
        // residue of every submap follows in submap order
        for (0..map.submaps) |submap| {
            var vectors: [root.max_channels][]f32 = undefined;
            var skip: [root.max_channels]bool = @splat(false);
            var count: usize = 0;
            for (0..self.channels) |channel| {
                if (map.mux[channel] != submap) continue;
                vectors[count] = self.scratch.spectrum[channel * self.bins..][
                    0..bins
                ];
                skip[count] = !nonzero[channel];
                count += 1;
            }
            if (count == 0) continue;
            const index = map.submap_residue[submap];
            try self.setup.residues[index]
                .decode(
                    self.setup.residue_types[index],
                    &reader,
                    self.setup.codebooks,
                    vectors[0..count],
                    skip[0..count],
                    self.scratch.classifications,
                    self.scratch.vectors,
                );
        }
        // coupling is undone in reverse order of the steps
        var step = map.coupling_steps;
        while (step > 0) {
            step -= 1;
            const magnitude = map.coupling_magnitude[step];
            const angle = map.coupling_angle[step];
            const m = self.scratch.spectrum[magnitude * self.bins..][0..bins];
            const a = self.scratch.spectrum[angle * self.bins..][0..bins];
            undoCoupling(m, a);
        }
        // floor curve scales the spectrum of every channel
        for (0..self.channels) |channel| {
            const spectrum = self.scratch.spectrum[channel * self.bins..][
                0..bins
            ];
            if (!nonzero[channel]) {
                @memset(spectrum, 0);
                continue;
            }
            const curve = self.scratch.curves[channel * self.bins..][0..bins];
            multiplySpectrum(spectrum, curve);
        }
        // inverse transform produces the time domain samples
        const transform = &self.transforms[if (long) 1 else 0];
        for (0..self.channels) |channel| {
            const buffer = self.scratch.samples[channel * self.block..][
                0..size
            ];
            @memcpy(
                buffer[0..bins],
                self.scratch.spectrum[channel * self.bins..][0..bins],
            );
            @memset(buffer[bins..size], 0);
            transform.inverseMdct(buffer, self.scratch.transform);
        }
        return self.lap(size, output);
    }

    /// build block window, long blocks use hybrid edges beside short blocks
    fn buildWindow(
        self: *Decoder,
        size: usize,
        long: bool,
        previous_flag: bool,
        next_flag: bool,
    ) void {
        const short = self.setup.info.blocksize0();
        var left_start: usize = 0;
        var left_end: usize = size / 2;
        var left_n: usize = size / 2;
        var right_start: usize = size / 2;
        var right_end: usize = size;
        var right_n: usize = size / 2;
        if (long and !previous_flag) {
            left_start = size / 4 - short / 4;
            left_end = size / 4 + short / 4;
            left_n = short / 2;
        }
        if (long and !next_flag) {
            right_start = size * 3 / 4 - short / 4;
            right_end = size * 3 / 4 + short / 4;
            right_n = short / 2;
        }
        self.scratch.window_start = left_start;
        self.scratch.window_end = right_end;
        const left = self.transforms[
            if (left_n == short / 2) @as(usize, 0) else 1
        ].window;
        const right = self.transforms[
            if (right_n == short / 2) @as(usize, 0) else 1
        ].window;
        const window = self.scratch.window[0..size];
        @memset(window[0..left_start], 0);
        @memcpy(window[left_start..left_end], left[0..left_n]);
        @memset(window[left_end..right_start], 1);
        for (window[right_start..right_end], 0..) |*value, index| {
            value.* = right[right_n - 1 - index];
        }
        @memset(window[right_end..], 0);
    }

    /// window and add block to overlap timeline
    /// return finalized samples between previous and current block centers
    fn lap(self: *Decoder, size: usize, output: []f32) Error!usize {
        // block starts where the three quarter point of the previous
        // block meets the quarter point of the current block
        var start: isize = 0;
        if (self.state.previous_size != 0) {
            start = self.state.previous_start
                + @as(isize, @intCast(self.state.previous_size * 3 / 4))
                - @as(isize, @intCast(size / 4));
        }
        const center = start + @as(isize, @intCast(size / 2));
        // samples outside of the window hold no energy
        const first = self.scratch.window_start;
        const last = self.scratch.window_end;
        const offset = start
            + @as(isize, @intCast(first))
            - self.state.timeline_start;
        if (offset < 0) return error.InvalidState;
        const length = last - first;
        const position: usize = @intCast(offset);
        if (position + length > self.block * 2) return error.InvalidState;
        // windowed samples join the timeline
        for (0..self.channels) |channel| {
            const buffer = self.scratch.samples[channel * self.block..][
                0..size
            ];
            const row = self.state.timeline[channel * (self.block * 2)..];
            overlapAdd(
                row[position..][0..length],
                buffer[first..][0..length],
                self.scratch.window[first..][0..length],
            );
        }
        self.state.timeline_len = @max(
            self.state.timeline_len,
            position + length,
        );
        const count: usize = @intCast(center - self.state.timeline_start);
        const frames = if (self.state.previous_size != 0) count else 0;
        for (0..self.channels) |channel| {
            const row = self.state.timeline[channel * (self.block * 2)..];
            for (0..frames) |index| {
                output[index * self.channels + channel] = row[index];
            }
        }
        // final samples leave the timeline; the tail overlaps the next block
        for (0..self.channels) |channel| {
            const row = self.state.timeline[channel * (self.block * 2)..];
            const rest = self.state.timeline_len - count;
            @memmove(row[0..rest], row[count..][0..rest]);
            @memset(row[rest..self.state.timeline_len], 0);
        }
        self.state.timeline_len -= count;
        self.state.timeline_start = center;
        self.state.previous_start = start;
        self.state.previous_size = size;
        return frames;
    }

    /// discard overlap history
    pub fn reset(self: *Decoder) void {
        @memset(self.state.timeline, 0);
        self.state = .{ .timeline = self.state.timeline };
    }
};
