//! channel mappings, coupling and packet modes
//! ref: libvorbis mapping0.c, spec section 4.4 and 4.5
const std = @import("std");
const root = @import("root.zig");

const Error = root.Error;
const Reader = root.Reader;

/// packet mode, the block size and mapping used by a packet
pub const Mode = struct {
    /// packet uses the long block size
    blockflag: bool = false,
    /// index of the mapping used by the mode
    mapping: u8 = 0,
};

/// channel mapping with its coupling steps and submaps
pub const Mapping = struct {
    /// number of submaps the channels are grouped into
    submaps: u8 = 1,
    /// number of coupling steps
    coupling_steps: u16 = 0,
    /// channel of the magnitude vector of every coupling step
    coupling_magnitude: []u8 = &.{},
    /// channel of the angle vector of every coupling step
    coupling_angle: []u8 = &.{},
    /// submap of every channel
    mux: []u8 = &.{},
    /// floor of every submap
    submap_floor: []u8 = &.{},
    /// residue of every submap
    submap_residue: []u8 = &.{},

    /// read a mapping and allocate its channel and submap tables
    /// caller owns the returned tables and must call deinit
    pub fn read(
        allocator: std.mem.Allocator,
        reader: *Reader,
        channels: u8,
    ) Error!Mapping {
        var self: Mapping = .{};
        errdefer self.deinit(allocator);
        // mapping type must be zero, vorbis one has a single type
        const mapping_type = try reader.read(16);
        if (mapping_type != 0) return error.UnsupportedEncoding;
        // submap count is only present when the flag is set
        if (try reader.read(1) == 1) {
            self.submaps = @intCast(try reader.read(4) + 1);
        }
        // coupling steps follow the submap flag
        if (try reader.read(1) == 1) {
            self.coupling_steps = @intCast(try reader.read(8) + 1);
            self.coupling_magnitude = try allocator.alloc(
                u8,
                self.coupling_steps,
            );
            self.coupling_angle = try allocator.alloc(u8, self.coupling_steps);
            const width = root.ilog(channels - 1);
            for (0..self.coupling_steps) |index| {
                const magnitude = try reader.read(width);
                const angle = try reader.read(width);
                if (magnitude >= channels or angle >= channels) {
                    return error.InvalidData;
                }
                if (magnitude == angle) return error.InvalidData;
                self.coupling_magnitude[index] = @intCast(magnitude);
                self.coupling_angle[index] = @intCast(angle);
            }
        }
        // two reserved bits follow the coupling steps
        if (try reader.read(2) != 0) return error.InvalidData;
        self.mux = try allocator.alloc(u8, channels);
        // channel multiplexing is only present with more than one submap
        if (self.submaps > 1) {
            for (self.mux) |*submap| {
                const value = try reader.read(4);
                if (value >= self.submaps) return error.InvalidData;
                submap.* = @intCast(value);
            }
        } else {
            @memset(self.mux, 0);
        }
        self.submap_floor = try allocator.alloc(u8, self.submaps);
        self.submap_residue = try allocator.alloc(u8, self.submaps);
        for (0..self.submaps) |index| {
            // time domain type must be zero
            if (try reader.read(8) != 0) return error.UnsupportedEncoding;
            self.submap_floor[index] = @intCast(try reader.read(8));
            self.submap_residue[index] = @intCast(try reader.read(8));
        }
        return self;
    }

    /// release every owned allocation
    pub fn deinit(self: *Mapping, allocator: std.mem.Allocator) void {
        allocator.free(self.coupling_magnitude);
        allocator.free(self.coupling_angle);
        allocator.free(self.mux);
        allocator.free(self.submap_floor);
        allocator.free(self.submap_residue);
        self.* = .{};
    }
};

/// read and validate modes; the caller owns the returned slice
pub fn readModes(allocator: std.mem.Allocator, reader: *Reader) Error![]Mode {
    const count = try reader.read(6) + 1;
    const modes = try allocator.alloc(Mode, count);
    errdefer allocator.free(modes);
    for (modes) |*mode| {
        mode.blockflag = try reader.read(1) == 1;
        // vorbis i reserves every nonzero window and transform type
        if (try reader.read(16) != 0) return error.UnsupportedEncoding;
        if (try reader.read(16) != 0) return error.UnsupportedEncoding;
        mode.mapping = @intCast(try reader.read(8));
    }
    return modes;
}
