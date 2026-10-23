//! parsed vorbis setup, shared by packet inspection and synthesis
//! ref: stb_vorbis.c start_decoder(), libvorbis info.c
const std = @import("std");
const root = @import("root.zig");
const headers = @import("headers.zig");
const codebook = @import("codebook.zig");
const floor = @import("floor.zig");
const residue = @import("residue.zig");
const mapping = @import("mapping.zig");
const Error = root.Error;
const Reader = root.Reader;

/// owns setup tables; packet inspection requires no synthesis buffers
pub const Setup = struct {
    allocator: std.mem.Allocator,
    /// stream params from the identification header
    info: headers.Info = .{},
    /// vector quantization books of the setup header
    codebooks: []codebook.Codebook = &.{},
    /// floor curves of the setup header
    floors: []floor.Floor = &.{},
    /// residue configs of the setup header
    residues: []residue.Residue = &.{},
    /// encoding variant of every residue
    residue_types: []u2 = &.{},
    /// channel mappings of the setup header
    mappings: []mapping.Mapping = &.{},
    /// packet modes of the setup header
    modes: []mapping.Mode = &.{},

    /// parse full setup packet including framing bit, info must be validated
    /// caller owns tables and must call deinit
    pub fn init(
        allocator: std.mem.Allocator,
        info: headers.Info,
        packet: []const u8,
    ) Error!Setup {
        var self: Setup = .{ .allocator = allocator, .info = info };
        errdefer self.deinit();
        try headers.validatePrefix(packet, headers.packet_setup);
        var reader: Reader = .init(packet[7..]);
        try self.read(&reader);
        return self;
    }

    /// transfer all tables; leave self empty and safe to deinit
    pub fn take(self: *Setup) Setup {
        const owned = self.*;
        self.* = .{ .allocator = self.allocator };
        return owned;
    }

    /// release setup tables with the allocator supplied to init
    pub fn deinit(self: *Setup) void {
        for (self.codebooks) |*book| book.deinit(self.allocator);
        self.allocator.free(self.codebooks);
        self.allocator.free(self.floors);
        self.allocator.free(self.residues);
        self.allocator.free(self.residue_types);
        for (self.mappings) |*map| map.deinit(self.allocator);
        self.allocator.free(self.mappings);
        self.allocator.free(self.modes);
        self.* = .{ .allocator = self.allocator };
    }

    fn read(self: *Setup, reader: *Reader) Error!void {
        const allocator = self.allocator;
        // codebooks
        const codebook_count = try reader.read(8) + 1;
        self.codebooks = try allocator.alloc(codebook.Codebook, codebook_count);
        @memset(self.codebooks, .{});
        for (self.codebooks) |*book| {
            book.* = try codebook.Codebook.read(allocator, reader);
        }
        // time domain transforms are placeholders that must be zero
        const time_count = try reader.read(6) + 1;
        for (0..time_count) |_| {
            if (try reader.read(16) != 0) return error.UnsupportedEncoding;
        }
        // floors
        const floor_count = try reader.read(6) + 1;
        self.floors = try allocator.alloc(floor.Floor, floor_count);
        for (self.floors) |*curve| {
            curve.* = try floor.Floor.read(reader, self.codebooks);
        }
        // residues
        const residue_count = try reader.read(6) + 1;
        self.residues = try allocator.alloc(residue.Residue, residue_count);
        self.residue_types = try allocator.alloc(u2, residue_count);
        for (self.residues, self.residue_types) |*vector, *kind| {
            const residue_type = try reader.read(16);
            if (residue_type > 2) return error.UnsupportedEncoding;
            kind.* = @intCast(residue_type);
            vector.* = try residue.Residue.read(reader, self.codebooks.len);
        }
        // mappings
        const mapping_count = try reader.read(6) + 1;
        self.mappings = try allocator.alloc(mapping.Mapping, mapping_count);
        @memset(self.mappings, .{});
        for (self.mappings) |*map| {
            map.* = try mapping.Mapping.read(
                allocator,
                reader,
                self.info.channels,
            );
        }
        // modes
        self.modes = try mapping.readModes(allocator, reader);
        if (try reader.read(1) != 1) return error.InvalidData;
        for (self.modes) |mode| {
            if (mode.mapping >= self.mappings.len) return error.InvalidData;
        }
        for (self.mappings) |map| {
            for (map.submap_floor) |index| {
                if (index >= self.floors.len) return error.InvalidData;
            }
            for (map.submap_residue) |index| {
                if (index >= self.residues.len) return error.InvalidData;
            }
        }
    }

    /// read packet type and mode to find block size
    pub fn blockSize(self: *const Setup, packet: []const u8) Error!u16 {
        var reader: Reader = .init(packet);
        if (try reader.read(1) != 0) return error.InvalidData;
        const mode = try self.readMode(&reader);
        const size = if (self.modes[mode].blockflag)
            self.info.blocksize1()
        else
            self.info.blocksize0();
        return @intCast(size);
    }

    /// read the mode number of a packet
    pub fn readMode(self: *const Setup, reader: *Reader) Error!usize {
        const mode = try reader.read(root.ilog(@intCast(self.modes.len - 1)));
        if (mode >= self.modes.len) return error.InvalidData;
        return mode;
    }
};
