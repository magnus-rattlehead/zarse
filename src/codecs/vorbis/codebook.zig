//! vector quantization books with huffman coded entries
//! ref: stb_vorbis.c start_decoder() codebook section, codebook_decode(),
//! libvorbis sharedbook.c and codebook.c, spec section 3.2
const std = @import("std");
const root = @import("root.zig");

const Error = root.Error;
const Reader = root.Reader;

/// marks a codebook entry that is not present in the stream
pub const no_code = 0;

/// one huffman coded entry of a book
pub const Codebook = struct {
    /// vector length of every entry
    dimensions: u16 = 0,
    /// number of entries in the book
    entries: u32 = 0,
    /// number of entries present in the stream
    used_entries: u32 = 0,
    /// present codewords grouped by length, then sorted by value
    sorted_codewords: []u32 = &.{},
    /// entry index of every slot of the sorted tables
    sorted_values: []u32 = &.{},
    /// first slot of every codeword length, one past the last length
    length_start: [root.max_code_length + 2]u32 = @splat(0),
    /// longest codeword of the book
    max_length: u8 = 0,
    /// zero for a plain book, one or two for a book with a value lookup
    lookup_type: u2 = 0,
    /// decoded value per present entry and dimension
    multiplicands: []f32 = &.{},

    /// parse a book at its sync pattern and allocate decode tables
    pub fn read(allocator: std.mem.Allocator, reader: *Reader) Error!Codebook {
        var self: Codebook = .{};
        errdefer self.deinit(allocator);
        // sync pattern marks the start of every book
        if (try reader.read(8) != 0x42) return error.InvalidData;
        if (try reader.read(8) != 0x43) return error.InvalidData;
        if (try reader.read(8) != 0x56) return error.InvalidData;
        const dimensions = try reader.read(16);
        const entries = try reader.read(24);
        // two counts must stay inside the addressable range
        if (root.ilog(dimensions) + root.ilog(entries) > 24)
            return error.InvalidData;
        if (dimensions == 0 and entries != 0) return error.InvalidData;
        self.dimensions = @intCast(dimensions);
        self.entries = entries;
        const ordered = try reader.read(1) == 1;
        const sparse = if (ordered) false else try reader.read(1) == 1;
        const lengths = try allocator.alloc(u8, entries);
        defer allocator.free(lengths);
        @memset(lengths, no_code);
        if (ordered) {
            // lengths are grouped, each group holds a run of entries
            var current_entry: u32 = 0;
            var current_length: u8 = @intCast(try reader.read(5) + 1);
            while (current_entry < entries) {
                const limit = entries - current_entry;
                const count = try reader.read(root.ilog(limit));
                if (current_length > root.max_code_length) {
                    return error.InvalidData;
                }
                if (count > limit) return error.InvalidData;
                @memset(lengths[current_entry..][0..count], current_length);
                current_entry += count;
                current_length += 1;
            }
        } else {
            for (lengths) |*length| {
                const present = if (sparse) try reader.read(1) == 1 else true;
                if (present) {
                    length.* = @intCast(try reader.read(5) + 1);
                } else {
                    length.* = no_code;
                }
            }
        }
        // codewords are needed before the lookup values are expanded
        try self.buildCodewords(allocator, lengths);
        const lookup_type = try reader.read(4);
        if (lookup_type > 2) return error.InvalidData;
        self.lookup_type = @intCast(lookup_type);
        if (self.lookup_type > 0) {
            var lookup: Lookup = .{
                .minimum = unpackFloat(try reader.read(32)),
                .delta = unpackFloat(try reader.read(32)),
                .bits = @intCast(try reader.read(4) + 1),
                .sequence = try reader.read(1) == 1,
                .values = 0,
            };
            lookup.values = switch (self.lookup_type) {
                1 => lookup1Values(self.entries, self.dimensions),
                2 => std.math.mul(u32, self.entries, self.dimensions) catch {
                    return error.InvalidData;
                },
                else => unreachable,
            };
            if (lookup.values == 0) return error.InvalidData;
            try self.readMultiplicands(allocator, reader, lookup);
        }
        return self;
    }

    pub fn deinit(self: *Codebook, allocator: std.mem.Allocator) void {
        allocator.free(self.sorted_codewords);
        allocator.free(self.sorted_values);
        allocator.free(self.multiplicands);
        self.* = .{};
    }

    /// temporary lookup params; only expanded vectors survive setup
    const Lookup = struct {
        minimum: f32,
        delta: f32,
        bits: u5,
        sequence: bool,
        values: u32,
    };

    /// expand lookup values into multiplicands
    /// type 2 stores values per entry and dimension, type 1 shares indexed values
    fn readMultiplicands(
        self: *Codebook,
        allocator: std.mem.Allocator,
        reader: *Reader,
        lookup: Lookup,
    ) Error!void {
        const raw = try allocator.alloc(u32, lookup.values);
        defer allocator.free(raw);
        for (raw) |*value| {
            value.* = try reader.read(lookup.bits);
        }
        const count = std.math.mul(usize, self.used_entries, self.dimensions)
            catch return error.InvalidData;
        const multiplicands = try allocator.alloc(f32, count);
        self.multiplicands = multiplicands;
        for (self.sorted_values, 0..) |entry, slot| {
            // accumulator restarts with every entry
            var last: f32 = 0;
            var lookup_entry = entry;
            for (0..self.dimensions) |dimension| {
                const index: u32 = switch (self.lookup_type) {
                    1 => lookup_entry % lookup.values,
                    2 => entry * self.dimensions
                        + @as(u32, @intCast(dimension)),
                    else => unreachable,
                };
                const raw_value: f32 = @floatFromInt(raw[index]);
                const value = raw_value * lookup.delta + lookup.minimum + last;
                multiplicands[slot * self.dimensions + dimension] = value;
                if (lookup.sequence) last = value;
                if (self.lookup_type == 1) lookup_entry /= lookup.values;
            }
        }
    }

    /// assign each entry the lowest unused codeword of its length
    /// reject books with more codewords than the tree holds
    fn buildCodewords(
        self: *Codebook,
        allocator: std.mem.Allocator,
        lengths: []const u8,
    ) Error!void {
        const entries = self.entries;
        const codewords = try allocator.alloc(u32, entries);
        defer allocator.free(codewords);
        @memset(codewords, 0);
        // markers hold the lowest free codeword of every length
        var marker: [root.max_code_length + 1]u32 = @splat(0);
        var used: u32 = 0;
        for (lengths, 0..) |length, entry| {
            if (length == no_code) continue;
            used += 1;
            if (length > root.max_code_length) return error.InvalidData;
            var codeword = marker[length];
            // tree cannot hold more codewords of this length
            if (
                length < root.max_code_length
                    and (codeword >> @as(u5, @intCast(length))) != 0
            ) {
                return error.InvalidData;
            }
            // codeword of the entry is the marker of its length before
            // walk below changes the markers
            const assigned = codeword;
            // move the markers of the shorter lengths to the next branch
            var index: u8 = length;
            while (index > 0) {
                if ((marker[index] & 1) != 0) {
                    if (index == 1) {
                        marker[1] += 1;
                    } else {
                        marker[index] = marker[index - 1] << 1;
                    }
                    break;
                }
                marker[index] += 1;
                index -= 1;
            }
            // prune the tree, the longer markers hang from the taken node
            index = length + 1;
            while (index <= root.max_code_length) : (index += 1) {
                if ((marker[index] >> 1) != codeword) break;
                codeword = marker[index];
                marker[index] = marker[index - 1] << 1;
            }
            codewords[entry] = assigned;
        }
        self.used_entries = used;
        if (used == 0) return error.InvalidData;
        // sorted tables hold the present entries by length and codeword
        const Entry = struct { length: u8, codeword: u32, entry: u32 };
        const list = try allocator.alloc(Entry, used);
        defer allocator.free(list);
        var slot: u32 = 0;
        for (lengths, 0..) |length, entry| {
            if (length == no_code) continue;
            list[slot] = .{
                .length = length,
                .codeword = codewords[entry],
                .entry = @intCast(entry),
            };
            slot += 1;
        }
        std.mem.sort(
            Entry,
            list,
            {},
            struct {
                fn lessThan(_: void, left: Entry, right: Entry) bool {
                    if (left.length != right.length)
                        return left.length < right.length;
                    return left.codeword < right.codeword;
                }
            }.lessThan,
        );
        const sorted_values = try allocator.alloc(u32, used);
        errdefer allocator.free(sorted_values);
        const sorted_codewords = try allocator.alloc(u32, used);
        errdefer allocator.free(sorted_codewords);
        for (list, 0..) |item, index| {
            sorted_codewords[index] = item.codeword;
            sorted_values[index] = item.entry;
            self.max_length = @max(self.max_length, item.length);
        }
        self.sorted_values = sorted_values;
        self.sorted_codewords = sorted_codewords;
        // first slot of every length bounds the search of a symbol
        @memset(&self.length_start, 0);
        var current: u32 = 0;
        var length: u8 = 1;
        while (length <= root.max_code_length) : (length += 1) {
            self.length_start[length] = current;
            while (current < used and list[current].length == length) {
                current += 1;
            }
        }
        self.length_start[root.max_code_length + 1] = used;
    }

    /// decode one huffman symbol, the index of the entry
    pub fn decodeSymbol(self: *const Codebook, reader: *Reader) Error!u32 {
        return self.sorted_values[try self.decodeSlot(reader)];
    }

    fn decodeSlot(self: *const Codebook, reader: *Reader) Error!usize {
        if (self.used_entries == 0) return error.InvalidData;
        var code: u32 = 0;
        var length: u8 = 1;
        while (length <= self.max_length) : (length += 1) {
            const bit = try reader.read(1);
            code = (code << 1) | bit;
            const start = self.length_start[length];
            const end = self.length_start[length + 1];
            if (start >= end) continue;
            const group = self.sorted_codewords[start..end];
            if (findCode(group, code)) |index| {
                return start + index;
            }
        }
        return error.InvalidData;
    }

    /// decode a vector into output, which must hold dimensions values
    /// scalar-only books return InvalidData
    pub fn decodeVector(
        self: *const Codebook,
        reader: *Reader,
        output: []f32,
    ) Error!void {
        if (self.lookup_type == 0) return error.InvalidData;
        const slot = try self.decodeSlot(reader);
        const start = slot * self.dimensions;
        @memcpy(
            output[0..self.dimensions],
            self.multiplicands[start..][0..self.dimensions],
        );
    }

    /// unpack vorbis float, 21 bit mantissa, sign bit, 10 bit exponent biased by 788
    pub fn unpackFloat(bits: u32) f32 {
        const mantissa = bits & 0x1fffff;
        const sign = bits & 0x80000000;
        const exponent = (bits & 0x7fe00000) >> 21;
        const magnitude: f64 = @floatFromInt(mantissa);
        const signed: f64 = if (sign != 0) -magnitude else magnitude;
        const scale = std.math.ldexp(
            signed,
            @as(i32, @intCast(exponent)) - 788,
        );
        return @floatCast(scale);
    }

    /// shared lookup 1 value count, largest n with n^dimensions <= entries
    fn lookup1Values(entries: u32, dimensions: u16) u32 {
        if (entries == 0 or dimensions == 0) return 0;
        const guess = @exp(
            @log(@as(f64, @floatFromInt(entries)))
                / @as(f64, @floatFromInt(dimensions)),
        );
        var values: u64 = @intFromFloat(@floor(guess));
        if (values < 1) values = 1;
        // floating point guess is only a starting point
        while (values > 1 and !powerFits(values, entries, dimensions)) {
            values -= 1;
        }
        while (powerFits(values + 1, entries, dimensions)) {
            values += 1;
        }
        if (values > std.math.maxInt(u32)) return 0;
        return @intCast(values);
    }

    /// check that the power of the base fits the entry count
    fn powerFits(base: u64, entries: u32, dimensions: u16) bool {
        var accumulator: u64 = 1;
        var index: u16 = 0;
        while (index < dimensions) : (index += 1) {
            if (accumulator > entries / base) return false;
            accumulator *= base;
        }
        return accumulator <= entries;
    }
};

/// find the codeword inside the group of one length
fn findCode(group: []const u32, code: u32) ?usize {
    var low: usize = 0;
    var high: usize = group.len;
    while (low < high) {
        const middle = low + (high - low) / 2;
        const value = group[middle];
        if (value == code) return middle;
        if (value < code) {
            low = middle + 1;
        } else {
            high = middle;
        }
    }
    return null;
}
