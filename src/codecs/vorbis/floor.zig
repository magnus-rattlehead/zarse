//! spectral floor curves
//! ref: stb_vorbis.c do_floor(), libvorbis floor0.c and floor1.c,
//! spec sections 4.3, 6 and 7
const std = @import("std");
const root = @import("root.zig");
const codebook = @import("codebook.zig");

const Error = root.Error;
const Reader = root.Reader;
const Codebook = codebook.Codebook;

/// amplitude range per multiplier, the spec table
const range_table = [4]u32{ 256, 128, 86, 64 };

/// a floor curve of one of the two defined types
pub const Floor = union(enum) {
    /// lsp curve, the older floor type
    floor0: Floor0,
    /// piecewise linear curve in the decibel domain
    floor1: Floor1,

    /// read one floor from the setup header
    /// reader must sit on the floor type field
    pub fn read(reader: *Reader, codebooks: []const Codebook) Error!Floor {
        const floor_type = try reader.read(16);
        return switch (floor_type) {
            0 => .{ .floor0 = try Floor0.read(reader, codebooks) },
            1 => .{ .floor1 = try Floor1.read(reader, codebooks.len) },
            else => error.UnsupportedEncoding,
        };
    }

    /// decode linear gain per spectral bin, return false for a silent channel
    pub fn decode(
        self: *const Floor,
        reader: *Reader,
        codebooks: []const Codebook,
        curve: []f32,
    ) Error!bool {
        return switch (self.*) {
            .floor0 => |*floor| floor.decode(reader, codebooks, curve),
            .floor1 => |*floor| floor.decode(reader, codebooks, curve),
        };
    }
};

/// lsp curve, the older floor type
pub const Floor0 = struct {
    /// lsp order
    order: u16 = 0,
    /// rate of the lsp curve
    rate: u32 = 0,
    /// bark map size
    bark_map_size: u16 = 0,
    /// amplitude bits
    amplitude_bits: u6 = 0,
    /// amplitude offset
    amplitude_offset: u8 = 0,
    /// books used by the curve
    books: [16]u16 = @splat(0),
    /// number of books in use
    book_count: u8 = 0,

    /// read one floor of type zero
    pub fn read(reader: *Reader, codebooks: []const Codebook) Error!Floor0 {
        var self: Floor0 = .{};
        self.order = @intCast(try reader.read(8));
        self.rate = try reader.read(16);
        self.bark_map_size = @intCast(try reader.read(16));
        self.amplitude_bits = @intCast(try reader.read(6));
        self.amplitude_offset = @intCast(try reader.read(8));
        self.book_count = @intCast(try reader.read(4) + 1);
        if (self.order == 0 or self.rate == 0 or self.bark_map_size == 0)
            return error.InvalidData;
        for (self.books[0..self.book_count]) |*book| {
            const value = try reader.read(8);
            if (value >= codebooks.len) return error.InvalidData;
            if (
                codebooks[value].lookup_type == 0
                    or codebooks[value].dimensions == 0
            )
                return error.InvalidData;
            book.* = @intCast(value);
        }
        return self;
    }

    /// decode the lsp curve of one channel
    pub fn decode(
        self: *const Floor0,
        reader: *Reader,
        codebooks: []const Codebook,
        curve: []f32,
    ) Error!bool {
        const low = try reader.read(@min(self.amplitude_bits, 32));
        const high = if (self.amplitude_bits > 32)
            try reader.read(self.amplitude_bits - 32)
        else
            0;
        const amplitude = @as(u64, low) | (@as(u64, high) << 32);
        if (amplitude == 0) return false;
        const book_number = try reader.read(root.ilog(self.book_count));
        if (book_number >= self.book_count) return error.InvalidData;
        const book = &codebooks[self.books[book_number]];
        if (book.dimensions == 0 or book.dimensions > 256) {
            return error.UnsupportedEncoding;
        }
        // coeffs are read in vectors, the last one may overshoot
        var coefficients: [256]f32 = undefined;
        var count: usize = 0;
        var last: f32 = 0;
        var vector: [256]f32 = undefined;
        while (count < self.order) {
            book.decodeVector(reader, vector[0..book.dimensions]) catch |err| {
                if (err == error.EndOfStream) return false;
                return err;
            };
            for (vector[0..book.dimensions]) |value| {
                if (count >= coefficients.len) break;
                coefficients[count] = value + last;
                count += 1;
            }
            // accumulator holds the last scalar of the vector
            last = coefficients[count - 1];
        }
        self.renderCurve(coefficients[0..count], amplitude, curve);
        return true;
    }

    /// render lsp curve, bark map selects polynomial evaluation points
    fn renderCurve(
        self: *const Floor0,
        coefficients: []const f32,
        amplitude: u64,
        curve: []f32,
    ) void {
        const n = curve.len;
        if (n == 0) return;
        const bark_size = @as(f64, @floatFromInt(self.bark_map_size));
        const rate = @as(f64, @floatFromInt(self.rate));
        const reference = bark(0.5 * rate);
        const maximum = @as(f64, @floatFromInt(self.bark_map_size - 1));
        var index: usize = 0;
        while (index < n) : (index += 1) {
            const position = bark(
                rate
                    * @as(f64, @floatFromInt(index))
                    / (2.0 * @as(f64, @floatFromInt(n))),
            );
            var mapped = @floor(position * bark_size / reference);
            if (mapped > maximum) mapped = maximum;
            if (mapped < 0) mapped = 0;
            const omega = std.math.pi * mapped / bark_size;
            const p = self.productP(coefficients, omega);
            const q = self.productQ(coefficients, omega);
            const scale = @as(f64, @floatFromInt(amplitude))
                * @as(f64, @floatFromInt(self.amplitude_offset))
                / (@as(
                    f64,
                    @floatFromInt((@as(u64, 1) << self.amplitude_bits) - 1),
                )
                    * @sqrt(p + q));
            const value = @exp(
                0.11512925
                    * (scale - @as(f64, @floatFromInt(self.amplitude_offset))),
            );
            curve[index] = @floatCast(value);
        }
    }

    /// lsp polynomial p factor, odd order uses odd coeffs and asymmetric weight
    /// even order uses even coeffs
    fn productP(
        self: *const Floor0,
        coefficients: []const f32,
        omega: f64,
    ) f64 {
        const cosine = @cos(omega);
        var product: f64 = 1.0;
        if (self.order & 1 == 1) {
            var index: usize = 1;
            while (index + 1 < self.order) : (index += 2) {
                const value = @cos(@as(f64, coefficients[index]));
                const difference = value - cosine;
                product *= 4.0 * difference * difference;
            }
            return product * (1.0 - cosine * cosine);
        }
        var index: usize = 1;
        while (index < self.order) : (index += 2) {
            const value = @cos(@as(f64, coefficients[index]));
            const difference = value - cosine;
            product *= 4.0 * difference * difference;
        }
        return product * (1.0 - cosine) / 2.0;
    }

    /// q factor of the lsp polynomial
    fn productQ(
        self: *const Floor0,
        coefficients: []const f32,
        omega: f64,
    ) f64 {
        const cosine = @cos(omega);
        var product: f64 = 1.0;
        if (self.order & 1 == 1) {
            var index: usize = 0;
            while (index < self.order) : (index += 2) {
                const value = @cos(@as(f64, coefficients[index]));
                const difference = value - cosine;
                product *= 4.0 * difference * difference;
            }
            return product / 4.0;
        }
        var index: usize = 0;
        while (index + 1 < self.order) : (index += 2) {
            const value = @cos(@as(f64, coefficients[index]));
            const difference = value - cosine;
            product *= 4.0 * difference * difference;
        }
        return product * (1.0 + cosine) / 2.0;
    }
};

/// piecewise linear curve in the decibel domain
pub const Floor1 = struct {
    /// number of partitions of the curve
    partitions: u8 = 0,
    /// partition class of every partition
    partition_class_list: [31]u8 = @splat(0),
    /// number of amplitude values of every class
    class_dimensions: [16]u8 = @splat(0),
    /// number of bits selecting a subclass book for each class
    class_subclasses: [16]u2 = @splat(0),
    /// master book of every class
    class_masterbooks: [16]u16 = @splat(0),
    /// alternate books of every class, negative marks an unused book
    subclass_books: [16][8]i16 = @splat(@splat(-1)),
    /// amplitude multiplier, one to four
    multiplier: u3 = 1,
    /// bit width of explicit horizontal post positions
    rangebits: u4 = 0,
    /// horizontal position of every amplitude value
    x_list: [65]u32 = @splat(0),
    /// number of amplitude values
    values: u8 = 0,
    /// index of the lower neighbour of every amplitude value
    low_neighbors: [65]u8 = @splat(0),
    /// index of the upper neighbour of every amplitude value
    high_neighbors: [65]u8 = @splat(0),
    /// amplitude values sorted by horizontal position
    sorted_order: [65]u8 = @splat(0),

    /// read one floor of type one
    pub fn read(reader: *Reader, codebook_count: usize) Error!Floor1 {
        var self: Floor1 = .{};
        self.partitions = @intCast(try reader.read(5));
        var class_count: u8 = 0;
        for (self.partition_class_list[0..self.partitions]) |*class| {
            class.* = @intCast(try reader.read(4));
            class_count = @max(class_count, class.* + 1);
        }
        for (0..class_count) |class| {
            self.class_dimensions[class] = @intCast(try reader.read(3) + 1);
            self.class_subclasses[class] = @intCast(try reader.read(2));
            if (self.class_subclasses[class] != 0) {
                const master = try reader.read(8);
                if (master >= codebook_count) return error.InvalidData;
                self.class_masterbooks[class] = @intCast(master);
            }
            const subclasses = @as(usize, 1) << self.class_subclasses[class];
            for (self.subclass_books[class][0..subclasses]) |*book| {
                // stored value is one less than the book number, zero
                // marks an unused book
                const value = try reader.read(8);
                if (value == 0) {
                    book.* = -1;
                } else {
                    if (value - 1 >= codebook_count) return error.InvalidData;
                    book.* = @intCast(value - 1);
                }
            }
        }
        self.multiplier = @intCast(try reader.read(2) + 1);
        self.rangebits = @intCast(try reader.read(4));
        // first two positions are implicit
        self.x_list[0] = 0;
        self.x_list[1] = @as(u32, 1) << self.rangebits;
        self.values = 2;
        const limit = @as(u32, 1) << self.rangebits;
        for (self.partition_class_list[0..self.partitions]) |class| {
            const dimensions = self.class_dimensions[class];
            for (0..dimensions) |_| {
                if (self.values >= self.x_list.len) return error.InvalidData;
                const value = try reader.read(self.rangebits);
                if (value >= limit) return error.InvalidData;
                self.x_list[self.values] = value;
                self.values += 1;
            }
        }
        // positions must be unique, they need not be ordered
        for (self.x_list[0..self.values], 0..) |value, index| {
            for (self.x_list[0..index]) |other| {
                if (other == value) return error.InvalidData;
            }
        }
        self.computeNeighbors();
        self.computeSortedOrder();
        return self;
    }

    /// decode channel floor curve from partition amplitudes
    pub fn decode(
        self: *const Floor1,
        reader: *Reader,
        codebooks: []const Codebook,
        curve: []f32,
    ) Error!bool {
        return self.decodeCurve(reader, codebooks, curve)
            catch |err| switch (err) {
                // an end of packet leaves the channel without energy
                error.EndOfStream => false,
                else => err,
            };
    }

    /// decode channel floor curve from partition amplitudes
    fn decodeCurve(
        self: *const Floor1,
        reader: *Reader,
        codebooks: []const Codebook,
        curve: []f32,
    ) Error!bool {
        const nonzero = try reader.read(1);
        if (nonzero == 0) return false;
        const range = range_table[self.multiplier - 1];
        const width = root.ilog(range - 1);
        var y_values: [65]u32 = @splat(0);
        var final_y: [65]u32 = @splat(0);
        var step2_flags: [65]bool = @splat(false);
        y_values[0] = try reader.read(width);
        y_values[1] = try reader.read(width);
        var offset: usize = 2;
        for (self.partition_class_list[0..self.partitions]) |class| {
            const dimensions = self.class_dimensions[class];
            const subclass_bits = self.class_subclasses[class];
            const subclass_mask = (@as(u32, 1) << subclass_bits) - 1;
            var value: u32 = 0;
            if (subclass_bits != 0) {
                value = try codebooks[self.class_masterbooks[class]]
                    .decodeSymbol(reader);
            }
            for (0..dimensions) |_| {
                const book = self.subclass_books[class][value & subclass_mask];
                value >>= subclass_bits;
                if (book >= 0) {
                    y_values[offset] = try codebooks[@intCast(book)]
                        .decodeSymbol(reader);
                } else {
                    y_values[offset] = 0;
                }
                offset += 1;
            }
        }
        self.synthesizeAmplitudes(range, &y_values, &final_y, &step2_flags);
        self.renderCurve(curve, &final_y, &step2_flags);
        return true;
    }

    /// unwrap amplitudes and apply line prediction before rendering the curve
    fn synthesizeAmplitudes(
        self: *const Floor1,
        range: u32,
        y_values: *[65]u32,
        final_y: *[65]u32,
        step2_flags: *[65]bool,
    ) void {
        step2_flags[0] = true;
        step2_flags[1] = true;
        final_y[0] = y_values[0];
        final_y[1] = y_values[1];
        var index: usize = 2;
        while (index < self.values) : (index += 1) {
            const low = self.low_neighbors[index];
            const high = self.high_neighbors[index];
            const predicted = renderPoint(
                self.x_list[low],
                final_y[low],
                self.x_list[high],
                final_y[high],
                self.x_list[index],
            );
            const high_room = range - predicted;
            const low_room = predicted;
            const room = @min(high_room, low_room) * 2;
            const value = y_values[index];
            if (value != 0) {
                step2_flags[low] = true;
                step2_flags[high] = true;
                step2_flags[index] = true;
                if (value >= room) {
                    if (high_room > low_room) {
                        final_y[index] = value - low_room + predicted;
                    } else {
                        final_y[index] = predicted - value + high_room - 1;
                    }
                } else if (value & 1 == 1) {
                    final_y[index] = predicted - ((value + 1) / 2);
                } else {
                    final_y[index] = predicted + value / 2;
                }
                // values stay inside the range of the multiplier
                final_y[index] = @min(final_y[index], range - 1);
            } else {
                step2_flags[index] = false;
                final_y[index] = predicted;
            }
        }
    }

    /// draw the curve of the amplitudes into the output
    fn renderCurve(
        self: *const Floor1,
        curve: []f32,
        final_y: *const [65]u32,
        step2_flags: *const [65]bool,
    ) void {
        const n = curve.len;
        @memset(curve, 0);
        var low_x: u32 = 0;
        var low_y: u32 = final_y[0] * self.multiplier;
        var high_x: u32 = 0;
        var high_y: u32 = low_y;
        for (1..self.values) |index| {
            const current = self.sorted_order[index];
            if (!step2_flags[current]) continue;
            high_y = final_y[current] * self.multiplier;
            high_x = self.x_list[current];
            renderLine(curve, low_x, low_y, high_x, high_y);
            low_x = high_x;
            low_y = high_y;
        }
        if (high_x < n) {
            renderLine(curve, high_x, high_y, @intCast(n), high_y);
        }
    }

    /// find lower and upper neighbours among earlier amplitudes
    /// fall back to the first two entries as libvorbis does
    fn computeNeighbors(self: *Floor1) void {
        for (0..self.values) |index| {
            const x = self.x_list[index];
            var low: u8 = 0;
            var high: u8 = 1;
            var low_x: u32 = 0;
            var high_x: u32 = @as(u32, 1) << self.rangebits;
            for (0..index) |candidate| {
                const candidate_x = self.x_list[candidate];
                if (candidate_x > low_x and candidate_x < x) {
                    low = @intCast(candidate);
                    low_x = candidate_x;
                }
                if (candidate_x < high_x and candidate_x > x) {
                    high = @intCast(candidate);
                    high_x = candidate_x;
                }
            }
            self.low_neighbors[index] = low;
            self.high_neighbors[index] = high;
        }
    }

    /// sort the amplitude values by horizontal position
    fn computeSortedOrder(self: *Floor1) void {
        for (0..self.values) |index| {
            self.sorted_order[index] = @intCast(index);
        }
        // insertion sort keeps the original order for equal positions
        var index: usize = 1;
        while (index < self.values) : (index += 1) {
            const value = self.sorted_order[index];
            var position = index;
            while (
                position > 0
                    and self.x_list[value]
                        < self.x_list[self.sorted_order[position - 1]]
            ) {
                self.sorted_order[position] = self.sorted_order[position - 1];
                position -= 1;
            }
            self.sorted_order[position] = value;
        }
    }
};

// precompute floor gains; packet decoding only indexes the table
const inverse_db = table: {
    @setEvalBranchQuota(100_000);
    var values: [256]f32 = undefined;
    for (&values, 0..) |*value, y| {
        const exponent = (@as(f64, @floatFromInt(y)) - 255.0) * 7.0 / 256.0;
        value.* = @floatCast(std.math.pow(f64, 10.0, exponent));
    }
    break :table values;
};

/// convert an amplitude in 0..255 to linear gain
pub fn inverseDb(y: u32) f32 {
    return inverse_db[y];
}

/// find line amplitude at x, spec helper render_point
pub fn renderPoint(x0: u32, y0: u32, x1: u32, y1: u32, x: u32) u32 {
    const dy: i64 = @as(i64, y1) - @as(i64, y0);
    const adx: i64 = @as(i64, x1) - @as(i64, x0);
    if (adx == 0) return y0;
    const ady = @abs(dy);
    const err = ady * (@as(u64, x) -| @as(u64, x0));
    const off = err / @as(u64, @intCast(adx));
    const result: i64 = if (dy < 0)
        @as(i64, y0) - @as(i64, @intCast(off))
    else
        @as(i64, y0) + @as(i64, @intCast(off));
    if (result < 0) return 0;
    return @intCast(result);
}

/// draw floor line as linear gain per spectral bin
/// ref: libvorbis floor1.c render_line()
pub fn renderLine(output: []f32, x0: u32, y0: u32, x1: u32, y1: u32) void {
    const dy: i64 = @as(i64, y1) - @as(i64, y0);
    const adx: i64 = @as(i64, x1) - @as(i64, x0);
    if (adx <= 0) return;
    const ady = @abs(dy);
    const base: i64 = @divTrunc(dy, adx);
    const sy: i64 = if (dy < 0) base - 1 else base + 1;
    var x: u32 = x0;
    var y: i64 = @as(i64, y0);
    var err: i64 = 0;
    const scaled = base * adx;
    const ady_remaining: i64 = @intCast(ady - @abs(scaled));
    if (x < output.len) {
        output[x] = inverseDb(clampAmplitude(y));
    }
    x += 1;
    while (x < x1) : (x += 1) {
        err += ady_remaining;
        if (err >= adx) {
            err -= adx;
            y += sy;
        } else {
            y += base;
        }
        if (x < output.len) {
            output[x] = inverseDb(clampAmplitude(y));
        }
    }
}

/// clamp an amplitude to the range of the decibel table
fn clampAmplitude(value: i64) u32 {
    if (value < 0) return 0;
    if (value > 255) return 255;
    return @intCast(value);
}

/// bark scale of a frequency
fn bark(value: f64) f64 {
    return 13.1 * std.math.atan(0.00074 * value)
        + 2.24 * std.math.atan(0.0000000185 * value * value)
        + 0.0001 * value;
}
