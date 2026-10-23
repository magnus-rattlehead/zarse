//! spectral residue vectors
//! ref: stb_vorbis.c decode_residue(), libvorbis res0.c,
//! spec section 8
const std = @import("std");
const root = @import("root.zig");
const codebook = @import("codebook.zig");

const Error = root.Error;
const Reader = root.Reader;
const Codebook = codebook.Codebook;

/// marks a book that a pass does not use
pub const unused = 0xffff;

/// residue vector of one submap
pub const Residue = struct {
    /// first partition of the vector
    begin: u32 = 0,
    /// partition after the last one
    end: u32 = 0,
    /// number of values in a partition
    partition_size: u32 = 0,
    /// number of partition classes
    classifications: u8 = 0,
    /// book that holds the partition classifications
    classbook: u16 = 0,
    /// passes of every class
    cascade: [64]u8 = @splat(0),
    /// books of every class and pass
    books: [64][8]u16 = @splat(@splat(unused)),

    /// read residue config, caller reads type first, fields shared by all types
    pub fn read(reader: *Reader, codebook_count: usize) Error!Residue {
        var self: Residue = .{};
        self.begin = try reader.read(24);
        self.end = try reader.read(24);
        self.partition_size = try reader.read(24) + 1;
        self.classifications = @intCast(try reader.read(6) + 1);
        const classbook = try reader.read(8);
        if (classbook >= codebook_count) return error.InvalidData;
        self.classbook = @intCast(classbook);
        // cascade bitmap of every class names the passes it uses
        for (0..self.classifications) |index| {
            const low = try reader.read(3);
            const flag = try reader.read(1);
            var high: u32 = 0;
            if (flag == 1) high = try reader.read(5);
            self.cascade[index] = @intCast(high * 8 + low);
        }
        // books of every class follow in the order of the bitmap
        for (0..self.classifications) |index| {
            for (0..8) |pass| {
                const bit = @as(u8, 1) << @intCast(pass);
                if (self.cascade[index] & bit == 0) continue;
                const value = try reader.read(8);
                if (value >= codebook_count) return error.InvalidData;
                self.books[index][pass] = @intCast(value);
            }
        }
        return self;
    }

    /// number of partition classes retained across all eight passes
    pub fn classificationCount(
        self: *const Residue,
        residue_type: u2,
        bins: usize,
        channels: usize,
    ) usize {
        const size = if (residue_type == 2) bins * channels else bins;
        const begin = @min(self.begin, size);
        const end = @min(self.end, size);
        if (end <= begin) return 0;
        const partitions = (end - begin) / self.partition_size;
        return partitions
            * (if (residue_type == 2) @as(usize, 1) else channels);
    }

    /// decode one vector per submap channel, one value per spectral bin
    /// type 0 interleaves partition values, type 1 keeps order, type 2 interleaves channels
    pub fn decode(
        self: *const Residue,
        residue_type: u2,
        reader: *Reader,
        codebooks: []const Codebook,
        vectors: [][]f32,
        do_not_decode: []const bool,
        classifications: []u8,
        vector_scratch: []f32,
    ) Error!void {
        const result = if (residue_type == 2)
            self.decodeResidue2(
                reader,
                codebooks,
                vectors,
                do_not_decode,
                classifications,
                vector_scratch,
            )
        else
            self.decodeResidue01(
                residue_type,
                reader,
                codebooks,
                vectors,
                do_not_decode,
                classifications,
                vector_scratch,
            );
        return result
            catch |err| switch (err) {
                // a truncated packet keeps the values decoded before its end
                error.EndOfStream => {},
                else => return err,
            };
    }

    /// decode submap by pass then partition then channel
    /// marked channels consume no residue bits
    fn decodeResidue01(
        self: *const Residue,
        residue_type: u2,
        reader: *Reader,
        codebooks: []const Codebook,
        vectors: [][]f32,
        do_not_decode: []const bool,
        classifications: []u8,
        vector_scratch: []f32,
    ) Error!void {
        const channels = vectors.len;
        if (channels == 0) return;
        const classbook = &codebooks[self.classbook];
        const classwords = classbook.dimensions;
        if (classwords == 0) return error.InvalidData;
        const size = vectors[0].len;
        const limit_begin = @min(self.begin, size);
        const limit_end = @min(self.end, size);
        if (limit_end <= limit_begin) return;
        const partitions = (limit_end - limit_begin) / self.partition_size;
        if (partitions == 0) return;
        if (classifications.len < channels * partitions)
            return error.InvalidData;
        // channels that are marked hold no residue
        var active = false;
        for (do_not_decode) |flag| {
            if (!flag) active = true;
        }
        if (!active) return;
        var pass: u8 = 0;
        while (pass < 8) : (pass += 1) {
            var partition: u32 = 0;
            while (partition < partitions) {
                if (pass == 0) {
                    // class of every partition of the group
                    for (0..channels) |channel| {
                        if (do_not_decode[channel]) continue;
                        var value = try classbook.decodeSymbol(reader);
                        var index: usize = classwords;
                        while (index > 0) {
                            index -= 1;
                            if (partition + index < partitions) {
                                classifications[
                                    channel * partitions + partition + index
                                ] = @intCast(value % self.classifications);
                            }
                            value /= self.classifications;
                        }
                    }
                }
                var word: usize = 0;
                while (word < classwords and partition < partitions)
                : (word += 1) {
                    for (0..channels) |channel| {
                        if (do_not_decode[channel]) continue;
                        const class = classifications[
                            channel * partitions + partition
                        ];
                        const index = self.books[class][pass];
                        if (index == unused) continue;
                        const book = &codebooks[index];
                        if (book.lookup_type == 0) return error.InvalidData;
                        const offset = limit_begin
                            + partition * self.partition_size;
                        try self.decodePartition(
                            residue_type,
                            book,
                            reader,
                            vectors,
                            channel,
                            offset,
                            vector_scratch,
                        );
                    }
                    partition += 1;
                }
            }
        }
    }

    /// decode interleaved submap channels as one vector with shared class words
    /// partition offsets and size apply to that vector
    fn decodeResidue2(
        self: *const Residue,
        reader: *Reader,
        codebooks: []const Codebook,
        vectors: [][]f32,
        do_not_decode: []const bool,
        classifications: []u8,
        vector_scratch: []f32,
    ) Error!void {
        const channels = vectors.len;
        if (channels == 0) return;
        const classbook = &codebooks[self.classbook];
        const classwords = classbook.dimensions;
        if (classwords == 0) return error.InvalidData;
        const size = vectors[0].len;
        // interleaved vector holds the values of every channel
        const actual_size = size * channels;
        const limit_begin = @min(self.begin, actual_size);
        const limit_end = @min(self.end, actual_size);
        if (limit_end <= limit_begin) return;
        const partitions = (limit_end - limit_begin) / self.partition_size;
        if (partitions == 0) return;
        if (classifications.len < partitions) return error.InvalidData;
        // every channel is decoded when one of them holds energy
        var active = false;
        for (do_not_decode) |flag| {
            if (!flag) active = true;
        }
        if (!active) return;
        var pass: u8 = 0;
        while (pass < 8) : (pass += 1) {
            var partition: u32 = 0;
            while (partition < partitions) {
                if (pass == 0) {
                    // group shares one class word
                    var value = try classbook.decodeSymbol(reader);
                    var index: usize = classwords;
                    while (index > 0) {
                        index -= 1;
                        if (partition + index < partitions) {
                            classifications[partition + index] = @intCast(
                                value % self.classifications,
                            );
                        }
                        value /= self.classifications;
                    }
                }
                var word: usize = 0;
                while (word < classwords and partition < partitions)
                : (word += 1) {
                    const class = classifications[partition];
                    const index = self.books[class][pass];
                    if (index == unused) {
                        partition += 1;
                        continue;
                    }
                    const book = &codebooks[index];
                    if (book.lookup_type == 0) return error.InvalidData;
                    const offset = limit_begin
                        + partition * self.partition_size;
                    try self.decodePartition(
                        2,
                        book,
                        reader,
                        vectors,
                        0,
                        offset,
                        vector_scratch,
                    );
                    partition += 1;
                }
            }
        }
    }

    /// decode partition, type 0 interleaves values, type 1 keeps order
    /// type 2 scatters values across channels
    fn decodePartition(
        self: *const Residue,
        residue_type: u2,
        book: *const Codebook,
        reader: *Reader,
        vectors: [][]f32,
        channel: usize,
        offset: u32,
        vector_scratch: []f32,
    ) Error!void {
        const dimensions = book.dimensions;
        if (dimensions == 0 or vector_scratch.len < dimensions) {
            return error.InvalidData;
        }
        const size = self.partition_size;
        if (residue_type == 0) {
            // values of the partition are interleaved
            const step = size / dimensions;
            if (step == 0) return error.InvalidData;
            var index: u32 = 0;
            while (index < step) : (index += 1) {
                try book.decodeVector(reader, vector_scratch[0..dimensions]);
                for (0..dimensions) |dimension| {
                    const position = offset
                        + index
                        + @as(u32, @intCast(dimension)) * step;
                    if (position >= vectors[channel].len) continue;
                    vectors[channel][position] += vector_scratch[dimension];
                }
            }
            return;
        }
        // values of the partition follow in order
        var index: u32 = 0;
        while (index < size) {
            try book.decodeVector(reader, vector_scratch[0..dimensions]);
            for (0..dimensions) |dimension| {
                if (index >= size) break;
                if (residue_type == 2) {
                    // type two variant scatters over the channels
                    const position = offset + index;
                    const target = position % vectors.len;
                    const sample = position / vectors.len;
                    if (sample < vectors[target].len) {
                        vectors[target][sample] += vector_scratch[dimension];
                    }
                } else {
                    const position = offset + index;
                    if (position < vectors[channel].len) {
                        vectors[channel][position] += vector_scratch[dimension];
                    }
                }
                index += 1;
            }
        }
    }
};
