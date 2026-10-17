//! print decoded pcm samples, one frame per line
const std = @import("std");
const zigaudioparser = @import("zigaudioparser");

/// decode file to text frames or raw little endian f32 samples
pub fn main(init: std.process.Init) !void {
    const allocator = init.arena.allocator();
    const args = try init.minimal.args.toSlice(allocator);
    // optional raw mode writes little endian float samples
    const raw_pcm = args.len == 3 and std.mem.eql(u8, args[1], "--pcm-f32le");
    if (args.len != 2 and !raw_pcm) {
        std.debug.print(
            "Usage: zigaudioparser-info [--pcm-f32le] <audio-file>\n",
            .{},
        );
        std.process.exit(1);
    }
    const file = try std.Io.Dir.cwd()
        .openFile(init.io, args[if (raw_pcm) 2 else 1], .{});
    defer file.close(init.io);
    // keep reader and its borrowed buffer alive through decoder cleanup
    var input_buffer: [64 * 1024]u8 = undefined;
    var reader = file.reader(init.io, &input_buffer);
    var decoder = try zigaudioparser.Decoder.init(
        allocator,
        zigaudioparser.Source.fromFile(&reader),
    );
    defer decoder.deinit();
    const info = decoder.info();
    // room for 1024 interleaved pcm frames per read
    const samples = try allocator.alloc(f32, @as(usize, info.channels) * 1024);
    defer allocator.free(samples);
    var buffer: [64 * 1024]u8 = undefined;
    var stdout = std.Io.File.Writer.init(.stdout(), init.io, &buffer);
    var frames: u64 = 0;
    while (true) {
        const count = try decoder.read(samples);
        if (count == 0) break;
        if (raw_pcm) {
            for (samples[0..count * info.channels]) |sample| {
                var bytes: [4]u8 = undefined;
                // write float bits in little endian order regardless of host byte order
                std.mem.writeInt(u32, &bytes, @bitCast(sample), .little);
                try stdout.interface.writeAll(&bytes);
            }
            frames += count;
            continue;
        }
        // text output groups one sample per channel on each line
        for (0..count) |frame| {
            for (
                samples[frame * info.channels..][0..info.channels],
                0..,
            ) |sample, channel| {
                if (channel != 0) try stdout.interface.writeByte(' ');
                try stdout.interface.print("{d}", .{sample});
            }
            try stdout.interface.writeByte('\n');
        }
        frames += count;
    }
    if (!raw_pcm)
        try stdout.interface.print(
            "{s}: {d} Hz, {d} channels, {d} frames decoded\n",
            .{ @tagName(info.format), info.sample_rate, info.channels, frames },
        );
    try stdout.interface.flush();
}
