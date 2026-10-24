const std = @import("std");

pub fn build(builder: *std.Build) !void {
    const target = builder.standardTargetOptions(.{});
    const optimize = builder.standardOptimizeOption(.{});
    const module = builder.addModule(
        "zarse",
        .{
            .root_source_file = builder.path("src/root.zig"),
            .target = target,
            .optimize = optimize,
        },
    );
    const wav = builder.option(bool, "wav", "Enable WAV and Wave64 containers")
        orelse true;
    const ogg = builder.option(bool, "ogg", "Enable the Ogg container")
        orelse true;
    const mp3 = builder.option(bool, "mp3", "Enable MP3 streams and decoding")
        orelse true;
    const flac = builder.option(
        bool,
        "flac",
        "Enable native FLAC streams and decoding",
    )
        orelse true;
    const pcm = builder.option(bool, "pcm", "Enable PCM decoding") orelse true;
    const wave_codecs = builder.option(
        bool,
        "wave_codecs",
        "Enable WAVE codec scaffolding (A-law, mu-law, ADPCM)",
    )
        orelse true;
    const opus = builder.option(bool, "opus", "Enable the libopus backend")
        orelse true;
    const vorbis = builder.option(bool, "vorbis", "Enable the vorbis backend")
        orelse true;
    const options = builder.addOptions();
    options.addOption(bool, "wav", wav);
    options.addOption(bool, "ogg", ogg);
    options.addOption(bool, "mp3", mp3);
    options.addOption(bool, "pcm", pcm);
    options.addOption(bool, "wave_codecs", wave_codecs);
    options.addOption(bool, "flac", flac);
    options.addOption(bool, "opus", opus);
    options.addOption(bool, "vorbis", vorbis);
    module.addOptions("build_options", options);
    if (opus) {
        if (builder.lazyDependency("opus", .{})) |dep| {
            @import("build/opus.zig").add(builder, module, dep);
        }
    }
    const example = builder.addExecutable(
        .{
            .name = "zarse-info",
            .root_module = builder.createModule(
                .{
                    .root_source_file = builder.path("src/main.zig"),
                    .target = target,
                    .optimize = optimize,
                    .imports = &.{.{ .name = "zarse", .module = module }},
                },
            ),
        },
    );
    builder.installArtifact(example);
    const run = builder.addRunArtifact(example);
    if (builder.args) |args| run.addArgs(args);
    builder.step("run", "Inspect and decode an audio file").dependOn(&run.step);

    const check = builder.step("check", "Compile the example");
    check.dependOn(&example.step);
}
