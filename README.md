# zarse (**z**ig **a**udio pa**rse**r)

A small audio reading and decoding library for **Zig 0.16.0**.

## Quick start

```sh
zig build
zig build run -- path/to/audio.wav
zig build run -- --pcm-f32le path/to/audio.wav > audio.f32le
```

All build features are enabled by default. Disable features for smaller builds with
`-Dfeature=false`. Currently available flags are `wav`, `ogg`, `mp3`, `flac`, `pcm`, `wave_codecs`,
`opus`, and `vorbis`:

```sh
# MP3 only
zig build -Dflac=false -Dwav=false -Dogg=false -Dpcm=false -Dwave_codecs=false -Dopus=false -Dvorbis=false

# WAV/PCM only
zig build -Dflac=false -Dogg=false -Dmp3=false -Dwave_codecs=false -Dopus=false -Dvorbis=false

# Ogg Opus only
zig build -Dflac=false -Dwav=false -Dmp3=false -Dpcm=false -Dwave_codecs=false -Dvorbis=false
```

## Roadmap

The project is still a work in progress. Ideally most common containers/codecs/stream sources will
be supported, along with the encoding step. Ideally for an inital release:

- [] Decoding API gets completed (or close enough)
- [] Create an encoding/writing API

expect breaking changes until then.
