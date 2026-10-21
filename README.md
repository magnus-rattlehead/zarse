# zigaudioparser

A small audio reading library for **Zig 0.16.0** codecs.

## Quick start

```sh
zig build
zig build run -- path/to/audio.wav
```

All container and codec support is enabled by default. Disable features for smaller builds with
`-Dfeature=false`:

```sh
# MP3 only
zig build -Dwav=false -Dogg=false -Dpcm=false -Dopus=false -Dvorbis=false
zig build run -Dwav=false -Dogg=false -Dpcm=false -Dopus=false -Dvorbis=false -- path/to/audio.mp3

# WAV/PCM only
zig build -Dogg=false -Dmp3=false -Dopus=false -Dvorbis=false

# Ogg Opus only
zig build -Dwav=false -Dmp3=false -Dpcm=false -Dvorbis=false
```

## Roadmap

The goal is to read, decode, convert, encode, and write mainstream audio formats.

### Documentation and testing

- [ ] create man pages
- [ ] create unit tests

### WAV container

- [x] RF64 audio reading.
- [x] BW64 audio reading.
- [x] RIFX.
- [x] Wave64.
- [x] Segmented RIFF/RIFX audio (`LIST/wavl`, `data`, `slnt`).
- [ ] Compressed WAVE support (IMA ADPCM, Microsoft ADPCM, MP3), including framing, timing,
      trimming, and seeking.
- [ ] Preserve extensible channel masks for seekable input.

### Ogg container

- [x] Opus and Vorbis reading (single logical stream).
- [ ] Chained streams.
- [ ] Audio selection from multiplexed streams.
- [ ] Recovery after damaged or missing pages.
- [ ] Ogg FLAC mapping.

### Audio processing and output

- [ ] Audio conversion: sample formats and layouts, resampling, channel mixing, and dithering,
      preserving timing and sample counts.
- [ ] Encoder and container writing support: codec settings, streaming, finalization, metadata, and
      remuxing.

### Metadata

- [ ] RIFF `LIST/INFO` tag reading.
- [ ] Technical metadata: stream properties and channel layouts.
- [ ] Descriptive tags: title, artist, album, track number, dates, and comments across formats.
- [ ] Embedded artwork.

### Codecs

- [ ] AAC.
- [ ] AC3.
- [ ] ADPCM IMA WAV.
- [ ] ADPCM Microsoft.
- [ ] A-law.
- [ ] ALAC.
- [ ] DTS.
- [ ] EAC3.
- [ ] FLAC.
- [x] MP3 (MPEG-1 Layer III decoder, MPEG2 and 2.5 still TODO).
- [ ] Mu-law.
- [x] Opus (temporary libopus backend).
- [x] Vorbis (temporary libvorbis backend).
- [ ] Windows Media Audio 1.
- [ ] Windows Media Audio 2.

### Common audio containers and streams

- [x] MP3 streams
- [x] Ogg container (Opus and Vorbis; remaining work above).
- [ ] ADTS.
- [ ] FLAC.
- [ ] M4A.
- [ ] AIFF.
- [ ] AIFC.
- [ ] CAF.
- [ ] WMA files.
- [ ] Raw PCM with caller supplied format.
