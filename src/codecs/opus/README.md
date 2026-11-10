# Built-in Opus decoder

Adapted from Jeffrey C. Ollie's zig-opus **v0.1.0** (Zig 0.16):
https://git.jcollie.dev/jeff/zig-opus/src/tag/v0.1.0

These sources compile directly with zarse when the `opus` build option is
enabled (the default). Use `zig build -Dopus=false` to exclude the decoder.
There is no external Opus package or build dependency.
`MultistreamDecoder.zig` handles packet decoding and
channel mapping; `../opus.zig` connects it to allocation, header gain, errors,
and the packet decoder interface.

The implementation is limited to what the Ogg pipeline uses: interleaved
floating-point output at 48 kHz, one to eight output channels, channel mapping
families 0 and 1, and reset after seeking. It retains SILK, CELT, hybrid mode,
all encoded bandwidths and frame durations, and internal concealment needed for
empty coded frames and mode transitions. SILK's internal sample-rate conversion
is still required even though output is always 48 kHz.

Encoder helpers, integer output, alternate output rates, projection mixing,
public packet-loss/FEC APIs, neural concealment, speech enhancement, DRED, and
upstream tests and tooling are omitted. SILK still consumes in-band redundancy
bits when present so the normal audio payload is decoded correctly.

Upstream ports portions of libopus. Preserve each file's SPDX copyright and
license declarations. Jeffrey C. Ollie's contributions use MIT; derived code
additionally uses BSD-2-Clause or BSD-3-Clause, as declared per file. Full license
texts are in `LICENSES/`. Redistributed binaries must include the applicable
copyright notices, license conditions, and disclaimers in accompanying material.
