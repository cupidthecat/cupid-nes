# Credits and references

Cupid is distributed under GPL-3.0-or-later; see [LICENSE](../LICENSE).
Project files with an author header credit
[@frankischilling](https://github.com/frankischilling). Keep existing copyright
and license notices with the files they describe. Imported components retain
their own notices.

## Cartridge board code

The [cartridge board modules](../src/rom/boards) include page-mapping and board
logic adapted from work by Sour and contributors, under GPL-3.0-or-later.
Those files retain their copyright and license notices. Cupid supplies the
loader, CPU/PPU bus integration, storage ownership, and regression fixtures.

## Bundled FM synthesis

The VRC7 implementation uses the bundled
[emu2413 source](../src/rom/emu2413.c) and [header](../src/rom/emu2413.h), with a
Cupid wrapper in [vrc7_audio.c](../src/rom/vrc7_audio.c). The source identifies
emu2413 v1.5.9, credits Mitsutaka Okazaki, and records its upstream project and
hardware references. Cupid's local corrections remain in the repository's Git
history.

The component's [MIT license](../src/rom/emu2413.LICENSE) is retained alongside
the source. Its notice is separate from Cupid's project license.

## Audio reconstruction

The APU uses Shay Green's [blip_buf](../src/third_party/blip_buf.c) to reconstruct
CPU-cycle output changes at the host sample rate. The source retains its
copyright notice and [LGPL-2.1-or-later license](../src/third_party/blip_buf.LICENSE).
The local adaptation supplies the C interface and avoids shifting negative
signed samples when updating the reconstruction integrator.

## EPSM sound engine

EPSM uses Aaron Giles's ymfm YMF288 implementation. The bundled OPN, SSG, and ADPCM sources retain their copyright notices and [BSD 3-Clause license](../src/third_party/ymfm/LICENSE). The [component notes](../src/third_party/ymfm/README.md) describe the local build edits. Cupid supplies the CPU-bus, timer, firmware, and stereo-output integration in [epsm.cpp](../src/apu/epsm.cpp).

The YMF288 percussion ROM is not included. [Configuration](configuration.md#epsm-sound) explains how to supply an existing 8 KiB file.

## SDL2

Cupid links against SDL2 for windows, rendering, controller events, and audio.
Install its development package as described in [getting started](getting-started.md).
The Windows build script copies the supplied `SDL2.dll` into the output
directory. SDL2's own package carries its license and notices.

## Diagnostic sources

The automated external test collection comes from
[christopherpow/nes-test-roms](https://github.com/christopherpow/nes-test-roms).
It supplies the canonical CPU trace and the diagnostic ROM groups listed in
[accuracy](accuracy.md). The complete cartridge baseline comes from
[100thCoin/AccuracyCoin](https://github.com/100thCoin/AccuracyCoin).

These are separate upstream projects. The [development guide](development.md)
records the pinned revisions and AccuracyCoin ROM hash used by the workflow.
Preserve each collection's accompanying documentation and notices. Files in
`test-roms/` are not a substitute for those pinned checkouts.

## Hardware references

The [NESdev wiki](https://www.nesdev.org/wiki/Nintendo_Entertainment_System)
documents the console and its variants. Its [PPU](https://www.nesdev.org/wiki/PPU),
[APU](https://www.nesdev.org/wiki/APU),
[mapper](https://www.nesdev.org/wiki/Mapper), and
[NES 2.0 submapper](https://www.nesdev.org/wiki/NES_2.0_submappers) pages are
starting points for hardware investigations. The emu2413 source header lists
the references used for its sound synthesis.

When a fix depends on a particular register edge, board revision, or hardware
measurement, cite the relevant source beside the regression or in the pull
request. The [accuracy notes](accuracy.md) record test-specific assumptions and
known gaps in Cupid's model.
