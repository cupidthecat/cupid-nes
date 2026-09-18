# Hardware and compatibility

[Documentation index](README.md)

Cupid models the CPU, picture processing unit (PPU), audio processing unit (APU), controller wiring, and the hardware in each supported cartridge. The [mapper table in the main README](../README.md#cartridges) lists implemented board families. A supported mapper can still reject a submapper or memory layout that the core cannot represent.

## Systems and timing

| System profile | Timing selection | Notes |
| --- | --- | --- |
| NES and Famicom cartridges | iNES or NES 2.0 header | NTSC, PAL, and Dendy timing are implemented |
| Famicom Disk System | NTSC | Requires a supplied BIOS and a supported disk image |
| VS System | NTSC | Requires supported console, PPU, input, and cartridge metadata |

NTSC uses 262 scanlines with vblank beginning at line 241. PAL uses 312 scanlines with vblank beginning at line 241, and Dendy uses 312 with vblank beginning at line 291. PAL advances the PPU at 3.2 clocks per CPU clock; NTSC and Dendy use 3. Only NTSC rendering skips a clock on odd frames.

The [timing table](../src/system/timing.c) supplies device rates and frame pacing. The [loader](../src/rom/rom.c) selects timing from the header. NES 2.0 dual-region images default to NTSC. Console wiring selected with `--console` is independent of this timing choice.

## What the cartridge header controls

An image begins with an iNES or NES 2.0 header that describes the board and its memory. A mapper number identifies a hardware family. A submapper narrows that choice to a wiring or chip variant, such as an IRQ-counter revision or a different register address layout.

| Metadata | How Cupid uses it |
| --- | --- |
| PRG ROM | CPU program data and bank geometry |
| CHR ROM or RAM | PPU pattern data and writable graphics memory |
| Mapper and submapper | Register decoding, banking, mirroring, and device behavior |
| Mirroring flags | Initial nametable layout, subject to board-specific wiring |
| Volatile and nonvolatile RAM | Allocation, addressability, and persistent storage |
| Timing and console type | Regional timing or a supported arcade configuration |
| Trainer flag | A 512-byte initialization window applied through cartridge handling |

The loader checks payload sizes and size overflows before activating a cartridge. A truncated or unsupported image returns an error and preserves an already loaded cartridge. The application exits when its initial load fails; preservation also matters to callers of the loader API.

Legacy iNES RAM fields are unreliable. Cupid uses board defaults and ignores byte 8 as a RAM-size override. A legacy PRG count of zero represents 256 banks of 16 KiB, or 4 MiB. The image must still contain the complete payload, and its mapper must support that size.

NES 2.0 RAM declarations are explicit, including zero RAM. Unsupported combinations are rejected. Do not change header bytes simply to make the loader accept an image: the resulting bank layout or save format could be wrong. Use the cartridge's board information when correcting a header, and record that correction in a bug report.

## Cartridge behavior

Bank switching changes which PRG or CHR storage appears at an address. It does not copy a new game into the CPU's internal RAM. Some boards also control nametable memory, expose cartridge RAM, generate interrupts, or add audio channels.

The implementation includes bus conflicts, initially unmapped windows, RAM permissions, persistent EEPROM and flash devices, and IRQ delivery through the CPU's normal polling path. These details are covered by board-specific tests in [mapper_accuracy.c](../src/tests/mapper_accuracy.c) and [bandai_accuracy.c](../src/tests/bandai_accuracy.c).

Some families have substantially different variants. Namco 175/340 variants do not all contain the N163 audio device. VRC2 variants differ from VRC4 IRQ-capable boards. The supported family name is not a claim that every member has every feature listed for that family.

Expansion sound includes MMC5 pulse/PCM, VRC6 pulse/saw, VRC7 FM, Namco 163 wavetable, Sunsoft 5B tone/noise/envelopes, and disk-system wavetable/modulation output. [Architecture](architecture.md) describes how the audio reaches the application.

## Disk-system media

The disk loader accepts supported headered and raw FDS images, plus the QD layouts recognized from their sizes. FDS sides contain 65,500 bytes; QD sides contain 65,536. An optional FDS header contributes another 16 bytes. The loader validates the declared number of sides and requires an 8 KiB BIOS.

The device supplies 32 KiB work RAM, 8 KiB CHR RAM, BIOS mapping, disk transport and block timing, CRC handling, timer and transfer interrupts, and audio. Side selection, ejection, and write protection are available through the application. [Disk tests](../src/tests/fds_accuracy.c) exercise the production loader and cartridge bus with synthetic media and BIOS data.

Writable images are updated at the loaded path. See [saves and media](saves.md) for backup and failure behavior.

## VS System

The supported VS configurations use mappers 0, 1, 2, or 99. NES 2.0 metadata selects the hardware type, PPU, and controller wiring. Legacy mapper 99 images use the implemented ROM-size convention to select single or dual operation. Cupid has no database that identifies a game's hardware from its hash.

The PPU choices include the 2C03 RGB palette, four 2C04 palettes, and the implemented 2C05 register/status variants. Cabinet handling includes DIP switches, coin and service inputs, controller routing, and the implemented protection-read sequences.

Dual mode maintains independent CPU, PPU, APU, internal RAM, input, and DMA state. The boards share cartridge RAM with ownership controlled by the hardware signal. Cross-CPU interrupts and synchronized stepping support communication between the two sides. Both screens are presented and both APUs feed mono output.

RP2C03G, VS Zapper input wiring, and unimplemented combinations of console, mapper, memory, or PPU metadata are rejected. The [VS tests](../src/tests/vs_accuracy.c) cover supported paths, including real CPU programs that communicate through shared RAM and produce separate video and audio.

## Reading accuracy results

The [checkpoint record](accuracy-checkpoints.md) identifies commits that passed all 144 AccuracyCoin tests without skipped or unfinished results. The [accuracy notes](accuracy.md) describe the separate CPU trace, diagnostic collection, focused hardware tests, and setup conditions for older ROMs.

These results are regression evidence. They do not prove compatibility with every game, physical console revision, or register interleaving. Optional OAM corruption and decay profiles are deterministic approximations. Analog output, arbitrary power-on alignment, MMC5 auxiliary I/O and `$5209/$520A` timers, and unlisted hardware remain outside the implemented or tested scope.

A compatibility report should identify the exact build, image hash, header, hardware profile, and failing behavior. The [reporting guide](../CONTRIBUTING.md#reporting-a-bug) explains how to make that reproducible.
