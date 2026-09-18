# Hardware and compatibility

[Documentation index](README.md)

Cupid models the CPU, picture processing unit (PPU), audio processing unit (APU), controller wiring, and the hardware in each supported cartridge. The [mapper table below](#cartridge-mappers) lists implemented board families. A supported mapper can still reject a submapper or memory layout that the core cannot represent.

## Systems and timing

| System profile | Timing selection | Notes |
| --- | --- | --- |
| NES and Famicom cartridges | iNES or NES 2.0 header | NTSC, PAL, and Dendy timing are implemented |
| Famicom Disk System | NTSC | Requires a supplied BIOS and a supported disk image |
| VS System | NTSC | Requires supported console, PPU, input, and cartridge metadata |
| NES with EPSM | NES 2.0 header | Extended subtype 4 adds an 8 MHz YMF288 with stereo output |

NTSC uses 262 scanlines with vblank beginning at line 241. PAL uses 312 scanlines with vblank beginning at line 241, and Dendy uses 312 with vblank beginning at line 291. PAL advances the PPU at 3.2 clocks per CPU clock; NTSC and Dendy use 3. The NTSC 2C02 skips one clock on rendered odd frames. VS RGB PPUs retain all 89,342 clocks on both frame parities, including both sides of a dual cabinet.

The [timing table](../src/system/timing.c) supplies device rates and frame pacing. The [loader](../src/rom/rom.c) selects timing from the header. NES 2.0 dual-region images default to NTSC. Console wiring selected with `--console` is independent of this timing choice. The application has no region override; the legacy PAL diagnostic runner has an explicit test-only mode.

## CPU, graphics, and audio

The CPU implements official and undocumented opcodes, page-crossing and read-modify-write bus accesses, interrupt polling, BRK vector hijacking, and JAM behavior. CPU bus accesses clock the PPU, APU, and cartridge. OAM and DMC DMA share that bus, including their halt, alignment, and overlap behavior.

The PPU renders 256 by 240 pixels. Scheduled pattern fetches feed background and sprite shifters, sprite evaluation, overflow, clipping, priority, and sprite-zero hits. Registers include palette mirrors, delayed address and data transfers, buffered reads, open bus, rendering-time address increments, and vblank/NMI edges. PAL has its own vblank OAM refresh and PAL/Dendy color-emphasis wiring.

The APU has two pulse channels, triangle, noise, and DMC. It implements envelopes, length and linear counters, sweep units, frame sequences and interrupts, and nonlinear channel mixing. DMC reads use the CPU DMA engine. The triangle DAC retains its value when the sequencer stops. PAL selects its own APU periods and frame events; Dendy uses NTSC APU periods at its CPU clock rate.

Power-on and soft reset are separate operations. See [architecture](architecture.md) for state ownership and [accuracy](accuracy.md) for bus phases and reset details.

## Cartridge mappers

PRG is the cartridge memory read by the CPU; CHR holds graphics patterns read by the PPU. A mapper controls bank selection, mirroring, and any additional hardware on the cartridge.

| Mapper | Board family | Implemented behavior |
| --- | --- | --- |
| 0 | NROM | Fixed PRG/CHR mapping and header mirroring |
| 1 | MMC1 / SxROM | Serial banking, consecutive-write filtering, mirroring, outer PRG selection, and supported SOROM/SXROM RAM layouts |
| 2 | UxROM | Switchable 16 KiB PRG bank and fixed upper bank |
| 3 | CNROM | CHR bank selection |
| 4 | MMC3 / MMC6 | PRG/CHR banking, filtered PPU A12 IRQ clocks, selectable MMC3 revision-A IRQ qualification, and RAM protection; submapper 1 selects MMC6 |
| 4, submapper 3 | MC-ACC | Falling-edge A12 filtering and IRQ timing |
| 5 | MMC5, partial | PRG/CHR banking, banked RAM, ExRAM/fill nametables, extended attributes, vertical split, multiplication, PPU-read-driven scanline IRQs, and pulse/PCM audio |
| 7 | AxROM | 32 KiB PRG banking and single-screen mirroring |
| 9 | MMC2 | PRG banking and pattern-fetch CHR latches |
| 10 | MMC4 | PRG banking and pattern-fetch CHR latches |
| 11 | Color Dreams | PRG/CHR bank selection and bus conflicts |
| 13 | CPROM | Banked CHR RAM |
| 15 | 100-in-1 | Address-selected PRG banking and mirroring |
| 16, 153, 157, 159 | Bandai FCG / LZ93D50 / Datach | Bank wiring, IRQs, serial EEPROM, outer PRG selection, and barcode signals |
| 18 | Jaleco SS88006 | Nibble-based bank registers and selectable IRQ counter widths |
| 19, 210 | Namco 163 / 175 / 340 | PRG/CHR banks, cartridge-backed nametables, RAM permissions, IRQs, and N163 wavetable audio |
| 21, 22, 23, 25, 27, 183 | VRC2 / VRC4 | Board-specific register wiring, bank selection, mirroring, and VRC4 IRQs |
| 24, 26 | VRC6 | PRG/CHR and nametable banking, IRQs, two pulse channels, and sawtooth audio |
| 28 | Action 53 | Outer and inner PRG selection, CHR RAM, mirroring, and startup mapping |
| 30 | UNROM 512 | PRG/CHR banking, cartridge nametable memory, and flash programming and erase commands |
| 32, 65 | Irem G-101 / H-3001 | PRG/CHR banking, board mirroring, and H-3001 IRQ timing |
| 33, 48 | Taito | PRG/CHR banking, mirroring, and mapper 48 IRQ timing |
| 80, 82, 207 | Taito X1-005 / X1-017 | PRG/CHR banking, protected cartridge RAM, CHR mode selection, and mapper 207 nametable routing |
| 34 | BNROM / NINA-001 | 32 KiB PRG banks, board-specific CHR/RAM access, and BNROM bus conflicts |
| 64, 158 | RAMBO-1 | PRG/CHR banks, CPU- or PPU-clocked IRQs, and mapper 158 nametable wiring |
| 66 | GxROM | Combined PRG/CHR bank selection and bus conflicts |
| 67 | Sunsoft 3 | 2 KiB CHR banks, switchable 16 KiB PRG, mirroring, and a one-shot CPU IRQ counter |
| 68 | Sunsoft 4 | 2 KiB CHR banks, CHR-backed nametables, protected cartridge RAM, and licensed external PRG selection |
| 69 | FME-7 / Sunsoft 5B | ROM/RAM bank selection, IRQ counter, and three-channel tone/noise/envelope audio |
| 71 | Codemasters | PRG banking and the single-screen board variant |
| 73 | VRC3 | Switchable 16 KiB PRG, fixed CHR, and 8- or 16-bit CPU-clocked IRQ counter |
| 75, 151 | VRC1 | Three switchable 8 KiB PRG windows, two 4 KiB CHR banks, and board mirroring |
| 76, 88, 95, 154, 206 | Namco 108 family | Variant-specific PRG/CHR banking, hardwired or register-controlled nametables, and no mapper IRQ source |
| 85 | VRC7 | PRG/CHR banking, IRQs, RAM control, and six-channel FM audio |
| 89, 93, 184 | Sunsoft discrete boards | Board-specific PRG/CHR selection, single-screen wiring, CHR access control, and paired 4 KiB CHR banks |
| 99 | VS System | Cabinet PRG/CHR selection, shared RAM permissions, and single/dual layouts |
| 105 | NES-EVENT | MMC1 serial control, competition PRG modes, fixed CHR RAM, cartridge RAM, and DIP-selected timer IRQ |
| 118 | TKSROM / TLSROM | MMC3 banking and IRQs with CHR-register-controlled nametable routing |
| 119 | TQROM | MMC3 banking and IRQs with mixed CHR ROM and RAM |
| 155 | MMC1A | MMC1 banking with the earlier revision's RAM-enable behavior |
| 232 | BF9096 | Outer PRG block and inner bank selection with the submapper-1 outer-bit wiring |

NES 2.0 submappers select supported wiring and revisions. Examples include MMC1 submapper 5, MMC6 submapper 1, MC-ACC submapper 3, and VRC register-wiring variants. UxROM, CNROM, and AxROM submapper 2 enable ROM bus conflicts. The loader rejects unsupported submappers and memory geometries even when the mapper family appears above. The complete checks are in [`mapper_init_from_header`](../src/rom/mapper.c).

## What the cartridge header controls

An image begins with an iNES or NES 2.0 header that describes the board and its memory. A mapper number identifies a hardware family. A submapper narrows that choice to a wiring or chip variant, such as an IRQ-counter revision or a different register address layout.

| Metadata | How Cupid uses it |
| --- | --- |
| PRG ROM | CPU program data and bank geometry |
| CHR ROM or RAM | PPU pattern data and writable graphics memory |
| Mapper and submapper | Register decoding, banking, mirroring, and device behavior |
| Mirroring flags | Initial nametable layout, subject to board-specific wiring |
| Volatile and nonvolatile RAM | Allocation, addressability, and persistent storage |
| Timing and console type | Regional timing, supported arcade configuration, or EPSM expansion sound |
| Trainer flag | A 512-byte initialization window applied through cartridge handling |

The loader checks payload sizes and size overflows before activating a cartridge. A truncated or unsupported image returns an error and preserves an already loaded cartridge. The application exits when its initial load fails; preservation also matters to callers of the loader API.

Legacy iNES RAM fields are unreliable. Cupid uses board defaults and ignores byte 8 as a RAM-size override. Most boards default to 8 KiB, MMC5 to 64 KiB, and FME-7 to 32 KiB. Legacy mapper 99 without a battery flag uses 2 KiB of volatile RAM. UNROM 512 has its own CHR RAM and flash layout. A legacy PRG count of zero represents 256 banks of 16 KiB, or 4 MiB. The image must still contain the complete payload, and its mapper must support that size.

NES 2.0 RAM declarations are explicit, including zero RAM. Unsupported combinations are rejected. Do not change header bytes simply to make the loader accept an image: the resulting bank layout or save format could be wrong. Use the cartridge's board information when correcting a header, and record that correction in a bug report.

Trainer initialization runs after save memory loads and copies the bytes into supported `$7000-$71FF` RAM windows. See [saves and media](saves.md) for save layouts, including MMC5 ExRAM and N163 audio RAM.

## Cartridge behavior

Bank switching changes which PRG or CHR storage appears at an address. It does not copy a new game into the CPU's internal RAM. Some boards also control nametable memory, expose cartridge RAM, generate interrupts, or add audio channels.

The implementation includes bus conflicts, initially unmapped windows, RAM permissions, persistent EEPROM and flash devices, and IRQ delivery through the CPU's normal polling path. These details are covered by board-specific tests in [mapper_accuracy.c](../src/tests/mapper_accuracy.c) and [bandai_accuracy.c](../src/tests/bandai_accuracy.c).

Some families have substantially different variants. Namco 175/340 variants do not all contain the N163 audio device. VRC2 variants differ from VRC4 IRQ-capable boards. The supported family name is not a claim that every member has every feature listed for that family.

Expansion sound includes MMC5 pulse/PCM, VRC6 pulse/saw, VRC7 FM, Namco 163 wavetable, Sunsoft 5B tone/noise/envelopes, and disk-system wavetable/modulation output. [Architecture](architecture.md) describes how the audio reaches the application.

## EPSM expansion sound

The YMF288 implementation provides six FM channels, three SSG tone/noise/envelope channels, and six ADPCM percussion voices. The CPU can write through `$401C-$401F` or the `$4016` data-bus/OUT-pin protocol. The delayed OUT1 edge samples the data bus at the time the pin changes. Timer IRQs join the CPU's ordinary interrupt polling path.

The device uses the same 8 MHz oscillator across NTSC, PAL, and Dendy CPU timings. Generated samples pass through the application's stereo callback; SSG output is centered, and FM and percussion retain their panning. A cold power-on restores the chip and protocol state. CPU soft reset preserves them.

Percussion needs a separate, exactly 8 KiB ADPCM ROM supplied with `--epsm-adpcm`. Without it, the device uses zero-filled data. Incorrect-size files are rejected, and failed cartridge or firmware loads preserve the active device. The [EPSM tests](../src/tests/epsm_accuracy.c) cover both bus protocols, CPU-driven delayed edges, timer IRQs, regional clocks, reset, stereo output, firmware validation, and transitions to other hardware.

## Disk-system media

The disk loader accepts supported headered and raw FDS images, plus the QD layouts recognized from their sizes. FDS sides contain 65,500 bytes; QD sides contain 65,536. An optional FDS header contributes another 16 bytes. The loader validates the declared number of sides and requires an 8 KiB BIOS.

The device supplies 32 KiB work RAM, 8 KiB CHR RAM, BIOS mapping, disk transport and block timing, CRC handling, timer and transfer interrupts, and audio. Side selection, ejection, and write protection are available through the application. [Disk tests](../src/tests/fds_accuracy.c) exercise the production loader and cartridge bus with synthetic media and BIOS data.

Writable images are updated at the loaded path. See [saves and media](saves.md) for backup and failure behavior.

## VS System

The supported VS configurations use mappers 0, 1, 2, 75, 99, or 151 with NTSC timing. Dual cabinets require mapper 99. NES 2.0 metadata selects the hardware type, PPU, and controller wiring. Legacy mapper 99 images use the implemented ROM-size convention to select single or dual operation. Cupid has no database that identifies a game's hardware from its hash.

The PPU choices include the 2C03 RGB palette, four 2C04 palettes, and the implemented 2C05 register/status variants. Cabinet handling includes DIP switches, coin and service inputs, controller routing, and the implemented protection-read sequences.

NES 2.0 console selector 3 with extended subtype 1 also selects VS hardware. That encoding uses the existing 2C03 profile as a compatibility fallback because its subtype occupies the direct descriptor's PPU field. It retains the cabinet type and input metadata. This fallback does not add RP2C03G emulation; direct VS descriptors keep their existing PPU selection and validation.

Dual mode maintains independent CPU, PPU, APU, internal RAM, input, and DMA state. The boards share cartridge RAM with ownership controlled by the hardware signal. Cross-CPU interrupts and synchronized stepping support communication between the two sides. Both screens are presented and both APUs feed mono output.

RP2C03G and unimplemented combinations of console, mapper, memory, or PPU metadata are rejected. NES 2.0 VS Zapper metadata selects the serial gun report on the first controller port. The [VS tests](../src/tests/vs_accuracy.c) cover supported paths, including real CPU programs that communicate through shared RAM and produce separate video and audio.

## Reading accuracy results

The [checkpoint record](accuracy-checkpoints.md) identifies commits that passed all 144 AccuracyCoin tests without skipped or unfinished results. The [accuracy notes](accuracy.md) describe the separate CPU trace, diagnostic collection, focused hardware tests, and setup conditions for older ROMs.

These results are regression evidence. They do not prove compatibility with every game, physical console revision, or register interleaving. Optional OAM corruption and decay profiles are deterministic approximations. Analog output, MMC5 auxiliary I/O and `$5209/$520A` timers, and unlisted hardware remain outside the implemented or tested scope.

A compatibility report should identify the exact build, image hash, header, hardware profile, and failing behavior. The [reporting guide](../CONTRIBUTING.md#reporting-a-bug) explains how to make that reproducible.
