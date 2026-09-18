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

The APU has two pulse channels, triangle, noise, and DMC. It implements envelopes, length and linear counters, sweep units, frame sequences and interrupts, and nonlinear channel mixing. DMC reads use the CPU DMA engine. Pulse and noise DACs latch their values between channel updates; pulse-register writes also refresh the output. The triangle DAC retains its value when the sequencer stops. PAL selects its own APU periods and frame events; Dendy uses NTSC APU periods at its CPU clock rate.

Audio reconstruction records CPU-cycle output changes before producing host-rate samples. This preserves short channel transitions that occur between sample boundaries. Cartridge expansion audio participates in the same reconstruction path; EPSM retains its separate stereo contribution.

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
| 11, 144 | Color Dreams | Four-bit PRG/CHR bank selection and bus conflicts; mapper 144 takes D0 from the ROM |
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
| 34 | BNROM / NINA-001 | 32 KiB PRG banks, board-specific CHR/RAM access, and BNROM bus conflicts |
| 64, 158 | RAMBO-1 | PRG/CHR banks, CPU- or PPU-clocked IRQs, and mapper 158 nametable wiring |
| 66 | GxROM | Combined PRG/CHR bank selection and bus conflicts |
| 67 | Sunsoft 3 | 2 KiB CHR banks, switchable 16 KiB PRG, mirroring, and a one-shot CPU IRQ counter |
| 68 | Sunsoft 4 | 2 KiB CHR banks, CHR-backed nametables, protected cartridge RAM, and licensed external PRG selection |
| 69 | FME-7 / Sunsoft 5B | ROM/RAM bank selection, IRQ counter, and three-channel tone/noise/envelope audio |
| 70, 152 | Bandai 74161/7432 | Shared 16 KiB PRG and 8 KiB CHR bank register, fixed upper PRG bank, cartridge RAM, and single-screen mirroring control |
| 71 | Codemasters | PRG banking and the single-screen board variant |
| 72, 78, 87, 92, 101, 140 | Jaleco discrete boards | Board-specific PRG/CHR banking, latch edges, mapper 78 mirroring, and applicable ROM bus conflicts |
| 73 | VRC3 | Switchable 16 KiB PRG, fixed CHR, and 8- or 16-bit CPU-clocked IRQ counter |
| 74 | MMC3 mixed CHR | MMC3 banking and IRQs with CHR pages $08-$09 routed to 2 KiB RAM |
| 75, 151 | VRC1 | Three switchable 8 KiB PRG windows, two 4 KiB CHR banks, and board mirroring |
| 76, 88, 95, 154, 206 | Namco 108 family | Variant-specific PRG/CHR banking, hardwired or register-controlled nametables, and no mapper IRQ source |
| 77 | Irem LROG017 | 32 KiB PRG banking with bus conflicts, cartridge RAM, banked 2 KiB CHR ROM, fixed 6 KiB CHR RAM, and four-screen nametables |
| 79, 113, 146 | NINA-03/06 variants | 32 KiB PRG and 8 KiB CHR banks, partially decoded expansion registers, cartridge RAM, and mapper 113's extra bank bits and mirroring control |
| 80, 82, 207 | Taito X1-005 / X1-017 | PRG/CHR banking, protected fixed-size cartridge RAM, CHR mode selection, and mapper 207 nametable routing |
| 85 | VRC7 | PRG/CHR banking, IRQs, RAM control, and six-channel FM audio |
| 89, 93, 184 | Sunsoft discrete boards | Board-specific PRG/CHR selection, single-screen wiring, CHR access control, and paired 4 KiB CHR banks |
| 90, 209, 211 | JY Company | PRG/CHR modes, register arithmetic, mapper-specific nametable routing, latches, and selectable IRQ clock sources |
| 94, 180 | UxROM variants | Mapper 94 uses D2-D4 to select the lower 16 KiB PRG bank; mapper 180 switches the upper bank and fixes the lower bank at zero |
| 96 | Oeka Kids | 32 KiB PRG selection, cartridge RAM, banked CHR RAM, PPU-address-driven inner CHR selection, and ROM bus conflicts |
| 97 | Irem TAM-S1 | Fixed lower 16 KiB PRG, switchable upper 16 KiB PRG, cartridge RAM, fixed CHR, and four mirroring modes |
| 99 | VS System | Cabinet PRG/CHR selection, shared RAM permissions, and single/dual layouts |
| 104 | Golden Five | Outer PRG block and inner 16 KiB bank selection, fixed bank within the selected block, and CHR RAM |
| 31 | NSF cartridge | Eight independent 4 KiB PRG windows selected through `$5000-$5FFF`, with fixed CHR mapping |
| 86 | Jaleco JF-13 | 32 KiB PRG and 8 KiB CHR selection through `$6000-$6FFF`; speech is not emulated |
| 218 | Magic Floor | Fixed PRG and shared pattern-table/nametable CIRAM with four header-selected address wirings |
| 105 | NES-EVENT | MMC1 serial control, competition PRG modes, fixed CHR RAM, cartridge RAM, and DIP-selected timer IRQ |
| 111 | GTROM | 32 KiB PRG flash banking, two CHR-RAM banks, banked cartridge nametable RAM, register-read latching, and flash persistence |
| 118 | TKSROM / TLSROM | MMC3 banking and IRQs with CHR-register-controlled nametable routing |
| 119 | TQROM | MMC3 banking and IRQs with mixed CHR ROM and RAM |
| 155 | MMC1A | MMC1 banking with the earlier revision's RAM-enable behavior |
| 185 | Protected CNROM | CHR protection latch, ROM bus conflicts, and D0 pull-up behavior while pattern-table ROM is disabled |
| 191, 192, 194, 195 | MMC3 mixed CHR | MMC3 banking and IRQs with board-specific CHR ROM/RAM selection ranges |
| 232 | BF9096 | Outer PRG block and inner bank selection with the submapper-1 outer-bit wiring |

NES 2.0 submappers select supported wiring and revisions. Examples include MMC1 submapper 5, MMC6 submapper 1, MC-ACC submapper 3, and VRC register-wiring variants. UxROM, CNROM, and AxROM submapper 2 enable ROM bus conflicts. The loader rejects unsupported submappers and memory geometries even when the mapper family appears above. The complete checks are in [`mapper_init_from_header`](../src/rom/mapper.c).

Bandai 70/152 start with vertical mirroring and accept their shared bank register throughout `$8000-$FFFF`, without ROM bus conflicts. Mapper 152 selects either single-screen page on every write. Mapper 70 retains vertical mirroring until a write sets D7; later writes then select either single-screen page. The register state survives CPU soft reset. Both IDs ignore the NES 2.0 submapper field. Their [board implementation](../src/rom/boards/bandai.hpp) uses complete memory pages, including small and irregular ROM images.

Golden Five starts with PRG bank 15 at `$C000-$FFFF`; the lower 16 KiB window remains on open bus until a bank write. Writes at `$8000-$9FFF` change the outer block when D3 is set. Writes at `$C000-$FFFF` select the inner bank, and writes at `$A000-$BFFF` have no effect. The registers survive CPU soft reset, and mirroring follows the header. The board uses the default CHR RAM window and does not select CHR ROM. Small PRG images use the shared page-mapping rules.

Magic Floor connects pattern-table addresses to the same two CIRAM pages used by the nametables. Horizontal and vertical header settings select the address wiring. With the four-screen header bit set, the low mirroring bit instead selects single-screen A or B wiring. Changes through either pattern or nametable addresses are visible through their aliases, including CPU accesses through PPUDATA. A CHR ROM declaration does not replace this routing. PRG uses a fixed 32 KiB window, with complete smaller images repeated where they fit.

Jaleco JF-13 starts at PRG bank zero. Writes throughout `$6000-$6FFF` select the 32 KiB PRG bank with D4-D5 and the 8 KiB CHR bank with D0-D1 and D6. CHR ROM remains unmapped until the first bank write. Mirroring follows the header, and bank state survives CPU soft reset. Register writes do not modify underlying PRG RAM. The speech device at `$7000-$7FFF` is not emulated; those writes currently have no effect.

Mapper 31 selects eight 4 KiB PRG windows with the low three address bits of writes at `$5000-$5FFF`. Startup maps bank 255 into `$F000-$FFFF`; the other windows remain on open bus until written. The bank number wraps over complete ROM pages, and CPU soft reset preserves the selected windows. CHR starts at bank zero, and nametable mirroring follows the header. This mapper loads as a cartridge image; NSF and NSFe file execution is a separate media path.

The mapper 72 and 92 cartridge banking and latch behavior is implemented. Optional speech hardware on those boards is not currently emulated.

Jaleco 72/78/92, Irem 77/97, and mapper 96 expose their mapped PRG RAM for CPU reads and writes. Jaleco 87/101/140 and Sunsoft 184 read PRG RAM at `$6000-$7FFF`, but writes in that window select banks instead of changing RAM. Explicit NES 2.0 zero-RAM declarations leave those reads on open bus. Trainers and battery saves can supply nonzero data to the readable RAM windows.

Mapper 77 always exposes 6 KiB of fixed CHR RAM at PPU `$0800-$1FFF`; the lower 2 KiB is banked CHR ROM. NES 2.0 images for this mixed layout must declare 8 KiB of volatile CHR RAM, which is the header size accepted by the loader for the board.

Mapper 185 submapper 0 keeps the legacy compatibility rule: CHR is enabled when the low nibble is nonzero except for latch value `0x13`. NES 2.0 submappers 4 through 7 use bits 0 and 1 as an exact enable value from 0 through 3.

Mappers 79 and 146 accept bank writes at `$4100-$5FFF` only when address bit A8 is high. Mapper 113 uses the same register decoding, with three PRG bank bits, four CHR bank bits, and vertical/horizontal mirroring selected by D7. These registers are write-only; reads retain open bus. All three boards keep ordinary RAM reads and writes at `$6000-$7FFF`. Their bank registers survive CPU soft reset.

Mappers 94 and 180 retain a fixed CHR window and header-selected mirroring. Mapper 94 uses three bits to select the lower PRG bank and fixes the upper window to the last available bank. Mapper 180 uses all eight bank bits for its upper window and starts with bank zero in both CPU windows. Mappers 79, 94, 113, 144, 146, and 180 select their wiring by mapper number and ignore the NES 2.0 submapper field.

Larger ROM images retain the board's implemented bank-selection bits. Bank numbers wrap across complete available pages; trailing partial pages do not add selectable banks. Small ROMs map their available pages, and uncovered addresses retain open bus. The RAM layout checks remain separate: these six variants reject simultaneous volatile and nonvolatile RAM declarations or CHR ROM combined with separate CHR RAM. Failed loads preserve the active cartridge.

Color Dreams writes resolve ROM bus conflicts before selecting banks. Both mapper 11 and mapper 144 retain all four PRG selection bits. Mapper 144 then takes D0 from the byte in the previously mapped ROM bank, so a CPU write of zero can still select an odd PRG bank.

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

NES 2.0 RAM declarations are normally explicit, including zero RAM. Taito X1-005/X1-017 boards are fixed-size exceptions: mappers 80 and 207 use a 256-byte cartridge-RAM allocation, while mapper 82 uses 5 KiB. For those boards, the battery flag chooses volatile or nonvolatile storage even when the NES 2.0 RAM-size fields are zero. Unsupported combinations are rejected. Do not change header bytes simply to make the loader accept an image: the resulting bank layout or save format could be wrong. Use the cartridge's board information when correcting a header, and record that correction in a bug report.

Trainer initialization runs before save memory loads and copies the bytes into supported `$7000-$71FF` RAM windows. Existing save bytes take precedence where they overlap the trainer. See [saves and media](saves.md) for save layouts, including MMC5 ExRAM and N163 audio RAM.

## Cartridge behavior

Bank switching changes which PRG or CHR storage appears at an address. It does not copy a new game into the CPU's internal RAM. Some boards also control nametable memory, expose cartridge RAM, generate interrupts, or add audio channels.

The implementation includes bus conflicts, initially unmapped windows, RAM permissions, persistent EEPROM and flash devices, and IRQ delivery through the CPU's normal polling path. These details are covered by board-specific tests in [mapper_accuracy.c](../src/tests/mapper_accuracy.c) and [bandai_accuracy.c](../src/tests/bandai_accuracy.c).

Some families have substantially different variants. Namco 175/340 variants do not all contain the N163 audio device. VRC2 variants differ from VRC4 IRQ-capable boards. The supported family name is not a claim that every member has every feature listed for that family.

Expansion sound includes MMC5 pulse/PCM, VRC6 pulse/saw, VRC7 FM, Namco 163 wavetable, Sunsoft 5B tone/noise/envelopes, and disk-system wavetable/modulation output. [Architecture](architecture.md) describes how the audio reaches the application.

## EPSM expansion sound

The YMF288 implementation provides six FM channels, three SSG tone/noise/envelope channels, and six ADPCM percussion voices. The CPU can write through `$401C-$401F` or the `$4016` data-bus/OUT-pin protocol. The delayed OUT1 edge samples the data bus at the time the pin changes. Timer IRQs join the CPU's ordinary interrupt polling path.

The device uses the same 8 MHz oscillator across NTSC, PAL, and Dendy CPU timings. Generated samples pass through the application's stereo callback; SSG output is centered, and FM and percussion retain their panning. A cold power-on restores the chip and protocol state. CPU soft reset preserves them.

Percussion needs a separate, exactly 8 KiB ADPCM ROM supplied with `--epsm-adpcm`. Without it, the device uses zero-filled data. Incorrect-size files are rejected, and failed cartridge or firmware loads preserve the active device. The [EPSM tests](../src/tests/epsm_accuracy.c) cover both bus protocols, CPU-driven delayed edges, timer IRQs, regional clocks, reset, stereo output, firmware validation, and transitions to other hardware.

## Controllers and expansion devices

Device selection changes the signals that CPU reads and writes see at `$4016` and `$4017`. The frontend supplies buttons, keys, pointer coordinates and barcode scans; the controller layer handles each device's latch, shift and timing rules.

| Connection | Devices | Implemented behavior |
| --- | --- | --- |
| Controller ports | Gamepads, Four Score, Arkanoid, Power Pad, Zapper | Serial reports, adapter signatures, paddle positions, mat wiring and beam-aware light sensing |
| Controller ports | SNES controller and mouse, NTT Data keypad, Virtual Boy controller | Device-specific serial packets, latch/strobe behavior, mouse sensitivity and both Virtual Boy D-pads |
| VS controller wiring | VS Zapper | Metadata-selected serial gun report, trigger state and beam timing |
| Famicom expansion | Two- and four-player adapters, Arkanoid, Family Trainer, Zapper | Expansion-line routing with separate ordinary controller bits |
| Famicom expansion | Family BASIC and Subor keyboards | Scanned key matrices; Family BASIC also supplies the data-recorder signal |
| Controller port 2 | Subor mouse | Button and signed movement reports, including extended packets |
| Famicom expansion | Hori Track | Latched buttons and signed trackball movement |
| Famicom expansion | Konami and Bandai Hyper Shot | Latched running/jumping controls, or serial controls with light-gun sensing |
| Famicom expansion | Party Tap, Pachinko, Exciting Boxing, Jissen Mahjong | Device-specific switch groups, serial reports, plunger state and matrix selection |
| Famicom expansion | Oeka Kids tablet | Coordinates, contact/click bits, ready line and an 18-bit serial report; mapper 96 supplies the cartridge's CHR latch |
| Cartridge or Famicom expansion | Datach and Barcode Battler | Separate barcode formats and CPU-clocked streams |
| Famicom expansion | ASCII Turbo File and BattleBox | Serial storage commands, memory contents and persistent save files |

The ASCII Turbo File has 8 KiB of storage with D1 reset, D2 clock, D0 write data and `$4017` D2 read data, including bit-position wrap. BattleBox has two 128-word serial chips with command decoding, D0 edge handshake, chip selection, D3 read data, alternating D4 output, write protection, programming and erase commands.

For Oeka Kids input, the frontend treats the left mouse button as both click and touch. Pointer hover counts as touch only while the pointer is on the emulated screen at Y 48 through 239. The tablet protocol then scales the screen coordinates into its serial coordinate fields.

Choose devices explicitly with the options in [configuration](configuration.md). An expansion adapter and another expansion device cannot occupy the same connector. [Controls](controls.md) lists the host input mappings, and [saves and media](saves.md) describes storage formats and failed-save handling.

## Disk-system media

The disk loader accepts supported headered and raw FDS images, plus the QD layouts recognized from their sizes. FDS sides contain 65,500 bytes; QD sides contain 65,536. An optional FDS header contributes another 16 bytes. The loader validates the declared number of sides and requires an 8 KiB BIOS.

The device supplies 32 KiB work RAM, 8 KiB CHR RAM, BIOS mapping, disk transport and block timing, CRC handling, timer and transfer interrupts, and audio. Side selection, ejection, and write protection are available through the application. [Disk tests](../src/tests/fds_accuracy.c) exercise the production loader and cartridge bus with synthetic media and BIOS data.

Writable images are updated at the loaded path. See [saves and media](saves.md) for backup and failure behavior.

## VS System

The supported VS configurations use mappers 0, 1, 2, 75, 99, or 151 with NTSC timing. Dual cabinets require mapper 99. NES 2.0 metadata selects the hardware type, PPU, and controller wiring. Legacy mapper 99 images use the implemented ROM-size convention to select single or dual operation. Cupid has no database that identifies a game's hardware from its hash.

The PPU choices include the 2C03 RGB palette, four 2C04 palettes, and the implemented 2C05 register/status variants. Cabinet handling includes DIP switches, coin and service inputs, controller routing, and the implemented protection-read sequences.

NES 2.0 console selector 3 with extended subtype 1 also selects VS hardware. That encoding uses the existing 2C03 profile as a compatibility fallback because its subtype occupies the direct descriptor's PPU field. It retains the cabinet type and input metadata. Direct VS descriptors with PPU code 1 (RP2C03G) or unknown codes 13 through 15 also use 2C03 behavior and print a diagnostic. Other defined PPU codes select their corresponding RGB profile.

Dual mode maintains independent CPU, PPU, APU, internal RAM, input, and DMA state. The boards share cartridge RAM with ownership controlled by the hardware signal. Cross-CPU interrupts and synchronized stepping support communication between the two sides. Both screens are presented and both APUs feed mono output.

There is no distinct RP2C03G palette or hardware model. The 2C03 fallback allows those images to load; it does not establish their palette accuracy. Unsupported console, mapper, memory, cabinet, and controller combinations are still rejected. NES 2.0 VS Zapper metadata selects the serial gun report on the first controller port. The [VS tests](../src/tests/vs_accuracy.c) cover the fallback and supported RGB profiles, including real CPU programs that communicate through shared RAM and produce separate video and audio.

## Reading accuracy results

The [checkpoint record](accuracy-checkpoints.md) identifies commits that passed all 144 AccuracyCoin tests without skipped or unfinished results. The [accuracy notes](accuracy.md) describe the separate CPU trace, diagnostic collection, focused hardware tests, and setup conditions for older ROMs.

These results are regression evidence. They do not prove compatibility with every game, physical console revision, or register interleaving. Optional OAM corruption and decay profiles are deterministic approximations. Analog output, MMC5 auxiliary I/O and `$5209/$520A` timers, and unlisted hardware remain outside the implemented or tested scope.

A compatibility report should identify the exact build, image hash, header, hardware profile, and failing behavior. The [reporting guide](../CONTRIBUTING.md#reporting-a-bug) explains how to make that reproducible.
