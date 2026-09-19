# NES accuracy checks

[Documentation index](README.md)

The hardware targets use NTSC, PAL, or Dendy timing with the cartridge and input devices listed in the [hardware reference](hardware.md). Disk and VS systems use NTSC timing. The tests below exercise the production core. They do not establish compatibility with every cartridge or hardware revision.

`make test` and the Windows build script run the synthetic hardware suite without external ROMs. The canonical trace and 91-ROM collection use `scripts/run-diagnostics.py`; AccuracyCoin is a separate runner mode. The [development guide](development.md) covers fixture setup, and [combined validation](accuracy-checkpoints.md#combined-validation) links a tested implementation to its CI results.

## Timing model

Each CPU read or write advances the PPU, APU, and cartridge. Reads and writes place the bus operation at different phases of the CPU clock. Fractional PPU clocks carry across CPU accesses. Interrupt lines are sampled at the CPU cycle boundaries used by instruction polling.

`--cpu-test-mode` enables ordinary 2A03 diagnostic reads: `$4018` packs pulse 1 in the low nibble and pulse 2 in the high nibble, `$4019` packs triangle and noise the same way, and `$401A` returns the DMC's seven-bit output. The values come from the channel DAC latches used by the mixer. Disabling a pulse or noise channel through `$4015` clears its length counter; its preceding DAC value remains visible until the next channel timer edge. Pulse-register writes also refresh their channel's output.

The default retains open bus at these addresses. The profile survives reset and leaves `$4015`'s internal-bus behavior intact. Writable test registers and DMA-specific test-pin interactions are outside this profile's scope.

| Region | Master clocks per CPU/PPU cycle | Scanlines | Vblank starts |
| --- | --- | ---: | ---: |
| NTSC | 12 / 4 | 262 | 241 |
| PAL | 16 / 5 | 312 | 241 |
| Dendy | 15 / 5 | 312 | 291 |

The final scanline is pre-render. The NTSC 2C02 skips a clock on rendered odd frames; VS RGB PPUs keep all 89,342 clocks on both frame parities. PAL selects its own APU periods and frame-counter events; Dendy keeps the NTSC APU periods at its clock rate. The application paces output from elapsed emulated CPU clocks and carries fractional host delays between frames.

`cpu_step()` includes device clocks and any DMA cycles it encounters. Its return value is elapsed CPU time, not an instruction-table estimate. The application and diagnostic runner must not clock the PPU again with that value.

OAM DMA begins when a CPU read can be halted. PAL DMA starts are restricted to opcode-fetch cycles, including the initial interrupt-sequence fetch. DMC requests go through the same bus scheduler. The scheduler handles alignment, DMC priority, OAM data puts, and canceled requests. Initial DMC divider phase keeps its output clocks aligned with CPU get cycles.

The CPU keeps separate internal and external data-bus latches. `$4015` updates the internal latch, including its floating bit 5, while DMA conflicts can drive a different byte on the external bus. Undocumented masked stores respond to a DMA stall during their indexed dummy read. Controller strobe writes commit on the APU output boundary, so consecutive writes can replace a pending value.

Reading `$4015` schedules the frame-IRQ flag clear for the next get cycle. The flag can remain visible in a consecutive read on the intervening put cycle. DMC cancellation distinguishes requests that have not halted the CPU from transfers already in progress; the one-byte reload case includes the early CPU variant's one-cycle aborted DMA.

PPU fetches put an address on the external bus on one clock and read the data on the next. The low address byte stays latched while the high address bits can change. This models hybrid addresses and overlapping address-latch/read strobes. Cartridge A12 notifications follow address changes; the mapper sees one read per data phase. The final nametable fetches begin at dots 337 and 339 and read at 338 and 340. An odd-frame skip omits the last data phase. The fetched tile also supplies the address driven at the next scanline's dot zero.

The second `$2006` write loads the current address after three PPU clocks. `$2007` transfers complete after five clocks and increment the address on the next clock. Reads within the six-clock recovery interval return the I/O latch without starting another transfer. These operations use the physical PPU bus address, including rendering collisions and cartridge address notifications. Sprite X counters keep counting while rendering is disabled, while the pattern shifters hold their data.

Secondary OAM keeps its address and increment-freeze state across rendering changes. Disabling rendering records the row affected by the compatibility OAM corruption path; the copy occurs on the next active rendering clock. `--ppu-oam-row-corruption` enables the deterministic NTSC worst-case row-copy approximation for alignment-dependent `$2003` writes and rendering transitions. `--ppu-revision 2c02-pre-e|2c02e-plus` selects whether the opt-in model includes the later 2C02 pre-render row-copy behavior. PAL excludes these row copies. The default keeps the established compatibility path. Sprite fetches walk the actual secondary-OAM address, including its dot-321 wrap. The CPU-facing OAM output latch follows the internal OAM bus one PPU clock later. `$2004` samples that output at the end of a CPU read. `$2002` latches vblank at the start of the read and sprite flags at the end. Sprite-zero and overflow conditions reach their status outputs on the following PPU clock; pre-render dot 1 clears both the flags and any pending output.

`--ppu-oam-decay` enables a deterministic primary-OAM decay approximation. Each eight-byte row records its last refresh CPU cycle. A primary-OAM read within 4500 CPU cycles refreshes that row; after the threshold, the row is replaced with its OAM addresses, with nonexistent attribute bits cleared. CPU writes, OAM DMA writes, sprite-evaluation reads, blanking, and PAL's late-vblank OAM refresh update the same row timestamps. Soft reset preserves OAM bytes and clears the timestamps, matching the selected model's reset behavior. The default leaves decay disabled.

Power-on and soft reset are distinct operations. The default startup alignment lets the PPU run one clock before the CPU begins its seven reset bus cycles. `--startup-phase CPU:PPU` selects another legal regional alignment; `--startup-seed SEED` generates repeatable choices on power-on. The applied CPU offset, PPU divider remainder, and supplied seed are recorded at startup. These options change divider alignment, not initial RAM contents. The canonical trace, pinned diagnostics, and AccuracyCoin gates use the unchanged default alignment.

`--ppu-startup-restriction` models the initial interval where writes to `$2000`, `$2001`, `$2005`, and `$2006` charge the PPU I/O latch but do not change their protected state. The restriction begins on both power-on and soft reset and ends when the PPU enters the next pre-render scanline, so the regional frame length determines the interval. The default keeps the established unrestricted startup behavior.

CPU reset reads the current PC twice, reads three stack locations while decrementing SP, and reloads PC from the reset vector. Soft reset preserves A, X, Y, CPU RAM, and the running divider phase; it updates the interrupt, unused, and break status bits. The frontend separately resets the PPU and APU. PPU palette/nametable/OAM memory and mapper state survive that operation, and PPU bus timestamps remain monotonic. [Architecture](architecture.md#power-on-and-reset) describes the reset entry points.

MMC5 detects scanline boundaries from repeated nametable reads and leaves the frame state after three CPU clocks without a PPU read. Address-only notifications do not count as reads. Extended attributes consume the next three physical reads after a qualifying nametable fetch, including reads that cross between the nametable and CHR ports. The mapper also supplies vertical-split tile data, separate CHR banking for large sprites, ExRAM permissions and persistence, and expansion pulse/PCM output. NMI-vector reads clear its frame IRQ state. PCM status follows the documented MMC5A revision, including its revision bit.

MMC5 pulse length reloads and halt changes commit at the end of the CPU clock. A simultaneous frame-counter decrement of a nonzero length takes precedence over its pending reload. Trainer initialization prefers a volatile PRG chip of at least 8 KiB, otherwise a persistent chip of at least 8 KiB, and writes at chip offset `$1000`. It does not depend on the initially mapped CPU bank. Save data then takes precedence where the overlays overlap.

MMC5 split and extended-attribute fetches mask the physical CHR address bits. This differs from ordinary bank selection on irregular ROM sizes. The extended-geometry regressions cover 8, 12, 20, and 28 KiB CHR images, CPU register writes, split and extended fetches, read-only ROM, failed-load preservation, and actual rendered pixels on a 12 KiB image.

## Family BASIC keyboard and tape

`--console famicom --expansion family-basic` connects the 72-key matrix and data recorder. Writes to `$4016` select a row and half; `$4017` returns the four active-low key bits. The tenth scan row is empty. Controller serial data and the microphone keep their own port bits.

The keyboard uses letter, number, arrow, modifier, and F1 through F8 keys. On a US keyboard, grave selects `@`, apostrophe selects `:`, equals selects `^`, backslash selects yen, and F9 selects underscore. Left Alt is GRPH, Right Alt is Kana, Home is CLEAR/HOME, and F12 is STOP. Backspace also acts as DELETE. Keyboard events go to BASIC while this device is connected, including letters and function keys otherwise used by emulator shortcuts.

Choose a raw tape with `--tape-play program.tap`, or a recording destination with `--tape-record program.tap`. Press F10 when BASIC is ready to read or write the tape; F11 stops playback or saves the recording. A recording also saves when the application exits. Tape files pack samples least-significant bit first at one sample per 88 emulated CPU cycles. Recording omits a partial final byte. This is a digital signal model; it does not decode WAV audio or model analog cassette noise.

The regression cases scan distinct simultaneous key patterns through all ten rows, run CPU loads across tape transitions, preserve controller and microphone signals, and round-trip a recorded byte through a file. A failed tape load leaves the current tape intact, and save replacement occurs only after the temporary file has been written and closed.

## Cartridge RAM and device regressions

The mapper tests distinguish mapped RAM, open bus, and write-register interception. Irem 77/97 and the applicable Jaleco boards execute CPU stores and loads under legacy RAM defaults and explicit NES 2.0 layouts. Jaleco 87/101/140 and Sunsoft 184 also load nonzero trainer or save data: a CPU write in their RAM-read window must change bank selection while a later read still returns the stored byte. Mapper 96 covers legacy RAM, CHR latch edges, reset, and battery persistence.

The discrete-board cases also run CPU instructions against mappers 79, 94, 113, 144, 146, and 180. They check ignored register addresses, fixed and switched PRG windows, all eight mapper 180 bank bits in a 4 MiB image, NINA CHR RAM and nametable routing, and mapper 144's ROM-driven D0. Mapper 11 has a separate check for PRG banks above 128 KiB. Loader cases distinguish legacy RAM from explicit zero, 2 KiB, and 8 KiB layouts, preserve trainer bytes, retain the active cartridge after failed loads, and round-trip PRG/CHR save memory.

ROM layout checks exercise complete-page bank wrapping, small images with unmapped addresses, and submapper fields ignored by these discrete boards. Trainer/save regressions load cartridge files with conflicting trainer and save bytes, including missing and short saves. Saved bytes must replace overlapping trainer data while bytes outside the saved portion retain their initialization.

These cases use production loading and CPU bus paths in [mapper_accuracy.c](../src/tests/mapper_accuracy.c). The [input regressions](../src/tests/input_accuracy.c) include the combined mapper 96/tablet fixture and separate Turbo File/BattleBox protocols and persistence. [EPSM regressions](../src/tests/epsm_accuracy.c) cover CPU writes, delayed output-pin transitions, timer IRQs, regional clocks, firmware validation, reset, and stereo output.

The [mixed-CHR MMC3 cases](../src/tests/board_mmc3_mixed_chr_accuracy.c) distinguish ROM from writable RAM across all six board-specific selection ranges, including RAM-only images, explicit zero RAM, smaller pages, larger allocations, IRQs, and persistent CHR storage. [Taito cases](../src/tests/board_taito_accuracy.c) check the physical work/save source, permission registers, mapper 552's reversed PRG bits, short saves, and bytes outside visible RAM windows. [NINA/FME-7 cases](../src/tests/board_nina_fme7_accuracy.c) exercise register writes over RAM, absent-chip selection, bank retention, IRQ timing, and audio state through CPU instructions.

[Native CHR tests](../src/tests/native_chr_capacity_accuracy.c) cover reduced Action 53 and Oeka Kids ROM slots, PPUADDR latch transitions, and unreachable storage. [Flash and CHR-source tests](../src/tests/native_flash_geometry_accuracy.c) cover protected CNROM writes, startup visibility, readonly ROM, nametable storage, and CHR-NVRAM reloads. [UNROM 512/GTROM PRG-RAM tests](../src/tests/mapper30_111_prg_ram_accuracy.c) separately program flash and write RAM, flush both save files, reload them, and shorten the RAM save to verify that trainer and flash bytes retain their ownership. Failed replacement cases check the still-active cartridge's data and bank state.

The [CPU diagnostic tests](../src/tests/cpu_accuracy.c) execute `$4015` disable and `$4018/$4019` read instructions before and after pulse/noise timer edges. This checks the output latches consumed by both diagnostic reads and the mixer. A correct packed value from manually seeded state alone would not cover that timing transition.

## Regression coverage

| Area | Checks |
| --- | --- |
| CPU | Official and undocumented instruction behavior, dummy reads/writes, page wraps, status flags, JAM, interrupt polling, vector selection, and power/reset bus sequences |
| DMA and controllers | OAM transfer parity and wrapping, DMC requests and cancellation, regional DMA start rules, overlapping transfers, strobe/shift behavior, and controller bus interactions |
| APU | Regional noise/DMC periods and frame events, LFSR taps, sweep targets/reloads, channel DAC latches, diagnostic reads across channel-disable transitions, delayed writes, length-counter collisions, DMC wrap/loop/IRQ behavior, and reset state |
| PPU | Palette mirrors, register-transfer delays and collisions, open bus, OAM access/refresh, sprite overflow/priority/shifter timing, sprite-zero hit, scrolling, regional frame timing, reset preservation and A12 events |
| Cartridge | Supported bank and nametable wiring, bus conflicts, startup mapping, RAM permissions, IRQ boundaries and CPU delivery, declared memory sizes, malformed images, failed-load preservation, EEPROM transactions, flash commands, and save round trips |
| Expansion audio | MMC5 pulse/PCM, VRC6 pulse/saw, VRC7 FM, N163 wavetable, Sunsoft 5B tone/noise/envelopes, and EPSM FM/SSG/ADPCM output; register access, delayed bus edges, timer IRQs, regional clocks, reset and stereo mixing |
| Disk system | BIOS/RAM mapping, timer and transfer IRQs, media insertion and side changes, transfer/CRC timing, disk persistence, failed-save preservation, reset, and wavetable/modulation audio |
| VS System | Direct and extended console descriptors, RGB frame lengths, 2C04 colors, 2C05 registers/status, serial Zapper reports, cabinet inputs and protection reads, mapper 99 banks and declared RAM, dual CPU/PPU/APU execution, independent DMA, shared RAM/IRQs, reset, both video outputs, and secondary audio through the production callback |
| Input devices | Console wiring, multiplayer adapters, paddles and mats, beam-aware light guns, Family BASIC keyboard/tape, Subor keyboard/mouse, Hori Track reports, Hyper Shot devices, Party Tap, Pachinko, Boxing and Mahjong switches, Oeka Kids tablet reports with a loaded mapper 96 cartridge, and separate Datach/Barcode Battler timing |
| Expansion storage | Turbo File bit positions and wrap, BattleBox command/word framing, write protection and erase, complete save round trips, and preservation after failed saves |

The canonical `nestest` comparison checks 8,991 PC/register/status/stack/cycle states and the diagnostic's result bytes. Its trace contains 225 distinct opcode values. The unit cases execute the remaining opcode values separately; neither number means that every possible operand or interrupt alignment has been exhausted. XAA/ANE and other unstable opcodes use a fixed silicon model.

`scripts/run-diagnostics.py` checks the pinned test collection at commit `95d8f621ae55cee0d09b91519a8989ae0e64753b`. It runs these groups:

| Group | ROMs |
| --- | ---: |
| Instruction tests v5 | 16 |
| Additional instruction bus/wrap tests | 4 |
| Instruction timing | 2 |
| CPU interrupt timing v2 | 5 |
| APU register and timing tests | 8 |
| PPU vblank/NMI timing | 10 |
| Overlapping OAM and DMC DMA | 2 |
| PPU open bus/read buffer, OAM reads, CPU dummy writes | 5 |
| CPU power/reset | 2 |
| APU power/reset | 6 |
| MMC3 with explicit RAM setup | 5 |
| Legacy PAL APU with explicit PAL timing | 10 |
| Sprite-zero hit | 11 |
| Sprite overflow | 5 |

The script checks each collection's expected file count and fails if files are missing. It reports four groups: 60 ordinary diagnostic ROMs, five MMC3 ROMs, ten PAL APU ROMs, and 16 sprite ROMs. A final line of `Diagnostic ROMs: 16 passed` describes only the last group; the complete run must also pass the CPU trace and the preceding groups.

The runner waits for a final result and returns a nonzero exit code for failure, timeout, or a missing result protocol. It honors `$6000=$81` reset requests after at least 100 milliseconds of emulated time, once per request, with a maximum of sixteen resets. The total frame limit still applies. A 7,200-frame limit accommodates the read-buffer test, which takes more than 1,200 frames. The other three diagnostic groups use 1,200 frames per ROM.

The separate `--accuracycoin` mode runs the 144-test cartridge at commit `9bc42d1e3acbeeaea215b1011d58f4ce72a8a49e`. Its ROM has SHA-256 `7e25ac08d2e7ed14c9b1f16bd853148fef09a824452164f8e0d69fd2bd96176c`. The runner reads the cartridge's test descriptors, presses Start through the controller, and waits for the complete result screen. It checks every stored result against the cartridge's final pass tally and requires all 144 tests to pass. Success codes for documented hardware variants count as passes; skipped tests do not. The optional PPM output contains the rendered framebuffer.

The recorded baseline passes all 144 AccuracyCoin tests with zero skipped or unfinished results, along with the 91-ROM collection and canonical CPU trace. The internal suite prints its group and assertion counts for each hardware area. These focused checks cover devices that AccuracyCoin does not exercise. [Per-issue checkpoints](accuracy-checkpoints.md) record the tested commits. [The result image](../img/coin.png) is the rendered output from the production core.

## Reproducing checks

Prepare the pinned checkouts described in [development and testing](development.md) before running these commands. On Linux:

```sh
make clean
make CC=gcc CXX=g++ CFLAGS='-std=c11 -Wall -Wextra -Werror -O2' all test
python3 scripts/run-diagnostics.py build/accuracy-tests build/diagnostic-roms
echo '7e25ac08d2e7ed14c9b1f16bd853148fef09a824452164f8e0d69fd2bd96176c  build/accuracycoin/AccuracyCoin.nes' | sha256sum --check --strict
build/accuracy-tests --accuracycoin 12000 build/accuracycoin/AccuracyCoin.nes build/accuracycoin.ppm
```

For memory and undefined-behavior checks:

```sh
make clean
export ASAN_OPTIONS=detect_leaks=1:halt_on_error=1
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
make CC=clang CXX=clang++ CFLAGS='-std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer' all test
python3 scripts/run-diagnostics.py build/accuracy-tests build/diagnostic-roms
build/accuracy-tests --accuracycoin 12000 build/accuracycoin/AccuracyCoin.nes
```

On Windows, the PowerShell build script accepts `-Sanitize` and writes the test executable to `build/windows-sanitized/accuracy-tests.exe`. Windows AddressSanitizer does not provide the Linux leak check. The Linux workflow enables leak detection explicitly.

The workflow pins both test collections and its checkout action. It runs on pushes and pull requests without a documentation-path exclusion. Each GCC and Clang job builds the application and test executable, runs the hardware suite, checks the trace and all 91 ROMs, verifies the AccuracyCoin ROM hash, and runs all 144 AccuracyCoin tests. Documentation commits therefore receive the same CI checks as source commits.

## Interpreting other ROMs

Some older ROMs report only on screen. Others write a result to cartridge RAM without first enabling that RAM. An absent `$6000` signature in either case is not evidence of a hardware-test pass or failure. Use a rendered frame or inspect the test's reporting code.

After the 60 ordinary runs, the script executes `mmc3_test_2` tests 1 through 5 with `--mmc3-rom`. These ROMs assume writable result RAM. That mode prints an explicit setup message and writes `$A001=$80` once after reset through the cartridge bus. It accepts only mapper 4/submapper 0. The emulator's normal MMC3 RAM-protection default remains disabled and has separate regression coverage. These five results must be reported with their setup condition.

The ten `pal_apu_tests` images have legacy headers without the PAL flag and report their result at `$F8`. `--legacy-pal-rom` selects PAL timing explicitly. The runner accepts the result only after the CPU reaches a stable terminal `JMP` loop with interrupts disabled; a temporary subtest value at `$F8` cannot pass the test. Code 1 means success in that older convention. This mode accepts only NROM images and does not change normal ROM-header handling.

The eleven `sprite_hit_tests_2005.10.05` and five `sprite_overflow_tests` images use the same `$F8` result convention. `--legacy-rom` applies that result check while retaining the ROM header's timing region. These suites cover sprite/background alignment, flips, clipping, 8x16 sprites, hit timing, overflow timing, and the overflow circuit's diagonal OAM scan.

The older `5.Emulator` overflow test depends on startup alignment. Its rendering-disable sequence can seed an OAM row copy that replaces two of the nine sprites it later expects to overflow. It passes under the fixed startup alignment described above. Both the OAM corruption model and the CPU's page-crossing cycle penalties remain active during the diagnostic runs.

The old test labeled `6-MMC6` expects an alternative MMC3 IRQ-counter revision; the later collection calls it `6-MMC3_alt`. It is not the acceptance test for NES 2.0 mapper 4/submapper 1. MMC6 here uses the Sharp counter behavior and has separate tests for its 1 KiB RAM and per-half permissions. See the [hardware retest discussion](https://forums.nesdev.org/viewtopic.php?t=6467) and [NES 2.0 board definitions](https://www.nesdev.org/wiki/NES_2.0_submappers).

## Remaining limits

The [hardware reference](hardware.md) lists the supported devices. Unsupported console types, mapper numbers, submappers, RAM geometries, and UNIF board names are rejected. Unlisted peripherals and audio chips, additional cartridge families, and a distinct RP2C03G palette remain outside the implemented system. Direct VS PPU codes 1 and 13 through 15 use a reported 2C03 fallback. Game-database entries can identify supported VS hardware from legacy images, but unsupported VS hardware and controller metadata are still rejected. The SSS-NROM-256 path covers the FamicomBox menu cartridge, not the complete kiosk. MMC5 coverage does not include its auxiliary I/O or `$5209/$520A` timer registers, every undocumented behavior, or every board revision.

OAM row corruption and decay are deterministic approximations when their opt-in profiles are selected. The `$2003` corruption path uses a worst-case CPU-bus alignment approximation. The 4500-cycle decay threshold and replacement values provide repeatable behavior for testing, but physical OAM charge loss varies with chip, temperature, and refresh history. Startup alignment can be selected or generated from a seed, and the register restriction ends at the next pre-render boundary. The current checks do not cover analog output effects or every DMA/register interleaving. Register-delay tests cover defined collision cases, not every possible interleaving. A new failure should be reduced to its bus operations and timing, then added as a regression. Avoid per-ROM behavior switches or expected-output substitutions in the core.
