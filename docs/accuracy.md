# NES accuracy checks

The hardware targets use NTSC, PAL, or Dendy timing with the cartridge and input devices listed in the README. Disk and VS systems use NTSC timing. The tests below exercise the production core. They do not establish compatibility with every cartridge or hardware revision.

## Timing model

Each CPU read or write advances the PPU, APU, and cartridge. Reads and writes place the bus operation at different phases of the CPU clock. Fractional PPU clocks carry across CPU accesses. Interrupt lines are sampled at the CPU cycle boundaries used by instruction polling.

| Region | Master clocks per CPU/PPU cycle | Scanlines | Vblank starts |
| --- | --- | ---: | ---: |
| NTSC | 12 / 4 | 262 | 241 |
| PAL | 16 / 5 | 312 | 241 |
| Dendy | 15 / 5 | 312 | 291 |

The final scanline is pre-render. Only NTSC rendering skips a clock on odd frames. PAL selects its own APU periods and frame-counter events; Dendy keeps the NTSC APU periods at its clock rate. The application paces output from elapsed emulated CPU clocks and carries fractional host delays between frames.

`cpu_step()` includes device clocks and any DMA cycles it encounters. Its return value is elapsed CPU time, not an instruction-table estimate. The application and diagnostic runner must not clock the PPU again with that value.

OAM DMA begins when a CPU read can be halted. PAL DMA starts are restricted to opcode-fetch cycles, including the initial interrupt-sequence fetch. DMC requests go through the same bus scheduler. The scheduler handles alignment, DMC priority, OAM data puts, and canceled requests. Initial DMC divider phase keeps its output clocks aligned with CPU get cycles.

The CPU keeps separate internal and external data-bus latches. `$4015` updates the internal latch, including its floating bit 5, while DMA conflicts can drive a different byte on the external bus. Undocumented masked stores respond to a DMA stall during their indexed dummy read. Controller strobe writes commit on the APU output boundary, so consecutive writes can replace a pending value.

Reading `$4015` schedules the frame-IRQ flag clear for the next get cycle. The flag can remain visible in a consecutive read on the intervening put cycle. DMC cancellation distinguishes requests that have not halted the CPU from transfers already in progress; the one-byte reload case includes the early CPU variant's one-cycle aborted DMA.

PPU fetches put an address on the external bus on one clock and read the data on the next. The low address byte stays latched while the high address bits can change. This models hybrid addresses and overlapping address-latch/read strobes. Cartridge A12 notifications follow address changes; the mapper sees one read per data phase. The final nametable fetches begin at dots 337 and 339 and read at 338 and 340. An odd-frame skip omits the last data phase. The fetched tile also supplies the address driven at the next scanline's dot zero.

The second `$2006` write loads the current address after three PPU clocks. `$2007` transfers complete after five clocks and increment the address on the next clock. Reads within the six-clock recovery interval return the I/O latch without starting another transfer. These operations use the physical PPU bus address, including rendering collisions and cartridge address notifications. Sprite X counters keep counting while rendering is disabled, while the pattern shifters hold their data.

Secondary OAM keeps its address and increment-freeze state across rendering changes. Disabling rendering records the row affected by the compatibility OAM corruption path; the copy occurs on the next active rendering clock. `--ppu-oam-row-corruption` enables the deterministic NTSC worst-case row-copy approximation for alignment-dependent `$2003` writes and rendering transitions. `--ppu-revision 2c02-pre-e|2c02e-plus` selects whether the opt-in model includes the later 2C02 pre-render row-copy behavior. PAL excludes these row copies. The default keeps the established compatibility path. Sprite fetches walk the actual secondary-OAM address, including its dot-321 wrap. The CPU-facing OAM output latch follows the internal OAM bus one PPU clock later. `$2004` samples that output at the end of a CPU read. `$2002` latches vblank at the start of the read and sprite flags at the end. Sprite-zero and overflow conditions reach their status outputs on the following PPU clock; pre-render dot 1 clears both the flags and any pending output.

`--ppu-oam-decay` enables a deterministic primary-OAM decay approximation. Each eight-byte row records its last refresh CPU cycle. A primary-OAM read within 4500 CPU cycles refreshes that row; after the threshold, the row is replaced with its OAM addresses, with nonexistent attribute bits cleared. CPU writes, OAM DMA writes, sprite-evaluation reads, blanking, and PAL's late-vblank OAM refresh update the same row timestamps. Soft reset preserves OAM bytes and clears the timestamps, matching the selected model's reset behavior. The default leaves decay disabled.

Power-on and soft reset are distinct operations. The fixed startup alignment lets the PPU run one clock before the CPU begins its seven reset bus cycles. `--ppu-startup-restriction` models the initial interval where writes to `$2000`, `$2001`, `$2005`, and `$2006` charge the PPU I/O latch but do not change their protected state. The restriction begins on both power-on and soft reset and ends when the PPU enters the next pre-render scanline, so the regional frame length determines the interval. The default keeps the established unrestricted startup behavior. CPU reset reads the current PC twice, reads three stack locations while decrementing SP, and reads the reset vector. Soft reset preserves the running divider phase, CPU registers and RAM, PPU palette/nametable/OAM memory, and mapper state. PPU bus timestamps remain monotonic through reset. Initial memory contents and alignment are deterministic; they do not simulate random power-up state across console revisions.

MMC5 detects scanline boundaries from repeated nametable reads and leaves the frame state after three CPU clocks without a PPU read. Address-only notifications do not count as reads. Extended attributes consume the next three physical reads after a qualifying nametable fetch, including reads that cross between the nametable and CHR ports. The mapper also supplies vertical-split tile data, separate CHR banking for large sprites, ExRAM permissions and persistence, and expansion pulse/PCM output. NMI-vector reads clear its frame IRQ state. PCM status follows the documented MMC5A revision, including its revision bit.

MMC5 pulse length reloads and halt changes commit at the end of the CPU clock. A simultaneous frame-counter decrement of a nonzero length takes precedence over its pending reload. Trainer initialization uses the RAM bank mapped at CPU `$7000-$71FF`, including the battery-backed socket on a two-socket MMC5 board.

## Family BASIC keyboard and tape

`--console famicom --expansion family-basic` connects the 72-key matrix and data recorder. Writes to `$4016` select a row and half; `$4017` returns the four active-low key bits. The tenth scan row is empty. Controller serial data and the microphone keep their own port bits.

The keyboard uses letter, number, arrow, modifier, and F1 through F8 keys. On a US keyboard, grave selects `@`, apostrophe selects `:`, equals selects `^`, backslash selects yen, and F9 selects underscore. Left Alt is GRPH, Right Alt is Kana, Home is CLEAR/HOME, and F12 is STOP. Backspace also acts as DELETE. Keyboard events go to BASIC while this device is connected, including letters and function keys otherwise used by emulator shortcuts.

Choose a raw tape with `--tape-play program.tap`, or a recording destination with `--tape-record program.tap`. Press F10 when BASIC is ready to read or write the tape; F11 stops playback or saves the recording. A recording also saves when the application exits. Tape files pack samples least-significant bit first at one sample per 88 emulated CPU cycles. Recording omits a partial final byte. This is a digital signal model; it does not decode WAV audio or model analog cassette noise.

The regression cases scan distinct simultaneous key patterns through all ten rows, run CPU loads across tape transitions, preserve controller and microphone signals, and round-trip a recorded byte through a file. A failed tape load leaves the current tape intact, and save replacement occurs only after the temporary file has been written and closed.

## Regression coverage

| Area | Checks |
| --- | --- |
| CPU | Official and undocumented instruction behavior, dummy reads/writes, page wraps, status flags, JAM, interrupt polling, vector selection, and power/reset bus sequences |
| DMA and controllers | OAM transfer parity and wrapping, DMC requests and cancellation, regional DMA start rules, overlapping transfers, strobe/shift behavior, and controller bus interactions |
| APU | Regional noise/DMC periods and frame events, LFSR taps, sweep targets/reloads, held triangle DAC, delayed writes, length-counter collisions, DMC wrap/loop/IRQ behavior, and reset state |
| PPU | Palette mirrors, register-transfer delays and collisions, open bus, OAM access/refresh, sprite overflow/priority/shifter timing, sprite-zero hit, scrolling, regional frame timing, reset preservation and A12 events |
| Cartridge | Supported bank and nametable wiring, bus conflicts, startup mapping, RAM permissions, IRQ boundaries and CPU delivery, declared memory sizes, malformed images, failed-load preservation, EEPROM transactions, flash commands, and save round trips |
| Expansion audio | MMC5 pulse/PCM, VRC6 pulse/saw, VRC7 FM, N163 wavetable, and Sunsoft 5B tone/noise/envelope output, including register access, reset, mute, and timing cases |
| Disk system | BIOS/RAM mapping, timer and transfer IRQs, media insertion and side changes, transfer/CRC timing, disk persistence, failed-save preservation, reset, and wavetable/modulation audio |
| VS System | Header validation, 2C04 colors, 2C05 registers/status, cabinet inputs and protection reads, mapper 99 banks and declared RAM, dual CPU/PPU/APU execution, independent DMA, shared RAM/IRQs, reset, both video outputs, and secondary audio through the production callback |
| Input devices | Console wiring, multiplayer adapters, Arkanoid serial reports, Power Pad and Family Trainer matrices, beam-aware Zapper reads, Family BASIC keyboard/tape signals, and Datach barcode timing |

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

The script runs 91 diagnostic ROMs and fails if expected files are missing. The runner waits for a final result and returns a nonzero exit code for failure, timeout, or a missing result protocol. It honors `$6000=$81` reset requests after at least 100 milliseconds of emulated time, once per request, with a maximum of sixteen resets. The total frame limit still applies. A 7,200-frame limit accommodates the read-buffer test, which takes more than 1,200 frames.

The separate `--accuracycoin` mode runs the 144-test cartridge at commit `9bc42d1e3acbeeaea215b1011d58f4ce72a8a49e`. Its ROM has SHA-256 `7e25ac08d2e7ed14c9b1f16bd853148fef09a824452164f8e0d69fd2bd96176c`. The runner reads the cartridge's test descriptors, presses Start through the controller, and waits for the complete result screen. It checks every stored result against the cartridge's final pass tally and requires all 144 tests to pass. Success codes for documented hardware variants count as passes; skipped tests do not. The optional PPM output contains the rendered framebuffer.

The current core passes all 144 AccuracyCoin tests with zero skipped or unfinished results, along with the 91-ROM collection and canonical CPU trace. The internal suite prints its group and assertion counts for each hardware area. These focused checks cover the newly added devices that AccuracyCoin does not exercise. [Per-issue checkpoints](accuracy-checkpoints.md) record the tested commits. `img/coin.png` is the rendered result from the production core.

## Reproducing checks

On Linux:

```sh
make clean
make CFLAGS='-std=c11 -Wall -Wextra -Werror -O2' all test
python3 scripts/run-diagnostics.py build/accuracy-tests build/diagnostic-roms
build/accuracy-tests --accuracycoin 12000 build/accuracycoin/AccuracyCoin.nes build/accuracycoin.ppm
```

For memory and undefined-behavior checks:

```sh
make clean
make CC=clang CFLAGS='-std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer' all test
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 python3 scripts/run-diagnostics.py build/accuracy-tests build/diagnostic-roms
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 build/accuracy-tests --accuracycoin 12000 build/accuracycoin/AccuracyCoin.nes
```

On Windows, the PowerShell build script accepts `-Sanitize` and writes the test executable to `build/windows-sanitized/accuracy-tests.exe`. Windows AddressSanitizer does not provide the Linux leak check. The Linux workflow enables leak detection explicitly.

The diagnostic ROMs are a separate checkout; the build does not download them. The workflow pins both the test collection and its checkout action. See the README for setup commands.

## Interpreting other ROMs

Some older ROMs report only on screen. Others write a result to cartridge RAM without first enabling that RAM. An absent `$6000` signature in either case is not evidence of a hardware-test pass or failure. Use a rendered frame or inspect the test's reporting code.

After the 60 ordinary runs, the script executes `mmc3_test_2` tests 1 through 5 with `--mmc3-rom`. These ROMs assume writable result RAM. That mode prints an explicit setup message and writes `$A001=$80` once after reset through the cartridge bus. It accepts only mapper 4/submapper 0. The emulator's normal MMC3 RAM-protection default remains disabled and has separate regression coverage. These five results must be reported with their setup condition.

The ten `pal_apu_tests` images have legacy headers without the PAL flag and report their result at `$F8`. `--legacy-pal-rom` selects PAL timing explicitly. The runner accepts the result only after the CPU reaches a stable terminal `JMP` loop with interrupts disabled; a temporary subtest value at `$F8` cannot pass the test. Code 1 means success in that older convention. This mode accepts only NROM images and does not change normal ROM-header handling.

The eleven `sprite_hit_tests_2005.10.05` and five `sprite_overflow_tests` images use the same `$F8` result convention. `--legacy-rom` applies that result check while retaining the ROM header's timing region. These suites cover sprite/background alignment, flips, clipping, 8x16 sprites, hit timing, overflow timing, and the overflow circuit's diagonal OAM scan.

The older `5.Emulator` overflow test depends on startup alignment. Its rendering-disable sequence can seed an OAM row copy that replaces two of the nine sprites it later expects to overflow. It passes under the fixed startup alignment described above. Both the OAM corruption model and the CPU's page-crossing cycle penalties remain active during the diagnostic runs.

The old test labeled `6-MMC6` expects an alternative MMC3 IRQ-counter revision; the later collection calls it `6-MMC3_alt`. It is not the acceptance test for NES 2.0 mapper 4/submapper 1. MMC6 here uses the Sharp counter behavior and has separate tests for its 1 KiB RAM and per-half permissions. See the [hardware retest discussion](https://forums.nesdev.org/viewtopic.php?t=6467) and [NES 2.0 board definitions](https://www.nesdev.org/wiki/NES_2.0_submappers).

## Remaining limits

The supported hardware list is explicit in the README. Unsupported console types, mapper numbers, submappers, and RAM geometries are rejected. Unlisted peripherals and audio chips, additional cartridge families, RP2C03G, and VS Zapper wiring remain outside the implemented system. VS games require valid header metadata; there is no per-game identification database. MMC5 coverage does not include its auxiliary I/O or `$5209/$520A` timer registers, every undocumented behavior, or every board revision.

OAM row corruption and decay are deterministic approximations when their opt-in profiles are selected. The `$2003` corruption path uses a worst-case CPU-bus alignment approximation. The 4500-cycle decay threshold and replacement values provide repeatable behavior for testing, but physical OAM charge loss varies with chip, temperature, and refresh history. The startup restriction uses Cupid's fixed power-on alignment and the next pre-render boundary; it does not simulate random power/reset phase variation between physical consoles. The current checks do not cover analog output effects or every DMA/register interleaving. Register-delay tests cover defined collision cases, not every possible interleaving. A new failure should be reduced to its bus operations and timing, then added as a regression. Avoid per-ROM behavior switches or expected-output substitutions in the core.
