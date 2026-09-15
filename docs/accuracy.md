# NES accuracy checks

The hardware targets use NTSC, PAL, or Dendy timing with standard controllers. The tests below exercise the production core. They do not establish compatibility with every cartridge or hardware revision.

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

PPU pattern reads occur in the background and sprite fetch slots. Sprite pixels use the previously fetched bytes. Nametable reads at dots 337 and 339 also supply the address driven at the following scanline's dot zero; that address change matters to cartridge IRQ circuitry even though it does not read another CHR byte.

The second `$2006` write loads the current address after three PPU clocks. `$2007` transfers complete after five clocks and increment the address on the next clock. Reads within the six-clock recovery interval return the I/O latch without starting another transfer. These operations use the physical PPU bus address, including rendering collisions and cartridge address notifications. Sprite X counters keep counting while rendering is disabled, while the pattern shifters hold their data.

Power-on and soft reset are distinct operations. CPU reset reads the current PC twice, reads three stack locations while decrementing SP, and reads the reset vector. Soft reset keeps CPU registers and RAM, PPU palette/nametable/OAM memory, and mapper state. PPU bus timestamps remain monotonic through reset. Initial memory contents and CPU/PPU alignment are deterministic; they do not simulate random power-up state across console revisions.

MMC5 detects scanline boundaries from repeated nametable reads and leaves the frame state after three CPU clocks without a PPU read. Address-only notifications do not count as reads. The mapper supplies extended attributes, vertical-split tile data, separate CHR banking for large sprites, ExRAM permissions and persistence, and expansion pulse/PCM output. NMI-vector reads clear its frame IRQ state. PCM status follows the documented MMC5A revision, including its revision bit.

MMC5 pulse length reloads and halt changes commit at the end of the CPU clock. A simultaneous frame-counter decrement of a nonzero length takes precedence over its pending reload. Trainer initialization uses the RAM bank mapped at CPU `$7000-$71FF`, including the battery-backed socket on a two-socket MMC5 board.

## Regression coverage

| Area | Checks |
| --- | --- |
| CPU | Official and undocumented instruction behavior, dummy reads/writes, page wraps, status flags, JAM, interrupt polling, vector selection, and power/reset bus sequences |
| DMA and controllers | OAM transfer parity and wrapping, DMC requests and cancellation, regional DMA start rules, overlapping transfers, strobe/shift behavior, and controller bus interactions |
| APU | Regional noise/DMC periods and frame events, LFSR taps, sweep targets/reloads, held triangle DAC, delayed writes, length-counter collisions, DMC wrap/loop/IRQ behavior, and reset state |
| PPU | Palette mirrors, register-transfer delays and collisions, open bus, OAM access/refresh, sprite overflow/priority/shifter timing, sprite-zero hit, scrolling, regional frame timing, reset preservation and A12 events |
| Cartridge | Supported banking/mirroring modes, MMC1 serial writes, MMC2/MMC4 latches, MMC3/MMC6 IRQs/protection, MMC5 rendering/audio/IRQs, declared RAM sizes, malformed images, regional headers, failed-load preservation and save round trips |

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

## Reproducing checks

On Linux:

```sh
make clean
make CFLAGS='-std=c11 -Wall -Wextra -Werror -O2' all test
python3 scripts/run-diagnostics.py build/accuracy-tests build/diagnostic-roms
```

For memory and undefined-behavior checks:

```sh
make clean
make CC=clang CFLAGS='-std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer' all test
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 python3 scripts/run-diagnostics.py build/accuracy-tests build/diagnostic-roms
```

On Windows, the PowerShell build script accepts `-Sanitize` and writes the test executable to `build/windows-sanitized/accuracy-tests.exe`. Windows AddressSanitizer does not provide the Linux leak check. The Linux workflow enables leak detection explicitly.

The diagnostic ROMs are a separate checkout; the build does not download them. The workflow pins both the test collection and its checkout action. See the README for setup commands.

## Interpreting other ROMs

Some older ROMs report only on screen. Others write a result to cartridge RAM without first enabling that RAM. An absent `$6000` signature in either case is not evidence of a hardware-test pass or failure. Use a rendered frame or inspect the test's reporting code.

After the 60 ordinary runs, the script executes `mmc3_test_2` tests 1 through 5 with `--mmc3-rom`. These ROMs assume writable result RAM. That mode prints an explicit setup message and writes `$A001=$80` once after reset through the cartridge bus. It accepts only mapper 4/submapper 0. The emulator's normal MMC3 RAM-protection default remains disabled and has separate regression coverage. These five results must be reported with their setup condition.

The ten `pal_apu_tests` images have legacy headers without the PAL flag and report their result at `$F8`. `--legacy-pal-rom` selects PAL timing explicitly. The runner accepts the result only after the CPU reaches a stable terminal `JMP` loop with interrupts disabled; a temporary subtest value at `$F8` cannot pass the test. Code 1 means success in that older convention. This mode accepts only NROM images and does not change normal ROM-header handling.

The eleven `sprite_hit_tests_2005.10.05` and five `sprite_overflow_tests` images use the same `$F8` result convention. `--legacy-rom` applies that result check while retaining the ROM header's timing region. These suites cover sprite/background alignment, flips, clipping, 8x16 sprites, hit timing, overflow timing, and the overflow circuit's diagonal OAM scan.

The old test labeled `6-MMC6` expects an alternative MMC3 IRQ-counter revision; the later collection calls it `6-MMC3_alt`. It is not the acceptance test for NES 2.0 mapper 4/submapper 1. MMC6 here uses the Sharp counter behavior and has separate tests for its 1 KiB RAM and per-half permissions. See the [hardware retest discussion](https://forums.nesdev.org/viewtopic.php?t=6467) and [NES 2.0 board definitions](https://www.nesdev.org/wiki/NES_2.0_submappers).

## Remaining limits

The supported mapper list is explicit in the README. Unsupported console types, mapper numbers, submappers, and RAM geometries are rejected. VS hardware, disk-system hardware, expansion peripherals, other expansion-audio chips, and additional cartridge families remain outside the implemented system. MMC5 coverage does not include its auxiliary I/O or `$5209/$520A` timer registers, every undocumented behavior, or every board revision.

The current checks do not cover every OAM corruption/decay case, reset/power-on alignment, analog output effect, or DMA interaction. Register-delay tests cover defined collision cases, not every possible interleaving. A new failure should be reduced to its bus operations and timing, then added as a regression. Avoid per-ROM behavior switches or expected-output substitutions in the core.
