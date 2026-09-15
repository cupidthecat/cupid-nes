# NES accuracy checks

The hardware target is an NTSC 2A03 CPU/APU and 2C02 PPU with standard controllers. The tests below exercise the production core. They do not establish compatibility with every cartridge or hardware revision.

## Timing model

Each CPU read or write advances the PPU, APU, and cartridge. Reads and writes place the bus operation at different phases of the twelve-master-clock CPU cycle. The PPU advances every four master clocks. Interrupt lines are sampled at the CPU cycle boundaries used by instruction polling.

`cpu_step()` includes device clocks and any DMA cycles it encounters. Its return value is elapsed CPU time, not an instruction-table estimate. The application and diagnostic runner must not clock the PPU again with that value.

OAM DMA begins when a CPU read can be halted. DMC requests go through the same bus scheduler. The scheduler handles alignment, DMC priority, OAM data puts, and canceled requests. Initial DMC divider phase keeps its output clocks aligned with CPU get cycles.

PPU pattern reads occur in the background and sprite fetch slots. Sprite pixels use the previously fetched bytes. Nametable reads at dots 337 and 339 also supply the address driven at the following scanline's dot zero; that address change matters to cartridge IRQ circuitry even though it does not read another CHR byte.

## Regression coverage

| Area | Checks |
| --- | --- |
| CPU | Official and undocumented instruction behavior, dummy reads/writes, page wraps, status flags, JAM, interrupt polling and vector selection |
| DMA and controllers | OAM transfer parity and wrapping, DMC requests and cancellation, overlapping transfers, strobe/shift behavior, and controller bus interactions |
| APU | All noise and DMC rate values, LFSR taps, sweep targets/reloads, held triangle DAC, frame events, delayed writes, length-counter collisions, DMC wrap/loop/IRQ behavior |
| PPU | Palette mirrors, PPUDATA buffering, open bus, rendering address increments, OAM access, sprite overflow and priority, sprite-zero hit, background scrolling, fetch positions, odd-frame timing and A12 events |
| Cartridge | Supported banking/mirroring modes, MMC1 serial-write filtering, MMC2/MMC4 latches, MMC3/MMC6 protection and IRQ clocks, declared RAM sizes, malformed images, failed-load preservation and persistent-memory round trips |

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

The script fails if expected test files are missing. The ROM runner waits for the diagnostic's result and returns a nonzero exit code for failure, timeout, a requested reset, or a missing result protocol. A 7,200-frame limit accommodates the read-buffer test, which takes more than 1,200 frames.

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

After the 52 ordinary runs, the script executes `mmc3_test_2` tests 1 through 5 with `--mmc3-rom`. These ROMs assume writable result RAM. That mode prints an explicit setup message and writes `$A001=$80` once after reset through the cartridge bus. It accepts only mapper 4/submapper 0. The emulator's normal MMC3 RAM-protection default remains disabled and has separate regression coverage. These five results must be reported with their setup condition.

The old test labeled `6-MMC6` expects an alternative MMC3 IRQ-counter revision; the later collection calls it `6-MMC3_alt`. It is not the acceptance test for NES 2.0 mapper 4/submapper 1. MMC6 here uses the Sharp counter behavior and has separate tests for its 1 KiB RAM and per-half permissions. See the [hardware retest discussion](https://forums.nesdev.org/viewtopic.php?t=6467) and [NES 2.0 board definitions](https://www.nesdev.org/wiki/NES_2.0_submappers).

## Remaining limits

The supported mapper list is explicit in the README. Unsupported mapper numbers, submappers, and RAM geometries are rejected. MMC5 still lacks extended-attribute/vertical-split rendering and expansion audio. PAL/Dendy clocks, VS hardware, disk-system hardware, and expansion peripherals are outside the implemented system.

The current checks do not cover every PPU register-pipeline delay, OAM corruption/decay case, reset/power-on state, analog output effect, or DMA interaction. A new failure should be reduced to its bus operations and timing, then added as a regression. Avoid per-ROM behavior switches or expected-output substitutions in the core.
