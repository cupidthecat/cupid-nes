# Development and testing

[Documentation index](README.md) | [Hardware reference](hardware.md)

The [setup guide](getting-started.md) covers compiler and SDL2 installation. This guide uses the production hardware runner and the pinned external test collections. Run commands from the repository root and record the commit being tested:

```sh
git rev-parse HEAD
git status --short
```

Results from a modified working tree should be identified as such. A passing historical commit does not validate a later patch.

## Build and run hardware regressions

On Linux, use the strict GCC flags from CI:

```sh
make clean
make -j2 CC=gcc CXX=g++ CFLAGS='-std=c11 -Wall -Wextra -Werror -O2' all test
```

The application is `./cupid-nes` and the runner is `build/accuracy-tests`. Running the test executable without arguments executes CPU, APU, PPU, cartridge, media, input, VS, and EPSM groups and returns failure if any group fails. Media tests include disk transport, StudyBox, and NSF/NSFe execution and sound. The C core uses C11; cartridge board modules, the metadata database, the EPSM wrapper, and ymfm use C++17.

The Makefile defaults to `CC=gcc` and `CFLAGS='-std=c11 -Wall -Wextra -O2'`. When `CXX` still has GNU Make's built-in default, the Makefile selects `g++` for GCC and `clang++` when `CC` contains `clang`. An explicitly supplied `CXX` is kept. Unless `CXXFLAGS` is supplied separately, the Makefile derives it from `CFLAGS`, removes any C language-standard flag, and appends `-std=c++17`. `LDLIBS` defaults to `-lSDL2 -lm`. Use `make clean` before changing compiler families or flag sets because those settings are not tracked as object-file dependencies.

On Windows:

```powershell
.\scripts\test-windows.ps1 -SdlRoot 'C:\dependencies\SDL2-2.32.10'
```

The script defaults to `clang` and selects `clang++`; `-Compiler gcc` selects `g++`. Any other C compiler basename requires an explicit `-CxxCompiler`. A normal build uses C11 or C++17 with `-Wall -Wextra -Werror -O2`, plus `_CRT_SECURE_NO_WARNINGS`, `SDL_MAIN_HANDLED`, and the generated SDL include path. `-Sanitize` replaces `-O2` with `-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer`. The script passes the matching flags to the final C++ link as well.

Normal Windows output is under `build/windows`; sanitized output is under `build/windows-sanitized`. Each directory contains `cupid-nes.exe`, `accuracy-tests.exe`, `SDL2.dll`, copied SDL headers under `include/SDL2`, and intermediate objects under `objects`. The SDL library input remains `lib/x64/SDL2.lib` from the supplied VC development package.

Tests use the device code listed in [Makefile](../Makefile) and [test-windows.ps1](../scripts/test-windows.ps1). Add any new production or test source to both lists. The older `src/tests/cpu_test.c` harness is excluded because its writable-ROM assumptions do not match the cartridge bus.

`python3 scripts/check-region-cli.py ./cupid-nes` checks the production launch parser and loader with a synthetic PAL cartridge. On Windows, use `python scripts/check-region-cli.py build/windows/cupid-nes.exe`. Its eleven cases cover Auto, explicit regions, independent console wiring, repeated selectors and invalid values. The fixture adds `--barcode` to an NROM image. The ROM loads, then the option is rejected because it requires a Datach cartridge, before SDL starts. CI runs the check with both compiler configurations and treats sanitizer diagnostics as failures.

Archive tests include owned synthetic cartridges in ZIP, LZMA 7z, and solid LZMA2 7z files. They exercise Unicode paths, member selection, corrupt archives, unsupported compression, size limits, separate cartridge saves, and database lookup after patching. The fixed patch examples cover every BPS command, reversible UPS changes, IPS records and generation, invalid offsets, and checksum failures. Disk-overlay tests drive real disk registers through writes and reloads for headered and headerless FDS and QD images, including read-only sources and failed save replacement.

The fixtures are checked into `src/tests/media_fixtures.h`; running the suite does not require an archive program. To regenerate those owned fixtures, run `python scripts/generate-media-fixtures.py --seven-zip /path/to/7z`. The generator uses only its own temporary directory under `build`. The bundled decoders, licenses, and local portability change are described in [archive codecs](../src/third_party/archive-codecs.md).

## Prepare the pinned test collections

The external ROM collections are separate from the internal hardware suite. Clone them once into an ignored build directory:

```sh
git clone https://github.com/christopherpow/nes-test-roms.git build/diagnostic-roms
git -C build/diagnostic-roms checkout 95d8f621ae55cee0d09b91519a8989ae0e64753b
git clone https://github.com/100thCoin/AccuracyCoin.git build/accuracycoin
git -C build/accuracycoin checkout 9bc42d1e3acbeeaea215b1011d58f4ce72a8a49e
```

For existing directories, inspect their checkout and local changes before updating them. Do not clone over an existing directory or replace a modified collection. The required revisions are also in [the CI workflow](../.github/workflows/accuracy.yml). `run-diagnostics.py` checks the expected files and group counts, but it does not inspect the diagnostic repository's Git revision. The AccuracyCoin runner also does not verify the ROM hash, so keep the revision and hash checks as separate validation steps.

| Collection | Required revision |
| --- | --- |
| Diagnostic ROMs and canonical CPU trace | `95d8f621ae55cee0d09b91519a8989ae0e64753b` |
| AccuracyCoin | `9bc42d1e3acbeeaea215b1011d58f4ce72a8a49e` |

AccuracyCoin's `AccuracyCoin.nes` must have SHA-256:

```text
7e25ac08d2e7ed14c9b1f16bd853148fef09a824452164f8e0d69fd2bd96176c
```

Verify it on Linux:

```sh
echo '7e25ac08d2e7ed14c9b1f16bd853148fef09a824452164f8e0d69fd2bd96176c  build/accuracycoin/AccuracyCoin.nes' | sha256sum --check --strict
```

Or in PowerShell:

```powershell
$coinHash = (Get-FileHash -Algorithm SHA256 -LiteralPath '.\build\accuracycoin\AccuracyCoin.nes').Hash.ToLowerInvariant()
if ($coinHash -ne '7e25ac08d2e7ed14c9b1f16bd853148fef09a824452164f8e0d69fd2bd96176c') {
    throw 'AccuracyCoin ROM hash does not match the pinned image'
}
```

## Run the complete baseline

On Linux:

```sh
python3 scripts/run-diagnostics.py build/accuracy-tests build/diagnostic-roms
echo '7e25ac08d2e7ed14c9b1f16bd853148fef09a824452164f8e0d69fd2bd96176c  build/accuracycoin/AccuracyCoin.nes' | sha256sum --check --strict
build/accuracy-tests --accuracycoin 12000 build/accuracycoin/AccuracyCoin.nes build/accuracycoin.ppm
```

On Windows:

```powershell
python scripts/run-diagnostics.py build/windows/accuracy-tests.exe build/diagnostic-roms
if ($LASTEXITCODE -ne 0) { throw 'Diagnostic collection failed' }
$coinHash = (Get-FileHash -Algorithm SHA256 -LiteralPath '.\build\accuracycoin\AccuracyCoin.nes').Hash.ToLowerInvariant()
if ($coinHash -ne '7e25ac08d2e7ed14c9b1f16bd853148fef09a824452164f8e0d69fd2bd96176c') {
    throw 'AccuracyCoin ROM hash does not match the pinned image'
}
.\build\windows\accuracy-tests.exe --accuracycoin 12000 .\build\accuracycoin\AccuracyCoin.nes .\build\accuracycoin.ppm
if ($LASTEXITCODE -ne 0) { throw 'AccuracyCoin failed' }
```

The diagnostic script checks 8,991 canonical CPU trace states and runs 91 ROMs. Its groups are 60 ordinary diagnostics, five MMC3 diagnostics with explicit result-RAM setup, ten older PAL APU diagnostics with explicit PAL timing, and sixteen older sprite-hit/overflow diagnostics. Missing files, a wrong group count, failures, and timeouts are errors.

AccuracyCoin is a separate run. Success requires all 144 stored test results to pass, with zero skipped or unfinished entries, and the cartridge's own final tally to agree. The recorded baseline completes in 4,182 frames. `12000` is the allowed frame limit, not an instruction to stop before the cartridge finishes its result checks. The optional PPM is the rendered framebuffer.

## Diagnostic runner modes

These modes belong to `accuracy-tests`, not the SDL application. Replace the Linux runner path with `build/windows/accuracy-tests.exe` on Windows.

| Mode | Arguments after the mode | Result |
| --- | --- | --- |
| No mode | None | Run the internal hardware suite |
| `--trace` | `ROM LOG` | Compare the canonical CPU trace |
| `--rom` | `FRAMES ROM...` | Run ROMs using the `$6000` result protocol |
| `--mmc3-rom` | `FRAMES ROM...` | Enable MMC3 result RAM once, then use the ordinary protocol |
| `--legacy-pal-rom` | `FRAMES ROM...` | Force PAL and inspect the older `$F8` terminal-loop result |
| `--legacy-rom` | `FRAMES ROM...` | Inspect the older result using ROM-header timing |
| `--render` | `FRAMES ROM OUTPUT.ppm` | Render a fixed frame count for inspection |
| `--accuracycoin` | `FRAMES ROM [OUTPUT.ppm]` | Require all 144 cartridge tests to pass |

Frame arguments must be between 1 and 100000. The ordinary diagnostic mode returns failure for a timeout or missing result protocol. The legacy modes also require a stable terminal CPU loop before treating `$F8` as a final result. A successful `--render` call means a frame was produced; it is not a test pass.

Unknown modes print the usage line and return status 2. An out-of-range or malformed frame count prints `Frame count must be between 1 and 100000` and also returns 2. `--render` requires exactly one ROM and one output path; `--accuracycoin FRAMES ROM` accepts one optional output path. Incorrect argument counts return 2, sometimes without a usage message.

Examples:

```sh
build/accuracy-tests --trace build/diagnostic-roms/other/nestest.nes build/diagnostic-roms/other/nestest.log
build/accuracy-tests --rom 7200 build/diagnostic-roms/ppu_read_buffer/test_ppu_read_buffer.nes
build/accuracy-tests --render 240 build/diagnostic-roms/oam_read/oam_read.nes build/oam.ppm
```

The read-buffer test needs more than 1,200 frames; the diagnostic collection grants 7,200. Older ROMs can assume result RAM is already enabled or report only on screen. Preserve the explicit setup conditions when reporting results. The [accuracy notes](accuracy.md#interpreting-other-roms) explain these conventions; [troubleshooting](troubleshooting.md#diagnostic-test-failures) covers missing data, timeouts, and CI failures.

## TAS regressions

The production runner includes FM2/FM3 codec, editable project, Lua script,
session/state, and desktop gesture tests in its ordinary hardware run. They
can also be run separately:

```sh
build/accuracy-tests --fm2
build/accuracy-tests --tas-project
build/accuracy-tests --tas-script
build/accuracy-tests --tas-session
build/accuracy-tests --tas-editor-input
```

The fixtures are synthetic and do not require a commercial game. Codec tests
cover text/binary input, Four Score and Zapper records, metadata, bounded
decoding, malformed offsets, and transactional failures. Project tests cover
grouping, undo/redo, branches, selections, persistence, and damaged files. A
synthetic CTAS version 1 fixture checks migration of populated undo and redo
stacks; FM3 tests check branch-change status after editing and round trips. Lua
tests include rollback, memory limits, and endless loops. Session tests cover
input ownership, checkpoint equality, recording, commands, startup-frame state
restoration, and restoration of live hardware options. Desktop tests use SDL's
dummy driver and exercise actual layout hit targets at several display scales.
They also check the shared native-window opening path, window reuse, movement
and resizing, keyboard routing, and project retention after closing the editor.
Editor regressions cover navigation between markers, playback following without
changing the selection, insertion of several frames as one undo item, and
column edits over selections with gaps. Sidebar controls are checked against
the rendered hit targets at multiple display scales.

For a supplied game and movie, use `--movie` to run a bounded prefix or all
recorded input. A frame limit of zero means the full movie; positive limits
must be at most 100,000.

```sh
build/accuracy-tests --movie 0 game.nes run.fm2 build/movie-final.ppm
build/accuracy-tests --movie-trace 0 game.nes project.fm3 build/project-trace
```

The trace command writes `.ram` (2,048 bytes per completed frame), `.csv`
(frame, cumulative lag, lag flag, and PC), `.pixels` (final 256 by 240 palette
indices), and `.ppm` (final RGB image). It also checks that Stop restores the
preceding machine state exactly. Output paths must differ from the input
game and movie. A successful run means playback and restoration completed;
compare the trace with an independent reference before claiming synchronization.

With a local FCEUX executable, the automated comparison runs both emulators and
binds their results to the source movie, ROM, and executable hashes:

```sh
python3 scripts/check-tas-movies.py --runner build/accuracy-tests \
  --reference /path/to/fceux --rom game.nes --movie run.fm2 \
  --output build/tas-reference-check
```

The output directory must be new. The check requires every frame's RAM and lag
to match and compares the final palette image. It reports CPU endpoint
differences separately; it does not claim cycle-by-cycle CPU equality. The
reference executable must support `--loadlua` and the standard FCEUX Lua movie,
memory, and screen APIs. Its emulation code does not need instrumentation.

## Sanitizers

Build a fresh Linux binary with Clang AddressSanitizer and UndefinedBehaviorSanitizer:

```sh
make clean
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 make -j2 CC=clang CXX=clang++ CFLAGS='-std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer' all test
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 python3 scripts/run-diagnostics.py build/accuracy-tests build/diagnostic-roms
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 build/accuracy-tests --accuracycoin 12000 build/accuracycoin/AccuracyCoin.nes
```

Windows uses a separate output directory:

```powershell
.\scripts\test-windows.ps1 -SdlRoot 'C:\dependencies\SDL2-2.32.10' -Sanitize
python scripts/run-diagnostics.py build/windows-sanitized/accuracy-tests.exe build/diagnostic-roms
if ($LASTEXITCODE -ne 0) { throw 'Sanitized diagnostic collection failed' }
.\build\windows-sanitized\accuracy-tests.exe --accuracycoin 12000 .\build\accuracycoin\AccuracyCoin.nes
if ($LASTEXITCODE -ne 0) { throw 'Sanitized AccuracyCoin failed' }
```

The Windows script copies the Clang AddressSanitizer runtime beside the executables. Windows AddressSanitizer does not provide the Linux leak check. Linux CI enables leak detection explicitly.

## Developing an accuracy fix

Identify the failing instruction, register access, board variant, or device interaction. Start with the relevant source and its callers in [the architecture guide](architecture.md), then reduce the failure to a fixture that can distinguish the correct behavior from the current one.

For mapper and loader work, exercise the normal image loader where metadata matters. Test successful activation, rejected-load preservation, bank boundaries, open bus, and persistence as applicable. For timing changes, use real CPU reads/writes and check the surrounding bus cycles; directly assigning a register may bypass the behavior being tested.

The focused suites live in `src/tests`:

| File | Main coverage |
| --- | --- |
| [cpu_accuracy.c](../src/tests/cpu_accuracy.c) | Instruction execution, bus ordering, interrupts, reset, and DMA |
| [ppu_accuracy.c](../src/tests/ppu_accuracy.c) | Registers, rendering timing, OAM, open bus, and fetch behavior |
| [apu_accuracy.c](../src/tests/apu_accuracy.c) | Audio channels, frame timing, DMC, registers, and reset |
| [mapper_accuracy.c](../src/tests/mapper_accuracy.c) | Loading, banks, IRQs, RAM, nametables, persistence, and expansion sound |
| [bandai_accuracy.c](../src/tests/bandai_accuracy.c) | Bandai/Datach cartridges and serial devices |
| [fds_accuracy.c](../src/tests/fds_accuracy.c) | Disk memory, controller, media, persistence, and audio |
| [input_accuracy.c](../src/tests/input_accuracy.c) | Controller wiring, adapters, and peripherals |
| [vs_accuracy.c](../src/tests/vs_accuracy.c) | VS metadata, machine contexts, input, DMA, shared RAM, video, and audio |
| [epsm_accuracy.c](../src/tests/epsm_accuracy.c) | EPSM metadata, delayed OUT-pin protocol, direct writes, timers, firmware, reset, and stereo audio |

Run the focused hardware regression first. After each implemented accuracy issue, run the complete pinned AccuracyCoin suite and retain the exact commit and result. Changes to the shared core also need the canonical trace, diagnostic collection, and sanitizer checks. The complete final CI result belongs to the final pushed revision.

Do not lower pass requirements, substitute expected output, add cartridge-specific success paths, or skip a failing test to make the result green. A missing result protocol and a timeout remain unresolved results. Use [the contribution guide](../CONTRIBUTING.md) for review and reporting details.

## Documentation-only changes

Check relative links and heading anchors, compare CLI examples with [main.c](../src/main.c), and confirm that build paths and test pins match the scripts. Review tables against their source constants and explain any unsupported path explicitly. A prose edit does not need a new C unit test. The existing CI workflow still runs the hardware suite, diagnostic collection, and pinned AccuracyCoin check on pushes and pull requests.
