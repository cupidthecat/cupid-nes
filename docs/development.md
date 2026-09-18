# Development and testing

[Documentation index](README.md)

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
make -j2 CC=gcc CFLAGS='-std=c11 -Wall -Wextra -Werror -O2' all test
```

The application is `./cupid-nes` and the runner is `build/accuracy-tests`. Running the test executable without arguments executes CPU/controller, APU, PPU, mapper, Bandai, disk, input, and VS groups and returns failure if any group fails.

On Windows:

```powershell
.\scripts\test-windows.ps1 -SdlRoot 'C:\dependencies\SDL2-2.32.10'
```

The script builds both executables and runs the hardware suite. Its output is under `build/windows`. Supplying `-Compiler` changes the Clang executable; the SDL library path remains the x64 VC SDK layout.

Tests use the device code listed in [Makefile](../Makefile) and [test-windows.ps1](../scripts/test-windows.ps1). Add any new production or test source to both lists. The older `src/tests/cpu_test.c` harness is excluded because its writable-ROM assumptions do not match the cartridge bus.

## Prepare the pinned test collections

The external ROM collections are separate from the internal hardware suite. Clone them once into an ignored build directory:

```sh
git clone https://github.com/christopherpow/nes-test-roms.git build/diagnostic-roms
git -C build/diagnostic-roms checkout 95d8f621ae55cee0d09b91519a8989ae0e64753b
git clone https://github.com/100thCoin/AccuracyCoin.git build/accuracycoin
git -C build/accuracycoin checkout 9bc42d1e3acbeeaea215b1011d58f4ce72a8a49e
```

For existing directories, inspect their checkout and local changes before updating them. Do not clone over an existing directory or replace a modified collection. The required revisions are also in [the CI workflow](../.github/workflows/accuracy.yml).

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
build/accuracy-tests --accuracycoin 12000 build/accuracycoin/AccuracyCoin.nes build/accuracycoin.ppm
```

On Windows:

```powershell
python scripts/run-diagnostics.py build/windows/accuracy-tests.exe build/diagnostic-roms
if ($LASTEXITCODE -ne 0) { throw 'Diagnostic collection failed' }
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

Examples:

```sh
build/accuracy-tests --trace build/diagnostic-roms/other/nestest.nes build/diagnostic-roms/other/nestest.log
build/accuracy-tests --rom 7200 build/diagnostic-roms/ppu_read_buffer/test_ppu_read_buffer.nes
build/accuracy-tests --render 240 build/diagnostic-roms/oam_read/oam_read.nes build/oam.ppm
```

The read-buffer test needs more than 1,200 frames; the diagnostic collection grants 7,200. Older ROMs can assume result RAM is already enabled or report only on screen. Preserve the explicit setup conditions when reporting results. The [accuracy notes](accuracy.md#interpreting-other-roms) explain these conventions.

## Sanitizers

Build a fresh Linux binary with Clang AddressSanitizer and UndefinedBehaviorSanitizer:

```sh
make clean
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 make -j2 CC=clang CFLAGS='-std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer' all test
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

Run the focused hardware regression first. After each implemented accuracy issue, run the complete pinned AccuracyCoin suite and retain the exact commit and result. Changes to the shared core also need the canonical trace, diagnostic collection, and sanitizer checks. The complete final CI result belongs to the final pushed revision.

Do not lower pass requirements, substitute expected output, add cartridge-specific success paths, or skip a failing test to make the result green. A missing result protocol and a timeout remain unresolved results. Use [the contribution guide](../CONTRIBUTING.md) for review and reporting details.

## Documentation-only changes

Check relative links and heading anchors, compare CLI examples with [main.c](../src/main.c), and confirm that build paths and test pins match the scripts. Review tables against their source constants and explain any unsupported path explicitly. A prose edit does not need a new C unit test. The existing CI workflow still runs its full hardware and diagnostic checks on pushes and pull requests.
