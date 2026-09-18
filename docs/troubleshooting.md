# Troubleshooting

[Documentation index](README.md)

Cupid prints loader, SDL, persistence, and option errors to the terminal that launched it. Keep that output when diagnosing a failure because the final `Failed to load ROM` line is often preceded by the more specific cause.

## Linux build errors

If compilation fails because `SDL2/SDL.h` or the SDL2 library cannot be found, install the development package and rebuild:

```sh
sudo apt update
sudo apt install build-essential libsdl2-dev
make clean
make
```

The Makefile requires C11 and C++17 compilers and links `-lSDL2 -lm`. It defaults to GCC/G++, and `make CC=clang` selects `clang++` automatically when `CXX` has not been overridden. If you supply custom `CXX` or `CXXFLAGS`, the Makefile keeps them. Clean the existing object files before changing compiler families or flag sets.

## Windows build errors

`scripts/test-windows.ps1` defaults to x64 Clang/clang++ with the Windows SDK/MSVC libraries available. `-Compiler gcc` selects `g++`; another compiler basename must be paired with `-CxxCompiler` or the script stops with `Specify -CxxCompiler for this C compiler`. `-SdlRoot` must point to the extracted SDL2 VC development package, not a directory containing only the runtime DLL.

The script checks for these files before compiling:

```text
include/SDL.h
lib/x64/SDL2.lib
lib/x64/SDL2.dll
```

An error such as `Missing SDL2 VC SDK file` means the supplied root does not match that layout. A compiler or linker error about Windows runtime libraries usually means the Windows SDK/MSVC build environment is not available to the Clang invocation. See [getting started](getting-started.md) for the expected output directories.

The normal script uses C11 or C++17 with `-Wall -Wextra -Werror -O2`. `-Sanitize` switches to `-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer` and requires the Clang AddressSanitizer runtime reported by `-print-resource-dir`.

## `Unexpected argument`

The application accepts one image path and startup options. Values must be separate arguments:

```sh
./cupid-nes --port2 zapper "game.nes"
```

`--port2=zapper`, an unknown switch, or a second image path reaches the `Unexpected argument` error. The full application option list is in [configuration](configuration.md). `cupid-nes` has no `--help` switch; launching it without an image prints the usage line and returns status 1.

## Cartridge will not load

Common loader messages describe different problems:

| Message | What to check |
| --- | --- |
| `open: ...` or `Failed to read ROM file` | Path, file permissions, and whether the image is still inside an archive |
| `Truncated iNES header` | File is shorter than a complete iNES header |
| `Invalid iNES signature` | File is not an unpacked iNES/NES 2.0 image |
| `Truncated iNES trainer` | Header declares a trainer that is missing from the file |
| `Truncated PRG-ROM or CHR-ROM payload` | Image is incomplete or its header sizes do not match the payload |
| `Unsupported mapper` or `Unsupported mapper/submapper` | That board selector is not implemented |
| `Unsupported ROM size`, `Unsupported ROM/RAM size`, or `Unsupported cartridge layout` | Mapper exists, but this image declares a layout the implementation rejects |
| `Unsupported VS System configuration: ...` | VS metadata requests unsupported mapper, timing, controller wiring, protection type, or layout |
| `VS PPU code 1 is not modeled separately; using 2C03 behavior` or `Unknown VS PPU code ...; using 2C03 behavior` | The image loads with the ordinary 2C03 profile. Its requested palette is not implemented separately; see [VS hardware](hardware.md#vs-system) |

See [hardware](hardware.md) for the supported board families and [accuracy](accuracy.md) for current implementation limits. A mapper family appearing in the project does not imply every submapper and ROM/RAM size is accepted.

## FDS image or BIOS will not load

`--fds-bios` switches the positional image to the disk-system loader. `Failed to read FDS disk or BIOS file` means one of the two paths could not be read.

`Invalid FDS disk image or BIOS` means the files were readable but failed format validation. The BIOS must be exactly 8 KiB. Disk images must contain one or more complete 65,500-byte FDS sides or 65,536-byte QD sides, optionally preceded by a valid 16-byte FDS header whose side count matches the file size.

`FDS side is outside the loaded disk image` means `--fds-side N` is larger than the image's side count. Side numbers on the command line start at 1.

## FDS save error keeps the window open

When a writable disk has changed, closing the window first tries to replace the launched image. If the replacement fails, Cupid displays an FDS save error and keeps the emulator open so the dirty media remains in memory.

Check that the disk directory is writable and has free space. If you enabled write protection after the game had already modified the disk, press F10 to disable it before retrying the close. Also check for a stale sibling file ending in `.cupid-fds.tmp`; a prior interrupted process can leave one behind and Cupid will not overwrite an existing temporary file.

Do not force-close the emulator while its in-memory disk changes are the only copy you have. More detail is in [saves and media](saves.md).

## Tape playback or recording fails

`--tape-play` and `--tape-record` require `--expansion family-basic` and cannot be used together. `Could not load tape` means the playback file could not be read into memory.

When F11 cannot save a recording, Cupid prints `Could not save tape; the captured signal remains in memory`. Fix the destination path or stale `.cupid-tape.tmp` problem and press F11 again. If the capture buffer itself runs out of memory, recording stops and shutdown reports that the tape capture buffer could not grow.

The tape file is raw packed signal data rather than WAV audio. See [saves and media](saves.md) for its exact layout.

## Audio is disabled

If SDL cannot open the requested audio device, Cupid prints `Warning: audio disabled (...)` and continues with video and input. Check the host audio device and SDL environment first. The emulator requests 44.1 kHz floating-point audio with a 1024-sample buffer. It requests one channel for the normal NES/VS path and two channels when EPSM is active. The startup log prints the sample rate and buffer size SDL actually opened.

## EPSM percussion firmware is rejected or missing

`--epsm-adpcm` accepts one external YMF288 ADPCM ROM that is exactly 8,192 bytes. If the path cannot be read or the file has any other size, startup stops with `Could not load the 8 KiB YMF288 ADPCM ROM: ...` before the cartridge is loaded.

An EPSM cartridge does not require the file to start. Without it, Cupid reports `EPSM percussion uses zero-filled data without --epsm-adpcm FILE`; FM and SSG audio continue, but percussion data is not present. The firmware is separate from the game image and is not included with Cupid.

## SDL window or renderer errors

`SDL_Init Error`, `SDL_CreateWindow Error`, `SDL_CreateRenderer Error`, and `SDL_CreateTexture Error` come from SDL setup before emulation starts. On Linux, confirm a working graphical session and SDL2 installation. On Windows, make sure the `SDL2.dll` copied by the build script remains beside `cupid-nes.exe` and that the executable is running in a desktop session with a usable graphics driver.

## Controller is ignored

The frontend opens devices SDL recognizes as GameControllers. Raw joysticks without an SDL GameController mapping are skipped. Supported host inputs are A, B, Back, Start, and the D-pad; analog-stick movement is not mapped.

Controllers fill player slots in discovery order, and a hot-plugged controller takes the first free slot. Player 3 and player 4 therefore require the third and fourth recognized controllers when a Four Score or dual VS setup needs them. [Controls](controls.md) lists the full routing.

## Keyboard shortcut does something else

Special peripherals get their keyboard input before the normal application shortcuts. With Family BASIC selected, R, F6, F7, function keys, letters, and punctuation belong to the BASIC keyboard. With a mat selected, `1 2 3 4`, `Q W E R`, and `A S D F` are mat positions; R therefore does not reset while that mat key is being handled.

## Palette file or paste is rejected

A dropped palette file must be exactly 192 bytes for 64 RGB triplets or 1536 bytes for eight emphasis tables. Ctrl+V accepts those same byte counts as raw hexadecimal text, or exactly 64 six-digit RGB tokens.

F6 restores the built-in palette if an experiment looks wrong. VS System rendering uses the emulated VS PPU palette mapping and is not replaced by the normal custom palette table.

## VS DIP or barcode option is rejected

`--vs-dip requires a VS System image` means the loaded cartridge metadata did not configure supported VS hardware. The option changes DIP bits only.

`Barcode input requires a Datach cartridge and 8 or 13 decimal digits` means either the mapper is not the supported Datach configuration or the barcode has the wrong length/content. [Configuration](configuration.md) has examples for both options.

## Diagnostic test failures

The project workflow requires the pinned diagnostic checkout. `run-diagnostics.py` itself verifies the required paths and expected number of ROMs in each group, but it does not inspect the checkout's Git revision. A missing file or wrong group count fails the run. Verify the revision separately before interpreting a result. AccuracyCoin is also a separate run; verify its pinned SHA-256 before launching it.

The ordinary runner expects a known result protocol. Some older ROMs report only on screen, assume writable cartridge RAM without enabling it, or have incorrect timing metadata. Use the documented mode for that collection. A missing signature, timeout, or skipped result is not a pass; the [accuracy notes](accuracy.md#interpreting-other-roms) describe the supported conventions.

The standalone `accuracy-tests` executable prints its usage and returns status 2 for an unknown mode. A malformed or out-of-range frame count prints `Frame count must be between 1 and 100000` and returns 2. The render and AccuracyCoin modes also return 2 when their positional argument counts are wrong.

For a CI failure, record the job, commit, command, and assertion. Reproduce its compiler flags and data pins. A Linux sanitizer failure can expose a defect that a normal Windows build does not exercise.
