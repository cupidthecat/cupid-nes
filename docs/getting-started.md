# Getting started

[Documentation index](README.md)

Cupid builds an SDL2 emulator application and a separate hardware test program. Building from source requires C11 and C++17 compilers, Make on Linux, and SDL2 development files. The C++ compiler builds the EPSM sound engine. Python 3 is used by the external diagnostic workflow described in [development](development.md), not by the emulator itself.

## Get the source

```sh
git clone https://github.com/cupidthecat/cupid-nes.git
cd cupid-nes
```

Run the remaining commands from the repository root. The build does not download SDL2, game images, an FDS BIOS, the optional EPSM ADPCM ROM, or diagnostic ROM collections.

## Linux

Ubuntu and the Linux CI job use the normal Makefile build. Install GCC/G++, Make, and the SDL2 development package:

```sh
sudo apt update
sudo apt install build-essential libsdl2-dev
make
./cupid-nes "path/to/game.nes"
```

`make` writes the emulator to `./cupid-nes`. `make test` additionally builds `build/accuracy-tests` and runs the production hardware regression suite. The default Makefile uses GCC for C11 and G++ for C++17. If `CC=clang` is supplied and `CXX` has not been overridden, it selects `clang++` automatically.

Clang is supported through the Makefile as well. On Ubuntu:

```sh
sudo apt install clang
make clean
make CC=clang
```

Use `make clean` before switching compiler families or compiler flags because those settings are not source-file dependencies in the Makefile.

## Windows

The repository includes a PowerShell build script for x64 Windows. It defaults to Clang and clang++, so install Clang plus the Windows SDK/MSVC build tools, then extract the SDL2 VC development package. The directory passed to `-SdlRoot` must contain:

```text
SDL2-directory/
  include/SDL.h
  lib/x64/SDL2.lib
  lib/x64/SDL2.dll
```

From PowerShell in the repository root:

```powershell
.\scripts\test-windows.ps1 -SdlRoot 'C:\dependencies\SDL2-2.32.10'
.\build\windows\cupid-nes.exe 'C:\games\game.nes'
```

The script builds both programs, copies `SDL2.dll` beside them, and runs the hardware regression suite. A normal build produces:

```text
build/windows/cupid-nes.exe
build/windows/accuracy-tests.exe
build/windows/SDL2.dll
```

`-Compiler gcc` selects `g++` automatically. The default `clang` selects `clang++`; any other compiler basename needs an explicit `-CxxCompiler`. Normal builds use C11 or C++17 with `-Wall -Wextra -Werror -O2`. `-Sanitize` writes the binaries to `build/windows-sanitized`, changes the optimization/debug flags, enables AddressSanitizer and UndefinedBehaviorSanitizer, and requires Clang's Windows AddressSanitizer runtime. See [development](development.md) for the exact flags and test workflows. This checkout has no dedicated macOS build script or CI job.

## Open a cartridge

```sh
./cupid-nes "games/game.nes"
```

On Windows, use `.\build\windows\cupid-nes.exe` in place of `./cupid-nes`. Cupid reads unpacked iNES and NES 2.0 images. Extract ZIP or other archive formats before launching the emulator.

The application accepts one image path per launch. Paths with spaces need quotes. Starting `cupid-nes` without an image prints its usage and exits with status 1; there is no `--help` switch. Command-line hardware selection is covered in [configuration](configuration.md).

The startup log prints the selected console and CPU/PPU profiles, cartridge metadata, mapper information, and audio device details. Cartridge metadata chooses NTSC, PAL, or Dendy timing. The supported boards and current hardware limits are listed in [hardware](hardware.md) and [accuracy](accuracy.md).

EPSM cartridges can use an external 8 KiB YMF288 ADPCM ROM for percussion:

```sh
./cupid-nes --epsm-adpcm "ymf288_adpcm_rom.bin" "epsm-game.nes"
```

The file must be exactly 8,192 bytes. Cupid does not provide it. An EPSM image still loads without the option, but percussion reads zero-filled data while FM and SSG audio remain available. An unreadable or incorrectly sized file is rejected before the cartridge is loaded. See [configuration](configuration.md#epsm-sound) for the hardware selector and option details.

Player 1 can use the keyboard immediately: Z/X are A/B, Right Shift and Enter are Select/Start, and the arrow keys are the D-pad. [Controls](controls.md) covers game controllers and special peripherals.

Close the game window for the normal shutdown path. This matters for persistent cartridge memory and writable disk media; see [saves and media](saves.md).

## Open an FDS or QD image

Disk-system loading requires an 8 KiB BIOS file:

```sh
./cupid-nes --fds-bios "games/disksys.rom" --fds-write-protect "games/game.fds"
```

This example starts the disk write-protected. If a game must write to disk, make a working copy of the image and omit `--fds-write-protect`. Cupid writes modified media back to the launched image during normal quit, unload, or replacement.

F8 ejects or reinserts the selected side, F9 advances to the next side, and F10 toggles write protection. Use `--fds-side N` to choose another initial side. The image may use normal 65,500-byte FDS sides or 65,536-byte QD sides, with or without a valid FDS header. See [saves and media](saves.md) before using writable media.

## Open a VS System image

VS hardware is selected from cartridge metadata. `--vs-dip` only sets the cabinet DIP value:

```sh
./cupid-nes --vs-dip 0x0000 "games/arcade.nes"
```

The option does not turn a normal cartridge into a VS image. A supported dual-system image opens a window wide enough for both 256 by 240 screens and mixes both APUs into the output stream. Keys 5 and 6 drive the main cabinet coin inputs, and F1 drives its service input. [Controls](controls.md) lists the secondary cabinet keys and player assignment.

If a game does not load, keep the terminal output. [Troubleshooting](troubleshooting.md) maps the common loader and runtime errors to the source checks that produced them.
