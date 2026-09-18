# Getting started

[Documentation index](README.md)

Cupid builds two programs: the SDL application and `accuracy-tests`, which runs hardware regressions and diagnostic cartridges. You need a C11 compiler and SDL2 development files. Python 3 is needed only for the external diagnostic collection.

## Get the source

```sh
git clone https://github.com/cupidthecat/cupid-nes.git
cd cupid-nes
```

For an existing checkout, run the following commands from its root. Select the branch you intend to test before building. The source tree does not fetch SDL2, game images, a disk BIOS, or the pinned diagnostic collections during a build.

## Linux

The CI build uses Ubuntu 24.04 with GCC and Clang. Install the development dependencies on Ubuntu:

```sh
sudo apt update
sudo apt install build-essential clang libsdl2-dev python3
make -j2 all test
./cupid-nes "path/to/game.nes"
```

`make` builds `./cupid-nes`. `make test` builds and runs `build/accuracy-tests`. The Makefile links SDL2 and the math library and tracks included headers through generated `.d` files.

To switch to Clang, rebuild the generated objects:

```sh
make clean
make -j2 CC=clang all test
```

The Makefile's clean target removes its application, test executable, object files, and dependency files. It leaves the downloaded test collections under `build/` in place. Rebuild when changing compilers or compiler flags; Make does not treat a changed `CFLAGS` value as a source dependency.

## Windows

Use an x64 Clang toolchain with the Windows SDK and MSVC linker libraries available. A developer PowerShell session from the Visual Studio Build Tools installation can supply the SDK environment. The build script invokes `clang` by default; `-Compiler` can select another Clang executable.

Extract the SDL2 **VC development package**. Pass the directory that contains these files:

```text
SDL2-directory/
  include/SDL.h
  lib/x64/SDL2.lib
  lib/x64/SDL2.dll
```

The runtime DLL alone is insufficient for compiling. From PowerShell in the repository root:

```powershell
.\scripts\test-windows.ps1 -SdlRoot 'C:\dependencies\SDL2-2.32.10'
.\build\windows\cupid-nes.exe 'C:\games\game.nes'
```

Replace the example SDL path with your extracted directory. The script builds the application and test runner, copies SDL headers and the DLL into its output directory, and runs the hardware regressions. It stops on a compiler or test failure.

| Build | Application | Test runner |
| --- | --- | --- |
| Normal | `build/windows/cupid-nes.exe` | `build/windows/accuracy-tests.exe` |
| With `-Sanitize` | `build/windows-sanitized/cupid-nes.exe` | `build/windows-sanitized/accuracy-tests.exe` |

The [development guide](development.md) covers sanitizer builds and the external ROM tests. macOS and other toolchains have no dedicated build script or CI job in this checkout.

## Open a cartridge

```sh
./cupid-nes "games/game.nes"
```

Use `build/windows/cupid-nes.exe` on Windows. Paths are relative to the terminal's current directory unless absolute, and paths containing spaces need quotes. Cupid reads an unpacked iNES or NES 2.0 image. Extract ZIP or other archives first.

The startup log reports the selected hardware and cartridge information. The ROM header chooses NTSC, PAL, or Dendy timing. Player 1 uses Z/X, Right Shift/Enter, and the arrow keys. Close the game window for normal shutdown and save flushing.

The application takes one game path per launch. Launching it without arguments prints its usage and returns exit code 1. It currently has no `--help` option. Drag-and-drop goes to the palette loader, so it cannot be used to switch games.

## Open a disk image

Supply an 8 KiB BIOS and a disk image:

```sh
./cupid-nes --fds-bios "games/disksys.rom" --fds-write-protect "games/game.fds"
```

This example starts with disk writes blocked. For a game that saves to disk, use a working copy of the image and omit `--fds-write-protect`. Successful saves replace that working image. F8 inserts or ejects the selected side, F9 advances to the next side, and F10 toggles write protection. See [saves and media](saves.md) before using writable disks.

## Open a VS image

```sh
./cupid-nes --vs-dip 0x0000 "games/arcade.nes"
```

VS hardware is selected from the image metadata. `--vs-dip` sets cabinet switches; it does not turn an ordinary cartridge into a VS game. Dual configurations display both screens and combine their audio. Keys 5/6 operate the first cabinet's coin slots and F1 operates its service input. See [controls](controls.md) for the second cabinet and additional players.

When loading fails, use the exact error in [troubleshooting](troubleshooting.md). A mapper number appearing in the README does not guarantee support for every ROM size or submapper of that board.
