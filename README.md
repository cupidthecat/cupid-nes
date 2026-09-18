# Cupid NES Emulator

Cupid is an NES emulator written in C11. It uses SDL2 for video, audio, and
controllers, and runs NES/Famicom cartridges, Famicom Disk System images with a
supplied BIOS, and supported VS System arcade images.

The core implements NTSC, PAL, and Dendy timing. Its recorded test baseline is
144/144 AccuracyCoin tests with zero skipped or unfinished results, the 91-ROM
diagnostic collection, and the 8,991-state canonical CPU trace. The
[accuracy notes](docs/accuracy.md) explain what those tests cover, and the
[checkpoint record](docs/accuracy-checkpoints.md) identifies the tested commits.

<p align="center">
  <img src="img/smb33.png" alt="Super Mario Bros. 3 running in Cupid">
</p>
<p align="center">
  <img src="img/coin.png" alt="AccuracyCoin results from the production core">
</p>

## Build and run

Clone the repository and run the build commands from its root:

```sh
git clone https://github.com/cupidthecat/cupid-nes.git
cd cupid-nes
```

### Linux

Install a C11 compiler, Make, and the SDL2 development libraries. On Ubuntu:

```sh
sudo apt install build-essential libsdl2-dev
make
./cupid-nes path/to/game.nes
```

Use `make CC=clang` for Clang. Clean the build before changing compilers or flags.

### Windows

Install Clang and the Windows SDK/MSVC build tools, and extract the SDL2 VC
development package. In PowerShell:

```powershell
.\scripts\test-windows.ps1 -SdlRoot 'C:\path\to\SDL2-2.32.10'
.\build\windows\cupid-nes.exe 'C:\path\to\game.nes'
```

The script builds the application and hardware tests, copies `SDL2.dll` beside
the executables, and runs the hardware suite. See
[getting started](docs/getting-started.md) for prerequisites, output paths, and
sanitizer builds.

Open games from the command line. File drops load palettes. Running the
application without arguments prints its usage; it does not have a `--help`
option.

## Controls

| Key | Action |
| --- | --- |
| Z / X | A / B |
| Right Shift / Enter | Select / Start |
| Arrow keys | D-pad |
| R | Soft reset |
| F7 / F6 | Show palette editor / restore default palette |
| Ctrl+V | Paste palette text |

The keyboard and first SDL game controller both drive player 1. More controllers
fill the remaining player slots; the selected multiplayer adapter or VS image
determines which players a game can read. See
[controls and peripherals](docs/controls.md) for controller assignment, light
guns, paddles, floor mats, the Famicom microphone, and Family BASIC.

## Hardware and saves

The [hardware reference](docs/hardware.md) lists supported mapper families,
expansion audio, console revisions, ROM-header requirements, and remaining
limits. Mapper support describes implemented hardware; it is not a per-game
compatibility guarantee.

To open a disk image with its 8 KiB BIOS:

```sh
./cupid-nes --fds-bios disksys.rom --fds-write-protect game.fds
```

F8 inserts or ejects the selected side, F9 changes sides, and F10 toggles write
protection. Cupid writes modified disk data back to the loaded image when it
flushes media on normal quit, unload, or replacement. Use a working copy to retain
the original. A failed save during normal quit keeps the application open with
the modified media still loaded.

Cartridge save files live beside their ROM. The
[saves and media guide](docs/saves.md) covers PRG/CHR save memory, EEPROM, flash,
disk images, and BASIC tapes, including their different failure behavior.

## Documentation

| Task | Guide |
| --- | --- |
| Build Cupid and start a game | [Getting started](docs/getting-started.md) |
| Look up options and defaults | [Configuration](docs/configuration.md) |
| Set up controllers and peripherals | [Controls](docs/controls.md) |
| Check supported hardware and ROM formats | [Hardware](docs/hardware.md) |
| Look up hardware and test terminology | [Hardware terminology](docs/glossary.md) |
| Locate saves and use writable media | [Saves and media](docs/saves.md) |
| Diagnose a build or runtime problem | [Troubleshooting](docs/troubleshooting.md) |
| Understand the core and source layout | [Architecture](docs/architecture.md) |
| Run tests or investigate an accuracy failure | [Development and testing](docs/development.md) |
| Submit a change | [Contributing](CONTRIBUTING.md) |

The [documentation index](docs/README.md) also links test evidence and component
credits. For questions and bug reports, see [support](SUPPORT.md).

## Tests

On Linux, `make test` builds and runs the hardware regressions against the
production core. The Windows build script runs the same suite. External test
ROMs are separate checkouts; the [development guide](docs/development.md) gives
their pinned revisions and the commands for the CPU trace, diagnostics, and
AccuracyCoin.

The [accuracy workflow](.github/workflows/accuracy.yml) builds with strict GCC
and Clang sanitizer settings and runs the pinned suites. Test results apply to
the checked commit and configuration. Analog output, arbitrary power-on phases,
and every possible DMA/register alignment remain outside the tested model.

## License and credits

Cupid is GPL-3.0-or-later; see [LICENSE](LICENSE). The bundled emu2413 component
retains its [MIT license](src/rom/emu2413.LICENSE). See
[credits and references](docs/credits.md) for component attribution, test
sources, and hardware documentation.
