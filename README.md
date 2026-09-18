# Cupid NES Emulator

Cupid runs NES and Famicom cartridges, Famicom Disk System images with a supplied
BIOS, and supported VS System arcade images. It uses SDL2 for video, audio, and
controllers. The CPU and PPU core is C11; cartridge board modules and the EPSM
YMF288 sound engine use C++17.

The core implements NTSC, PAL, and Dendy timing. The
[tested implementation](docs/accuracy-checkpoints.md#combined-validation) passes
all 144 AccuracyCoin tests with zero skipped or unfinished results, the 91-ROM
diagnostic collection, and the 8,991-state canonical CPU trace. The
[accuracy notes](docs/accuracy.md) describe the test setup and coverage. Mapper,
input, storage, and expansion-audio regressions cover hardware that AccuracyCoin
does not exercise.

<p align="center">
  <img src="img/smb33.png" alt="Super Mario Bros. 3 running in Cupid">
</p>
<p align="center">
  <img src="img/coin.png" alt="AccuracyCoin results from the production core">
</p>

## Emulated hardware

| Area | Implemented behavior |
| --- | --- |
| CPU and PPU | Official and undocumented instructions, shared OAM/DMC DMA timing, register delays, sprite evaluation, regional frame timing, and selectable startup alignment |
| Cartridges | Board-specific PRG/CHR banking, nametable routing, bus conflicts, RAM permissions, IRQs, EEPROM, and flash; the [mapper table](docs/hardware.md#cartridge-mappers) lists supported families and variants |
| Sound | Five base APU channels, cartridge and disk expansion audio, and EPSM stereo output with timer IRQs |
| Controllers and storage | Gamepads, multiplayer adapters, light guns, paddles, mats, Family BASIC and Subor keyboards, mouse/trackball/tablet input, specialty expansion controllers, Turbo File, and BattleBox |
| VS System | Header-selected RGB PPU and controller behavior, cabinet controls and protection, and dual machines with shared RAM, two screens, and mixed audio |

Device selection and timing follow the ROM header and
[command-line options](docs/configuration.md). Supported mapper families can still
reject unsupported submappers or memory layouts. The
[hardware guide](docs/hardware.md#reading-accuracy-results) records remaining
limits, including optional Jaleco speech, a distinct RP2C03G palette, and MMC5
auxiliary I/O/timers. VS images requesting RP2C03G use the documented 2C03 fallback.

## Build and run

Clone the repository and run the build commands from its root:

```sh
git clone https://github.com/cupidthecat/cupid-nes.git
cd cupid-nes
```

### Linux

Install C11 and C++17 compilers, Make, and the SDL2 development libraries. On Ubuntu:

```sh
sudo apt install build-essential libsdl2-dev
make
./cupid-nes path/to/game.nes
```

Use `make CC=clang CXX=clang++` for Clang. Both language compilers are required
even when the loaded game does not use EPSM. Run `make clean` before changing
compilers or flags.

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
option. It accepts one image per launch and has no configuration-file loader.

For example, an Oeka Kids cartridge needs its tablet selected explicitly:

```sh
./cupid-nes --console famicom --expansion oeka-kids-tablet path/to/oeka-kids.nes
```

EPSM is selected by NES 2.0 console metadata. A separately supplied 8 KiB
percussion ROM can be loaded with `--epsm-adpcm`; that option alone does not
enable EPSM. [Configuration](docs/configuration.md) covers these choices and
other peripheral combinations.

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
guns, paddles, floor mats, keyboards, mouse and tablet input, and the Famicom
expansion devices. Peripheral key handlers can take precedence over the shortcuts
above; Family BASIC, Subor keyboards, and active mats consume R as device input.

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
the original.

Cartridge save files live beside their ROM. The
[saves and media guide](docs/saves.md) covers PRG/CHR save memory, EEPROM, flash,
disk images, BASIC tapes, and Turbo File/BattleBox storage. Cartridge RAM can
remain readable at addresses where writes select banks. Its layout and
persistence follow the selected board and header.

Disk and expansion-storage save failures keep the window open during normal
quit. Cartridge saves and tape recordings have different failure paths; the
save guide explains which changes can be retried and which are lost when the
process exits.

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
and Clang sanitizer settings on pushes and pull requests. Both jobs run the
hardware suite, CPU trace, all 91 diagnostic ROMs, and AccuracyCoin. The Clang job
enables AddressSanitizer, UndefinedBehaviorSanitizer, and Linux leak detection.

Test results apply to the checked commit and configuration. The baseline uses
the default startup alignment. Explicit phases, seeded startup choices, and
optional hardware profiles have separate focused checks; the baseline does not
establish every possible DMA/register alignment or analog output effect.

## License and credits

Cupid is GPL-3.0-or-later; see [LICENSE](LICENSE). The bundled emu2413 component
retains its [MIT license](src/rom/emu2413.LICENSE), and ymfm retains its
[BSD 3-Clause license](src/third_party/ymfm/LICENSE). See
[credits and references](docs/credits.md) for component attribution, test
sources, and hardware documentation.
