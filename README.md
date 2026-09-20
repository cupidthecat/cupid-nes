# Cupid NES Emulator

Cupid runs NES and Famicom cartridges, UNIF images, NSF and NSFe music, Famicom
Disk System disks, StudyBox media, and supported VS System arcade images. Disk
System and StudyBox images require their respective BIOS files. It uses SDL2 for
video, audio, and controllers, with Clay layouts and cached TrueType text for
the desktop interface. A menu bar and toolbar provide the main actions; nested
menus keep longer command lists on screen. Settings use checkboxes, option
lists, editable numeric fields, and native file pickers. Audio output uses a
list of detected devices. Settings and tools open in separate resizable windows;
the debugger can stay open while the game runs. The interface opens and switches games,
remembers settings and input profiles, and provides access to save states,
rewind, debugging, cheats, music playback, netplay, HD packs, and screenshots or recordings.
The CPU and PPU core is C11; cartridge board modules and the EPSM YMF288 sound
engine use C++17.

The core implements NTSC, PAL, and Dendy timing. The
[tested implementation](docs/accuracy-checkpoints.md#regional-timing-checkpoint) passes
all 144 AccuracyCoin tests with zero skipped or unfinished results, the 91-ROM
diagnostic collection, and the 8,991-state canonical CPU trace in normal and
sanitizer builds. The
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
| CPU and PPU | Official and undocumented instructions, shared OAM/DMC DMA timing, register delays, selectable PPU readback and sprite-evaluation behavior, regional frame timing, startup alignment and RAM contents, and optional PPU reset suppression |
| Cartridges | Board-specific PRG/CHR banking, mixed CHR RAM/NVRAM with separate persistence, nametable routing, bus conflicts, RAM permissions, IRQs, EEPROM, and flash; the [mapper table](docs/hardware.md#cartridge-mappers) lists supported families and variants |
| Sound | Five base APU channels, optional oldest-Famicom noise and clone pulse duty behavior, CPU-cycle band-limited reconstruction, cartridge and disk expansion audio, and EPSM stereo output with timer IRQs |
| Music and other media | NSF/NSFe initialization and play scheduling, track transport, timed fades, silence-based progression, repeat and shuffle, bank switching and expansion sound; StudyBox tape transport and audio; Famicom Network System RAM, character-ROM, and controller interfaces |
| Controllers and storage | NES, SNES, and Virtual Boy gamepads; SNES and Subor mice; NTT Data keypads; multiplayer adapters, light guns, paddles, mats, keyboards, trackballs, tablets, specialty expansion controllers, Turbo File, and BattleBox |
| VS System | Header-selected RGB PPU and controller behavior, hardware-palette light sensing, cabinet controls and protection, and dual machines with shared RAM, two screens, and mixed audio |

Cartridge loading supports iNES, NES 2.0, named UNIF boards, and an optional CRC
database for legacy corrections and recognized headerless images. Cupid looks
for `NesDB.txt` in its application data folder; `--data-dir` changes that folder
and `--game-db` selects a specific database. Database records also select the
Zapper connector: Famicom and Dendy games use the
expansion port, and NES games use controller port two. Explicit controller
options take precedence. Small and
irregular images use the board's implemented page mapping, including open bus
where a complete page cannot be mapped. CHR ROM and RAM follow each board's
source selectors, startup mappings, and write permissions. Declared memory can
remain allocated even when the board cannot address every byte. Work RAM and
save RAM retain separate ownership; UNROM 512 and GTROM also keep ordinary RAM
saves independent of their writable flash images. The
[cartridge checkpoints](docs/cartridge-checkpoints.md#memory-and-review-checkpoints)
record the tested memory and banking fixes.

For NSF and NSFe files that use MMC5, multiplier operands survive soft reset
and track changes. Loading a music image initializes both operands to zero.
Music INIT receives the PAL flag only in PAL mode; Dendy keeps its own clock
while using `X=0` for initialization.

VRC7 cartridges reset their FM synthesizer on console reset and retain their
bank and IRQ registers.
CPU reset preserves NMI edges raised during its bus cycles, including vblank
edges when PPU reset suppression keeps the raster running.
Disk-adapter work RAM and CHR RAM use the selected power-on profile when a disk
is loaded and retain their contents across soft reset.

Device selection and timing follow the ROM header and
[command-line options](docs/configuration.md). `--region auto` uses image metadata
and applicable database corrections;
`--region ntsc`, `--region pal`, and `--region dendy` select an explicit timing
profile for supported hardware. Console wiring remains a separate choice.
Supported mapper families can still
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

Run the application without an image to open the desktop, then use **Open** or
**File > Open Game**. The recent list remembers successfully opened images,
including the selected member of a ZIP or 7z archive and any applied patch.
Open another image in the same window to switch games. A failed load or save
keeps the current session available.

An image argument still opens a game directly. Explicit command-line choices
take precedence over saved settings for that launch. The application data
folder contains `settings.ini`, `recent.ini`, and the optional `NesDB.txt`;
`--data-dir` selects another folder. See [configuration](docs/configuration.md)
for the options and [getting started](docs/getting-started.md) for file dialogs,
archives, and platform details.

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
| Ctrl+O | Open a game |
| Ctrl+P / Ctrl+. | Pause or resume / advance one frame while paused |
| Ctrl+R / Ctrl+Shift+R | Soft reset / power cycle |
| Ctrl+Alt+R | Reload the current image |
| Ctrl+F / Ctrl+Shift+F | Hold fast-forward / toggle fast-forward |
| Ctrl+1 / Ctrl+2 / Ctrl+3 | Half speed / normal speed / double speed |
| F5 / F7 | Quick save / quick load the selected state slot |
| Ctrl+F5 / Ctrl+F7 | Save / load a state file |
| F6 | Restore the default palette |
| Page Up / Page Down | Next / previous NSF or NSFe track |
| Ctrl+Space / Ctrl+End / Ctrl+Home | Music play-pause / stop / restart |
| Ctrl+V | Paste palette text |
| F12 / Ctrl+F12 / Shift+F12 | Screenshot / audio recording / video recording |
| Ctrl+Shift+F12 | Stop and finalize a recording |

These are the default bindings. Settings can select another input profile and
change keyboard or gamepad assignments. The keyboard and first SDL game
controller both drive player 1 by default. More controllers fill the remaining
player slots; the selected multiplayer adapter or VS image determines which
players a game can read. See
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
protection. The desktop defaults to overlay saving: disk writes go into a
separate IPS file and leave the original image unchanged. In-place saving is
also available. Archived or patched disks use their own overlay identity.

Cartridge save files live beside their ROM. The
[saves and media guide](docs/saves.md) covers PRG/CHR save memory, EEPROM, flash,
disk images, BASIC tapes, and Turbo File/BattleBox storage. Cartridge RAM can
remain readable at addresses where writes select banks. Its layout and
persistence follow the selected board and header.

Normal quit and image replacement finalize pending recordings and persistent
data before discarding the running session. Failed writes keep the session
available for retry. Save states restore the machine as well as its memory;
input movies and network sessions use isolated timelines so their writes cannot
silently replace ordinary progress. See [states and replay](docs/replay.md) and
the save guide for those ownership rules.

## Documentation

| Task | Guide |
| --- | --- |
| Build Cupid and start a game | [Getting started](docs/getting-started.md) |
| Look up options and defaults | [Configuration](docs/configuration.md) |
| Set up controllers and peripherals | [Controls](docs/controls.md) |
| Check supported hardware and ROM formats | [Hardware](docs/hardware.md) |
| Look up hardware and test terminology | [Hardware terminology](docs/glossary.md) |
| Locate saves and use writable media | [Saves and media](docs/saves.md) |
| Use states, rewind, run-ahead, or input movies | [States and replay](docs/replay.md) |
| Play NSF and NSFe tracks | [Music player](docs/music.md) |
| Save screenshots, audio, or video | [Screenshots and recordings](docs/capture.md) |
| Inspect execution and hardware | [Debugger and Lua](docs/debugging.md) |
| Manage per-game cheat codes | [Cheats](docs/cheats.md) |
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
hardware suite, eleven region launch cases, CPU trace, all 91 diagnostic ROMs,
and AccuracyCoin. The Clang job enables AddressSanitizer,
UndefinedBehaviorSanitizer, and Linux leak detection.

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

The [desktop guide](docs/desktop.md) covers menus, settings, feature panels, and storage locations.
