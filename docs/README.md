# Cupid documentation

Cupid has an SDL application for running games and music and a separate executable for testing the hardware core. Both link the same C11 device code and C++17 cartridge and expansion-sound implementations. Start with [getting started](getting-started.md) to build the application, or [development and testing](development.md) to investigate a failure.

## Using the emulator

| Guide | Contents |
| --- | --- |
| [Desktop interface](desktop.md) | Menus, settings, feature panels, storage paths, and rendered examples |
| [Getting started](getting-started.md) | Linux and Windows builds, output paths, opening cartridges, disks, and VS images |
| [Configuration](configuration.md) | Application options, device selection, startup phases, diagnostic reads, EPSM firmware, and valid combinations |
| [CPU overclock](overclock.md) | Optional blank scanlines, sound compatibility, and state behavior |
| [Controls and peripherals](controls.md) | Gamepads, adapters, paddles, light guns, mats, keyboards, Subor mouse, Hori Track, Oeka tablet, and specialty expansion controls |
| [Hardware and compatibility](hardware.md) | Supported systems and mappers, RAM/register behavior, expansion sound, ROM metadata, and implementation limits |
| [Saves and media](saves.md) | Cartridge saves, EEPROM and flash, Turbo File/BattleBox storage, disk images, tapes, and save failures |
| [States and replay](replay.md) | State slots and files, rewind, run-ahead, deterministic input movies, and isolated save ownership |
| [TAS movies and editing](tas.md) | FM2/FM3 playback, CTAS projects, input editing, branches, recording, seeking, and Lua editing |
| [Movie preferences](movie-preferences.md) | Playback endings, recording defaults, subtitles, overlays, and backup copies |
| [Family BASIC keyboard](keyboard.md) | Mouse, touch, physical-key feedback, and latched keys |
| [Cheat database](cheat-database.md) | Game identity matching, code previews, built-in entries, and custom catalogs |
| [Cartridge header editor](header-editor.md) | Inspect and save a validated copy of iNES and NES 2.0 headers |
| [Presentation tools](presentation-tools.md) | Shader presets, HD drafts, frame timing, history viewing, and native audio backends |
| [Session tools](session-tools.md) | Unloading games, automatic resume, state recording, game settings, and updates |
| [Debugger tools](debugger-tools.md) | Code coverage, profiles, event traces, symbols, source lines, and text hooks |
| [Netplay](netplay.md) | Host and guest setup, slot assignment, synchronization, and disconnect behavior |
| [HD packs](hd-packs.md) | Replacement graphics and audio, installation, selection, export, and capture |
| [Music player](music.md) | NSF/NSFe track selection, pause and stop, metadata, fades, silence detection, repeat, and shuffle |
| [Screenshots and recordings](capture.md) | PNG screenshots, stereo WAV and AVI recording, capture timing, output paths, and file limits |
| [Debugger and Lua](debugging.md) | Breakpoints, stepping, side-effect-free hardware inspection, Lua callbacks, and overlays |
| [Cheat codes](cheats.md) | Supported code formats, compare behavior, saved per-game lists, and deterministic-session policy |
| [Troubleshooting](troubleshooting.md) | Common error messages and the information needed for a reproducible report |

## Working on the core

| Guide | Contents |
| --- | --- |
| [Hardware terminology](glossary.md) | CPU, PPU, mapper, memory, bus, interrupt, DMA, sound, peripheral, and test terminology |
| [Architecture](architecture.md) | Source layout, bus timing, state ownership, reset behavior, dual systems, DAC latches, and audio threading |
| [Development and testing](development.md) | Build commands, pinned test data, diagnostic modes, and regression workflow |
| [Accuracy notes](accuracy.md) | Detailed CPU, PPU, APU, DMA, and cartridge behavior, test conventions, and limits |
| [Implementation checkpoints](accuracy-checkpoints.md) | Tested issue and review commits, full AccuracyCoin results, and combined CI evidence |
| [Cartridge and media checkpoints](cartridge-checkpoints.md) | Issue #77 onward: geometry, cartridge families, music, media, startup behavior, and controllers |
| [Contributing](../CONTRIBUTING.md) | Patch scope, review expectations, test evidence, and documentation changes |
| [Credits and references](credits.md) | Component attribution, licenses, external tests, and hardware references |

Command examples assume the repository root as the working directory unless stated otherwise. Game paths are examples; supply your own files. The docs describe the code in the checkout containing them. When testing a pull request, use its head branch and include its commit in any report.

The [main README](../README.md) gives an overview; the [hardware guide](hardware.md#cartridge-mappers) contains the mapper table. [Support](../SUPPORT.md) explains where to ask a question or report a problem. Public interfaces are in the headers beside their implementations; start with [CPU](../src/cpu/cpu.h), [PPU](../src/ppu/ppu.h), [APU](../src/apu/apu.h), [cartridge](../src/rom/mapper.h), [controllers](../src/joypad/joypad.h), and the [EPSM C interface](../src/apu/epsm.h).

## Keeping the guides current

Check application options against [main.c](../src/main.c), build instructions against the [Makefile](../Makefile) and [Windows script](../scripts/test-windows.ps1), and test pins against [the workflow](../.github/workflows/accuracy.yml). Update the relevant guide in the same pull request as a behavior change. Historical results remain attached to the commits that were tested.
