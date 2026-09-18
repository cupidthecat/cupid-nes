# Cupid documentation

Cupid has an SDL application for running games and a separate executable for testing the hardware core. Both use the same emulation code. Start with [getting started](getting-started.md) to build the application, or [development and testing](development.md) to investigate a failure.

## Using the emulator

| Guide | Contents |
| --- | --- |
| [Getting started](getting-started.md) | Linux and Windows builds, output paths, opening cartridges, disks, and VS images |
| [Configuration](configuration.md) | Every application command-line option, defaults, and valid combinations |
| [Controls and peripherals](controls.md) | Keyboard and controller mapping, multiplayer adapters, paddles, light guns, mats, and BASIC |
| [Hardware and compatibility](hardware.md) | Supported systems, mapper selection, ROM metadata, and implementation limits |
| [Saves and media](saves.md) | Cartridge save names, writable disk images, tape recordings, and backups |
| [Troubleshooting](troubleshooting.md) | Common error messages and the information needed for a reproducible report |

## Working on the core

| Guide | Contents |
| --- | --- |
| [Hardware terminology](glossary.md) | CPU, PPU, mapper, memory, bus, interrupt, DMA, and test terminology |
| [Architecture](architecture.md) | Source layout, bus timing, state ownership, dual systems, and audio threading |
| [Development and testing](development.md) | Build commands, pinned test data, diagnostic modes, and regression workflow |
| [Accuracy notes](accuracy.md) | Detailed CPU, PPU, APU, DMA, and cartridge behavior, test conventions, and limits |
| [Implementation checkpoints](accuracy-checkpoints.md) | Historical per-issue commits with hardware and full AccuracyCoin results |
| [Contributing](../CONTRIBUTING.md) | Patch scope, review expectations, test evidence, and documentation changes |
| [Credits and references](credits.md) | Component attribution, licenses, external tests, and hardware references |

Command examples assume the repository root as the working directory unless stated otherwise. Game paths are examples; supply your own files. The docs describe the code in the checkout containing them. When testing a pull request, use its head branch and include its commit in any report.

The [main README](../README.md) gives an overview; the [hardware guide](hardware.md#cartridge-mappers) contains the mapper table. [Support](../SUPPORT.md) explains where to ask a question or report a problem. Public C interfaces are in the headers beside their implementations; start with [CPU](../src/cpu/cpu.h), [PPU](../src/ppu/ppu.h), [APU](../src/apu/apu.h), and [cartridge](../src/rom/mapper.h).

## Keeping the guides current

Check application options against [main.c](../src/main.c), build instructions against the [Makefile](../Makefile) and [Windows script](../scripts/test-windows.ps1), and test pins against [the workflow](../.github/workflows/accuracy.yml). Update the relevant guide in the same pull request as a behavior change. Historical results remain attached to the commits that were tested.
