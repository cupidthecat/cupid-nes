# Hardware terminology

[Project README](../README.md)

This guide explains the terms used in Cupid's source, startup logs, and accuracy reports.

## The emulated machine

| Term | Meaning in Cupid |
| --- | --- |
| CPU | The processor that executes instructions and accesses devices through its bus. See [the CPU core](../src/cpu/cpu.c). |
| PPU | The picture processing unit. It fetches background and sprite data and produces the framebuffer. See [the PPU core](../src/ppu/ppu.c). |
| APU | The audio processing unit, including pulse, triangle, noise, and DMC channels. See [the APU core](../src/apu/apu.c). |
| EPSM | Optional expansion sound selected by NES 2.0 extended console subtype 4. Cupid models an 8 MHz YMF288 with stereo output, timer IRQs, and optional 8 KiB percussion ROM data. See [the EPSM interface](../src/apu/epsm.h). |
| Mapper | Cartridge hardware that controls address decoding, bank selection, and optional board devices. A mapper number identifies the board family. |
| Submapper | An NES 2.0 field distinguishing wiring or revisions within a mapper family. Not every submapper is implemented. |
| Frontend | The SDL application in [src/main.c](../src/main.c), which handles input, display, audio-device setup, and shutdown. |
| Core | The emulation code shared by the application and the hardware test executable. |

Timing region and controller wiring are separate properties. The ROM header selects the [timing profile](../src/system/timing.c); `--console famicom` selects [controller-port wiring](../src/system/hardware.c).

## Memory

| Term | Meaning |
| --- | --- |
| PRG ROM | Program data mapped into CPU address windows. |
| CHR ROM | Stored pattern data read by the PPU. |
| CHR RAM | Writable pattern memory into which a game can load tiles. |
| Work RAM | Volatile memory used while the machine runs. |
| NVRAM | Nonvolatile memory backed by save files for supported cartridge layouts. |
| Bank | A chunk of memory selected for an address window. |
| Nametable | Background tile-layout data addressed by the PPU. |
| Mirroring | Multiple addresses reaching the same backing memory or register. Nametable mirroring describes how logical pages address their backing memory. |
| Trainer | An optional 512-byte image block copied to the supported cartridge RAM window at `$7000-$71FF` after persistent memory is loaded. It does not create RAM when the board has no suitable cartridge RAM. |

CPU and PPU addresses belong to separate buses. CPU `$0000` is internal RAM; PPU `$0000` is pattern space. `$2002` is hexadecimal notation, equivalent to `0x2002` in C.

The [mapper interface](../src/rom/mapper.h) separates CPU access, PPU pattern access, and nametable access. A bank write changes which memory a later access reaches. It does not copy an entire ROM bank into CPU RAM.

## Bus behavior and timing

An open-bus read occurs when a device drives only some data bits or none of them. Cupid combines driven bits with retained bus values. The CPU keeps separate internal and external data-bus latches, and the PPU has its own open-bus state. Cartridge and write-only I/O reads use the CPU's external latch when nothing drives the bus; `$4015`, for example, gets floating bit 5 from the CPU's internal latch. See `read_bus_target()` in [cpu.c](../src/cpu/cpu.c).

A bus conflict occurs when cartridge ROM output affects a CPU write to a bank register. It depends on board wiring. Dummy accesses are instruction reads or writes whose data is not used as the final result; they still consume cycles and can affect devices.

| Term | Meaning |
| --- | --- |
| PPU dot | One PPU clock position within a scanline, including clocks used for internal work. |
| Scanline | One row of PPU timing. Frames include visible scanlines and blanking intervals. |
| Vblank | The nonvisible vertical-blank interval represented by the PPU's status and NMI behavior. |
| IRQ | A maskable interrupt request, including APU, cartridge, EPSM, and peer VS sources. |
| NMI | A nonmaskable interrupt input affected by PPU vblank and control-register state. |
| OAM | Object attribute memory containing sprite attributes. |
| OAM DMA | A transfer requested through `$4014` that copies a 256-byte CPU page into sprite memory. |
| DMC DMA | A memory fetch for the APU sample channel, sharing the CPU bus with other work. |
| DAC/output latch | The last digital level a sound channel drives toward the mixer. Pulse and noise update their stored output on channel edges, pulse-register writes refresh it immediately, and the triangle retains its last level while its sequencer is stopped. |

A pending interrupt, its status bit, and entry into a CPU interrupt handler are distinct events. Their ordering is part of the timing model.

`cpu_step()` returns time already spent clocking the CPU and connected devices, including DMA. Callers must not clock the PPU again using its return value. Power-on and soft reset are also distinct: `cpu_reset()` is the power-on compatibility entry point, while the frontend's R key soft-resets the CPU, PPU, APU, and VS peer state without calling the cartridge mapper's reset callback. Mapper and cartridge RAM state therefore remain live across that frontend soft reset unless the hardware changes them through normal bus activity. The [accuracy notes](accuracy.md) describe the reset and bus sequences.

## Test terminology

A hardware test group contains related cases; its count is not a count of supported mapper numbers. A canonical CPU trace checks recorded states. Its opcode count does not exhaust all operands or interrupt alignments.

Diagnostic cartridges report results through particular memory conventions. A timeout or absent result signature is a failure to obtain a passing result. Some legacy diagnostics need explicit region or RAM setup, which must accompany their reported results.

AccuracyCoin is the separate 144-test cartridge used for the regression baseline. A complete pass requires 144/144 with no skipped or unfinished tests and agreement with the cartridge's tally. It does not test every mapper or peripheral.

A pinned revision identifies test sources, a ROM SHA-256 identifies the tested bytes, and the emulator commit identifies the code under test. An older passing commit does not validate subsequent changes. The [checkpoint record](accuracy-checkpoints.md) preserves historical results; [CI](../.github/workflows/accuracy.yml) checks the branch being built.
