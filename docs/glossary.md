# Hardware terminology

[Project README](../README.md)

This guide explains the terms used in Cupid's source, startup logs, and accuracy reports.

## The emulated machine

| Term | Meaning in Cupid |
| --- | --- |
| CPU | The processor that executes instructions and accesses devices through its bus. See [the CPU core](../src/cpu/cpu.c). |
| PPU | The picture processing unit. It fetches background and sprite data and produces the framebuffer. See [the PPU core](../src/ppu/ppu.c). |
| APU | The audio processing unit, including pulse, triangle, noise, and DMC channels. See [the APU core](../src/apu/apu.c). |
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
| Trainer | An optional 512-byte image block initialized into the cartridge RAM window after persistent memory is loaded. |

CPU and PPU addresses belong to separate buses. CPU `$0000` is internal RAM; PPU `$0000` is pattern space. `$2002` is hexadecimal notation, equivalent to `0x2002` in C.

The [mapper interface](../src/rom/mapper.h) separates CPU access, PPU pattern access, and nametable access. A bank write changes which memory a later access reaches. It does not copy an entire ROM bank into CPU RAM.

## Bus behavior and timing

An open-bus read occurs when a device does not drive every data bit. Cupid combines driven bits with the appropriate retained bus value. CPU and PPU latches are separate; `$4015`, for example, gets floating bit 5 from the CPU's internal latch. See `read_bus_target()` in [cpu.c](../src/cpu/cpu.c).

A bus conflict occurs when cartridge ROM output affects a CPU write to a bank register. It depends on board wiring. Dummy accesses are instruction reads or writes whose data is not used as the final result; they still consume cycles and can affect devices.

| Term | Meaning |
| --- | --- |
| PPU dot | One PPU clock position within a scanline, including clocks used for internal work. |
| Scanline | One row of PPU timing. Frames include visible scanlines and blanking intervals. |
| Vblank | The nonvisible vertical-blank interval represented by the PPU's status and NMI behavior. |
| IRQ | A maskable interrupt request, including APU, cartridge, and peer VS sources. |
| NMI | A nonmaskable interrupt input affected by PPU vblank and control-register state. |
| OAM | Object attribute memory containing sprite attributes. |
| OAM DMA | A transfer requested through `$4014` that copies a 256-byte CPU page into sprite memory. |
| DMC DMA | A memory fetch for the APU sample channel, sharing the CPU bus with other work. |

A pending interrupt, its status bit, and entry into a CPU interrupt handler are distinct events. Their ordering is part of the timing model.

`cpu_step()` returns time already spent clocking the CPU and connected devices, including DMA. Callers must not clock the PPU again using its return value. Power-on and soft reset are also distinct: `cpu_reset()` is the power-on compatibility entry point, while the frontend's R key invokes explicit soft-reset functions. The [accuracy notes](accuracy.md) describe the reset and bus sequences.

## Test terminology

A hardware test group contains related cases; its count is not a count of supported mapper numbers. A canonical CPU trace checks recorded states. Its opcode count does not exhaust all operands or interrupt alignments.

Diagnostic cartridges report results through particular memory conventions. A timeout or absent result signature is a failure to obtain a passing result. Some legacy diagnostics need explicit region or RAM setup, which must accompany their reported results.

AccuracyCoin is the separate 144-test cartridge used for the regression baseline. A complete pass requires 144/144 with no skipped or unfinished tests and agreement with the cartridge's tally. It does not test every mapper or peripheral.

A pinned revision identifies test sources, a ROM SHA-256 identifies the tested bytes, and the emulator commit identifies the code under test. An older passing commit does not validate subsequent changes. The [checkpoint record](accuracy-checkpoints.md) preserves historical results; [CI](../.github/workflows/accuracy.yml) checks the branch being built.
