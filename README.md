# Cupid NES Emulator

Cupid NES Emulator is a minimal NES emulator implemented in C. It is designed as a learning tool to explore the 6502 CPU architecture and basic NES hardware emulation. The project focuses on accurately emulating the CPU instruction set, various addressing modes, and simulating the NES memory map.

## 🎮 Major Milestone: Donkey Kong Playable!

**BREAKTHROUGH:** The emulator can now successfully run and play Donkey Kong! This represents a major achievement in NES emulation, demonstrating that the core CPU, PPU, and memory systems are working correctly enough to run one of the most iconic NES games.

![Donkey Kong Gameplay 1](img/donkykong1.png)
![Donkey Kong Gameplay 2](img/donkykong2.png)

This milestone validates:
- ✅ Complete 6502 CPU instruction set implementation
- ✅ Proper PPU background and sprite rendering
- ✅ Accurate memory mapping and ROM loading
- ✅ Working joypad input system
- ✅ Correct timing and interrupt handling
- ✅ Full NES hardware compatibility for classic games

## Features

### 6502 CPU Emulation

Implements a comprehensive 6502 instruction set including:

- **Data Transfer:** LDA, STA, TAX, TXA, LDY, LDX, STX, STY, LAX (undocumented)
- **Arithmetic:** ADC, SBC, INC, DEC, INX, INY, DEX, DEY
- **Logical:** AND, ORA, EOR, BIT
- **Comparison:** CMP, CPX, CPY
- **Branching:** BEQ, BNE, BCC, BCS, BMI, BPL, BVS, BVC
- **Jump/Call:** JMP (Absolute, Indirect), JSR/RTS, RTI
- **Stack Operations:** PHP/PLP, PHA/PLA, TSX/TXS
- **Shift/Rotate:** ASL, LSR, ROL, ROR
- **Undocumented Instructions:** DCP, RLA, SLO, SRE, RRA, SAX, ISC, SHX, SHY, SHA, TAS, LAS, and various undocumented NOPs
- **Control Flow:** BRK, NMI/IRQ interrupt handling with accurate timing
- **Interrupt Latency:** Proper CLI/SEI/PLP timing windows for accurate interrupt behavior

### Multiple Addressing Modes

Supports all major 6502 addressing modes:

- Immediate
- Zero Page, Zero Page,X, Zero Page,Y
- Absolute, Absolute,X, Absolute,Y
- Indexed Indirect ((Indirect,X)) and Indirect Indexed ((Indirect),Y)
- Absolute Indirect (for JMP)
- Relative addressing for branch instructions
- Proper 6502 page-boundary crossing behavior and dummy reads

### Memory Map Simulation

- 2KB internal RAM with mirroring (0x0000–0x1FFF)
- PPU registers (0x2000–0x3FFF, mirrored every 8 bytes)
- APU/I-O registers (0x4000–0x401F)
- Joypad input (0x4016, 0x4017)
- PRG-RAM (0x6000–0x7FFF, 8KB)
- Program ROM (PRG-ROM) loaded into 0x8000–0xFFFF

### ROM Loading

Supports loading NES ROMs in the standard iNES file format. The loader extracts:

- PRG-ROM data (16KB per bank)
- CHR-ROM data (8KB per bank) or CHR-RAM if no CHR-ROM is present
- Mirroring mode (Horizontal/Vertical/Single-Screen/Four-Screen) based on header flags
- Automatic PRG-ROM mirroring for single-bank ROMs (NROM)
- Mapper identification and initialization

### PPU Emulation

- **VRAM and OAM:** Full PPU memory spaces with proper mirroring
- **PPU Registers:** Complete implementation of $2000–$2007 with proper scroll and address handling
- **OAM DMA:** Full 256-byte DMA transfer via $4014
- **Background Rendering:** Renders full 256×240 background using nametables, attribute tables, and pattern tables
- **Sprite Rendering:** Renders up to 64 sprites from OAM with proper palette lookup
- **VBlank Interrupt:** NMI generation at VBlank based on PPUCTRL bit 7
- **Palette Support:** Full 64-color NES palette with palette mirroring
- **Nametable Mirroring:** Configurable horizontal/vertical/single-screen mirroring modes
- **Scrolling:** Proper PPUSCROLL handling with fine X/Y and coarse coordinate management
- **Cycle Timing:** Approximate NTSC timing with VBlank scheduling

### Joypad Input

- Full controller support (A, B, Select, Start, Up, Down, Left, Right)
- Proper strobe and shift register behavior matching NES hardware
- SDL keyboard mapping for intuitive control

### Mapper Support

The emulator includes support for multiple memory mappers, enabling compatibility with a wide range of NES games:

- **Mapper 0 (NROM):** Basic mapper with 16KB or 32KB PRG-ROM
- **Mapper 1 (MMC1/SxROM):** Configurable PRG/CHR banking with programmable mirroring
- **Mapper 2 (UxROM):** Switchable 16KB PRG banks with fixed upper bank
- **Mapper 3 (CNROM):** Simple CHR-ROM banking
- **Mapper 4 (MMC3/TxROM):** Advanced mapper with IRQ support (partial)
- **Mapper 5 (MMC5/ExROM):** Simplified implementation for expansion audio games
- **Mapper 7 (AOROM):** 32KB PRG banking with single-screen mirroring control
- **Mapper 9 (MMC2/PxROM):** Latch-based CHR switching (Punch-Out!!)
- **Mapper 10 (MMC4/FxROM):** Similar to MMC2 with extended banking
- **Mapper 11 (Color Dreams):** Simple PRG and CHR banking
- **Mapper 13 (CPROM):** CHR-RAM banking (Videomation)
- **Mapper 15 (100-in-1):** Multi-mode mapper for multicarts

### APU (Audio Processing Unit) Emulation

Full implementation of the NES APU with all sound channels:

- **Pulse Channels (2):** Duty cycle control, envelope, sweep, and length counter
- **Triangle Channel:** Linear counter and frequency control
- **Noise Channel:** Configurable period and mode (15-bit/7-bit LFSR)
- **Frame Sequencer:** Both 4-step and 5-step modes with IRQ support
- **Audio Output:** Real-time audio playback via SDL2 at 44.1kHz
- **Accurate Timing:** Envelope, sweep, and length counter clocking synced to CPU cycles
- **Nonlinear Mixing:** Proper NES audio mixing formula for authentic sound

### Graphical Output

- Real-time rendering via SDL2
- Display of background and sprite layers with proper priority
- 2x pixel scaling for better visibility
- Color emphasis support (red, green, blue tinting via PPUMASK)

## Test Results

The emulator has been tested with various NES test ROMs to verify correct implementation of CPU instructions, PPU functionality, and overall emulation accuracy.

### CPU Tests

**Nestest Results**
![NES Test 1](img/nes_test1.png)
![NES Test 2](img/nes_test2.png)

The nestest ROM validates CPU instruction execution, addressing modes, and flag behavior. Both test screens show successful completion with all tests passing.

**CPU Execution Space Tests**
![CPU Execution Space PPU I/O](img/test_cpu_exec_space_ppuio.png)

This test verifies proper CPU execution in PPU I/O space, ensuring correct memory mapping and register behavior.

### PPU and Graphics Tests

**Color Test**
![Color Test](img/color_test.png)

The color test validates the PPU's palette rendering capabilities, showing proper color output and palette management.

**CLI Latency Test**
![CLI Latency](img/1-cli_latency.png)

This test verifies interrupt handling and timing accuracy, particularly for CLI (Clear Interrupt Disable) instruction behavior.

### Test ROMs Included

The project includes several test ROMs for validation:
- `nestest.nes` - Comprehensive CPU instruction testing
- `1-cli_latency.nes` - Interrupt timing validation
- `color_test.nes` - PPU color and palette testing
- `test_cpu_exec_space_ppuio.nes` - CPU execution in PPU I/O space
- `test_cpu_exec_space_apu.nes` - CPU execution in APU space

All tests demonstrate successful emulation of the NES hardware components.

## Project Structure

```
cupid-nes/
├── Makefile                 # Build configuration using gcc and SDL2
├── README.md                # Project documentation
├── include/
│   └── globals.h            # Global constants and shared definitions
└── src/
    ├── main.c               # Main emulation loop and SDL setup
    ├── cpu/
    │   ├── cpu.c            # 6502 CPU implementation with full instruction set
    │   └── cpu.h            # CPU interface and status flag definitions
    ├── ppu/
    │   ├── ppu.c            # PPU functionality, rendering, mirroring, and VBlank
    │   └── ppu.h            # PPU interface
    ├── apu/
    │   ├── apu.c            # APU sound generation and channel management
    │   └── apu.h            # APU interface and data structures
    ├── rom/
    │   ├── rom.c            # ROM loading and iNES header parsing
    │   ├── rom.h            # ROM interface and iNES header structure
    │   ├── mapper.c         # Mapper implementations (0-15)
    │   └── mapper.h         # Mapper interface and abstractions
    └── joypad/
        ├── joypad.c         # Joypad controller state management
        └── joypad.h         # Joypad interface
```

## Requirements

- **Compiler:** GCC (or any C compiler that supports C99)
- **Operating System:** Linux or any POSIX-compliant system
- **Build Tools:** Make
- **Libraries:** SDL2 (for graphical output)

## Installation & Usage

### Building the Emulator

From the root directory of the project, run:

```bash
make
```

This will compile all source files and create an executable named `cupid-nes`.

### Running with a ROM

To load and run an NES ROM:

```bash
./cupid-nes path/to/rom.nes
```

The emulator will:
- Load the ROM and print header information
- Initialize the CPU and PPU
- Open an SDL window and begin execution
- Display background and sprite graphics in real time

### Keyboard Controls

- **Z** – A button
- **X** – B button
- **Right Shift** – Select button
- **Enter** – Start button
- **Arrow Keys** – D-Pad (Up, Down, Left, Right)

## Emulation Details

### CPU Cycle Timing

The emulator approximates NTSC NES timing:

- CPU frequency: 1,789,773 Hz
- Frame rate: 60 Hz
- CPU cycles per frame: ~29,796
- Frames include a VBlank period where the PPU stops rendering

### Addressing Mode Implementation

The CPU includes proper 6502 quirks:

- Page-boundary crossing with dummy reads on indexed addressing modes
- 6502 indirect addressing bug in JMP (Absolute Indirect)
- Stack pointer wrapping at 0x00/0xFF
- Proper flag updates and preserved carry behavior in shift/rotate operations

### PPU Memory Management

- Nametable mirroring respects ROM configuration (horizontal/vertical/single-screen/four-screen)
- Palette RAM with special $3F10/$3F14/$3F18/$3F1C mirroring to $3F00
- Proper VRAM address latching for $2005 (scroll) and $2006 (address) writes
- Read buffer behavior for $2007 (palette reads bypass buffer)
- Open bus behavior for unused register reads
- Sprite priority handling (background vs foreground)

## Limitations and Future Improvements

- **Mapper Support:** Currently supports mappers 0-4, 7, 9-11, 13, and 15. Additional mappers would expand ROM compatibility
- **Cycle Accuracy:** Current timing is approximate; true cycle-accurate emulation would improve compatibility
- **APU Features:** DMC (sample playback) channel is stubbed out and not fully implemented
- **MMC3 IRQ:** Mapper 4 IRQ support is implemented but may need refinement for some games
- **Debugging Tools:** An integrated debugger with breakpoints and memory inspection would aid development
- **Performance:** Further optimization for high-resolution or high-speed rendering
- **Save States:** Not yet implemented

## Contributing

Contributions are welcome! To contribute:

1. Fork the repository
2. Create a feature branch for your changes
3. Test your implementation thoroughly
4. Submit a pull request with a detailed description
5. Update documentation as needed

## License

This project is open source under the GNU v3 License. See the [LICENSE](LICENSE) file for more details.

## Resources

- [NESdev Wiki](https://www.nesdev.org/wiki/Nintendo_Entertainment_System)
- [6502 CPU Documentation](http://www.6502.org/tutorials/6502opcodes.html)
- [SDL2 Documentation](https://wiki.libsdl.org/)
- [NES PPU Documentation](https://www.nesdev.org/wiki/PPU)
- [NES APU Documentation](https://www.nesdev.org/wiki/APU)
- [Undocumented 6502 Instructions](https://www.nesdev.org/undocumented_opcodes)
