# Cupid NES Emulator

Cupid NES Emulator is a minimal NES emulator implemented in C. It is designed as a learning tool to explore the 6502 CPU architecture and basic NES hardware emulation. The project focuses on accurately emulating the CPU instruction set, various addressing modes, and simulating the NES memory map.

## Features

- **6502 CPU Emulation**  
  Implements a core subset of the 6502 instruction set including:
  - **Data Transfer:** LDA, STA, TAX, TXA, LDY, LDX  
  - **Arithmetic:** ADC, SBC  
  - **Logical:** AND, ORA, EOR, BIT  
  - **Comparison:** CMP, CPX, CPY  
  - **Branching:** BEQ, BNE, BCC, BCS, BMI, BPL, BVS, BVC  
  - **Jump/Call:** JMP (Absolute, Indirect), JSR/RTS  
  - **Stack Operations:** PHP/PLP, PHA/PLA  
  - **Shift/Rotate:** ASL, LSR, ROL, ROR  
  - **Other:** BRK and interrupt flag handling

- **Multiple Addressing Modes**  
  Supports:
  - Immediate, Zero Page, Zero Page, X
  - Absolute, Absolute, X, Absolute, Y
  - Indexed Indirect ((Indirect,X)) and Indirect Indexed ((Indirect),Y)
  - Absolute Indirect (for JMP)
  - Relative addressing for branch instructions

- **Memory Map Simulation**  
  - 2KB internal RAM with mirroring (0x0000–0x1FFF)
  - PPU registers (0x2000–0x3FFF, mirrored)
  - APU/I-O registers (0x4000–0x401F)
  - Program ROM (PRG-ROM) loaded into 0x8000–0xFFFF (read/write in our test environment)

- **ROM Loading**  
  Supports loading NES ROMs in the standard iNES file format. The loader extracts:
  - PRG-ROM data (16KB per bank)
  - CHR-ROM data (8KB per bank or CHR-RAM if no CHR-ROM is present)
  - Mirroring mode (Horizontal/Vertical) based on the header flags

- **Integrated Test Suite**  
  A comprehensive set of tests exercises every instruction and addressing mode. The tests print PASS/FAIL messages to the console.

- **Basic PPU Emulation & New PPU Enhancements**  
  - **Tile Rendering:**  
    Renders the first pattern table (CHR-ROM data) as a grid of 16×16 8×8 tiles for visualizing tile data from loaded ROMs.
  - **Sprite Zero Hit:**  
    A specific signal triggered when the first sprite (sprite zero) overlaps with the background. It’s used by some games for timing. The implementation will detect when pixels from sprite zero overlap with background pixels.
  - **Sprite Priority:**  
    Determines whether a sprite should be drawn in front of or behind the background based on its attributes and the background pixel values.
  - **OAM (Object Attribute Memory):**  
    Sprites (or “objects”) are defined in OAM. Each sprite entry includes the tile index, x and y position, attributes (such as palette selection, priority, and flipping), etc.

## Project Structure

```
cupid-nes/
├── Makefile           # Build configuration using gcc and SDL2
├── README.md          # Project documentation (this file)
└── src/
    ├── cpu/
    │   ├── cpu.c     # 6502 CPU implementation
    │   └── cpu.h     # CPU interface and flag definitions
    ├── ppu/
    │   ├── ppu.c     # Basic PPU functionality and tile rendering, including new sprite features
    │   └── ppu.h     # PPU interface
    ├── rom/
    │   ├── rom.c     # ROM loading and iNES header parsing
    │   └── rom.h     # ROM interface and iNES header structure
    └── main.c
```

## Requirements

- **Compiler:** GCC (or any C compiler that supports C99)
- **Operating System:** Linux or any POSIX‑compliant system
- **Build Tools:** Make
- **Libraries:** SDL2 (for graphical output)

## Installation & Usage

### Building the Emulator

From the root directory of the project, run:

```bash
make
```

This will compile all source files and create an executable named `cupid-nes`.

### Running the CPU Test Suite

To run the integrated CPU instruction tests (which verify each instruction's behavior):

```bash
./cupid-nes
```

The test suite prints PASS/FAIL messages for each instruction (e.g., "LDA Immediate: PASS"). These tests do not invoke the SDL display.

### Running in Graphical Mode

To run the emulator with graphical output (which displays tile data from CHR-ROM), pass the path to a ROM file as an argument:

```bash
./cupid-nes path/to/rom.nes
```

The emulator will:
- Load the ROM and print header information.
- Open an SDL window and display the first pattern table (tiles) from the ROM.

## Future Improvements

- **Cycle-Accurate Emulation:** Improve cycle counting.
- **Full PPU and APU Emulation:** Expand beyond basic tile rendering.
- **Mapper Support:** Implement support for additional mappers found in NES ROMs.
- **Debugging Tools:** Integrate an in-emulator debugger with breakpoints and memory viewing.

## Contributing

Contributions are welcome! To contribute:
1. Fork the repository.
2. Create a feature branch.
3. Submit a pull request with a detailed description of your changes.
4. Update tests and documentation as necessary.

## License

This project is open source under the GNU v3 License. See the [LICENSE](LICENSE) file for more details.

## Resources

- [NESdev Wiki](https://www.nesdev.org/wiki/Nintendo_Entertainment_System)
- [6502 CPU Documentation](http://www.6502.org/tutorials/6502opcodes.html)
- [SDL2 Documentation](https://wiki.libsdl.org/)
