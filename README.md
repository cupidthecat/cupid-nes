# Cupid NES Emulator

A minimal NES emulator implemented in C, designed as a learning tool for understanding the 6502 CPU architecture and NES hardware.

### New Features
- Implemented JSR (Jump to Subroutine) instruction
- Implemented RTS (Return from Subroutine) instruction
- Added subroutine test case

### Technical Details
#### JSR (0x20)
- Pushes the return address (current PC + 2) onto the stack
- Jumps to the specified subroutine address
- Stack behavior:
  - High byte pushed first
  - Low byte pushed second
  - Stack pointer decremented twice

#### RTS (0x60)
- Pulls the return address from the stack
- Increments the address by 1
- Jumps back to the return address
- Stack behavior:
  - Low byte pulled first
  - High byte pulled second
  - Stack pointer incremented twice

## Features

- Basic 6502 CPU emulation
- Support for fundamental instructions (LDA, BRK, NOP)
- Memory management with 64KB address space
- Simple test program execution
- Implemented JSR (Jump to Subroutine) instruction
- Implemented RTS (Return from Subroutine) instruction
- Added subroutine test case

## Project Structure

```
cupid-nes/
├── Makefile           # Build configuration
├── README.md          # Project documentation
└── src/
    └── cpu/
        └── cpu.c      # 6502 CPU implementation and test program
```

## Requirements

- **Compiler:** GCC or compatible C compiler
- **Environment:** Linux or POSIX-compliant system
- **Build Tools:** make

## Installation & Usage

### Building the Emulator

```bash
make
```

This will generate an executable named `cupid-nes`.

### Running the Emulator

```bash
./cupid-nes
```

Expected output:
```
CPU Test Results:
Accumulator: 0x42
Stack Pointer: 0xFD
Status Register: 0x24
Program Counter: 0x8007

Test Program:
0x8000: 0xA9
0x8001: 0x42
0x8002: 0x08
0x8003: 0x48
0x8004: 0xA9
0x8005: 0x00
0x8006: 0x68
0x8007: 0x28
0x8008: 0x00
```

## Technical Details

The emulator initializes with a test program that:
1. Sets the reset vector to address `0x8000`
2. Executes the following instructions:
   - `LDA #$42` - Load 0x42 into the accumulator
   - `PHP` - Push processor status (initially 0x24) onto stack
   - `PHA` - Push accumulator (0x42) onto stack
   - `LDA #$00` - Load 0x00 into accumulator (sets Zero flag)
   - `PLA` - Pull accumulator from stack (restores 0x42)
   - `PLP` - Pull processor status from stack (restores 0x24)
   - `BRK` - End program
3. The status register (NV-BDIZC) bits are:
   - N (Negative): Set if bit 7 of result is set
   - V (Overflow): Not implemented
   - B (Break): Set by BRK instruction
   - D (Decimal): Not implemented
   - I (Interrupt): Not implemented
   - Z (Zero): Set if result is zero
   - C (Carry): Not implemented

The CPU implementation includes:
- 6502 registers (A, X, Y, PC, SP, Status)
- Basic memory read/write operations
- Core instruction execution

## Roadmap

### Short-term Goals
- Expand CPU instruction set
- Implement additional addressing modes
- Add memory mapping support

### Long-term Vision
- PPU (Picture Processing Unit) implementation
- APU (Audio Processing Unit) support
- Input handling for controllers
- ROM loading capability
- Comprehensive test suite

## Contributing

Contributions are welcome! Please follow these guidelines:
1. Fork the repository
2. Create a feature branch
3. Submit a pull request with detailed description

## License

This project is open source under the MIT License. See [LICENSE](LICENSE) for details.

## Resources & References

- [NESdev Wiki](https://www.nesdev.org/wiki/Nintendo_Entertainment_System)
- [6502 CPU Reference](http://www.6502.org/tutorials/6502opcodes.html)
- [Open-source NES Emulators](https://github.com/topics/nes-emulator)

# NES Emulator TODO List

## Core CPU Features
- [ ] Implement all 6502 instructions
- [ ] Add support for all addressing modes
- [ ] Implement interrupt handling (NMI, IRQ, RESET)
- [ ] Add cycle-accurate timing
- [ ] Implement stack operations (PHA, PLA, PHP, PLP)

## Memory Management
- [ ] Implement memory mapping for NES hardware
- [ ] Add support for ROM loading
- [ ] Implement mirroring for RAM and PPU registers
- [ ] Add memory access timing

## Graphics (PPU)
- [ ] Implement PPU registers
- [ ] Add pattern table rendering
- [ ] Implement nametable and attribute table handling
- [ ] Add sprite rendering
- [ ] Implement background scrolling

## Audio (APU)
- [ ] Implement pulse wave channels
- [ ] Add triangle wave channel
- [ ] Implement noise channel
- [ ] Add DMC (Delta Modulation Channel)
- [ ] Implement audio mixing

## Input
- [ ] Add controller input handling
- [ ] Implement standard NES controller support
- [ ] Add support for Zapper (light gun)
- [ ] Implement input polling timing

## Testing & Debugging
- [ ] Add CPU test suite
- [ ] Create PPU test patterns
- [ ] Implement debugger interface
- [ ] Add memory viewer
- [ ] Create instruction tracer

## Optimization
- [ ] Profile CPU emulation
- [ ] Optimize PPU rendering
- [ ] Add JIT compilation option
- [ ] Implement multi-threading for PPU/APU

## User Interface
- [ ] Add ROM file loading
- [ ] Implement save state support
- [ ] Add configuration options
- [ ] Create debug visualization tools
- [ ] Implement frame stepping

## Documentation
- [ ] Write technical documentation
- [ ] Create architecture diagrams
- [ ] Add code comments
- [ ] Write user guide
