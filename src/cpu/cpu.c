#include <stdint.h>
#include <stdio.h>
#include "cpu.h"

uint8_t memory[0x10000]; // 64KB memory

void cpu_reset(CPU* cpu) {
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->sp = 0xFD;
    cpu->status = 0x24; // Default flag state
    cpu->pc = (memory[0xFFFD] << 8) | memory[0xFFFC]; // Reset vector
}

uint8_t read_mem(uint16_t addr) {
    return memory[addr];
}

void write_mem(uint16_t addr, uint8_t value) {
    memory[addr] = value;
}

void execute(CPU* cpu, uint8_t opcode) {
    switch(opcode) {
        case 0xA9: // LDA Immediate
            cpu->a = read_mem(cpu->pc++);
            cpu->status = (cpu->a == 0) ? ZERO_FLAG :
                          (cpu->a & NEGATIVE_FLAG) ? NEGATIVE_FLAG : 0;
            break;

        case 0x48: // PHA - Push Accumulator onto Stack
            write_mem(0x0100 + cpu->sp, cpu->a);
            cpu->sp--;  // Stack pointer decrements after push
            break;

        case 0x68: // PLA - Pull Accumulator from Stack
            cpu->sp++;  // Increment SP before pulling the value
            cpu->a = read_mem(0x0100 + cpu->sp);
            cpu->status = (cpu->a == 0) ? ZERO_FLAG :
                          (cpu->a & NEGATIVE_FLAG) ? NEGATIVE_FLAG : 0;
            break;

        case 0x08: // PHP - Push Processor Status onto Stack
            write_mem(0x0100 + cpu->sp, cpu->status);
            cpu->sp--; 
            break;

        case 0x28: // PLP - Pull Processor Status from Stack
            cpu->sp++; 
            cpu->status = read_mem(0x0100 + cpu->sp);
            break;

        case 0x00: // BRK - Break (halts execution)
            cpu->status |= BREAK_FLAG;
            break;

        case 0xEA: // NOP
            break;

        case JSR_OPCODE: // JSR - Jump to Subroutine
            {
                // Fetch operand bytes (little endian)
                uint8_t low = read_mem(cpu->pc);
                uint8_t high = read_mem(cpu->pc + 1);
                cpu->pc += 2;  // Advance PC past the operand bytes
                uint16_t target_addr = (high << 8) | low;
                
                // Push return address (PC - 1) onto the stack
                uint16_t return_addr = cpu->pc - 1;
                write_mem(0x0100 + cpu->sp, (return_addr >> 8) & 0xFF); // High byte
                cpu->sp--;
                write_mem(0x0100 + cpu->sp, return_addr & 0xFF);        // Low byte
                cpu->sp--;
                
                // Jump to subroutine
                cpu->pc = target_addr;
            }
            break;

        case RTS_OPCODE: // RTS - Return from Subroutine
            {
                // Pull return address from the stack
                cpu->sp++;
                uint8_t low = read_mem(0x0100 + cpu->sp);
                cpu->sp++;
                uint8_t high = read_mem(0x0100 + cpu->sp);
                uint16_t return_addr = (high << 8) | low;
                // Set PC to return address + 1
                cpu->pc = return_addr + 1;
            }
            break;

        default:
            printf("Unknown opcode: 0x%02X\n", opcode);
            break;
    }
}

void cpu_step(CPU* cpu) {
    uint8_t opcode = read_mem(cpu->pc++);
    execute(cpu, opcode);
}
