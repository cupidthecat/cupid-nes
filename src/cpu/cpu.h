#ifndef CPU_H
#define CPU_H

#include <stdint.h>
#include "../joypad/joypad.h"

typedef struct { 
    uint8_t a;         // Accumulator
    uint8_t x;         // X register
    uint8_t y;         // Y register
    uint16_t pc;       // Program Counter
    uint8_t sp;        // Stack Pointer
    uint8_t status;    // Status register (NV-BDIZC)
    uint8_t irq_delay; // IRQ delay 
    uint8_t sei_delay; // SEI delay 
    int extra_cycles;  // extra cycles 
} CPU;

typedef enum {
    CARRY_FLAG     = 0x01,
    ZERO_FLAG      = 0x02,
    INTERRUPT_FLAG = 0x04,
    DECIMAL_FLAG   = 0x08,
    BREAK_FLAG     = 0x10,
    UNUSED_FLAG    = 0x20,  
    OVERFLOW_FLAG  = 0x40,
    NEGATIVE_FLAG  = 0x80,
    JSR_OPCODE     = 0x20,
    RTS_OPCODE     = 0x60
} StatusFlags;

extern uint8_t memory[0x10000];

// Add joypad extern declarations
extern Joypad pad1, pad2;
extern CPU cpu;

void cpu_reset(CPU* cpu);
uint8_t read_mem(uint16_t addr);
void write_mem(uint16_t addr, uint8_t value);
void execute(CPU* cpu, uint8_t opcode);
int cpu_step(CPU* cpu);
void cpu_nmi(CPU *cpu);
void cpu_irq(CPU *cpu);

#endif // CPU_H
