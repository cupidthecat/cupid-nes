#include <stdio.h>
#include "../cpu/cpu.h"

void run_cpu_test() {
    CPU cpu;
    
    // Test Program:
    // JSR $8004     ; 0x20 0x04 0x80 => Jump to subroutine at 0x8004
    // BRK           ; 0x00         => End main program
    //
    // Subroutine at 0x8004:
    // LDA #$42      ; 0xA9 0x42    => Load 0x42 into accumulator
    // RTS           ; 0x60         => Return from subroutine
    uint8_t test_program[] = {
        0x20, 0x04, 0x80, // JSR $8004
        0x00,            // BRK (end main program)
        0xA9, 0x42,      // LDA #$42 (subroutine starts here at 0x8004)
        0x60             // RTS
    };
    
    // Load test program into memory starting at 0x8000
    for (size_t i = 0; i < sizeof(test_program); i++) {
        write_mem(0x8000 + i, test_program[i]);
    }
    
    // Set reset vector to point to our test program
    write_mem(0xFFFC, 0x00);
    write_mem(0xFFFD, 0x80);
    
    // Reset CPU
    cpu_reset(&cpu);
    
    // Execute test program until BRK is encountered
    while (!(cpu.status & BREAK_FLAG)) {
        cpu_step(&cpu);
    }
    
    // Print test results
    printf("CPU Test Results:\n");
    printf("Accumulator: 0x%02X\n", cpu.a);
    printf("Stack Pointer: 0x%02X\n", cpu.sp);
    printf("Status Register: 0x%02X\n", cpu.status);
    printf("Program Counter: 0x%04X\n", cpu.pc);
    
    printf("\nTest Program:\n");
    for (size_t i = 0; i < sizeof(test_program); i++) {
        printf("0x%04lX: 0x%02X\n", (unsigned long)(0x8000 + i), test_program[i]);
    }
}

int main() {
    run_cpu_test();
    return 0;
}
