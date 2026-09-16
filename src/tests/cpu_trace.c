/* SPDX-License-Identifier: GPL-3.0-or-later
 * Compare a canonical instruction trace with the running CPU.
 */
#include <stdio.h>
#include <string.h>
#include "../cpu/cpu.h"
#include "../apu/apu.h"
#include "../ppu/ppu.h"
#include "../rom/rom.h"

extern uint8_t ram[0x0800];

// Run in a test process: loading the ROM replaces the active cartridge and CPU state.
int test_cpu_trace(const char *rom_path, const char *trace_path) {
    if (!rom_path || !trace_path) {
        fprintf(stderr, "CPU trace: ROM and trace paths are required\n");
        return 1;
    }
    FILE *trace = fopen(trace_path, "rb");
    if (!trace) {
        perror("CPU trace");
        return 1;
    }
    if (load_rom(rom_path) != 0) {
        fclose(trace);
        return 1;
    }

    // The automated entry starts at $C000 immediately after the seven-cycle
    // CPU power-on reset sequence.
    memset(&cpu, 0, sizeof(cpu));
    memset(ram, 0, 0x0800);
    memset(&pad1, 0, sizeof(pad1));
    memset(&pad2, 0, sizeof(pad2));
    cpu_total_cycles = 0;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cpu.pc = 0xC000;

    char line[512], previous[512] = "";
    unsigned long lines = 0;
    bool seen[256] = {0};
    while (fgets(line, sizeof(line), trace)) {
        unsigned pc, opcode, a, x, y, p, sp;
        unsigned long long cycles;
        const char *registers = strstr(line, "A:");
        const char *timing = strstr(line, "CYC:");
        ++lines;
        if (!registers || !timing || sscanf(line, "%x %x", &pc, &opcode) != 2 ||
            sscanf(registers, "A:%x X:%x Y:%x P:%x SP:%x", &a, &x, &y, &p, &sp) != 5 ||
            sscanf(timing, "CYC:%llu", &cycles) != 1 || opcode > 0xFF || pc > 0xFFFF ||
            a > 0xFF || x > 0xFF || y > 0xFF || p > 0xFF || sp > 0xFF) {
            fprintf(stderr, "CPU trace: malformed entry at line %lu\n", lines);
            fclose(trace);
            return 1;
        }
        if (cpu.pc != pc || cpu.a != a || cpu.x != x || cpu.y != y ||
            cpu.status != p || cpu.sp != sp || cpu_total_cycles != cycles) {
            fprintf(stderr, "CPU trace mismatch at line %lu\nPrevious: %sExpected: %s"
                    "Actual: PC:%04X A:%02X X:%02X Y:%02X P:%02X SP:%02X CYC:%llu\n",
                    lines, previous, line, cpu.pc, cpu.a, cpu.x, cpu.y, cpu.status, cpu.sp,
                    (unsigned long long)cpu_total_cycles);
            fclose(trace);
            return 1;
        }
        seen[opcode] = true;
        memcpy(previous, line, strlen(line) + 1);
        cpu_step(&cpu);
    }
    int read_error = ferror(trace);
    fclose(trace);
    if (read_error || lines == 0) {
        fprintf(stderr, "CPU trace: %s\n", read_error ? "read failed" : "empty trace");
        return 1;
    }
    if (ram[2] != 0 || ram[3] != 0) {
        fprintf(stderr, "CPU trace: diagnostic result $02/$03=%02X/%02X\n", ram[2], ram[3]);
        return 1;
    }
    unsigned unique = 0;
    for (unsigned i = 0; i < 256; ++i) unique += seen[i];
    printf("CPU canonical trace: PASS (%lu states, %u unique opcodes, $02/$03=00/00)\n", lines, unique);
    printf("Opcodes absent from this trace:");
    for (unsigned i = 0; i < 256; ++i) if (!seen[i]) printf(" %02X", i);
    puts("");
    return 0;
}
