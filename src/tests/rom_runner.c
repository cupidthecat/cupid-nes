/* SPDX-License-Identifier: GPL-3.0-or-later
 * Bounded diagnostic-ROM runner. A timeout is never treated as a pass.
 */
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../apu/apu.h"
#include "../rom/rom.h"
#include "../rom/mapper.h"
#include "../system/timing.h"
#include "../../include/globals.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern uint8_t ram[0x800];

typedef enum {
    DIAGNOSTIC_NORMAL, DIAGNOSTIC_MMC3, DIAGNOSTIC_LEGACY_PAL, DIAGNOSTIC_LEGACY
} DiagnosticMode;

static int load_test_image(const char *path, DiagnosticMode mode) {
    FILE *file = fopen(path, "rb");
    if (!file) {
        perror(path);
        return 1;
    }
    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        return 1;
    }
    long length = ftell(file);
    if (length < 16 || fseek(file, 0, SEEK_SET) != 0) {
        fclose(file);
        fprintf(stderr, "Invalid diagnostic image: %s\n", path);
        return 1;
    }
    uint8_t *image = malloc((size_t)length);
    if (!image) {
        fclose(file);
        return 1;
    }
    size_t bytes = fread(image, 1, (size_t)length, file);
    fclose(file);
    int result = bytes == (size_t)length ? load_rom_memory(image, bytes) : -1;
    free(image);
    if (result < 0) return 1;
    if (mode == DIAGNOSTIC_LEGACY_PAL || mode == DIAGNOSTIC_LEGACY) {
        if (rom_mapper_number(&ines_header) != 0) {
            fprintf(stderr, "Legacy diagnostics require an NROM image\n");
            return 2;
        }
        if (mode == DIAGNOSTIC_LEGACY_PAL) nes_set_region(NES_REGION_PAL);
        printf("Diagnostic setup for %s: %s timing; legacy $F8 result with a terminal CPU loop\n",
               path, mode == DIAGNOSTIC_LEGACY_PAL ? "explicit PAL" : "ROM-header");
    }
    memset(ram, 0, 0x800);
    memset(&pad1, 0, sizeof(pad1));
    memset(&pad2, 0, sizeof(pad2));
    cpu_total_cycles = 0;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    ppu_begin_frame_render(framebuffer);
    return 0;
}

static bool has_status_protocol(void) {
    return cart_cpu_read(0x6001) == 0xDE && cart_cpu_read(0x6002) == 0xB0
        && cart_cpu_read(0x6003) == 0x61;
}

static void print_test_output(void) {
    for (uint16_t address = 0x6004; address < 0x8000; ++address) {
        uint8_t value = cart_cpu_read(address);
        if (!value) break;
        putchar(value >= 0x20 && value <= 0x7E ? value : (value == '\n' ? '\n' : '?'));
    }
    putchar('\n');
}

static int write_frame(const char *path) {
    FILE *file = fopen(path, "wb");
    if (!file) {
        perror(path);
        return 1;
    }
    fprintf(file, "P6\n256 240\n255\n");
    for (unsigned i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; ++i) {
        uint32_t pixel = framebuffer[i];
        fputc((int)((pixel >> 16) & 0xFF), file);
        fputc((int)((pixel >> 8) & 0xFF), file);
        fputc((int)(pixel & 0xFF), file);
    }
    int failed = ferror(file);
    if (fclose(file) != 0) failed = 1;
    return failed;
}

static int run_image(const char *path, unsigned limit, const char *frame_path, DiagnosticMode mode) {
    if (load_test_image(path, mode)) return 1;
    if (mode == DIAGNOSTIC_MMC3) {
        bool nes2 = (ines_header.flags7 & 0x0C) == 8;
        if (rom_mapper_number(&ines_header) != 4 || (nes2 && (ines_header.prg_ram_size >> 4))) {
            fprintf(stderr, "MMC3 diagnostic setup requires mapper 4/submapper 0\n");
            return 2;
        }
        // These diagnostics write results at $6000 without enabling WRAM.
        // This explicit test setup leaves the emulator's reset defaults intact.
        printf("Diagnostic setup for %s: write $A001=$80 to enable result RAM\n", path);
        write_mem(0xA001, 0x80);
    }
    unsigned frames = 0;
    unsigned instructions = 0;
    bool protocol = false;
    bool reset_requested = false;
    bool reset_armed = true;
    uint64_t reset_request_cycle = 0;
    unsigned resets = 0;
    unsigned loop_instructions = 0;
    while (frames < limit) {
        uint16_t previous_pc = cpu.pc;
        int cycles = cpu_step(&cpu);
        if (cycles <= 0 || cycles > 1000000) {
            fprintf(stderr, "Invalid CPU elapsed cycles: %d\n", cycles);
            return 1;
        }
        if (ppu.frame_complete) {
            frames++;
            start_frame();
        }
        if (mode == DIAGNOSTIC_LEGACY_PAL || mode == DIAGNOSTIC_LEGACY) {
            if (cpu.pc == previous_pc && cycles == 3 && (cpu.status & INTERRUPT_FLAG) && !(ppu.ctrl & 0x80))
                loop_instructions++;
            else
                loop_instructions = 0;
            // The old suite stores its current subtest code at $F8 while it
            // runs. Only read the final code once its explicit exit loop runs.
            if (loop_instructions >= 1024 && cpu.pc >= 0x8000
                && cart_cpu_read(cpu.pc) == 0x4C
                && cart_cpu_read((uint16_t)(cpu.pc + 1)) == (uint8_t)cpu.pc
                && cart_cpu_read((uint16_t)(cpu.pc + 2)) == (uint8_t)(cpu.pc >> 8)) {
                uint8_t result = ram[0xF8];
                printf("ROM %s: %s (legacy code %u, %u frames)\n", path, result == 1 ? "PASS" : "FAIL", result, frames);
                return result == 1 ? 0 : 1;
            }
            continue;
        }
        if (!frame_path && ((++instructions & 1023) == 0)) {
            protocol = protocol || has_status_protocol();
            if (protocol) {
                uint8_t status = cart_cpu_read(0x6000);
                if (status < 0x80) {
                    printf("ROM %s: %s (code %u, %u frames)\n", path, status ? "FAIL" : "PASS", status, frames);
                    print_test_output();
                    return status ? 1 : 0;
                }
                if (status == 0x81) {
                    // A reset preserves cartridge RAM. Some diagnostics spend
                    // several frames checking reset timing before replacing
                    // the old request byte, so honor each request only once.
                    if (!reset_armed) continue;
                    if (!reset_requested) {
                        reset_requested = true;
                        reset_request_cycle = cpu_total_cycles;
                    } else if ((double)(cpu_total_cycles - reset_request_cycle) >= nes_timing()->cpu_hz * 0.1) {
                        if (++resets > 16) {
                            fprintf(stderr, "ROM %s: exceeded sixteen requested resets\n", path);
                            return 2;
                        }
                        printf("ROM %s: soft reset %u after the requested 100 ms delay\n", path, resets);
                        ppu_soft_reset(&ppu);
                        apu_soft_reset(&apu);
                        cpu_soft_reset(&cpu);
                        reset_requested = false;
                        reset_armed = false;
                    }
                } else {
                    reset_requested = false;
                    reset_armed = true;
                }
            }
        }
    }
    if (frame_path) {
        int result = write_frame(frame_path);
        printf("Rendered %u frames from %s to %s\n", frames, path, frame_path);
        return result;
    }
    printf("ROM %s: TIMEOUT (%u frames, %s status protocol)\n", path, frames, protocol ? "recognized" : "no");
    if (protocol) print_test_output();
    return 2;
}

int run_diagnostic_rom(const char *path, unsigned frame_limit) {
    int result = run_image(path, frame_limit, NULL, DIAGNOSTIC_NORMAL);
    unload_rom();
    return result;
}

int render_diagnostic_rom(const char *path, unsigned frames, const char *output) {
    int result = run_image(path, frames, output, DIAGNOSTIC_NORMAL);
    unload_rom();
    return result;
}

int run_mmc3_diagnostic_rom(const char *path, unsigned frame_limit) {
    int result = run_image(path, frame_limit, NULL, DIAGNOSTIC_MMC3);
    unload_rom();
    return result;
}

int run_legacy_pal_diagnostic_rom(const char *path, unsigned frame_limit) {
    int result = run_image(path, frame_limit, NULL, DIAGNOSTIC_LEGACY_PAL);
    unload_rom();
    return result;
}

int run_legacy_diagnostic_rom(const char *path, unsigned frame_limit) {
    int result = run_image(path, frame_limit, NULL, DIAGNOSTIC_LEGACY);
    unload_rom();
    return result;
}

typedef struct {
    char name[64];
    uint16_t result_address;
    uint8_t page, row;
} CoinTest;

static bool coin_read_text(uint32_t *address, char *text, size_t capacity) {
    size_t length = 0;
    while (*address < 0x10000 && length + 1 < capacity) {
        uint8_t value = cart_cpu_read((uint16_t)(*address)++);
        if (value == 0xFF) {
            text[length] = '\0';
            return length != 0;
        }
        if (value < 0x20 || value > 0x7E) return false;
        text[length++] = (char)value;
    }
    return false;
}

static unsigned coin_read_tests(CoinTest tests[144]) {
    static const char first_suite[] = "CPU Behavior";
    uint32_t address;
    for (address = 0x8000; address < 0xFFF0; ++address) {
        unsigned i;
        for (i = 0; i < sizeof(first_suite) - 1; ++i)
            if (cart_cpu_read((uint16_t)(address + i)) != (uint8_t)first_suite[i]) break;
        if (i == sizeof(first_suite) - 1 && cart_cpu_read((uint16_t)(address + i)) == 0xFF) break;
    }
    unsigned count = 0;
    for (unsigned page = 1; page <= 22; ++page) {
        char suite[64];
        if (!coin_read_text(&address, suite, sizeof(suite))) return 0;
        unsigned row = 0;
        while (address < 0xFFFC && cart_cpu_read((uint16_t)address) != 0xFF) {
            CoinTest test = {0};
            if (++row > 10 || !coin_read_text(&address, test.name, sizeof(test.name))) return 0;
            uint16_t result = cart_cpu_read((uint16_t)address);
            result |= (uint16_t)cart_cpu_read((uint16_t)(address + 1)) << 8;
            address += 4; // The next word is the ROM's test entry point.
            if ((result >> 8) == 3) continue; // Power-on displays do not report pass/fail.
            if (result < 0x400 || result > 0x4FF || count >= 144) return 0;
            for (unsigned i = 0; i < count; ++i)
                if (tests[i].result_address == result) return 0;
            test.result_address = result;
            test.page = (uint8_t)page;
            test.row = (uint8_t)row;
            tests[count++] = test;
        }
        if (address >= 0xFFFC) return 0;
        address++;
    }
    return count;
}

int run_accuracycoin_rom(const char *path, unsigned limit, const char *output) {
    int result = 2;
    CoinTest tests[144];
    if (load_test_image(path, DIAGNOSTIC_NORMAL)) goto done;
    if (rom_mapper_number(&ines_header) != 0 || coin_read_tests(tests) != 144) {
        fprintf(stderr, "AccuracyCoin requires the 144-test NROM image\n");
        goto done;
    }
    printf("AccuracyCoin: 144 test descriptors read from the cartridge\n");
    unsigned frames = 0, pressed_frame = 0, finished_frame = 0;
    bool started = false;
    uint16_t last_test = 0;
    while (frames < limit) {
        int cycles = cpu_step(&cpu);
        if (cycles <= 0 || cycles > 1000000 || cpu.halted) {
            fprintf(stderr, "AccuracyCoin stopped at PC=$%04X after %u frames\n", cpu.pc, frames);
            goto done;
        }
        if (!ppu.frame_complete) continue;
        ++frames;
        start_frame();
        // Operate the original menu through the controller. The runner never
        // changes ROM bytes, the program counter, test RAM, or result bytes.
        if (!pressed_frame && frames >= 120 && ram[0xEC] >= 0x0A
            && ram[0x16] == 0xFF && (ppu.ctrl & 0x80)) {
            joypad_set(&pad1, BTN_START, 1);
            pressed_frame = frames;
        }
        if (pressed_frame && frames >= pressed_frame + 2)
            joypad_set(&pad1, BTN_START, 0);
        if (ram[0x35] == 1) started = true;
        if (started && ram[0x35] == 1) {
            uint16_t current = (uint16_t)(ram[0x1E] | ((uint16_t)ram[0x1F] << 8));
            if (current != last_test) {
                last_test = current;
                for (unsigned i = 0; i < 144; ++i) {
                    if (tests[i].result_address != current) continue;
                    printf("AccuracyCoin running %u/%u: %s (frame %u)\n",
                           tests[i].page, tests[i].row, tests[i].name, frames);
                    fflush(stdout);
                    break;
                }
            }
        }
        if (started && ram[0x35] == 0 && ram[0x37] == 144 && (ppu.ctrl & 0x80)) {
            if (!finished_frame) finished_frame = frames;
            if (frames >= finished_frame + 2) {
                unsigned passed = 0, skipped = 0, unfinished = 0;
                for (unsigned i = 0; i < 144; ++i) {
                    uint8_t value = ram[tests[i].result_address];
                    bool pass = value != 0xFF && (value & 1);
                    passed += pass;
                    skipped += value == 0xFF;
                    unfinished += value == 0;
                    printf("AccuracyCoin %2u/%u $%04X %-27s: %s (raw $%02X, code %u)\n",
                           tests[i].page, tests[i].row, tests[i].result_address, tests[i].name,
                           pass ? "PASS" : value == 0xFF ? "SKIP" : value ? "FAIL" : "UNFINISHED",
                           value, value >> 2);
                }
                printf("AccuracyCoin: %u/144 passed, %u skipped, %u unfinished (%u frames); ROM tally %u/144\n",
                       passed, skipped, unfinished, frames, ram[0x38]);
                result = passed == 144 && ram[0x38] == 144 && !ram[0x3F] ? 0 : 1;
                if (output && write_frame(output)) result = 2;
                goto done;
            }
        }
    }
    fprintf(stderr, "AccuracyCoin TIMEOUT: %u frames, PC=$%04X, startup=$%02X, running=%u, test=$%04X\n",
            frames, cpu.pc, ram[0xEC], ram[0x35], last_test);
    if (output) (void)write_frame(output);
done:
    unload_rom();
    return result;
}
