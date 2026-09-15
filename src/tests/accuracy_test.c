/* SPDX-License-Identifier: GPL-3.0-or-later
 * Hardware tests use the production CPU, PPU, APU, controller, and cartridge.
 */
#include "../cpu/cpu.h"
#include "../rom/rom.h"
#include "../../include/globals.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];
Joypad pad1 = {0};
Joypad pad2 = {0};

int test_cpu_accuracy(void);
int test_apu_accuracy(void);
int test_ppu_accuracy(void);
int test_mapper_accuracy(void);
int test_cpu_trace(const char *rom_path, const char *trace_path);
int run_diagnostic_rom(const char *path, unsigned frame_limit);
int run_mmc3_diagnostic_rom(const char *path, unsigned frame_limit);
int render_diagnostic_rom(const char *path, unsigned frames, const char *output);

int main(int argc, char **argv) {
    if (argc > 1) {
        if (argc == 4 && strcmp(argv[1], "--trace") == 0) {
            int result = test_cpu_trace(argv[2], argv[3]);
            unload_rom();
            return result;
        }
        if (argc >= 4 && (strcmp(argv[1], "--rom") == 0 || strcmp(argv[1], "--render") == 0
            || strcmp(argv[1], "--mmc3-rom") == 0)) {
            char *end;
            long frames = strtol(argv[2], &end, 10);
            if (*end || frames < 1 || frames > 100000) {
                fprintf(stderr, "Frame count must be between 1 and 100000\n");
                return 2;
            }
            if (strcmp(argv[1], "--render") == 0) {
                if (argc != 5) return 2;
                return render_diagnostic_rom(argv[3], (unsigned)frames, argv[4]);
            }
            int failed = 0;
            for (int i = 3; i < argc; ++i) {
                int result = strcmp(argv[1], "--mmc3-rom") == 0
                           ? run_mmc3_diagnostic_rom(argv[i], (unsigned)frames)
                           : run_diagnostic_rom(argv[i], (unsigned)frames);
                if (result != 0) failed++;
            }
            printf("Diagnostic ROMs: %d passed, %d failed or unfinished\n", argc - 3 - failed, failed);
            return failed ? 1 : 0;
        }
        fprintf(stderr, "Usage: %s [--trace ROM LOG | --rom FRAMES ROM... | --mmc3-rom FRAMES ROM... | --render FRAMES ROM OUTPUT.ppm]\n", argv[0]);
        return 2;
    }
    int failures = 0;
    failures += test_cpu_accuracy();
    failures += test_apu_accuracy();
    failures += test_ppu_accuracy();
    failures += test_mapper_accuracy();
    unload_rom();
    printf("Hardware regressions: %s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
