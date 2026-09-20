/*
 * accuracy_test.c - Hardware regression test entry point
 *
 * Author: @frankischilling
 *
 * This file runs the hardware regression suites and command line diagnostic modes against
 * the production CPU, PPU, APU, controller, cartridge, and timing implementations.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
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
int test_region_accuracy(void);
int test_file_io_accuracy(void);
int test_persistence_accuracy(void);
int test_native_flash_geometry_accuracy(void);
int test_mapper30_111_prg_ram_accuracy(void);
int test_board_accuracy(void);
int test_board_codemasters_accuracy(void);
int test_board_magic_floor_accuracy(void);
int test_board_jaleco_accuracy(void);
int test_board_nsf_cart_accuracy(void);
int test_board_ffe_accuracy(void);
int test_board_farid_accuracy(void);
int test_board_sealie_accuracy(void);
int test_board_ntdec_accuracy(void);
int test_board_racermate_accuracy(void);
int test_board_taito_accuracy(void);
int test_board_sachen_accuracy(void);
int test_board_kaiser_accuracy(void);
int test_board_mmc3_accuracy(void);
int test_board_mmc3_mixed_chr_accuracy(void);
int test_board_sachen_late_accuracy(void);
int test_board_jy_small_accuracy(void);
int test_board_drip_accuracy(void);
int test_board_rainbow_accuracy(void);
int test_board_mmc3_96_accuracy(void);
int test_board_mmc3_97_accuracy(void);
int test_board_mmc3_98_accuracy(void);
int test_board_unlicensed_109_accuracy(void);
int test_board_unlicensed_111_accuracy(void);
int test_board_unlicensed_112_accuracy(void);
int test_board_txc_107_accuracy(void);
int test_board_unlicensed_113_accuracy(void);
int test_board_unlicensed_114_accuracy(void);
int test_board_unlicensed_110_accuracy(void);
int test_board_unlicensed_115_accuracy(void);
int test_rom_database_defaults_accuracy(void);
int test_game_database_discovery_accuracy(void);
int test_unif_accuracy(void);
int test_board_waixing_116_accuracy(void);
int test_board_whirlwind_117_accuracy(void);
int test_mmc5_extended_geometry_accuracy(void);
int test_native_ram_accuracy(void);
int test_board_irem77_accuracy(void);
int test_default_prg_ram_geometry_accuracy(void);
int test_board_nina_fme7_accuracy(void);
int test_native_chr_capacity_accuracy(void);
int test_native_mixed_chr_accuracy(void);
int test_bandai_accuracy(void);
int test_fds_accuracy(void);
int test_studybox_accuracy(void);
int test_nsf_accuracy(void);
int test_input_accuracy(void);
int test_vs_accuracy(void);
int test_epsm_accuracy(void);
int test_frontend_accuracy(void);
int test_cpu_trace(const char *rom_path, const char *trace_path);
int run_diagnostic_rom(const char *path, unsigned frame_limit);
int run_mmc3_diagnostic_rom(const char *path, unsigned frame_limit);
int run_legacy_pal_diagnostic_rom(const char *path, unsigned frame_limit);
int run_legacy_diagnostic_rom(const char *path, unsigned frame_limit);
int render_diagnostic_rom(const char *path, unsigned frames, const char *output);
int run_accuracycoin_rom(const char *path, unsigned frames, const char *output);

int main(int argc, char **argv) {
    if (argc > 1) {
        if (argc == 4 && strcmp(argv[1], "--trace") == 0) {
            int result = test_cpu_trace(argv[2], argv[3]);
            unload_rom();
            return result;
        }
        if (argc >= 4 && (strcmp(argv[1], "--rom") == 0 || strcmp(argv[1], "--render") == 0
            || strcmp(argv[1], "--accuracycoin") == 0
            || strcmp(argv[1], "--mmc3-rom") == 0 || strcmp(argv[1], "--legacy-pal-rom") == 0
            || strcmp(argv[1], "--legacy-rom") == 0)) {
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
            if (strcmp(argv[1], "--accuracycoin") == 0) {
                if (argc > 5) return 2;
                return run_accuracycoin_rom(argv[3], (unsigned)frames, argc == 5 ? argv[4] : NULL);
            }
            int failed = 0;
            for (int i = 3; i < argc; ++i) {
                int result = strcmp(argv[1], "--mmc3-rom") == 0
                           ? run_mmc3_diagnostic_rom(argv[i], (unsigned)frames)
                           : strcmp(argv[1], "--legacy-pal-rom") == 0
                           ? run_legacy_pal_diagnostic_rom(argv[i], (unsigned)frames)
                           : strcmp(argv[1], "--legacy-rom") == 0
                           ? run_legacy_diagnostic_rom(argv[i], (unsigned)frames)
                           : run_diagnostic_rom(argv[i], (unsigned)frames);
                if (result != 0) failed++;
            }
            printf("Diagnostic ROMs: %d passed, %d failed or unfinished\n", argc - 3 - failed, failed);
            return failed ? 1 : 0;
        }
        fprintf(stderr, "Usage: %s [--trace ROM LOG | --rom FRAMES ROM... | --mmc3-rom FRAMES ROM... | --legacy-pal-rom FRAMES ROM... | --legacy-rom FRAMES ROM... | --render FRAMES ROM OUTPUT.ppm | --accuracycoin FRAMES ROM [OUTPUT.ppm]]\n", argv[0]);
        return 2;
    }
    int failures = 0;
    failures += test_cpu_accuracy();
    failures += test_apu_accuracy();
    failures += test_ppu_accuracy();
    failures += test_mapper_accuracy();
    failures += test_native_flash_geometry_accuracy();
    failures += test_mapper30_111_prg_ram_accuracy();
    failures += test_board_accuracy();
    failures += test_board_codemasters_accuracy();
    failures += test_board_magic_floor_accuracy();
    failures += test_board_jaleco_accuracy();
    failures += test_board_nsf_cart_accuracy();
    failures += test_board_ffe_accuracy();
    failures += test_board_farid_accuracy();
    failures += test_board_sealie_accuracy();
    failures += test_board_ntdec_accuracy();
    failures += test_board_racermate_accuracy();
    failures += test_board_taito_accuracy();
    failures += test_board_sachen_accuracy();
    failures += test_board_kaiser_accuracy();
    failures += test_board_mmc3_accuracy();
    failures += test_board_mmc3_mixed_chr_accuracy();
    failures += test_board_sachen_late_accuracy();
    failures += test_board_jy_small_accuracy();
    failures += test_board_drip_accuracy();
    failures += test_board_rainbow_accuracy();
    failures += test_board_mmc3_96_accuracy();
    failures += test_board_mmc3_97_accuracy();
    failures += test_board_mmc3_98_accuracy();
    failures += test_board_unlicensed_109_accuracy();
    failures += test_board_unlicensed_111_accuracy();
    failures += test_board_unlicensed_112_accuracy();
    failures += test_board_txc_107_accuracy();
    failures += test_board_unlicensed_113_accuracy();
    failures += test_board_unlicensed_114_accuracy();
    failures += test_board_unlicensed_110_accuracy();
    failures += test_board_unlicensed_115_accuracy();
    failures += test_rom_database_defaults_accuracy();
    failures += test_game_database_discovery_accuracy();
    failures += test_unif_accuracy();
    failures += test_board_waixing_116_accuracy();
    failures += test_board_whirlwind_117_accuracy();
    failures += test_mmc5_extended_geometry_accuracy();
    failures += test_native_ram_accuracy();
    failures += test_board_irem77_accuracy();
    failures += test_default_prg_ram_geometry_accuracy();
    failures += test_board_nina_fme7_accuracy();
    failures += test_native_chr_capacity_accuracy();
    failures += test_native_mixed_chr_accuracy();
    failures += test_bandai_accuracy();
    failures += test_fds_accuracy();
    failures += test_studybox_accuracy();
    failures += test_nsf_accuracy();
    failures += test_input_accuracy();
    failures += test_vs_accuracy();
    failures += test_epsm_accuracy();
    failures += test_region_accuracy();
    failures += test_file_io_accuracy();
    failures += test_persistence_accuracy();
    failures += test_frontend_accuracy();
    unload_rom();
    printf("Hardware regressions: %s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
