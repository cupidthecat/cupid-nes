/*
 * mapper_accuracy.c - Cartridge mapper regression tests
 *
 * Author: @frankischilling
 *
 * This file uses synthetic PRG and CHR images to test mapper banking, mirroring, IRQs,
 * RAM layouts, save persistence, bus conflicts, MMC5 features, and loader error handling.
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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif
#include "../rom/mapper.h"
#include "../rom/game_db.h"
#include "../rom/unif.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../apu/apu.h"
#include "../joypad/joypad.h"
#include "../system/timing.h"
#include "../system/vs_system.h"
#include "../../include/globals.h"

extern uint64_t cpu_total_cycles;

static uint8_t fixture_prg[0x200000];
static uint8_t fixture_chr[0x80000];
static uint8_t *image_for(const iNESHeader *h, size_t prg_bytes, size_t chr_bytes, size_t *size);
static void prepare_mapper_cpu_nops(void);
static void run_mapper_nops(unsigned count);
static void sunsoft69_command(uint8_t command, uint8_t value);
static int discrete_cpu_store(uint16_t address, uint8_t value);

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static iNESHeader header_for(unsigned mapper, size_t prg_bytes, bool chr_ram) {
    iNESHeader h = {0};
    memcpy(h.signature, "NES\x1A", 4);
    h.prg_rom_chunks = (uint8_t)(prg_bytes / 0x4000);
    h.chr_rom_chunks = chr_ram ? 0 : 1;
    h.flags6 = (uint8_t)(mapper << 4);
    h.flags7 = (uint8_t)(mapper & 0xF0);
    return h;
}

typedef struct {
    uint8_t *data;
    size_t size;
} UnifFixture;

static int unif_fixture_begin(UnifFixture *fixture) {
    fixture->data = (uint8_t *)calloc(1, 32);
    if (!fixture->data) return -1;
    fixture->size = 32;
    memcpy(fixture->data, "UNIF", 4);
    fixture->data[4] = 7;
    return 0;
}

static int unif_fixture_chunk(UnifFixture *fixture, const char id[4],
                              const void *payload, size_t length) {
    if (!fixture || !fixture->data || length > UINT32_MAX
        || fixture->size > SIZE_MAX - 8 - length) return -1;
    size_t old_size = fixture->size;
    size_t new_size = old_size + 8 + length;
    uint8_t *grown = (uint8_t *)realloc(fixture->data, new_size);
    if (!grown) return -1;
    fixture->data = grown;
    memcpy(grown + old_size, id, 4);
    uint32_t len = (uint32_t)length;
    grown[old_size + 4] = (uint8_t)len;
    grown[old_size + 5] = (uint8_t)(len >> 8);
    grown[old_size + 6] = (uint8_t)(len >> 16);
    grown[old_size + 7] = (uint8_t)(len >> 24);
    if (length) memcpy(grown + old_size + 8, payload, length);
    fixture->size = new_size;
    return 0;
}

static void unif_fixture_end(UnifFixture *fixture) {
    if (!fixture) return;
    free(fixture->data);
    fixture->data = NULL;
    fixture->size = 0;
}

static int fixture_with_header(const iNESHeader *h, size_t prg_bytes, size_t chr_bytes) {
    // Finish using the previous borrowed buffers before refilling them.
    mapper_shutdown();
    for (size_t i = 0; i < prg_bytes; ++i) fixture_prg[i] = (uint8_t)(i / 0x2000);
    for (size_t i = 0; i < chr_bytes; ++i) fixture_chr[i] = (uint8_t)(i / 0x0400);
    cpu_total_cycles = 0;
    return mapper_init_from_header(h, fixture_prg, prg_bytes, fixture_chr, chr_bytes);
}

static int fixture(unsigned mapper, size_t prg_bytes, size_t chr_bytes, bool chr_ram) {
    iNESHeader h = header_for(mapper, prg_bytes, chr_ram);
    return fixture_with_header(&h, prg_bytes, chr_bytes);
}

static void serial_write(uint16_t addr, uint8_t value) {
    for (unsigned bit = 0; bit < 5; ++bit) {
        cpu_total_cycles += 2;
        cart_cpu_write(addr, (uint8_t)((value >> bit) & 1));
    }
}

#include "mapper_accuracy_nintendo.h"
#include "mapper_accuracy_discrete.h"
#include "mapper_accuracy_conflicts.h"
#include "mapper_accuracy_loader.h"
#include "mapper_accuracy_storage.h"
#include "mapper_accuracy_persistence.h"
#include "mapper_accuracy_native.h"
#include "mapper_accuracy_irq.h"
#include "mapper_accuracy_vrc7_reset.h"
#include "mapper_accuracy_sunsoft.h"
#include "mapper_accuracy_followups.h"

int test_mapper_accuracy(void) {
    static int (*const tests[])(void) = {
        test_small_cartridges, test_mmc1_banks_and_ram, test_mmc1_serial_timing,
        test_mapper105_competition_board,
        test_mapper105_fixed_chr_and_serial_timing, test_mapper105_dips_and_cpu_irq,
        test_mapper232_multicart_banks,
        test_mapper232_loader_and_cpu_bus,
        test_mmc1a_ram_revision, test_mmc1a_cpu_serial_writes, test_mmc1a_ram_layouts_and_loader,
        test_mmc1_outer_and_fixed_banks, test_mmc_latch_address_notifications, test_mmc2_banks_and_latches,
        test_mmc4_latches_and_chr_ram, test_mmc3_banks_and_protection,
        test_mmc3_irq_edges, test_mmc3_revision_a_irq, test_mmc3_revision_a_cpu_irq,
        test_mmc3_render_trace, test_tqrom_mixed_chr_memory,
        test_mmc3_mixed_chr_variants, test_mmc3_mixed_chr_source_page_geometry,
        test_jy_prg_chr_modes, test_jy_mapper_variants_and_readback,
        test_jy_chr_ram_nametable_reads,
        test_jy_irq_sources_and_cpu_delivery, test_jy_loader_and_reset,
        test_mcacc_irq_divider, test_mcacc_irq_reload_and_enable,
        test_mcacc_banks_ram_and_loader, test_mcacc_cpu_ppu_irq_path,
        test_taito_banks_aliases_and_mirroring, test_taito48_irq, test_taito_loader_transaction,
        test_taito48_cpu_irq,
        test_txsrom_nametable_routing, test_txsrom_startup_ram_and_loader,
        test_rambo1_banks_and_modes, test_rambo1_irq_sources, test_rambo158_nametables,
        test_rambo1_ram_and_odd_chr_banks, test_rambo1_irq_boundaries,
        test_rambo1_cpu_and_rendering_irq, test_rambo1_loader,
        test_simple_mapper_registers,
        test_vrc6_startup_banks_wiring_and_ram, test_vrc6_nametable_modes,
        test_vrc6_irq_timing, test_vrc6_pulse_and_saw_audio,
        test_vrc6_loader_rejection_preserves_cart,
        test_vrc6_cpu_boot_and_odd_prg_size,
        test_colordreams_bus_conflicts, test_bus_conflict_submappers,
        test_colordreams_high_banks_and_mapper144, test_unrom94_180_cpu_banks,
        test_nina_cpu_decode_and_mirroring, test_nina_chr_ram_banks,
        test_discrete_followup_loader_and_ram, test_mapper180_full_prg_range,
        test_uxrom94_180_sub16_prg_geometry, test_native_8k_prg_page_geometry,
        test_native_reduced_prg_page_geometry, test_remaining_native_geometry_caps,
        test_native_shrunk_chr8_windows, test_discrete_followup_page_geometry,
        test_discrete_followup_ignored_submappers,
        test_discrete_followup_saves,
        test_trainer_and_existing_saves, test_trainer_volatile_ram_ignores_save,
        test_mapper15_modes, test_action53_banks_and_mirroring,
        test_action53_game_sizes, test_action53_largest_image,
        test_unrom512_banks_flash_and_mirroring, test_mmc5_memory_windows,
        test_unrom512_physical_flash_address, test_unrom512_cpu_flash,
        test_mapper111_banks_flash_and_nametables, test_mapper111_cpu_soft_reset_preserves_state,
        test_mapper96_banks_latch_and_loader,
        test_mmc5_exram_and_irq, test_mmc5_chr_fetch_modes, test_mmc5_extended_rendering,
        test_mmc5_rendered_ppu_paths, test_mmc5_audio_and_pcm, test_header_and_mapper_rejection,
        test_loader_trainers_and_sizes, test_loader_rejection_preserves_cart,
        test_loader_small_and_irregular_rom_pages, test_loader_existing_board_page_geometry,
        test_loader_region_and_console_type,
        test_ram_header_sizes, test_prg_ram_capacity, test_mmc1_banked_ram,
        test_mmc5_banked_ram, test_loader_ram_layouts, test_extended_ram_layouts_and_ownership,
        test_unmapped_chr_storage_ownership,
        test_prg_nvram_persistence,
        test_mapper96_legacy_nvram_persistence,
        test_vs_nvram_persistence,
        test_unrom512_flash_persistence, test_mapper111_flash_persistence,
        test_mmc5_persistence,
        test_chr_nvram_persistence, test_chr_nvram_writers,
        test_mmc6_ram_mirroring, test_mmc6_protection, test_mmc6_banks_and_irq,
        test_namco163_banks_ram_and_nametables, test_namco163_irq_and_audio,
        test_namco175_340_variants, test_namco163_persistence_and_loader,
        test_namco163_source_page_geometry,
        test_mapper34_bnrom_banking_and_ram, test_mapper34_nina_banks_and_selection,
        test_mapper34_loader_preserves_cart, test_mapper34_image_loading,
        test_gxrom_banks_reset_and_ram, test_gxrom_chr_ram_and_loader,
        test_mapper71_variants_and_mirroring, test_mapper71_loader_and_chr_rom,
        test_mapper71_image_loading, test_legacy_loader_ram_and_large_prg_metadata,
        test_namco108_banks_aliases_and_irq_absence, test_namco108_submapper_loader_and_chr_ram,
        test_namco108_variants, test_namco108_variant_loader_rejection,
        test_namco108_variant_image_loading, test_native_small_chr_bank_windows,
        test_native_small_chr_1k_families, test_remaining_small_chr_single_source,
        test_sunsoft69_banks_ram_and_startup, test_sunsoft69_legacy_ram_defaults,
        test_sunsoft69_irq_cpu_clock,
        test_sunsoft5b_tone_noise_envelope, test_sunsoft69_persistence_and_loader,
        test_irregular_native_page_safety, test_nintendo_shrunk_chr_pages,
        test_jaleco18_banks_ram_and_mirroring, test_jaleco18_irq_and_cpu_clock,
        test_jaleco18_irq_width_transitions,
        test_jaleco18_loader_validation,
        test_irem32_banks_variants_and_ram, test_irem65_banks_decode_and_ram,
        test_irem65_irq_and_loader_validation,
        test_irem_ram_and_irq_boundaries,
        test_vrc24_variant_register_wiring, test_vrc24_ram_latch_and_mapper183_window,
        test_vrc24_irq_variants_and_phase, test_vrc24_loader_rejection_preserves_cart,
        test_vrc7_banks_wiring_and_ram, test_vrc7_irq_timing, test_vrc7_fm_audio,
        test_vrc7_console_reset, test_vrc7_reset_address_latch,
        test_vrc7_register_boundaries,
        test_vrc7_loader_rejection_preserves_cart,
        test_vrc1_banks_mirroring_and_reset, test_vrc1_loader_rejection_preserves_cart,
        test_vrc3_banks_irq_and_reset, test_vrc3_cpu_irq_and_loader,
        test_sunsoft3_banks_mirroring_and_irq, test_sunsoft3_cpu_irq_and_loader,
        test_sunsoft4_banks_nametables_and_timer, test_sunsoft4_chr_ram_persistence_and_loader,
        test_sunsoft4_cpu_licensed_reads,
        test_sunsoft_discrete_boards, test_sunsoft_shrunk_chr_pages,
        test_sunsoft_discrete_loader_rejection,
        test_sunsoft_discrete_image_loading, test_sunsoft184_inherited_ram_reads,
        test_taito_x1005_and_207, test_taito_x1017_banks_chr_and_ram,
        test_taito_x1_persistence_and_loader,
        test_irem97_prg_and_mirroring, test_irem97_loader_validation,
        test_irem97_cpu_ram_layouts, test_irem97_ram_persistence,
        test_jaleco72_92_latches_cpu_and_bus_conflicts,
        test_jaleco78_banking_and_submapper_mirroring,
        test_jaleco87_101_140_banks_and_register_ranges,
        test_jaleco_discrete_ram_cpu_paths,
        test_jaleco_intercepted_ram_reads_and_soft_reset,
        test_jaleco_discrete_loader_validation,
        test_cnrom185_submapper_latches_and_ppu_bus,
        test_cnrom185_cpu_bus_conflict_control,
        test_cnrom185_loader_and_cnrom_regression,
        test_cartridge_bus_reads, test_mmc6_persistence,
        test_game_database_and_headerless_loading, test_unif_loading_and_named_boards,
        test_cartridge_unload
    };
    int failures = 0;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) failures += tests[i]();
    unload_rom();
    printf("Mapper accuracy: %zu groups, %d failures\n", sizeof(tests) / sizeof(tests[0]), failures);
    return failures;
}
