/*
 * cheat_accuracy.c - Cheat decoding, bus behavior and persistence regressions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "../apu/apu.h"
#include "../cheats/cheats.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../rom/mapper.h"
#include "../rom/rom.h"
#include "../system/execution_policy.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include "../util/file_io.h"
#include "../ui/cheat_frontend.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static unsigned cheat_checks;

#define CHECK(condition) do { \
    ++cheat_checks; \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static int parse_formats(void) {
    cheats_init();
    CheatRecord code;
    CHECK(cheats_parse("SXIOPO", &code) == CHEAT_OK);
    CHECK(code.format == CHEAT_FORMAT_GAME_GENIE && code.address == 0x91D9);
    CHECK(code.value == 0xAD && !code.has_compare);
    CHECK(cheats_parse("GZUXNGEI", &code) == CHEAT_OK);
    CHECK(code.address == 0xAC3F && code.value == 0x24);
    CHECK(code.has_compare && code.compare == 0xD0);
    CHECK(cheats_parse("12345678", &code) == CHEAT_OK);
    CHECK(code.format == CHEAT_FORMAT_PAR && code.address == 0xAD85);
    CHECK(code.value == 0x3C && code.compare == 0xC1 && code.has_compare);
    CHECK(cheats_parse("8000:9A", &code) == CHEAT_OK);
    CHECK(code.format == CHEAT_FORMAT_RAW && code.address == 0x8000 && code.value == 0x9A);
    CHECK(cheats_parse("1234:56:78", &code) == CHEAT_OK && code.compare == 0x78);
    CHECK(cheats_parse("SXIOP!", &code) == CHEAT_INVALID_CODE);
    CHECK(cheats_parse("123:45", &code) == CHEAT_INVALID_CODE);
    CHECK(cheats_parse("10000:12", &code) == CHEAT_INVALID_CODE);
    CHECK(cheats_parse("8000:100", &code) == CHEAT_INVALID_CODE);
    return 0;
}

static uint8_t uxrom_image[16 + 0x10000 + 0x2000];

static void build_uxrom(void) {
    memset(uxrom_image, 0, sizeof(uxrom_image));
    memcpy(uxrom_image, "NES\x1A", 4);
    uxrom_image[4] = 4;
    uxrom_image[5] = 1;
    uxrom_image[6] = 0x20;
    uint8_t *prg = uxrom_image + 16;
    memset(prg + 0x0000, 0x11, 0x4000);
    memset(prg + 0x4000, 0x22, 0x4000);
    memset(prg + 0x8000, 0x33, 0x4000);
    memset(prg + 0xC000, 0x44, 0x4000);
    prg[0xC000] = 0xAD; prg[0xC001] = 0x00; prg[0xC002] = 0x80; /* LDA $8000 */
    prg[0xC003] = 0x85; prg[0xC004] = 0x10;                 /* STA $10 */
    prg[0xC005] = 0x4C; prg[0xC006] = 0x00; prg[0xC007] = 0xC0;
    prg[0xFFFA] = 0x00; prg[0xFFFB] = 0xC0;
    prg[0xFFFC] = 0x00; prg[0xFFFD] = 0xC0;
    prg[0xFFFE] = 0x00; prg[0xFFFF] = 0xC0;
}

static int start_uxrom(void) {
    if (!unload_rom()) return -1;
    if (!nes_set_region_mode(NES_REGION_MODE_NTSC)) return -1;
    build_uxrom();
    if (load_rom_memory(uxrom_image, sizeof(uxrom_image)) != 0) return -1;
    ppu_power_on(&ppu); apu_power_on(&apu); cpu_total_cycles = 0;
    return cpu_power_on(&cpu) ? 0 : -1;
}

static int banked_compare_and_bus_hook(void) {
    cheats_init();
    CHECK(start_uxrom() == 0);
    uint32_t id = 0;
    CHECK(cheats_add("8000:99:11", "bank zero", true, &id) == CHEAT_OK && id != 0);
    CHECK(cart_cpu_peek_bus(0x8000, 0xFF) == 0x11);
    CHECK(cpu_step(&cpu) > 0 && cpu.a == 0x99);
    CHECK(cart_cpu_peek_bus(0x8000, 0xFF) == 0x11);

    cart_cpu_write(0x8000, 1);
    CHECK(cart_cpu_peek_bus(0x8000, 0xFF) == 0x22);
    cpu.pc = 0xC000;
    CHECK(cpu_step(&cpu) > 0 && cpu.a == 0x22);
    CHECK(cheats_edit(id, "8000:99:22", "bank one", true) == CHEAT_OK);
    cpu.pc = 0xC000;
    CHECK(cpu_step(&cpu) > 0 && cpu.a == 0x99);

    CHECK(cheats_clear() == CHEAT_OK);
    uint32_t first = 0, second = 0;
    CHECK(cheats_add("8000:55", "first", true, &first) == CHEAT_OK);
    CHECK(cheats_add("8000:66", "second", true, &second) == CHEAT_OK);
    CHECK(cheats_apply_read(0x8000, 0x22) == 0x66);
    CHECK(cheats_move(first, 1) == CHEAT_OK);
    CHECK(cheats_apply_read(0x8000, 0x22) == 0x55);
    CHECK(cheats_set_enabled(first, false) == CHEAT_OK);
    CHECK(cheats_apply_read(0x8000, 0x22) == 0x66);

    write_mem(0x0010, 0x42);
    CHECK(cpu_peek_internal_ram(0x0010) == 0x42);
    uint32_t raw = 0;
    CHECK(cheats_add("0010:9A", "RAM read", true, &raw) == CHEAT_OK);
    CHECK(cpu_peek_internal_ram(0x0010) == 0x42);
    CHECK(cheats_apply_read(0x0010, cpu_peek_internal_ram(0x0010)) == 0x9A);
    return 0;
}

static int crud_policy_and_hash(void) {
    cheats_init(); cheats_set_game_identity(0x2468ACE0u);
    uint32_t a = 0, b = 0;
    CHECK(cheats_add("8000:10", "a", true, &a) == CHEAT_OK);
    CHECK(cheats_add("8001:20", "b", false, &b) == CHEAT_OK);
    CHECK(cheats_count() == 2);
    uint32_t hash_a = cheats_compatibility_hash();
    CHECK(hash_a != 0);
    CHECK(cheats_set_enabled(b, true) == CHEAT_OK);
    uint32_t hash_b = cheats_compatibility_hash();
    CHECK(hash_b != hash_a);
    CHECK(cheats_move(b, 0) == CHEAT_OK);
    CHECK(cheats_compatibility_hash() != hash_b);
    CheatRecord record;
    CHECK(cheats_at(0, &record) && record.id == b);

    CHECK(nes_execution_set_policy(NES_EXECUTION_MOVIE_RECORDING));
    CHECK(cheats_add("8002:30", "blocked", true, NULL) == CHEAT_DETERMINISTIC_MODE);
    CHECK(cheats_set_enabled(a, false) == CHEAT_DETERMINISTIC_MODE);
    CHECK(cheats_edit(a, "8000:11", "blocked", true) == CHEAT_DETERMINISTIC_MODE);
    CHECK(cheats_remove(a) == CHEAT_DETERMINISTIC_MODE);
    CHECK(cheats_move(a, 0) == CHEAT_DETERMINISTIC_MODE);
    CHECK(cheats_clear() == CHEAT_DETERMINISTIC_MODE);
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    cheats_set_execution_policy(NES_EXECUTION_NETPLAY);
    CHECK(cheats_set_enabled(a, false) == CHEAT_DETERMINISTIC_MODE);
    cheats_set_execution_policy(NES_EXECUTION_LIVE);
    CHECK(cheats_set_enabled(a, false) == CHEAT_OK);
    CHECK(cheats_remove(a) == CHEAT_OK && cheats_count() == 1);
    return 0;
}

static int persistence_and_retention(void) {
    cheats_init(); cheats_set_game_identity(0x1234ABCDu);
    uint32_t first = 0, second = 0;
    CHECK(cheats_add("SXIOPO", "Möbius 猫", true, &first) == CHEAT_OK);
    CHECK(cheats_add("1234:56:78", "compare", false, &second) == CHEAT_OK);
    char path[160];
    snprintf(path, sizeof(path), "build/cheat-accuracy-%lu.txt", (unsigned long)time(NULL));
    (void)nes_file_remove(path);
    CHECK(cheats_save_file(path) == CHEAT_OK);
    CHECK(cheats_clear() == CHEAT_OK && cheats_count() == 0);
    CHECK(cheats_load_file(path) == CHEAT_OK && cheats_count() == 2);
    CheatRecord loaded;
    CHECK(cheats_at(0, &loaded));
    CHECK(loaded.enabled && strcmp(loaded.code, "SXIOPO") == 0);
    CHECK(strcmp(loaded.description, "Möbius 猫") == 0);
    CHECK(cheats_at(1, &loaded) && !loaded.enabled && loaded.compare == 0x78);

    size_t retained_count = cheats_count();
    static const char malformed[] = "CUPID-CHEATS\t1\t1234ABCD\n1\tNOT-A-CODE\tbad\n";
    CHECK(nes_file_write_atomic(path, malformed, sizeof(malformed) - 1) == NES_FILE_OK);
    CHECK(cheats_load_file(path) == CHEAT_INVALID_CODE);
    CHECK(cheats_count() == retained_count);
    CHECK(cheats_at(0, &loaded) && strcmp(loaded.description, "Möbius 猫") == 0);

    static const char wrong_game[] = "CUPID-CHEATS\t1\tFFFFFFFF\n1\tSXIOPO\twrong\n";
    CHECK(nes_file_write_atomic(path, wrong_game, sizeof(wrong_game) - 1) == NES_FILE_OK);
    CHECK(cheats_load_file(path) == CHEAT_INVALID_CODE && cheats_count() == retained_count);

    static const uint8_t invalid_utf8[] = {
        'C','U','P','I','D','-','C','H','E','A','T','S','\t','1','\t','1','2','3','4','A','B','C','D','\n',
        '1','\t','S','X','I','O','P','O','\t',0xC0,0xAF,'\n'
    };
    CHECK(nes_file_write_atomic(path, invalid_utf8, sizeof(invalid_utf8)) == NES_FILE_OK);
    CHECK(cheats_load_file(path) == CHEAT_INVALID_CODE && cheats_count() == retained_count);
    CHECK(nes_file_remove(path) == NES_FILE_OK);
    return 0;
}

static int game_replacement_identity(void) {
    cheats_init();
    CHECK(start_uxrom() == 0);
    uint32_t identity = cheats_game_identity();
    CHECK(identity == rom_file_crc32());
    CHECK(cheats_add("8000:77", "keep", true, NULL) == CHEAT_OK && cheats_count() == 1);
    uint8_t invalid[16] = {0};
    CHECK(load_rom_memory(invalid, sizeof(invalid)) != 0);
    CHECK(cheats_game_identity() == identity && cheats_count() == 1);
    CHECK(unload_rom());
    CHECK(cheats_game_identity() == 0 && cheats_count() == 0);
    return 0;
}

static int frontend_panel_crud(void) {
    char path[160];
    const uint32_t identity = 0xA11CE123u;
    snprintf(path, sizeof(path), "build/cheats-%08X.txt", identity);
    (void)nes_file_remove(path);
    frontend_commands_reset();
    frontend_panels_reset();
    frontend_command_set_session_active(true);
    frontend_panel_set_session_active(true);
    CheatFrontend *frontend = cheat_frontend_create("build/");
    CHECK(frontend != NULL);
    CHECK(cheat_frontend_register_ui(frontend));
    cheat_frontend_image_changed(frontend, identity);

    FrontendPanelControl controls[16];
    FrontendPanelModel model = {.controls = controls, .capacity = 16};
    char error[160] = {0};
    CHECK(frontend_panel_snapshot(CHEATS_FRONTEND_PANEL, &model, error, sizeof(error)));
    CHECK(model.count == 13 && controls[0].id == CHEAT_CONTROL_LIST);
    CHECK(frontend_panel_action(CHEATS_FRONTEND_PANEL, CHEAT_CONTROL_CODE,
                                "8000:7F", -1, error, sizeof(error)));
    CHECK(frontend_panel_action(CHEATS_FRONTEND_PANEL, CHEAT_CONTROL_DESCRIPTION,
                                "frontend", -1, error, sizeof(error)));
    CHECK(frontend_panel_action(CHEATS_FRONTEND_PANEL, CHEAT_CONTROL_ENABLED,
                                NULL, -1, error, sizeof(error)));
    CHECK(frontend_panel_action(CHEATS_FRONTEND_PANEL, CHEAT_CONTROL_ADD,
                                NULL, -1, error, sizeof(error)));
    CHECK(cheats_count() == 1);
    CheatRecord record;
    CHECK(cheats_at(0, &record) && !record.enabled && !strcmp(record.code, "8000:7F"));
    uint8_t *saved = NULL; size_t saved_size = 0;
    CHECK(nes_file_read_all(path, 4096, &saved, &saved_size) == NES_FILE_OK);
    CHECK(saved_size > 0);
    free(saved);
    CHECK(frontend_panel_action(CHEATS_FRONTEND_PANEL, CHEAT_CONTROL_ENABLED,
                                NULL, -1, error, sizeof(error)));
    CHECK(cheats_at(0, &record) && record.enabled);
    CHECK(frontend_panel_action(CHEATS_FRONTEND_PANEL, CHEAT_CONTROL_REMOVE,
                                NULL, -1, error, sizeof(error)));
    CHECK(cheats_count() == 0);
    cheat_frontend_destroy(frontend);
    CHECK(!frontend_panel_get(CHEATS_FRONTEND_PANEL, &(FrontendPanelInfo){0}));
    CHECK(nes_file_remove(path) == NES_FILE_OK);
    return 0;
}

int test_cheat_accuracy(void) {
    cheat_checks = 0;
    int failures = 0;
    failures += parse_formats();
    failures += banked_compare_and_bus_hook();
    failures += crud_policy_and_hash();
    failures += persistence_and_retention();
    failures += game_replacement_identity();
    failures += frontend_panel_crud();
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    cheats_set_execution_policy(NES_EXECUTION_LIVE);
    unload_rom();
    printf("Cheats: %u checks, %d failures\n", cheat_checks, failures);
    return failures;
}
