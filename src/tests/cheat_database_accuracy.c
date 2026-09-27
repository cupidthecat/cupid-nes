/*
 * cheat_database_accuracy.c - Catalog identity and atomic import regressions
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "board_tests.h"
#include "../cheats/cheat_database.h"
#include "../system/execution_policy.h"
#include "../ui/cheat_database_frontend.h"
#include "../ui/frontend_panels.h"
#include "../ui/platform_frontend.h"
#include "../util/sha1.h"
#include "../util/file_io.h"

static int catalog_checks(void) {
    CheatDatabase *database = cheat_database_create();
    BOARD_CHECK(database);
    const char *identity = "fefa1097449a3a11ebf8c6199e905996c5dc8fbd";
    BOARD_CHECK(cheat_database_count(database, identity) == 2);
    CheatDatabaseEntry entry;
    BOARD_CHECK(cheat_database_entry(database, identity, 0, &entry));
    BOARD_CHECK(!strcmp(entry.codes, "SSASSA"));
    const struct { const char *sha1; const char *codes[2]; } builtins[] = {
        {"fefa1097449a3a11ebf8c6199e905996c5dc8fbd", {"SSASSA", "VZTLTN"}},
        {"979494e7869ac7ab4815fdbd1dc99f893f713fbf", {"SXKVPZAX", "SLTIYG"}},
        {"2ec08f9341003ded125458df8697ca5ef09d2209", {"EIUGVTEY", "SXXTPSSE"}}
    };
    for (size_t game = 0; game < sizeof(builtins) / sizeof(builtins[0]); ++game) {
        BOARD_CHECK(cheat_database_count(database, builtins[game].sha1) == 2);
        for (size_t code = 0; code < 2; ++code) {
            CheatRecord parsed;
            BOARD_CHECK(cheat_database_entry(database, builtins[game].sha1, code, &entry));
            BOARD_CHECK(!strcmp(entry.codes, builtins[game].codes[code]));
            BOARD_CHECK(cheats_parse(entry.codes, &parsed) == CHEAT_OK);
        }
    }
    BOARD_CHECK(!cheat_database_entry(database, "no match", 0, &entry));
    const char rows[] =
        "# custom catalog\n0123456789012345678901234567890123456789\tTest game\tTwo writes\t0010:33;0011:55\n";
    char error[256] = {0};
    BOARD_CHECK(cheat_database_parse(database, rows, sizeof(rows) - 1, error, sizeof(error)));
    BOARD_CHECK(cheat_database_count(database, "0123456789012345678901234567890123456789") == 1);
    BOARD_CHECK(cheat_database_entry(database, "0123456789012345678901234567890123456789", 0, &entry));
    cheats_init();
    uint32_t original;
    BOARD_CHECK(cheats_add("0020:77", "Existing", true, &original) == CHEAT_OK);
    uint32_t hash = cheats_compatibility_hash();
    BOARD_CHECK(cheat_database_add(&entry, identity, false) == CHEAT_INVALID_ARGUMENT && cheats_count() == 1);
    BOARD_CHECK(cheat_database_add(&entry, entry.sha1, false) == CHEAT_OK && cheats_count() == 3);
    BOARD_CHECK(cheats_compatibility_hash() == hash);
    CheatRecord record;
    BOARD_CHECK(cheats_at(1, &record) && record.address == 0x10 && record.value == 0x33 && !record.enabled);
    BOARD_CHECK(cheats_at(2, &record) && record.address == 0x11 && record.value == 0x55 && !record.enabled);
    cheats_set_execution_policy(NES_EXECUTION_MOVIE_PLAYBACK);
    BOARD_CHECK(cheat_database_add(&entry, entry.sha1, true) == CHEAT_DETERMINISTIC_MODE && cheats_count() == 3);
    cheats_set_execution_policy(NES_EXECUTION_LIVE);
    const char *invalid[] = {"0010:01", "bad code"};
    BOARD_CHECK(cheats_add_group(invalid, 2, "Invalid group", true) == CHEAT_INVALID_CODE && cheats_count() == 3);
    const char bad[] = "0123456789012345678901234567890123456789\tTest\tInvalid\t0010:33;bogus\n";
    BOARD_CHECK(!cheat_database_parse(database, bad, sizeof(bad) - 1, error, sizeof(error)));
    BOARD_CHECK(cheat_database_count(database, entry.sha1) == 1 && cheats_count() == 3);
    BOARD_CHECK(!cheat_database_parse(database, "bad\0data", 8, error, sizeof(error)));
    BOARD_CHECK(!cheat_database_parse(database, "missing tabs", 12, error, sizeof(error)));
    BOARD_CHECK(cheat_database_count(database, entry.sha1) == 1);
    cheat_database_destroy(database);
    BOARD_CHECK(cheats_clear() == CHEAT_OK);
    return 0;
}

static int production_identity(void) {
    BoardImage image = {0};
    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    char actual[41], expected[41];
    nes_sha1(prg_rom, prg_size, expected);
    BOARD_CHECK(cheat_database_game_identity(actual) && !strcmp(actual, expected));
    frontend_panels_reset();
    frontend_panel_set_session_active(true);
    BOARD_CHECK(cheat_database_frontend_register());
    char data[256], error[256] = {0};
    int length = snprintf(data, sizeof(data), "%s\tFixture\tRAM test\t0010:44\n", expected);
    const char *path = "build/cheat-catalog.tsv";
    BOARD_CHECK(nes_file_write_atomic(path, data, (size_t)length) == NES_FILE_OK);
    BOARD_CHECK(frontend_panel_action(CHEAT_DATABASE_PANEL, 1, path, 0, error, sizeof(error)));
    FrontendPanelControl controls[16];
    FrontendPanelModel model = {.controls = controls, .capacity = 16};
    BOARD_CHECK(frontend_panel_snapshot(CHEAT_DATABASE_PANEL, &model, error, sizeof(error)));
    BOARD_CHECK(controls[1].selected == FRONTEND_OPEN_CHEAT_DATABASE);
    BOARD_CHECK(controls[2].item_count == 1 && !strcmp(controls[2].items[0], "RAM test"));
    BOARD_CHECK(frontend_panel_action(CHEAT_DATABASE_PANEL, 6, NULL, 0, error, sizeof(error)));
    CheatRecord added;
    BOARD_CHECK(cheats_count() == 1 && cheats_at(0, &added) && !added.enabled && added.address == 0x10);
    uint8_t old = prg_rom[0];
    prg_rom[0] ^= 1;
    BOARD_CHECK(!frontend_panel_action(CHEAT_DATABASE_PANEL, 6, NULL, 0, error, sizeof(error)) && cheats_count() == 1);
    prg_rom[0] = old;
    cheat_database_frontend_unregister();
    frontend_panels_reset();
    BOARD_CHECK(unload_rom());
    board_image_free(&image);
    (void)nes_file_remove(path);
    return 0;
}

int test_cheat_database_accuracy(void) {
    int failures = catalog_checks() + production_identity();
    printf("Cheat database: matching, atomic groups, policies and native panel, %d failures\n", failures);
    return failures;
}
