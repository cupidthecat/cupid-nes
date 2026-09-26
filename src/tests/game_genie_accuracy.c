/*
 * game_genie_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Game Genie conversion and editor regressions. SPDX-License-Identifier: GPL-3.0-or-later */
#include "../cheats/cheats.h"
#include "../system/execution_policy.h"
#include "../ui/cheat_frontend.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"
#include "../ui/game_genie_frontend.h"

#include <stdio.h>
#include <string.h>

#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition);                                            \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

static int vectors(void) {
    static const struct {
        const char *code;
        uint16_t address;
        uint8_t value;
        int compare;
    } cases[] = {{"AAAAAA", 0x8000, 0, -1},        {"AAEAAAAA", 0x8000, 0, 0},   {"NNYNNN", 0xFFFF, 0xFF, -1},
                 {"NNNNNNNN", 0xFFFF, 0xFF, 0xFF}, {"SXIOPO", 0x91D9, 0xAD, -1}, {"GZUXNGEI", 0xAC3F, 0x24, 0xD0}};

    for (size_t i = 0; i < sizeof(cases) / sizeof(*cases); ++i) {
        CheatRecord decoded;
        char encoded[9];
        CHECK(cheats_game_genie_decode(cases[i].code, &decoded) == CHEAT_OK);
        CHECK(decoded.address == cases[i].address && decoded.value == cases[i].value);
        CHECK(decoded.has_compare == (cases[i].compare >= 0));
        CHECK(!decoded.has_compare || decoded.compare == cases[i].compare);
        CHECK(cheats_game_genie_encode(cases[i].address, cases[i].value, cases[i].compare, encoded, sizeof(encoded)) ==
              CHEAT_OK);
        CHECK(!strcmp(encoded, cases[i].code));
    }

    CheatRecord decoded;
    char canonical[9];
    CHECK(cheats_game_genie_decode("aAeAaA", &decoded) == CHEAT_OK);
    CHECK(decoded.address == 0x8000 && decoded.value == 0 && !decoded.has_compare);
    CHECK(cheats_game_genie_encode(decoded.address, decoded.value, -1, canonical, sizeof(canonical)) == CHEAT_OK);
    CHECK(!strcmp(canonical, "AAAAAA"));
    CHECK(cheats_game_genie_decode("aaaaaaaa", &decoded) == CHEAT_OK);
    CHECK(decoded.has_compare && decoded.compare == 0);
    CHECK(cheats_game_genie_encode(decoded.address, decoded.value, decoded.compare, canonical, sizeof(canonical)) ==
          CHEAT_OK);
    CHECK(!strcmp(canonical, "AAEAAAAA"));
    return 0;
}

static int round_trip(uint32_t address, uint32_t value, int compare) {
    char code[9];
    CheatRecord decoded, active;
    CHECK(cheats_game_genie_encode(address, value, compare, code, sizeof(code)) == CHEAT_OK);
    CHECK(strlen(code) == (compare < 0 ? 6u : 8u));
    CHECK(cheats_game_genie_decode(code, &decoded) == CHEAT_OK);
    CHECK(decoded.address == address && decoded.value == value);
    CHECK(decoded.has_compare == (compare >= 0));
    CHECK(!decoded.has_compare || decoded.compare == compare);
    CHECK(cheats_parse(code, &active) == CHEAT_OK);
    CHECK(active.format == decoded.format && active.address == decoded.address && active.value == decoded.value);
    CHECK(active.has_compare == decoded.has_compare && active.compare == decoded.compare);
    return 0;
}

static int all_fields(void) {
    for (uint32_t address = 0x8000; address <= 0xFFFF; ++address) {
        CHECK(round_trip(address, (address >> 7) & 255, -1) == 0);
        CHECK(round_trip(address, address & 255, (address ^ 0xA5) & 255) == 0);
    }

    for (uint32_t value = 0; value <= 255; ++value) {
        for (int compare = 0; compare <= 255; ++compare) {
            CHECK(round_trip(0x8123, value, compare) == 0);
        }
    }

    /* Every byte in every character position, including lower case and
     * non-ASCII input. Only the documented sixteen letters are accepted. */
    const char *letters = "APZLGITYEOXUKSVNapzlgityeoxuksvn";
    for (size_t length = 6; length <= 8; length += 2) {
        for (size_t i = 0; i < length; ++i) {
            for (unsigned byte = 1; byte <= 255; ++byte) {
                char code[9] = "AAAAAAAA";
                code[length] = 0;
                code[i] = (char)byte;
                CheatRecord decoded;
                bool accepted = strchr(letters, (int)byte) != NULL;
                CHECK((cheats_game_genie_decode(code, &decoded) == CHEAT_OK) == accepted);
            }
        }
    }

    return 0;
}

static int rejected_input(void) {
    CheatRecord decoded = {.id = 73, .address = 0x1234, .value = 0x56};
    CheatRecord previous;
    memcpy(&previous, &decoded, sizeof(previous));
    const char *invalid[] = {"",        "A",      "AAAAA",    "AAAAAAA", "AAAAAAAAA", " SXIOPO",
                             "SXIOPO ", "SXIOP!", "12345678", "8000:FF", "\377AAAAA"};
    for (size_t i = 0; i < sizeof(invalid) / sizeof(*invalid); ++i) {
        CHECK(cheats_game_genie_decode(invalid[i], &decoded) == CHEAT_INVALID_CODE);
        CHECK(!memcmp(&previous, &decoded, sizeof(previous)));
    }

    CHECK(cheats_game_genie_decode(NULL, &decoded) == CHEAT_INVALID_ARGUMENT);
    CHECK(cheats_game_genie_decode("AAAAAA", NULL) == CHEAT_INVALID_ARGUMENT);
    CHECK(cheats_game_genie_encode(0x8000, 0, -1, NULL, 9) == CHEAT_INVALID_ARGUMENT);
    char out[10] = "unchanged";
    for (int compare = -1; compare <= 0; ++compare) {
        for (size_t capacity = 0; capacity <= (compare < 0 ? 6u : 8u); ++capacity) {
            CHECK(cheats_game_genie_encode(0x8000, 0, compare, out, capacity) == CHEAT_OUT_OF_RANGE);
            CHECK(!strcmp(out, "unchanged"));
        }
    }

    const uint32_t addresses[] = {0, 0x7FFF, 0x10000, UINT32_MAX};
    for (size_t i = 0; i < sizeof(addresses) / sizeof(*addresses); ++i) {
        CHECK(cheats_game_genie_encode(addresses[i], 0, -1, out, sizeof(out)) == CHEAT_OUT_OF_RANGE);
        CHECK(!strcmp(out, "unchanged"));
    }

    CHECK(cheats_game_genie_encode(0x8000, 256, -1, out, sizeof(out)) == CHEAT_OUT_OF_RANGE);
    CHECK(cheats_game_genie_encode(0x8000, UINT32_MAX, -1, out, sizeof(out)) == CHEAT_OUT_OF_RANGE);
    CHECK(cheats_game_genie_encode(0x8000, 0, -2, out, sizeof(out)) == CHEAT_OUT_OF_RANGE);
    CHECK(cheats_game_genie_encode(0x8000, 0, 256, out, sizeof(out)) == CHEAT_OUT_OF_RANGE);
    CHECK(!strcmp(out, "unchanged"));
    return 0;
}

static bool edit(unsigned id, const char *value) {
    return frontend_panel_action(CHEATS_GAME_GENIE_PANEL, id, value, 0, NULL, 0);
}

static int utility(void) {
    frontend_commands_reset();
    frontend_panels_reset();
    CheatFrontend *editor = cheat_frontend_create("build/");
    CHECK(editor && cheat_frontend_register_ui(editor));
    FrontendPanelInfo info;
    CHECK(frontend_panel_get(CHEATS_GAME_GENIE_PANEL, &info) && info.enabled);
    CHECK(!edit(GAME_GENIE_SEND, NULL));
    CHECK(edit(GAME_GENIE_CODE, "gzuxngei"));
    FrontendPanelControl controls[16];
    FrontendPanelModel model = {.controls = controls, .capacity = 16};
    CHECK(frontend_panel_snapshot(CHEATS_GAME_GENIE_PANEL, &model, NULL, 0));
    CHECK(model.count == 7);
    CHECK(!strcmp(controls[0].value, "GZUXNGEI") && !strcmp(controls[1].value, "AC3F"));
    CHECK(!strcmp(controls[2].value, "24") && !strcmp(controls[3].value, "D0"));

    const struct {
        unsigned id;
        const char *value;
    } invalid[] = {{GAME_GENIE_CODE, "12345678"},
                   {GAME_GENIE_ADDRESS, "7FFF"},
                   {GAME_GENIE_ADDRESS, "10000"},
                   {GAME_GENIE_ADDRESS, "0x8000"},
                   {GAME_GENIE_VALUE, "100"},
                   {GAME_GENIE_VALUE, "-1"},
                   {GAME_GENIE_VALUE, ""},
                   {GAME_GENIE_COMPARE, "G0"},
                   {GAME_GENIE_COMPARE, "100"},
                   {GAME_GENIE_ADDRESS, NULL},
                   {999, "00"}};

    for (size_t i = 0; i < sizeof(invalid) / sizeof(*invalid); ++i) {
        CHECK(!edit(invalid[i].id, invalid[i].value));
        CHECK(!strcmp(controls[0].value, "GZUXNGEI") && !strcmp(controls[1].value, "AC3F"));
        CHECK(!strcmp(controls[2].value, "24") && !strcmp(controls[3].value, "D0"));
    }

    CHECK(edit(GAME_GENIE_COMPARE, ""));
    CHECK(strlen(controls[0].value) == 6 && !controls[3].value[0]);
    CHECK(edit(GAME_GENIE_ADDRESS, "ffff") && edit(GAME_GENIE_VALUE, "ff"));
    CHECK(!strcmp(controls[0].value, "NNYNNN"));
    CHECK(edit(GAME_GENIE_COMPARE, "ff") && !strcmp(controls[0].value, "NNNNNNNN"));
    CHECK(edit(GAME_GENIE_DESCRIPTION, "Example draft"));
    frontend_panel_set_session_active(true);
    uint32_t hash = cheats_compatibility_hash();
    CHECK(edit(GAME_GENIE_SEND, NULL));
    CHECK(cheats_count() == 0 && cheats_compatibility_hash() == hash);
    CHECK(frontend_panel_snapshot(CHEATS_FRONTEND_PANEL, &model, NULL, 0));
    CHECK(controls[0].selected == 0 && !strcmp(controls[1].value, "NNNNNNNN"));
    CHECK(!strcmp(controls[2].value, "Example draft"));
    const uint32_t policies[] = {NES_EXECUTION_MOVIE_RECORDING, NES_EXECUTION_MOVIE_PLAYBACK, NES_EXECUTION_NETPLAY};
    for (size_t i = 0; i < sizeof(policies) / sizeof(*policies); ++i) {
        CHECK(nes_execution_set_policy(policies[i]));
        CHECK(edit(GAME_GENIE_SEND, NULL));
        CHECK(!frontend_panel_action(CHEATS_FRONTEND_PANEL, CHEAT_CONTROL_ADD, NULL, 0, NULL, 0));
        CHECK(cheats_count() == 0 && cheats_compatibility_hash() == hash);
    }

    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    CHECK(frontend_panel_action(CHEATS_FRONTEND_PANEL, CHEAT_CONTROL_ADD, NULL, 0, NULL, 0));
    CheatRecord active;
    CHECK(cheats_at(0, &active) && active.address == 0xFFFF && active.value == 255 && active.compare == 255);
    CHECK(!cheat_frontend_prepare(editor, "invalid", "bad", NULL, 0));
    CHECK(cheats_count() == 1);
    cheat_frontend_destroy(editor);
    CHECK(!frontend_panel_get(CHEATS_GAME_GENIE_PANEL, &info));
    CHECK(!frontend_panel_get(CHEATS_FRONTEND_PANEL, &info));
    CHECK(cheats_clear() == CHEAT_OK);
    frontend_panel_set_session_active(false);
    frontend_command_set_session_active(false);
    return 0;
}

static bool collision_command(void *context, char *error, size_t capacity) {
    (void)error;
    (void)capacity;
    ++*(unsigned *)context;
    return true;
}

static bool collision_panel(void *context, FrontendPanelModel *model, char *error, size_t capacity) {
    (void)model;
    return collision_command(context, error, capacity);
}

static int registration_rollback(void) {
    const unsigned ids[] = {CHEATS_ADD_COMMAND, CHEATS_TOGGLE_COMMAND, CHEATS_FRONTEND_PANEL, CHEATS_GAME_GENIE_PANEL};
    for (size_t i = 0; i < sizeof(ids) / sizeof(*ids); ++i) {
        frontend_commands_reset();
        frontend_panels_reset();
        unsigned calls = 0;
        bool command = i < 2;
        FrontendCommandSpec command_spec = {ids[i], "Existing command", "Tools", "", 0, collision_command, &calls};
        FrontendPanelSpec panel_spec = {ids[i], "Existing panel", "Tools", 0, collision_panel, NULL, &calls};
        CHECK(command ? frontend_command_register(&command_spec) : frontend_panel_register(&panel_spec));
        CheatFrontend *editor = cheat_frontend_create("build/");
        CHECK(editor && !cheat_frontend_register_ui(editor));
        cheat_frontend_destroy(editor);
        CHECK(frontend_command_count() == (command ? 1u : 0u));
        CHECK(frontend_panel_count() == (command ? 0u : 1u));
        FrontendPanelControl control;
        FrontendPanelModel model = {.controls = &control, .capacity = 1};
        CHECK(command ? frontend_command_invoke(ids[i], NULL, 0) : frontend_panel_snapshot(ids[i], &model, NULL, 0));
        CHECK(calls == 1);
        CHECK(command ? frontend_command_unregister(ids[i]) : frontend_panel_unregister(ids[i]));
        editor = cheat_frontend_create("build/");
        CHECK(editor && cheat_frontend_register_ui(editor));
        cheat_frontend_destroy(editor);
        CHECK(frontend_command_count() == 0 && frontend_panel_count() == 0);
    }

    return 0;
}

int test_game_genie_accuracy(void) {
    int failures = vectors() + all_fields() + rejected_input() + utility() + registration_rollback();
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    printf("Game Genie: vectors, field ranges, round trips, utility, session policy and rollback; %d failures\n",
           failures);
    return failures;
}
