/*
 * game_genie_frontend.c - Game Genie conversion and cheat-editor drafts
 *
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "game_genie_frontend.h"
#include "frontend_panels.h"
#include "../cheats/cheats.h"

#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct GameGenieFrontend {
    CheatFrontend *editor;
    char code[9];
    char address[5];
    char value[3];
    char compare[3];
    char description[96];
    char status[256];
    bool registered;
};

static bool report(GameGenieFrontend *frontend, char *error, size_t capacity, const char *message, bool ok) {
    snprintf(frontend->status, sizeof(frontend->status), "%s", message);
    if (error && capacity) {
        snprintf(error, capacity, "%s", ok ? "" : message);
    }

    return ok;
}

static bool hex_value(const char *text, size_t limit, uint32_t *value) {
    if (!text || !*text) {
        return false;
    }

    uint32_t parsed = 0;
    size_t i = 0;
    for (; i < limit && text[i]; ++i) {
        unsigned char c = (unsigned char)text[i];
        unsigned digit;
        if (c >= '0' && c <= '9') {
            digit = c - '0';
        } else if (c >= 'a' && c <= 'f') {
            digit = c - 'a' + 10;
        } else if (c >= 'A' && c <= 'F') {
            digit = c - 'A' + 10;
        } else {
            return false;
        }

        parsed = (parsed << 4) | digit;
    }

    if (text[i]) {
        return false;
    }

    *value = parsed;
    return true;
}

static void set_fields(GameGenieFrontend *frontend, const CheatRecord *code) {
    snprintf(frontend->address, sizeof(frontend->address), "%04X", (unsigned)code->address);
    snprintf(frontend->value, sizeof(frontend->value), "%02X", (unsigned)code->value);
    frontend->compare[0] = '\0';
    if (code->has_compare) {
        snprintf(frontend->compare, sizeof(frontend->compare), "%02X", (unsigned)code->compare);
    }
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t error_size) {
    GameGenieFrontend *frontend = context;
    FrontendPanelControl controls[] = {
        {GAME_GENIE_CODE, FRONTEND_PANEL_TEXT, "Game Genie code", frontend->code, NULL, 0, 0, true, false},
        {GAME_GENIE_ADDRESS, FRONTEND_PANEL_TEXT, "Address (8000-FFFF hex)", frontend->address, NULL, 0, 0, true,
         false},
        {GAME_GENIE_VALUE, FRONTEND_PANEL_TEXT, "Value (00-FF hex)", frontend->value, NULL, 0, 0, true, false},
        {GAME_GENIE_COMPARE, FRONTEND_PANEL_TEXT, "Compare (hex, blank for none)", frontend->compare, NULL, 0, 0, true,
         false},
        {GAME_GENIE_DESCRIPTION, FRONTEND_PANEL_TEXT, "Description", frontend->description, NULL, 0, 0, true, false},
        {GAME_GENIE_SEND, FRONTEND_PANEL_ACTION, "Send to cheat editor", "", NULL, 0, 0,
         frontend_panel_session_active(), false},
        {GAME_GENIE_COPY, FRONTEND_PANEL_ACTION, "Copy code", "", NULL, 0, 0, true, false}};
    for (size_t i = 0; i < sizeof(controls) / sizeof(*controls); ++i) {
        if (!frontend_panel_add_control(model, &controls[i])) {
            return report(frontend, error, error_size, "Game Genie panel is too small", false);
        }
    }

    model->status = frontend->status;
    return true;
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t error_size) {
    GameGenieFrontend *frontend = context;
    (void)selected;
    if (id == GAME_GENIE_SEND) {
        if (!frontend_panel_session_active()) {
            return report(frontend, error, error_size, "Open a game before sending a cheat", false);
        }

        if (!cheat_frontend_prepare(frontend->editor, frontend->code, frontend->description, error, error_size)) {
            return false;
        }

        return report(frontend, error, error_size, "Draft sent to Cheats. Choose Add new cheat to enable it.", true);
    }

    if (id == GAME_GENIE_COPY) {
        if (SDL_SetClipboardText(frontend->code) < 0) {
            return report(frontend, error, error_size, SDL_GetError(), false);
        }

        return report(frontend, error, error_size, "Code copied", true);
    }

    if (id == GAME_GENIE_DESCRIPTION) {
        if (!value || strlen(value) >= sizeof(frontend->description)) {
            return report(frontend, error, error_size, "Description is too long", false);
        }

        strcpy(frontend->description, value);
        return report(frontend, error, error_size, "Description updated", true);
    }

    CheatRecord decoded;
    if (id == GAME_GENIE_CODE) {
        CheatResult result = cheats_game_genie_decode(value, &decoded);
        if (result != CHEAT_OK) {
            return report(frontend, error, error_size, "Use six or eight Game Genie letters: APZLGITYEOXUKSVN", false);
        }

        set_fields(frontend, &decoded);
        (void)cheats_game_genie_encode(decoded.address, decoded.value, decoded.has_compare ? decoded.compare : -1,
                                       frontend->code, sizeof(frontend->code));
        return report(frontend, error, error_size, "Code decoded", true);
    }

    if (id != GAME_GENIE_ADDRESS && id != GAME_GENIE_VALUE && id != GAME_GENIE_COMPARE) {
        return report(frontend, error, error_size, "Unknown Game Genie control", false);
    }

    (void)cheats_game_genie_decode(frontend->code, &decoded);
    uint32_t number = 0;
    bool no_compare = id == GAME_GENIE_COMPARE && value && !*value;
    if (!no_compare && !hex_value(value, id == GAME_GENIE_ADDRESS ? 4 : 2, &number)) {
        return report(frontend, error, error_size, "Enter hexadecimal digits within the displayed range", false);
    }

    uint32_t address = id == GAME_GENIE_ADDRESS ? number : decoded.address;
    uint32_t byte = id == GAME_GENIE_VALUE ? number : decoded.value;
    int compare =
        id == GAME_GENIE_COMPARE ? (no_compare ? -1 : (int)number) : (decoded.has_compare ? decoded.compare : -1);
    char encoded[9];
    CheatResult result = cheats_game_genie_encode(address, byte, compare, encoded, sizeof(encoded));
    if (result != CHEAT_OK) {
        return report(frontend, error, error_size, "Address must be 8000-FFFF; value and compare must be 00-FF", false);
    }

    (void)cheats_game_genie_decode(encoded, &decoded);
    set_fields(frontend, &decoded);
    strcpy(frontend->code, encoded);
    return report(frontend, error, error_size, "Code encoded", true);
}

GameGenieFrontend *game_genie_frontend_create(CheatFrontend *editor) {
    if (!editor) {
        return NULL;
    }

    GameGenieFrontend *frontend = calloc(1, sizeof(*frontend));
    if (frontend) {
        frontend->editor = editor;
        strcpy(frontend->code, "AAAAAA");
        strcpy(frontend->address, "8000");
        strcpy(frontend->value, "00");
        strcpy(frontend->status, "Edit a code or hexadecimal field and press Enter. Blank compare gives six letters.");
    }

    return frontend;
}

bool game_genie_frontend_register(GameGenieFrontend *frontend) {
    if (!frontend || frontend->registered) {
        return false;
    }

    FrontendPanelSpec panel = {CHEATS_GAME_GENIE_PANEL, "Game Genie", "Tools", 0, snapshot, action, frontend};
    frontend->registered = frontend_panel_register(&panel);
    return frontend->registered;
}

void game_genie_frontend_destroy(GameGenieFrontend *frontend) {
    if (frontend && frontend->registered) {
        (void)frontend_panel_unregister(CHEATS_GAME_GENIE_PANEL);
    }

    free(frontend);
}
