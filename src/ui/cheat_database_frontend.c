/*
 * cheat_database_frontend.c - Preview and import game-matched cheats
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "cheat_database_frontend.h"
#include "frontend_panels.h"
#include "platform_frontend.h"
#include "../cheats/cheat_database.h"
#include "../system/execution_policy.h"
#include <stdio.h>
#include <string.h>

enum { LOAD = 1, SELECT, PREVIOUS, NEXT, ENABLED, ADD, IDENTITY, GAME, CODES, PAGE_SIZE = 16 };

static struct {
    CheatDatabase *database;
    char sha1[41], path[1024], status[256], identity[64];
    CheatDatabaseEntry entries[PAGE_SIZE], selected;
    const char *labels[PAGE_SIZE];
    size_t first, count, total, selected_index;
    bool enabled;
} catalog;

static bool editable(void) {
    return !(nes_execution_policy() & (NES_EXECUTION_MOVIE_RECORDING | NES_EXECUTION_MOVIE_PLAYBACK |
                                       NES_EXECUTION_NETPLAY | NES_EXECUTION_REWIND | NES_EXECUTION_SPECULATIVE));
}

void cheat_database_frontend_image_changed(void) {
    catalog.sha1[0] = 0;
    (void)cheat_database_game_identity(catalog.sha1);
    catalog.first = catalog.selected_index = 0;
    memset(&catalog.selected, 0, sizeof(catalog.selected));
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    (void)context;
    (void)error;
    (void)size;
    catalog.total = cheat_database_count(catalog.database, catalog.sha1);
    catalog.count = 0;
    for (size_t i = 0; i < PAGE_SIZE && catalog.first + i < catalog.total; ++i) {
        if (!cheat_database_entry(catalog.database, catalog.sha1, catalog.first + i, &catalog.entries[i])) {
            break;
        }
        catalog.labels[i] = catalog.entries[i].description;
        ++catalog.count;
    }
    bool selected = cheat_database_entry(catalog.database, catalog.sha1, catalog.selected_index, &catalog.selected);
    snprintf(catalog.identity, sizeof(catalog.identity), "%s", catalog.sha1[0] ? catalog.sha1 : "No cartridge loaded");
    FrontendPanelControl controls[] = {
        {IDENTITY, FRONTEND_PANEL_TEXT, "PRG SHA-1", catalog.identity, NULL, 0, 0, false, true},
        {LOAD, FRONTEND_PANEL_FILE_OPEN, "Load database (.tsv)", catalog.path, NULL, 0, FRONTEND_OPEN_CHEAT_DATABASE, true,
         false},
        {SELECT, FRONTEND_PANEL_LIST, "Matching cheats", NULL, catalog.labels, catalog.count,
         selected ? (int)(catalog.selected_index - catalog.first) : -1, catalog.count != 0, false},
        {PREVIOUS, FRONTEND_PANEL_ACTION, "Previous page", NULL, NULL, 0, 0, catalog.first != 0, false},
        {NEXT, FRONTEND_PANEL_ACTION, "Next page", NULL, NULL, 0, 0, catalog.first + catalog.count < catalog.total,
         false},
        {GAME, FRONTEND_PANEL_TEXT, "Game", selected ? catalog.selected.game : "No matching catalog entries", NULL, 0,
         0, false, true},
        {CODES, FRONTEND_PANEL_TEXT, "Codes", selected ? catalog.selected.codes : "", NULL, 0, 0, false, true},
        {ENABLED, FRONTEND_PANEL_CHECKBOX, "Enable added cheats", NULL, NULL, 0, catalog.enabled, editable(), false},
        {ADD, FRONTEND_PANEL_ACTION, "Add selected cheat", NULL, NULL, 0, 0, selected && editable(), false}};
    for (size_t i = 0; i < sizeof(controls) / sizeof(*controls); ++i) {
        if (!frontend_panel_add_control(model, &controls[i])) {
            return false;
        }
    }
    model->status = catalog.status;
    return true;
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    (void)context;
    if (error && size) error[0] = 0;
    bool ok = false;
    if (id == LOAD) {
        ok = value && strlen(value) < sizeof(catalog.path) && cheat_database_load(catalog.database, value, error, size);
        if (ok) {
            strcpy(catalog.path, value);
            catalog.first = catalog.selected_index = 0;
        }
    } else if (id == SELECT && selected >= 0 && (size_t)selected < catalog.count) {
        catalog.selected_index = catalog.first + (size_t)selected;
        ok = true;
    } else if (id == PREVIOUS && catalog.first) {
        catalog.first = catalog.first >= PAGE_SIZE ? catalog.first - PAGE_SIZE : 0;
        catalog.selected_index = catalog.first;
        ok = true;
    } else if (id == NEXT && catalog.first + catalog.count < catalog.total) {
        catalog.first += PAGE_SIZE;
        catalog.selected_index = catalog.first;
        ok = true;
    } else if (id == ENABLED && editable()) {
        catalog.enabled = selected != 0;
        ok = true;
    } else if (id == ADD && editable()) {
        char current[41];
        CheatDatabaseEntry entry;
        if (cheat_database_game_identity(current) && !strcmp(current, catalog.sha1) &&
            cheat_database_entry(catalog.database, current, catalog.selected_index, &entry)) {
            CheatResult result = cheat_database_add(&entry, current, catalog.enabled);
            ok = result == CHEAT_OK;
            if (!ok && error && size) {
                snprintf(error, size, "%s", cheats_result_message(result));
            }
        }
    }
    if (ok) {
        snprintf(catalog.status, sizeof(catalog.status), "%s",
                 id == ADD ? "Added to the active cheat list" : "Catalog ready");
        if (error && size) {
            error[0] = 0;
        }
    } else if (error && size && !error[0]) {
        snprintf(error, size, "Cheat database action is unavailable or invalid");
    }
    return ok;
}

bool cheat_database_frontend_register(void) {
    if (catalog.database) return false;
    memset(&catalog, 0, sizeof(catalog));
    catalog.database = cheat_database_create();
    if (!catalog.database) {
        return false;
    }
    cheat_database_frontend_image_changed();
    strcpy(catalog.status, "Choose a matching cheat and review its codes. Added cheats are disabled by default.");
    FrontendPanelSpec panel = {
        CHEAT_DATABASE_PANEL, "Cheat Database", "Tools", FRONTEND_PANEL_NEEDS_SESSION, snapshot, action, NULL};
    if (frontend_panel_register(&panel)) {
        return true;
    }
    cheat_database_destroy(catalog.database);
    catalog.database = NULL;
    return false;
}

void cheat_database_frontend_unregister(void) {
    frontend_panel_unregister(CHEAT_DATABASE_PANEL);
    cheat_database_destroy(catalog.database);
    memset(&catalog, 0, sizeof(catalog));
}
