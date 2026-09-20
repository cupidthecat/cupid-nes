/*
 * cheat_frontend.c - Desktop cheat-code controls
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "cheat_frontend.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "../cheats/cheats.h"
#include "../util/file_io.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { CHEAT_UI_LIMIT = 256 };

struct CheatFrontend {
    int selected;
    char draft_code[32];
    char draft_description[96];
    bool draft_enabled;
    char storage_directory[NES_FILE_PATH_LIMIT];
    char path[NES_FILE_PATH_LIMIT];
    char status[256];
    char info[192];
    char labels[CHEAT_UI_LIMIT + 1][144];
    const char *items[CHEAT_UI_LIMIT + 1];
    bool registered;
};

static bool fail(CheatFrontend *frontend, char *error, size_t error_size,
                 const char *message) {
    if (frontend) snprintf(frontend->status, sizeof(frontend->status), "%s", message);
    if (error && error_size) snprintf(error, error_size, "%s", message);
    return false;
}

static bool report(CheatFrontend *frontend, CheatResult result, const char *success,
                   char *error, size_t error_size) {
    if (result != CHEAT_OK)
        return fail(frontend, error, error_size, cheats_result_message(result));
    snprintf(frontend->status, sizeof(frontend->status), "%s", success);
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool selected_record(CheatFrontend *frontend, CheatRecord *record, size_t *index) {
    if (!frontend || frontend->selected <= 0) return false;
    size_t at = (size_t)(frontend->selected - 1);
    if (!cheats_at(at, record)) return false;
    if (index) *index = at;
    return true;
}

static bool save_default(CheatFrontend *frontend, char *error, size_t error_size) {
    if (!frontend || !frontend->path[0]) return true;
    return report(frontend, cheats_save_file(frontend->path), "Cheats saved",
                  error, error_size);
}

static bool edit_selected(CheatFrontend *frontend, const char *code,
                          const char *description, bool enabled,
                          char *error, size_t error_size) {
    CheatRecord record;
    if (!selected_record(frontend, &record, NULL))
        return fail(frontend, error, error_size, "Choose a cheat first");
    CheatResult result = cheats_edit(record.id, code ? code : record.code,
                                     description ? description : record.description,
                                     enabled);
    if (!report(frontend, result, "Cheat updated", error, error_size)) return false;
    return save_default(frontend, error, error_size);
}

static void select_new(CheatFrontend *frontend) {
    frontend->selected = 0;
    frontend->draft_code[0] = '\0';
    frontend->draft_description[0] = '\0';
    frontend->draft_enabled = true;
}

static bool add_cheat(void *context, char *error, size_t error_size) {
    CheatFrontend *frontend = context;
    if (!frontend || frontend->selected != 0)
        return fail(frontend, error, error_size, "Select New cheat before adding");
    uint32_t id = 0;
    if (!report(frontend, cheats_add(frontend->draft_code, frontend->draft_description,
                                     frontend->draft_enabled, &id),
                "Cheat added", error, error_size)) return false;
    frontend->selected = (int)cheats_count();
    snprintf(frontend->status, sizeof(frontend->status), "Cheat #%u added", id);
    return save_default(frontend, error, error_size);
}

static bool toggle_cheat(void *context, char *error, size_t error_size) {
    CheatFrontend *frontend = context;
    CheatRecord record;
    if (!selected_record(frontend, &record, NULL))
        return fail(frontend, error, error_size, "Choose a cheat first");
    if (!report(frontend, cheats_set_enabled(record.id, !record.enabled),
                record.enabled ? "Cheat disabled" : "Cheat enabled",
                error, error_size)) return false;
    return save_default(frontend, error, error_size);
}

static bool refresh_items(CheatFrontend *frontend, size_t *count) {
    size_t records = cheats_count();
    if (records > CHEAT_UI_LIMIT) records = CHEAT_UI_LIMIT;
    strcpy(frontend->labels[0], "New cheat");
    frontend->items[0] = frontend->labels[0];
    for (size_t i = 0; i < records; ++i) {
        CheatRecord record;
        if (!cheats_at(i, &record)) return false;
        snprintf(frontend->labels[i + 1], sizeof(frontend->labels[i + 1]),
                 "%s #%u %s %.72s", record.enabled ? "[x]" : "[ ]", record.id,
                 record.code, record.description);
        frontend->items[i + 1] = frontend->labels[i + 1];
    }
    *count = records + 1;
    if (frontend->selected < 0 || (size_t)frontend->selected >= *count)
        frontend->selected = 0;
    return true;
}

static bool snapshot(void *context, FrontendPanelModel *model,
                     char *error, size_t error_size) {
    CheatFrontend *frontend = context;
    size_t count = 0;
    if (!frontend || !refresh_items(frontend, &count)) return false;
    const char *code = frontend->draft_code;
    const char *description = frontend->draft_description;
    bool enabled = frontend->draft_enabled;
    CheatRecord record;
    if (selected_record(frontend, &record, NULL)) {
        code = record.code;
        description = record.description;
        enabled = record.enabled;
        snprintf(frontend->info, sizeof(frontend->info),
                 "#%u %s | $%04X = $%02X%s", record.id,
                 record.format == CHEAT_FORMAT_GAME_GENIE ? "Game Genie"
                 : record.format == CHEAT_FORMAT_PAR ? "Pro Action Replay" : "Raw",
                 record.address, record.value, record.has_compare ? " | compare active" : "");
    } else {
        CheatRecord parsed;
        CheatResult result = cheats_parse(frontend->draft_code, &parsed);
        if (frontend->draft_code[0] && result == CHEAT_OK)
            snprintf(frontend->info, sizeof(frontend->info),
                     "Valid | $%04X = $%02X%s", parsed.address, parsed.value,
                     parsed.has_compare ? " | compare active" : "");
        else snprintf(frontend->info, sizeof(frontend->info), "%s",
                      frontend->draft_code[0] ? cheats_result_message(result)
                                               : "Enter a Game Genie, PAR, or raw code");
    }
    FrontendPanelControl controls[] = {
        {CHEAT_CONTROL_LIST, FRONTEND_PANEL_CHOICE, "Cheat", NULL,
         frontend->items, count, frontend->selected, true, false},
        {CHEAT_CONTROL_CODE, FRONTEND_PANEL_TEXT, "Code", code, NULL, 0, 0, true, false},
        {CHEAT_CONTROL_DESCRIPTION, FRONTEND_PANEL_TEXT, "Description", description,
         NULL, 0, 0, true, false},
        {CHEAT_CONTROL_ENABLED, FRONTEND_PANEL_CHECKBOX, "Enabled", NULL,
         NULL, 0, enabled, true, false},
        {CHEAT_CONTROL_ADD, FRONTEND_PANEL_ACTION, "Add new cheat", "", NULL, 0, 0,
         frontend->selected == 0, false},
        {CHEAT_CONTROL_REMOVE, FRONTEND_PANEL_ACTION, "Remove selected", "", NULL, 0, 0,
         frontend->selected > 0, false},
        {CHEAT_CONTROL_UP, FRONTEND_PANEL_ACTION, "Move up", "", NULL, 0, 0,
         frontend->selected > 1, false},
        {CHEAT_CONTROL_DOWN, FRONTEND_PANEL_ACTION, "Move down", "", NULL, 0, 0,
         frontend->selected > 0 && (size_t)frontend->selected + 1 < count, false},
        {CHEAT_CONTROL_CLEAR, FRONTEND_PANEL_ACTION, "Clear all cheats", "", NULL, 0, 0,
         cheats_count() != 0, false},
        {CHEAT_CONTROL_PATH, FRONTEND_PANEL_TEXT, "Cheat file", frontend->path,
         NULL, 0, 0, true, false},
        {CHEAT_CONTROL_LOAD, FRONTEND_PANEL_ACTION, "Load cheat file", "", NULL, 0, 0,
         true, false},
        {CHEAT_CONTROL_SAVE, FRONTEND_PANEL_ACTION, "Save cheat file", "", NULL, 0, 0,
         true, false},
        {CHEAT_CONTROL_INFO, FRONTEND_PANEL_TEXT, "Decoded", frontend->info,
         NULL, 0, 0, true, true}
    };
    for (size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); ++i)
        if (!frontend_panel_add_control(model, &controls[i])) return false;
    model->status = frontend->status;
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool action(void *context, unsigned id, const char *value, int selected,
                   char *error, size_t error_size) {
    CheatFrontend *frontend = context;
    CheatRecord record;
    size_t index = 0;
    if (id == CHEAT_CONTROL_LIST && selected >= 0) {
        frontend->selected = selected;
        return true;
    }
    if (id == CHEAT_CONTROL_CODE) {
        if (!value || strlen(value) >= sizeof(frontend->draft_code))
            return fail(frontend, error, error_size, "Cheat code is too long");
        if (frontend->selected == 0) {
            CheatRecord parsed;
            CheatResult result = cheats_parse(value, &parsed);
            if (result != CHEAT_OK)
                return fail(frontend, error, error_size, cheats_result_message(result));
            strcpy(frontend->draft_code, value);
            return true;
        }
        if (!selected_record(frontend, &record, NULL))
            return fail(frontend, error, error_size, "Cheat is unavailable");
        return edit_selected(frontend, value, record.description, record.enabled,
                             error, error_size);
    }
    if (id == CHEAT_CONTROL_DESCRIPTION) {
        if (!value || strlen(value) >= sizeof(frontend->draft_description))
            return fail(frontend, error, error_size, "Description is too long");
        if (frontend->selected == 0) {
            strcpy(frontend->draft_description, value);
            return true;
        }
        if (!selected_record(frontend, &record, NULL))
            return fail(frontend, error, error_size, "Cheat is unavailable");
        return edit_selected(frontend, record.code, value, record.enabled,
                             error, error_size);
    }
    if (id == CHEAT_CONTROL_ENABLED) {
        if (frontend->selected == 0) {
            frontend->draft_enabled = !frontend->draft_enabled;
            return true;
        }
        return toggle_cheat(frontend, error, error_size);
    }
    if (id == CHEAT_CONTROL_ADD) return add_cheat(frontend, error, error_size);
    if (id == CHEAT_CONTROL_REMOVE) {
        if (!selected_record(frontend, &record, NULL))
            return fail(frontend, error, error_size, "Choose a cheat first");
        if (!report(frontend, cheats_remove(record.id), "Cheat removed",
                    error, error_size)) return false;
        if ((size_t)frontend->selected > cheats_count()) frontend->selected--;
        return save_default(frontend, error, error_size);
    }
    if (id == CHEAT_CONTROL_UP || id == CHEAT_CONTROL_DOWN) {
        if (!selected_record(frontend, &record, &index))
            return fail(frontend, error, error_size, "Choose a cheat first");
        size_t target = id == CHEAT_CONTROL_UP ? index - 1u : index + 1u;
        if (!report(frontend, cheats_move(record.id, target), "Cheat moved",
                    error, error_size)) return false;
        frontend->selected = (int)target + 1;
        return save_default(frontend, error, error_size);
    }
    if (id == CHEAT_CONTROL_CLEAR) {
        if (!report(frontend, cheats_clear(), "Cheats cleared",
                    error, error_size)) return false;
        select_new(frontend);
        return save_default(frontend, error, error_size);
    }
    if (id == CHEAT_CONTROL_PATH) {
        if (!value || strlen(value) >= sizeof(frontend->path))
            return fail(frontend, error, error_size, "Cheat file path is too long");
        strcpy(frontend->path, value);
        return true;
    }
    if (id == CHEAT_CONTROL_LOAD) {
        if (!frontend->path[0])
            return fail(frontend, error, error_size, "Choose a cheat file first");
        if (!report(frontend, cheats_load_file(frontend->path), "Cheats loaded",
                    error, error_size)) return false;
        select_new(frontend);
        return true;
    }
    if (id == CHEAT_CONTROL_SAVE) return save_default(frontend, error, error_size);
    return fail(frontend, error, error_size, "Unknown cheat control");
}

CheatFrontend *cheat_frontend_create(const char *storage_directory) {
    if (!storage_directory || !*storage_directory
        || strlen(storage_directory) >= NES_FILE_PATH_LIMIT) return NULL;
    CheatFrontend *frontend = calloc(1, sizeof(*frontend));
    if (!frontend) return NULL;
    strcpy(frontend->storage_directory, storage_directory);
    frontend->draft_enabled = true;
    cheats_init();
    return frontend;
}

bool cheat_frontend_register_ui(CheatFrontend *frontend) {
    if (!frontend || frontend->registered) return false;
    FrontendCommandSpec add = {
        CHEATS_ADD_COMMAND, "Add Configured Cheat", "Tools", "",
        FRONTEND_COMMAND_NEEDS_SESSION, add_cheat, frontend
    };
    FrontendCommandSpec toggle = {
        CHEATS_TOGGLE_COMMAND, "Toggle Selected Cheat", "Tools", "",
        FRONTEND_COMMAND_NEEDS_SESSION, toggle_cheat, frontend
    };
    if (!frontend_command_register(&add) || !frontend_command_register(&toggle)) {
        (void)frontend_command_unregister(CHEATS_ADD_COMMAND);
        (void)frontend_command_unregister(CHEATS_TOGGLE_COMMAND);
        return false;
    }
    FrontendPanelSpec panel = {
        CHEATS_FRONTEND_PANEL, "Cheats", "Tools", FRONTEND_PANEL_NEEDS_SESSION,
        snapshot, action, frontend
    };
    if (!frontend_panel_register(&panel)) {
        (void)frontend_command_unregister(CHEATS_ADD_COMMAND);
        (void)frontend_command_unregister(CHEATS_TOGGLE_COMMAND);
        return false;
    }
    frontend->registered = true;
    return true;
}

void cheat_frontend_image_changed(CheatFrontend *frontend, uint32_t game_identity) {
    if (!frontend) return;
    cheats_set_game_identity(game_identity);
    select_new(frontend);
    frontend->status[0] = '\0';
    int written = snprintf(frontend->path, sizeof(frontend->path), "%scheats-%08X.txt",
                           frontend->storage_directory, game_identity);
    if (written < 0 || (size_t)written >= sizeof(frontend->path)) {
        frontend->path[0] = '\0';
        return;
    }
    FILE *file = nes_file_open(frontend->path, "rb");
    if (!file) return;
    fclose(file);
    CheatResult result = cheats_load_file(frontend->path);
    if (result != CHEAT_OK)
        snprintf(frontend->status, sizeof(frontend->status), "%s",
                 cheats_result_message(result));
}

void cheat_frontend_destroy(CheatFrontend *frontend) {
    if (!frontend) return;
    if (frontend->registered) {
        (void)frontend_command_unregister(CHEATS_ADD_COMMAND);
        (void)frontend_command_unregister(CHEATS_TOGGLE_COMMAND);
        (void)frontend_panel_unregister(CHEATS_FRONTEND_PANEL);
    }
    free(frontend);
}
