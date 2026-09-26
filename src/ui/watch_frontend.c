/*
 * watch_frontend.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Memory watch table and editing controls. SPDX-License-Identifier: GPL-3.0-or-later */
#include "watch_frontend.h"
#include "frontend_panels.h"
#include "output_guard.h"
#include "platform_frontend.h"
#include "../debugger/debugger.h"
#include "../debugger/memory_watch.h"
#include "../util/file_io.h"

#include <SDL2/SDL.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct WatchFrontend {
    FrontendExecutionRuntime *execution;
    DebugMemorySpace space;
    int width, format, sort;
    bool descending, registered;
    uint32_t selected_id, row_ids[WATCH_PAGE_SIZE];
    size_t page;
    char expression[DEBUG_EXPRESSION_LIMIT], label[48], count[4];
    char path[NES_FILE_PATH_LIMIT], status[256], info[160], selection[384], values[192];
    char cells[WATCH_PAGE_SIZE][5][96];
    const char *items[WATCH_PAGE_SIZE][5];
};

static bool report(WatchFrontend *frontend, char *error, size_t size, const char *message, bool success) {
    snprintf(frontend->status, sizeof(frontend->status), "%s", message);
    if (error && size) snprintf(error, size, "%s", success ? "" : message);
    return success;
}

static bool number(const char *text, uint32_t limit, uint32_t *out) {
    if (!text || !*text) return false;
    uint64_t value = 0;
    for (const char *p = text; *p; ++p) {
        if (*p < '0' || *p > '9') return false;
        value = value * 10 + (unsigned)(*p - '0');
        if (value > limit) return false;
    }
    *out = (uint32_t)value;
    return true;
}

static bool find(uint32_t id, DebugWatchSample *row, size_t *index) {
    if (!id) return false;
    for (size_t i = 0; i < debugger_watch_count(); ++i) {
        if (debug_watch_at(i, row) && row->id == id) {
            if (index) *index = i;
            return true;
        }
    }
    return false;
}

static bool pick(WatchFrontend *frontend, uint32_t id) {
    DebugWatchSample row;
    size_t index;
    if (!find(id, &row, &index)) return false;
    frontend->selected_id = id;
    frontend->page = index / WATCH_PAGE_SIZE;
    frontend->space = row.spec.space;
    frontend->width = row.spec.width - 1;
    frontend->format = (int)row.spec.format;
    strcpy(frontend->expression, row.spec.expression);
    strcpy(frontend->label, row.spec.label);
    strcpy(frontend->count, "1");
    return true;
}

static void preserve_selection(WatchFrontend *frontend) {
    DebugWatchSample row;
    size_t index;
    if (find(frontend->selected_id, &row, &index)) frontend->page = index / WATCH_PAGE_SIZE;
    else frontend->selected_id = 0;
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t error_size) {
    WatchFrontend *frontend = context;
    debug_watch_refresh();
    static const char *space_items[DEBUG_MEMORY_SPACE_COUNT];
    static const char *const widths[] = {"8 bit", "16 bit", "24 bit", "32 bit"};
    static const char *const formats[] = {"Hexadecimal", "Unsigned decimal", "Signed decimal", "Binary", "ASCII"};
    static const char *const columns[] = {"Address", "Current", "Previous", "Changes", "Label"};
    for (unsigned i = 0; i < DEBUG_MEMORY_SPACE_COUNT; ++i) space_items[i] = debug_memory_name(i);
    size_t count = debugger_watch_count(), selected_index = 0;
    size_t pages = count ? (count - 1) / WATCH_PAGE_SIZE + 1 : 1;
    if (frontend->page >= pages) frontend->page = pages - 1;
    DebugWatchSample selected;
    bool chosen = find(frontend->selected_id, &selected, &selected_index);
    frontend->values[0] = '\0';
    if (chosen) {
        snprintf(frontend->selection, sizeof(frontend->selection), "#%" PRIu32 " | %s | %s => $%04" PRIX32
                 " | %s", selected.id, debug_memory_name(selected.spec.space), selected.spec.expression,
                 selected.address, selected.valid ? selected.spec.label : selected.error);
        char current[40] = "Unavailable", previous[40] = "Unavailable";
        if (selected.valid)
            (void)debug_watch_format(selected.current, selected.spec.width, selected.spec.format, current, sizeof(current));
        if (selected.previous_valid)
            (void)debug_watch_format(selected.previous, selected.spec.width, selected.spec.format, previous, sizeof(previous));
        snprintf(frontend->values, sizeof(frontend->values), "Current %s | Previous %s | Changes %" PRIu32,
                 current, previous, selected.changes);
    } else {
        frontend->selected_id = 0;
        snprintf(frontend->selection, sizeof(frontend->selection), "Select a row to edit, disable or reorder it.");
    }
    snprintf(frontend->info, sizeof(frontend->info), "%zu of %u watches | Page %zu of %zu | "
             "Hex addresses; #decimal, %%binary, PC/A/X/Y/SP", count, DEBUG_WATCH_LIMIT, frontend->page + 1, pages);
    FrontendPanelControl controls[] = {
        {WATCH_SPACE, FRONTEND_PANEL_CHOICE, "Memory space", NULL, space_items, DEBUG_MEMORY_SPACE_COUNT,
         (int)frontend->space, true, false},
        {WATCH_EXPRESSION, FRONTEND_PANEL_TEXT, "Address expression", frontend->expression, NULL, 0, 0, true, false},
        {WATCH_WIDTH, FRONTEND_PANEL_CHOICE, "Width (little endian)", NULL, widths, 4, frontend->width, true, false},
        {WATCH_FORMAT, FRONTEND_PANEL_CHOICE, "Display format", NULL, formats, DEBUG_WATCH_FORMAT_COUNT,
         frontend->format, true, false},
        {WATCH_LABEL, FRONTEND_PANEL_TEXT, "Label", frontend->label, NULL, 0, 0, true, false},
        {WATCH_COUNT, FRONTEND_PANEL_TEXT, "Array count (decimal)", frontend->count, NULL, 0, 0, true, false},
        {WATCH_ADD, FRONTEND_PANEL_ACTION, "Add / add array", "", NULL, 0, 0, count < DEBUG_WATCH_LIMIT, false},
        {WATCH_UPDATE, FRONTEND_PANEL_ACTION, "Update selected", "", NULL, 0, 0, chosen, false},
        {WATCH_TOGGLE, FRONTEND_PANEL_ACTION, "Enable / disable", "", NULL, 0, 0, chosen, false},
        {WATCH_REMOVE, FRONTEND_PANEL_ACTION, "Remove selected", "", NULL, 0, 0, chosen, false},
        {WATCH_PREVIOUS, FRONTEND_PANEL_ACTION, "Capture previous", "", NULL, 0, 0, count != 0, false},
        {WATCH_FREEZE, FRONTEND_PANEL_CHECKBOX, "Freeze previous values", NULL, NULL, 0,
         debug_watch_previous_frozen(), true, false},
        {WATCH_MOVE_UP, FRONTEND_PANEL_ACTION, "Move up", "", NULL, 0, 0, chosen && selected_index > 0, false},
        {WATCH_MOVE_DOWN, FRONTEND_PANEL_ACTION, "Move down", "", NULL, 0, 0, chosen && selected_index + 1 < count, false},
        {WATCH_COPY, FRONTEND_PANEL_ACTION, "Copy table (CSV)", "", NULL, 0, 0, count != 0, false},
        {WATCH_EXPORT_PATH, FRONTEND_PANEL_FILE_SAVE, "CSV export path", frontend->path, NULL, 0,
         FRONTEND_SAVE_CSV, true, false},
        {WATCH_EXPORT, FRONTEND_PANEL_ACTION, "Export CSV", "", NULL, 0, 0, count != 0 && frontend->path[0], false},
        {WATCH_PREV_PAGE, FRONTEND_PANEL_ACTION, "Previous page", "", NULL, 0, 0, frontend->page > 0, false},
        {WATCH_NEXT_PAGE, FRONTEND_PANEL_ACTION, "Next page", "", NULL, 0, 0, frontend->page + 1 < pages, false},
        {WATCH_INFO, FRONTEND_PANEL_TEXT, "Watch list", frontend->info, NULL, 0, 0, true, true},
        {WATCH_SELECTION, FRONTEND_PANEL_TEXT, "Selection / error", frontend->selection, NULL, 0, 0, true, true},
        {WATCH_VALUES, FRONTEND_PANEL_TEXT, "Selected values", frontend->values, NULL, 0, 0, true, true}
    };
    for (size_t i = 0; i < sizeof(controls) / sizeof(*controls); ++i)
        if (!frontend_panel_add_control(model, &controls[i])) return false;
    for (unsigned col = 0; col < 5; ++col) {
        const char *order = frontend->sort == (int)col ? (frontend->descending ? "descending" : "ascending") : "";
        FrontendPanelControl control = {WATCH_SORT + col, FRONTEND_PANEL_ACTION, columns[col], order,
                                       NULL, 0, 0, count != 0, false};
        if (!frontend_panel_add_control(model, &control)) return false;
    }
    for (unsigned i = 0; i < WATCH_PAGE_SIZE; ++i) {
        DebugWatchSample row;
        bool exists = debug_watch_at(frontend->page * WATCH_PAGE_SIZE + i, &row);
        frontend->row_ids[i] = exists ? row.id : 0;
        memset(frontend->cells[i], 0, sizeof(frontend->cells[i]));
        if (exists) {
            static const char *const spaces[] = {"RAM", "Cart", "CPU", "PPU", "OAM", "PAL"};
            snprintf(frontend->cells[i][0], sizeof(frontend->cells[i][0]), "%s:%04" PRIX32,
                     spaces[row.spec.space], row.address);
            if (row.valid) {
                (void)debug_watch_format(row.current, row.spec.width, row.spec.format,
                                         frontend->cells[i][1], sizeof(frontend->cells[i][1]));
            } else {
                snprintf(frontend->cells[i][1], sizeof(frontend->cells[i][1]), "%s", row.spec.enabled ? "ERROR" : "Disabled");
            }
            if (row.previous_valid)
                (void)debug_watch_format(row.previous, row.spec.width, row.spec.format,
                                         frontend->cells[i][2], sizeof(frontend->cells[i][2]));
            snprintf(frontend->cells[i][3], sizeof(frontend->cells[i][3]), "%" PRIu32, row.changes);
            snprintf(frontend->cells[i][4], sizeof(frontend->cells[i][4]), "%s", row.spec.label);
        }
        for (unsigned col = 0; col < 5; ++col) frontend->items[i][col] = frontend->cells[i][col];
        const char *label = exists && !row.valid ? "Unavailable watch" :
                            exists && row.previous_valid && row.current != row.previous ? "Changed watch" : "Watch";
        FrontendPanelControl control = {WATCH_ROW + i, FRONTEND_PANEL_ACTION, label,
            frontend->cells[i][0], frontend->items[i], 5, exists && row.id == frontend->selected_id, exists, false};
        if (!frontend_panel_add_control(model, &control)) return false;
    }
    model->status = frontend->status;
    if (error && error_size) error[0] = '\0';
    return true;
}

static DebugWatchSpec editor_spec(const WatchFrontend *frontend, bool enabled) {
    DebugWatchSpec spec = {.space = frontend->space, .width = (uint8_t)(frontend->width + 1),
                          .format = (DebugWatchFormat)frontend->format, .enabled = enabled};
    strcpy(spec.expression, frontend->expression);
    strcpy(spec.label, frontend->label);
    return spec;
}

static bool export_table(WatchFrontend *frontend, bool clipboard, char *error, size_t error_size) {
    char text[DEBUG_WATCH_EXPORT_LIMIT], message[256];
    if (!debug_watch_export(text, sizeof(text)))
        return report(frontend, error, error_size, "The watch table exceeds the export limit", false);
    if (clipboard) {
        if (SDL_SetClipboardText(text) < 0) return report(frontend, error, error_size, SDL_GetError(), false);
    } else {
        if (!frontend_output_path_allowed(frontend->path, frontend->execution, NULL, 0, message, sizeof(message)))
            return report(frontend, error, error_size, message, false);
        NesFileResult result = nes_file_write_atomic(frontend->path, text, strlen(text));
        if (result != NES_FILE_OK) return report(frontend, error, error_size, nes_file_result_message(result), false);
    }
    return report(frontend, error, error_size, clipboard ? "Watch table copied as CSV" : "Watch table exported as CSV", true);
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t error_size) {
    WatchFrontend *frontend = context;
    if (id == WATCH_EXPRESSION || id == WATCH_LABEL || id == WATCH_COUNT || id == WATCH_EXPORT_PATH) {
        char *target = id == WATCH_EXPRESSION ? frontend->expression : id == WATCH_LABEL ? frontend->label :
                       id == WATCH_COUNT ? frontend->count : frontend->path;
        size_t size = id == WATCH_EXPRESSION ? sizeof(frontend->expression) : id == WATCH_LABEL ? sizeof(frontend->label) :
                      id == WATCH_COUNT ? sizeof(frontend->count) : sizeof(frontend->path);
        if (!value || strlen(value) >= size) return report(frontend, error, error_size, "Watch field is too long", false);
        strcpy(target, value);
        return true;
    }
    if (id == WATCH_SPACE && selected >= 0 && selected < DEBUG_MEMORY_SPACE_COUNT) {
        frontend->space = (DebugMemorySpace)selected;
        return true;
    }
    if (id == WATCH_WIDTH && selected >= 0 && selected < 4) { frontend->width = selected; return true; }
    if (id == WATCH_FORMAT && selected >= 0 && selected < DEBUG_WATCH_FORMAT_COUNT) {
        frontend->format = selected;
        return true;
    }
    if (id == WATCH_FREEZE && (selected == 0 || selected == 1)) {
        debug_watch_freeze_previous(selected != 0);
        return true;
    }
    if (id == WATCH_PREVIOUS) {
        debug_watch_capture_previous();
        return report(frontend, error, error_size, "Current values captured as the previous values", true);
    }
    if (id == WATCH_COPY || id == WATCH_EXPORT) return export_table(frontend, id == WATCH_COPY, error, error_size);
    if (id == WATCH_PICK || (id >= WATCH_ROW && id < WATCH_ROW + WATCH_PAGE_SIZE)) {
        uint32_t picked = 0;
        if (id == WATCH_PICK) {
            if (!number(value, UINT32_MAX, &picked)) return report(frontend, error, error_size, "Invalid watch ID", false);
        } else picked = frontend->row_ids[id - WATCH_ROW];
        return pick(frontend, picked) || report(frontend, error, error_size, "The selected watch no longer exists", false);
    }
    if (id >= WATCH_SORT && id < WATCH_SORT + DEBUG_WATCH_SORT_COUNT) {
        int sort = (int)(id - WATCH_SORT);
        bool descending = frontend->sort == sort && !frontend->descending;
        if (!debug_watch_sort((DebugWatchSort)sort, descending)) return false;
        frontend->sort = sort;
        frontend->descending = descending;
        preserve_selection(frontend);
        return true;
    }
    size_t count = debugger_watch_count();
    if (id == WATCH_PREV_PAGE && frontend->page > 0) { --frontend->page; return true; }
    if (id == WATCH_NEXT_PAGE && (frontend->page + 1) * WATCH_PAGE_SIZE < count) { ++frontend->page; return true; }
    char message[256];
    if (id == WATCH_ADD) {
        uint32_t amount;
        if (!number(frontend->count, DEBUG_WATCH_LIMIT, &amount) || !amount)
            return report(frontend, error, error_size, "Array count must be between 1 and 64", false);
        DebugWatchSpec spec = editor_spec(frontend, true);
        uint32_t added = debug_watch_add(&spec, amount, message, sizeof(message));
        if (!added) return report(frontend, error, error_size, message, false);
        (void)pick(frontend, added);
        frontend->sort = -1;
        return report(frontend, error, error_size, amount == 1 ? "Watch added" : "Watch array added", true);
    }
    DebugWatchSample row;
    size_t index;
    if (!find(frontend->selected_id, &row, &index))
        return report(frontend, error, error_size, "Select a watch first", false);
    if (id == WATCH_UPDATE || id == WATCH_TOGGLE) {
        DebugWatchSpec spec = id == WATCH_UPDATE ? editor_spec(frontend, row.spec.enabled) : row.spec;
        if (id == WATCH_TOGGLE) spec.enabled = !spec.enabled;
        if (!debug_watch_update(row.id, &spec, message, sizeof(message)))
            return report(frontend, error, error_size, message, false);
        frontend->sort = -1;
        return report(frontend, error, error_size, "Watch updated", true);
    }
    if (id == WATCH_REMOVE) {
        if (!debugger_remove_watch(row.id)) return false;
        frontend->selected_id = 0;
        count = debugger_watch_count();
        if (count && debug_watch_at(index < count ? index : count - 1, &row)) (void)pick(frontend, row.id);
        return report(frontend, error, error_size, "Watch removed", true);
    }
    if ((id == WATCH_MOVE_UP && index > 0) || (id == WATCH_MOVE_DOWN && index + 1 < count)) {
        if (!debug_watch_move(row.id, id == WATCH_MOVE_UP ? index - 1 : index + 1)) return false;
        preserve_selection(frontend);
        frontend->sort = -1;
        return true;
    }
    return report(frontend, error, error_size, "Watch action is unavailable", false);
}

WatchFrontend *watch_frontend_create(FrontendExecutionRuntime *execution) {
    if (!execution) return NULL;
    WatchFrontend *frontend = calloc(1, sizeof(*frontend));
    if (!frontend) return NULL;
    frontend->execution = execution;
    frontend->sort = -1;
    strcpy(frontend->expression, "$0000");
    strcpy(frontend->count, "1");
    return frontend;
}

bool watch_frontend_register(WatchFrontend *frontend) {
    if (!frontend || frontend->registered) return false;
    FrontendPanelSpec spec = {WATCH_FRONTEND_PANEL, "Memory Watches", "Tools", FRONTEND_PANEL_NEEDS_SESSION,
                             snapshot, action, frontend};
    frontend->registered = frontend_panel_register(&spec);
    return frontend->registered;
}

void watch_frontend_image_changed(WatchFrontend *frontend) {
    if (!frontend) return;
    frontend->selected_id = 0;
    frontend->page = 0;
    frontend->status[0] = '\0';
    memset(frontend->row_ids, 0, sizeof(frontend->row_ids));
}

void watch_frontend_destroy(WatchFrontend *frontend) {
    if (!frontend) return;
    if (frontend->registered) (void)frontend_panel_unregister(WATCH_FRONTEND_PANEL);
    free(frontend);
}

bool watch_frontend_select(uint32_t id) {
    char text[16];
    snprintf(text, sizeof(text), "%" PRIu32, id);
    return frontend_panel_action(WATCH_FRONTEND_PANEL, WATCH_PICK, text, 0, NULL, 0);
}
