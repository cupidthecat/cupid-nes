/*
 * hex_frontend.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Native hex editor controls. SPDX-License-Identifier: GPL-3.0-or-later */
#include "hex_frontend.h"
#include "frontend_panels.h"
#include "output_guard.h"
#include "platform_frontend.h"
#include "watch_frontend.h"
#include "../debugger/debugger.h"
#include "../debugger/expression.h"
#include "../debugger/memory_editor.h"
#include "../debugger/memory_watch.h"
#include "../system/execution_policy.h"
#include "../util/file_io.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct HexFrontend {
    FrontendExecutionRuntime *execution;
    DebugMemoryPage page;
    DebugMemorySpace space;
    uint32_t first, last, anchor, cursor;
    bool live, text_mode, wrap, registered;
    char address[DEBUG_EXPRESSION_LIMIT + 1], count[24], bytes[DEBUG_MEMORY_PAGE_SIZE * 3 + 1];
    char pattern[DEBUG_MEMORY_PATTERN_LIMIT * 3 + 1];
    char import_path[NES_FILE_PATH_LIMIT], export_path[NES_FILE_PATH_LIMIT];
    char status[256], info[192], backing[256], selection[192];
    char row_label[HEX_ROWS][8], row_value[HEX_ROWS][96];
    char cells[HEX_ROWS][HEX_COLUMNS][3], ascii[HEX_ROWS][HEX_COLUMNS + 1], changed[HEX_ROWS][HEX_COLUMNS + 1];
    const char *items[HEX_ROWS][HEX_COLUMNS + 2];
};

static bool report(HexFrontend *frontend, char *error, size_t size, const char *message, bool success) {
    snprintf(frontend->status, sizeof(frontend->status), "%s", message);
    if (error && size) snprintf(error, size, "%s", success ? "" : message);
    return success;
}

static bool capture(HexFrontend *frontend, uint32_t first, bool baseline) {
    if (!frontend_panel_session_active()) return false;
    frontend_execution_begin_machine_change(frontend->execution);
    bool ok = debug_memory_capture(&frontend->page, frontend->space, first, baseline);
    frontend_execution_end_machine_change_preserving_audio(frontend->execution);
    return ok;
}

static bool current(const HexFrontend *frontend) {
    return frontend_panel_session_active() && frontend->page.valid
        && frontend->page.session == debugger_session_revision();
}

static bool editable(const HexFrontend *frontend) {
    return current(frontend) && frontend->live && frontend_execution_paused(frontend->execution)
        && nes_execution_policy() == NES_EXECUTION_LIVE;
}

static size_t selected_count(const HexFrontend *frontend) {
    return (size_t)(frontend->last - frontend->first) + 1;
}

static void selection_fields(HexFrontend *frontend) {
    snprintf(frontend->address, sizeof(frontend->address), "$%04" PRIX32, frontend->first);
    snprintf(frontend->count, sizeof(frontend->count), "%zu", selected_count(frontend));
}

static bool select_range(HexFrontend *frontend, uint32_t first, size_t count, uint32_t cursor, bool extend) {
    uint32_t low, high;
    if (!count || !debug_memory_bounds(frontend->space, &low, &high) || first < low || first > high
        || count > (size_t)(high - first) + 1 || cursor < first || cursor - first >= count)
        return false;
    uint32_t page = low + ((cursor - low) / DEBUG_MEMORY_PAGE_SIZE) * DEBUG_MEMORY_PAGE_SIZE;
    if ((!current(frontend) || page != frontend->page.first) && !capture(frontend, page, true)) return false;
    frontend->first = first;
    frontend->last = first + (uint32_t)count - 1;
    frontend->cursor = cursor;
    if (!extend) frontend->anchor = first;
    selection_fields(frontend);
    return true;
}

static bool decimal(const char *text, uint32_t *out) {
    uint32_t value = 0;
    if (!text || !*text) return false;
    for (const char *p = text; *p; ++p) {
        if (*p < '0' || *p > '9') return false;
        value = value * 10 + (unsigned)(*p - '0');
        if (value > DEBUG_MEMORY_EDIT_LIMIT) return false;
    }
    *out = value;
    return value != 0;
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t error_size) {
    HexFrontend *frontend = context;
    uint32_t low, high;
    if (!debug_memory_bounds(frontend->space, &low, &high)) return false;
    if (frontend->first < low || frontend->last > high) {
        frontend->first = frontend->last = frontend->anchor = frontend->cursor = low;
        frontend->page.valid = false;
        selection_fields(frontend);
    }
    uint32_t page_start = low + ((frontend->cursor - low) / DEBUG_MEMORY_PAGE_SIZE) * DEBUG_MEMORY_PAGE_SIZE;
    if ((frontend->live || !current(frontend)) && !capture(frontend, page_start, false))
        return report(frontend, error, error_size, "Memory is unavailable; load a game first", false);
    bool can_edit = editable(frontend);
    const DebugMemoryByte *byte = &frontend->page.bytes[frontend->cursor - frontend->page.first];
    snprintf(frontend->info, sizeof(frontend->info), "$%04" PRIX32 "-$%04" PRIX32
             " | %s | * value or backing changed; ?? unreadable", frontend->page.first,
             frontend->page.first + (uint32_t)frontend->page.count - 1,
             frontend->live ? "Live view" : "Frozen view");
    snprintf(frontend->backing, sizeof(frontend->backing), "$%04" PRIX32 " | %s%s + $%zX | %s",
             frontend->cursor, byte->physical ? "" : "Observational: ", byte->backing, byte->offset,
             !byte->readable ? "Unreadable" : !byte->writable ? "Read only" : can_edit ? "Writable" : "Pause a live game to edit");
    snprintf(frontend->selection, sizeof(frontend->selection), "$%04" PRIX32 "-$%04" PRIX32
             " (%zu bytes). Shift-click or Shift-arrow extends the selection.",
             frontend->first, frontend->last, selected_count(frontend));
    static const char *space_items[DEBUG_MEMORY_SPACE_COUNT];
    for (unsigned i = 0; i < DEBUG_MEMORY_SPACE_COUNT; ++i) space_items[i] = debug_memory_name(i);
    FrontendPanelControl controls[] = {
        {HEX_SPACE, FRONTEND_PANEL_CHOICE, "Memory space", NULL, space_items, DEBUG_MEMORY_SPACE_COUNT,
         (int)frontend->space, true, false},
        {HEX_ADDRESS, FRONTEND_PANEL_TEXT, "Address / expression", frontend->address, NULL, 0, 0, true, false},
        {HEX_COUNT, FRONTEND_PANEL_TEXT, "Selection bytes (decimal)", frontend->count, NULL, 0, 0, true, false},
        {HEX_SELECT, FRONTEND_PANEL_ACTION, "Go / select", "", NULL, 0, 0, true, false},
        {HEX_LIVE, FRONTEND_PANEL_CHECKBOX, "Live view", NULL, NULL, 0, frontend->live, true, false},
        {HEX_REFRESH, FRONTEND_PANEL_ACTION, "Refresh", "", NULL, 0, 0, true, false},
        {HEX_BASELINE, FRONTEND_PANEL_ACTION, "Reset changed bytes", "", NULL, 0, 0, true, false},
        {HEX_PREV_PAGE, FRONTEND_PANEL_ACTION, "Previous page", "", NULL, 0, 0, frontend->page.first > low, false},
        {HEX_NEXT_PAGE, FRONTEND_PANEL_ACTION, "Next page", "", NULL, 0, 0,
         high - frontend->page.first >= DEBUG_MEMORY_PAGE_SIZE, false},
        {HEX_BYTES, FRONTEND_PANEL_TEXT, "Replacement bytes / text", frontend->bytes, NULL, 0, 0, true, false},
        {HEX_TEXT_MODE, FRONTEND_PANEL_CHECKBOX, "Text mode (UTF-8)", NULL, NULL, 0, frontend->text_mode, true, false},
        {HEX_APPLY, FRONTEND_PANEL_ACTION, "Apply at selection", "", NULL, 0, 0, can_edit && frontend->bytes[0], false},
        {HEX_COPY, FRONTEND_PANEL_ACTION, "Copy selection", "", NULL, 0, 0, true, false},
        {HEX_PASTE, FRONTEND_PANEL_ACTION, "Paste at selection", "", NULL, 0, 0, can_edit, false},
        {HEX_PATTERN, FRONTEND_PANEL_TEXT, "Find bytes / text", frontend->pattern, NULL, 0, 0, true, false},
        {HEX_FIND, FRONTEND_PANEL_ACTION, "Find next", "", NULL, 0, 0, frontend->pattern[0] != 0, false},
        {HEX_WRAP, FRONTEND_PANEL_CHECKBOX, "Wrap search", NULL, NULL, 0, frontend->wrap, true, false},
        {HEX_IMPORT_PATH, FRONTEND_PANEL_FILE_OPEN, "Binary import path", frontend->import_path,
         NULL, 0, FRONTEND_OPEN_MEMORY, true, false},
        {HEX_IMPORT, FRONTEND_PANEL_ACTION, "Import at selection", "", NULL, 0, 0, can_edit && frontend->import_path[0], false},
        {HEX_EXPORT_PATH, FRONTEND_PANEL_FILE_SAVE, "Binary export path", frontend->export_path,
         NULL, 0, FRONTEND_SAVE_MEMORY, true, false},
        {HEX_EXPORT, FRONTEND_PANEL_ACTION, "Export selection", "", NULL, 0, 0, frontend->export_path[0] != 0, false},
        {HEX_WATCH, FRONTEND_PANEL_ACTION, "Add watch / array", "", NULL, 0, 0,
         selected_count(frontend) <= DEBUG_WATCH_LIMIT, false},
        {HEX_INFO, FRONTEND_PANEL_TEXT, "Memory page", frontend->info, NULL, 0, 0, true, true},
        {HEX_BACKING, FRONTEND_PANEL_TEXT, "Backing / protection", frontend->backing, NULL, 0, 0, true, true},
        {HEX_SELECTION, FRONTEND_PANEL_TEXT, "Selection", frontend->selection, NULL, 0,
         (int)frontend->cursor, true, true}
    };
    for (size_t i = 0; i < sizeof(controls) / sizeof(*controls); ++i)
        if (!frontend_panel_add_control(model, &controls[i])) return false;
    for (unsigned row = 0; row < HEX_ROWS; ++row) {
        size_t start = row * HEX_COLUMNS;
        bool exists = start < frontend->page.count;
        uint32_t address = frontend->page.first + (uint32_t)start;
        unsigned selection = 0;
        frontend->row_value[row][0] = '\0';
        snprintf(frontend->row_label[row], sizeof(frontend->row_label[row]), "$%04" PRIX32, address);
        for (unsigned col = 0; col < HEX_COLUMNS; ++col) {
            bool present = start + col < frontend->page.count;
            const DebugMemoryByte *cell = present ? &frontend->page.bytes[start + col] : NULL;
            if (present && cell->readable) snprintf(frontend->cells[row][col], 3, "%02X", cell->value);
            else strcpy(frontend->cells[row][col], present ? "??" : "  ");
            frontend->ascii[row][col] = !present ? ' ' : cell->readable && cell->value >= 32 && cell->value <= 126
                ? (char)cell->value : '.';
            frontend->changed[row][col] = present && cell->changed ? '*' : ' ';
            frontend->items[row][col] = frontend->cells[row][col];
            if (present && address + col >= frontend->first && address + col <= frontend->last) selection |= 1u << col;
            snprintf(frontend->row_value[row] + col * 3, sizeof(frontend->row_value[row]) - col * 3,
                     "%s ", frontend->cells[row][col]);
        }
        frontend->ascii[row][HEX_COLUMNS] = frontend->changed[row][HEX_COLUMNS] = '\0';
        frontend->items[row][HEX_COLUMNS] = frontend->ascii[row];
        frontend->items[row][HEX_COLUMNS + 1] = frontend->changed[row];
        snprintf(frontend->row_value[row] + HEX_COLUMNS * 3,
                 sizeof(frontend->row_value[row]) - HEX_COLUMNS * 3, "|%s|", frontend->ascii[row]);
        FrontendPanelControl control = {HEX_ROW + row, FRONTEND_PANEL_ACTION, frontend->row_label[row],
            frontend->row_value[row], frontend->items[row], HEX_COLUMNS + 2, (int)selection, exists, false};
        if (!frontend_panel_add_control(model, &control)) return false;
    }
    model->status = frontend->status;
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool apply(HexFrontend *frontend, const uint8_t *bytes, size_t count, char *error, size_t error_size) {
    if (!editable(frontend))
        return report(frontend, error, error_size,
                      "Refresh the view and pause a live game to edit. Frozen, movie, rewind and netplay views are read-only", false);
    frontend_execution_begin_machine_change(frontend->execution);
    debugger_pause();
    frontend_execution_sync_debugger(frontend->execution);
    bool changed = false;
    char message[256];
    bool ok = debug_memory_edit(frontend->space, frontend->first, bytes, count, frontend->page.session,
                                &changed, message, sizeof(message));
    if (ok && changed) frontend_execution_clear_timeline(frontend->execution);
    frontend_execution_end_machine_change_preserving_audio(frontend->execution);
    if (!ok) return report(frontend, error, error_size, message, false);
    (void)select_range(frontend, frontend->first, count, frontend->first, false);
    (void)capture(frontend, frontend->page.first, false);
    return report(frontend, error, error_size, changed ? "Memory updated; rewind history cleared" : "The bytes already match", true);
}

static bool apply_text(HexFrontend *frontend, const char *text, char *error, size_t error_size) {
    uint8_t *bytes = malloc(DEBUG_MEMORY_EDIT_LIMIT);
    if (!bytes) return report(frontend, error, error_size, "Could not allocate the paste buffer", false);
    size_t count = 0;
    char message[256];
    bool ok = debug_memory_parse_bytes(text, frontend->text_mode, bytes, DEBUG_MEMORY_EDIT_LIMIT,
                                       &count, message, sizeof(message));
    if (ok) ok = apply(frontend, bytes, count, error, error_size);
    else (void)report(frontend, error, error_size, message, false);
    free(bytes);
    return ok;
}

static bool read_selection(HexFrontend *frontend, uint8_t *bytes, size_t count, char *error, size_t error_size) {
    if (!current(frontend)) return report(frontend, error, error_size, "Refresh the memory view first", false);
    if (!frontend->live) {
        if (frontend->first < frontend->page.first
            || frontend->last - frontend->page.first >= frontend->page.count)
            return report(frontend, error, error_size, "A frozen selection must fit in the captured page; enable Live for larger ranges", false);
        for (size_t i = 0; i < count; ++i) {
            const DebugMemoryByte *byte = &frontend->page.bytes[frontend->first - frontend->page.first + i];
            if (!byte->readable) return report(frontend, error, error_size, "The selection contains an unreadable byte", false);
            bytes[i] = byte->value;
        }
        return true;
    }
    char message[256];
    frontend_execution_begin_machine_change(frontend->execution);
    bool ok = debug_memory_copy(frontend->space, frontend->first, count, bytes, count, message, sizeof(message));
    frontend_execution_end_machine_change_preserving_audio(frontend->execution);
    return ok || report(frontend, error, error_size, message, false);
}

/* SDL clipboards contain UTF-8, not arbitrary byte strings or embedded NULs. */
static bool clipboard_text(const uint8_t *bytes, size_t count) {
    for (size_t i = 0; i < count;) {
        uint32_t code = bytes[i++], minimum;
        unsigned following;
        if (!code) return false;
        if (code < 0x80) continue;
        if (code >= 0xC2 && code <= 0xDF) { code &= 0x1F; minimum = 0x80; following = 1; }
        else if (code >= 0xE0 && code <= 0xEF) { code &= 0x0F; minimum = 0x800; following = 2; }
        else if (code >= 0xF0 && code <= 0xF4) { code &= 7; minimum = 0x10000; following = 3; }
        else return false;
        if (following > count - i) return false;
        while (following--) {
            uint8_t next = bytes[i++];
            if ((next & 0xC0) != 0x80) return false;
            code = (code << 6) | (next & 0x3F);
        }
        if (code < minimum || code > 0x10FFFF || (code >= 0xD800 && code <= 0xDFFF)) return false;
    }
    return true;
}

static bool copy_or_export(HexFrontend *frontend, bool clipboard, char *error, size_t error_size) {
    size_t count = selected_count(frontend);
    uint8_t *bytes = malloc(count + 1);
    if (!bytes) return report(frontend, error, error_size, "Could not allocate the memory selection", false);
    bool ok = read_selection(frontend, bytes, count, error, error_size);
    char message[256];
    if (ok && clipboard) {
        bytes[count] = 0;
        char *text = NULL;
        if (frontend->text_mode) {
            if (!clipboard_text(bytes, count))
                ok = report(frontend, error, error_size, "The selection is not UTF-8 text without NUL bytes; copy it in hex mode", false);
            else text = (char *)bytes;
        } else {
            text = malloc(count * 3);
            if (!text || !debug_memory_format_hex(bytes, count, text, count * 3))
                ok = report(frontend, error, error_size, "Could not format the memory selection", false);
        }
        if (ok && SDL_SetClipboardText(text) < 0) ok = report(frontend, error, error_size, SDL_GetError(), false);
        if (text != (char *)bytes) free(text);
    } else if (ok) {
        if (!frontend_output_path_allowed(frontend->export_path, frontend->execution, NULL, 0, message, sizeof(message)))
            ok = report(frontend, error, error_size, message, false);
        else {
            NesFileResult result = nes_file_write_atomic(frontend->export_path, bytes, count);
            if (result != NES_FILE_OK) ok = report(frontend, error, error_size, nes_file_result_message(result), false);
        }
    }
    free(bytes);
    return ok && report(frontend, error, error_size, clipboard ? "Selection copied" : "Selection exported as binary bytes", true);
}

static bool add_watch(HexFrontend *frontend, char *error, size_t error_size) {
    FrontendPanelInfo panel;
    size_t count = selected_count(frontend);
    if (!current(frontend) || count > DEBUG_WATCH_LIMIT
        || !frontend_panel_get(WATCH_FRONTEND_PANEL, &panel) || !panel.enabled)
        return report(frontend, error, error_size, "Select up to 64 bytes in a current view with Memory Watches available", false);
    DebugWatchSpec spec = {.space = frontend->space, .width = count <= 4 ? (uint8_t)count : 1,
        .format = frontend->text_mode ? DEBUG_WATCH_ASCII : DEBUG_WATCH_HEX, .enabled = true};
    snprintf(spec.expression, sizeof(spec.expression), "$%04" PRIX32, frontend->first);
    strcpy(spec.label, "Hex selection");
    size_t rows = count <= 4 ? 1 : count;
    char message[256];
    frontend_execution_begin_machine_change(frontend->execution);
    uint32_t id = debug_watch_add(&spec, rows, message, sizeof(message));
    frontend_execution_end_machine_change_preserving_audio(frontend->execution);
    if (!id) return report(frontend, error, error_size, message, false);
    if (!watch_frontend_select(id)) {
        frontend_execution_begin_machine_change(frontend->execution);
        for (size_t i = 0; i < rows; ++i) (void)debugger_remove_watch(id + (uint32_t)i);
        frontend_execution_end_machine_change_preserving_audio(frontend->execution);
        return report(frontend, error, error_size, "Memory Watches could not select the new watch", false);
    }
    return report(frontend, error, error_size, rows == 1 ? "Watch added" : "Byte watch array added", true);
}

static bool find_frozen(const HexFrontend *frontend, const uint8_t *pattern, size_t count, uint32_t *found) {
    const DebugMemoryPage *page = &frontend->page;
    if (!count || count > page->count) return false;
    size_t end = page->count - count + 1;
    size_t next = frontend->first < page->first ? 0 : (size_t)(frontend->first - page->first) + 1;
    if (next > end) next = end;
    for (unsigned pass = 0; pass < (frontend->wrap ? 2u : 1u); ++pass) {
        size_t start = pass ? 0 : next, limit = pass ? next : end;
        for (size_t i = start; i < limit; ++i) {
            size_t matched = 0;
            while (matched < count && page->bytes[i + matched].readable &&
                   page->bytes[i + matched].value == pattern[matched]) ++matched;
            if (matched == count) { *found = page->first + (uint32_t)i; return true; }
        }
    }
    return false;
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t error_size) {
    HexFrontend *frontend = context;
    char *target = NULL;
    size_t size = 0;
    switch (id) {
        case HEX_ADDRESS: target = frontend->address; size = sizeof(frontend->address); break;
        case HEX_COUNT: target = frontend->count; size = sizeof(frontend->count); break;
        case HEX_BYTES: target = frontend->bytes; size = sizeof(frontend->bytes); break;
        case HEX_PATTERN: target = frontend->pattern; size = sizeof(frontend->pattern); break;
        case HEX_IMPORT_PATH: target = frontend->import_path; size = sizeof(frontend->import_path); break;
        case HEX_EXPORT_PATH: target = frontend->export_path; size = sizeof(frontend->export_path); break;
        default: break;
    }
    if (target) {
        if (!value || strlen(value) >= size) return report(frontend, error, error_size, "The hex editor field is too long", false);
        strcpy(target, value);
        if (error && error_size) error[0] = '\0';
        return true;
    }
    if ((id == HEX_LIVE || id == HEX_TEXT_MODE || id == HEX_WRAP) && (selected == 0 || selected == 1)) {
        if (id == HEX_LIVE) frontend->live = selected != 0;
        else if (id == HEX_TEXT_MODE) frontend->text_mode = selected != 0;
        else frontend->wrap = selected != 0;
        return true;
    }
    uint32_t low, high;
    if (!debug_memory_bounds(frontend->space, &low, &high)) return false;
    if (id == HEX_SPACE && selected >= 0 && selected < DEBUG_MEMORY_SPACE_COUNT) {
        frontend->space = (DebugMemorySpace)selected;
        (void)debug_memory_bounds(frontend->space, &low, &high);
        frontend->page.valid = false;
        return select_range(frontend, low, 1, low, false);
    }
    if (id == HEX_SELECT) {
        uint32_t address, count;
        char message[192];
        if (!debug_expression_eval(frontend->address, &address, message, sizeof(message)))
            return report(frontend, error, error_size, message, false);
        if (!decimal(frontend->count, &count) || !select_range(frontend, address, count, address, false))
            return report(frontend, error, error_size, "The selection is outside this memory space", false);
        return report(frontend, error, error_size, "Selection updated", true);
    }
    if (id == HEX_REFRESH || id == HEX_BASELINE) {
        uint32_t first = low + ((frontend->cursor - low) / DEBUG_MEMORY_PAGE_SIZE) * DEBUG_MEMORY_PAGE_SIZE;
        return capture(frontend, first, id == HEX_BASELINE)
            ? report(frontend, error, error_size, id == HEX_BASELINE ? "Changed-byte baseline reset" : "Memory refreshed", true)
            : report(frontend, error, error_size, "The memory view is unavailable", false);
    }
    if (id == HEX_COPY || id == HEX_EXPORT) return copy_or_export(frontend, id == HEX_COPY, error, error_size);
    if (id == HEX_APPLY) return apply_text(frontend, frontend->bytes, error, error_size);
    if (id == HEX_PASTE) {
        char *text = SDL_GetClipboardText();
        if (!text) return report(frontend, error, error_size, SDL_GetError(), false);
        bool ok = apply_text(frontend, text, error, error_size);
        SDL_free(text);
        return ok;
    }
    if (id == HEX_IMPORT) {
        uint8_t *bytes = NULL;
        size_t count = 0;
        NesFileResult result = nes_file_read_all(frontend->import_path, DEBUG_MEMORY_EDIT_LIMIT, &bytes, &count);
        if (result != NES_FILE_OK) return report(frontend, error, error_size, nes_file_result_message(result), false);
        bool ok = count ? apply(frontend, bytes, count, error, error_size)
            : report(frontend, error, error_size, "The memory import is empty", false);
        free(bytes);
        return ok;
    }
    if (id == HEX_WATCH) return add_watch(frontend, error, error_size);
    if (id == HEX_FIND) {
        if (!current(frontend)) return report(frontend, error, error_size, "Refresh the memory view first", false);
        uint8_t pattern[DEBUG_MEMORY_PATTERN_LIMIT];
        size_t count;
        uint32_t found;
        char message[256];
        if (!debug_memory_parse_bytes(frontend->pattern, frontend->text_mode, pattern, sizeof(pattern),
                                      &count, message, sizeof(message)))
            return report(frontend, error, error_size, message, false);
        if (frontend->first == high && !frontend->wrap)
            return report(frontend, error, error_size, "The byte sequence was not found", false);
        bool ok;
        if (frontend->live) {
            frontend_execution_begin_machine_change(frontend->execution);
            ok = debug_memory_find(frontend->space, frontend->first == high ? low : frontend->first + 1,
                                   pattern, count, frontend->wrap, &found, message, sizeof(message));
            frontend_execution_end_machine_change_preserving_audio(frontend->execution);
        } else {
            ok = find_frozen(frontend, pattern, count, &found);
            snprintf(message, sizeof(message), "The byte sequence was not found in the frozen page");
        }
        if (!ok) return report(frontend, error, error_size, message, false);
        if (!select_range(frontend, found, count, found, false)) return false;
        if (frontend->live) (void)capture(frontend, frontend->page.first, false);
        return report(frontend, error, error_size, frontend->live ? "Match selected" : "Match selected in the frozen page", true);
    }
    if (!current(frontend)) return report(frontend, error, error_size, "Refresh the memory view first", false);
    if (id == HEX_PREV_PAGE || id == HEX_NEXT_PAGE) {
        uint32_t first = frontend->page.first;
        if ((id == HEX_PREV_PAGE && first - low < DEBUG_MEMORY_PAGE_SIZE)
            || (id == HEX_NEXT_PAGE && high - first < DEBUG_MEMORY_PAGE_SIZE))
            return report(frontend, error, error_size, "This is the end of the memory space", false);
        first = id == HEX_PREV_PAGE ? first - DEBUG_MEMORY_PAGE_SIZE : first + DEBUG_MEMORY_PAGE_SIZE;
        return select_range(frontend, first, 1, first, false);
    }
    if (id >= HEX_ROW && id < HEX_ROW + HEX_ROWS) {
        uint32_t address = frontend->page.first + (id - HEX_ROW) * HEX_COLUMNS;
        return select_range(frontend, address, 1, address, false);
    }
    if ((id == HEX_PICK || id == HEX_EXTEND) && selected >= 0) {
        uint32_t cursor = (uint32_t)selected, anchor = id == HEX_EXTEND ? frontend->anchor : cursor;
        uint32_t first = cursor < anchor ? cursor : anchor, last = cursor > anchor ? cursor : anchor;
        if (!select_range(frontend, first, (size_t)(last - first) + 1, cursor, id == HEX_EXTEND))
            return report(frontend, error, error_size, "The byte is outside this memory space", false);
        return true;
    }
    return report(frontend, error, error_size, "The hex editor action is unavailable", false);
}

HexFrontend *hex_frontend_create(FrontendExecutionRuntime *execution) {
    if (!execution) return NULL;
    HexFrontend *frontend = calloc(1, sizeof(*frontend));
    if (!frontend) return NULL;
    frontend->execution = execution;
    frontend->live = frontend->wrap = true;
    selection_fields(frontend);
    return frontend;
}

bool hex_frontend_register(HexFrontend *frontend) {
    if (!frontend || frontend->registered) return false;
    FrontendPanelSpec spec = {HEX_FRONTEND_PANEL, "Hex / Memory Editor", "Tools", FRONTEND_PANEL_NEEDS_SESSION,
                             snapshot, action, frontend};
    frontend->registered = frontend_panel_register(&spec);
    return frontend->registered;
}

void hex_frontend_unregister(HexFrontend *frontend) {
    if (!frontend || !frontend->registered) return;
    (void)frontend_panel_unregister(HEX_FRONTEND_PANEL);
    frontend->registered = false;
}

void hex_frontend_image_changed(HexFrontend *frontend) {
    if (!frontend) return;
    uint32_t low, high;
    (void)debug_memory_bounds(frontend->space, &low, &high);
    frontend->page.valid = false;
    frontend->first = frontend->last = frontend->anchor = frontend->cursor = low;
    frontend->bytes[0] = frontend->status[0] = '\0';
    selection_fields(frontend);
}

void hex_frontend_destroy(HexFrontend *frontend) {
    if (!frontend) return;
    hex_frontend_unregister(frontend);
    free(frontend);
}
