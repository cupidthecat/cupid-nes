/*
 * memory_search_frontend.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Memory search, comparison history and cheat drafts. SPDX-License-Identifier: GPL-3.0-or-later */
#include "memory_search_frontend.h"
#include "frontend_panels.h"
#include "watch_frontend.h"
#include "../debugger/memory_search.h"
#include "../debugger/memory_watch.h"
#include "../debugger/debugger.h"
#include "../cheats/cheats.h"
#include "../system/execution_policy.h"

#include <SDL2/SDL.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    DebugMemorySearch *search;
    CheatFrontend *editor;
    unsigned id;
    uint64_t session;
    DebugMemorySpace space;
    int width, format, comparison, reference, byte;
    DebugSearchSort sort;
    bool aligned, descending, registered;
    size_t page, selected_index;
    uint32_t selected_address;
    char first[5], last[5], value[32], replacement[3], compare[3], description[96];
    char status[256], info[256], selection[128];
    char cells[MEMORY_SEARCH_PAGE_SIZE][5][32];
    const char *items[MEMORY_SEARCH_PAGE_SIZE][5];
} SearchPanel;

struct MemoryToolsFrontend {
    SearchPanel panels[2];
};

static const uint8_t widths[] = {1, 2, 4};
static const char *const sizes[] = {"8 bit", "16 bit", "32 bit"};
static const char *const formats[] = {"Hexadecimal", "Unsigned decimal", "Signed decimal"};
static const char *const comparisons[] = {"Equal", "Different", "Greater", "Less", "At least", "At most"};
static const char *const references[] = {"Constant", "Last sample", "Initial value", "Address (hex)", "Last search"};
static const char *const byte_names[] = {"0 (low)", "1", "2", "3 (high)"};
static const char *const columns[] = {"Address", "Current", "Last sample", "Initial", "Changes"};

static bool report(SearchPanel *panel, char *error, size_t size, const char *message, bool success) {
    snprintf(panel->status, sizeof(panel->status), "%s", message);
    if (error && size) {
        snprintf(error, size, "%s", success ? "" : message);
    }

    return success;
}

static void deselect(SearchPanel *panel) {
    panel->selected_index = SIZE_MAX;
    panel->selected_address = UINT32_MAX;
    panel->byte = 0;
    panel->replacement[0] = panel->compare[0] = '\0';
}

static void clear(SearchPanel *panel) {
    debug_search_clear(panel->search);
    panel->session = debugger_session_revision();
    panel->page = 0;
    deselect(panel);
    snprintf(panel->status, sizeof(panel->status), "Choose New search to capture memory.");
}

static void sync_session(SearchPanel *panel) {
    if (panel->session != debugger_session_revision()) {
        clear(panel);
        snprintf(panel->status, sizeof(panel->status), "The game session changed. Choose New search.");
    }
}

/* Parse without strtol overflow, whitespace, locale, or platform-long-width
 * differences. Signed input becomes a width-limited two's-complement value. */
static bool number(const char *text, unsigned bits, int format, uint32_t *out) {
    if (!text || !*text || !bits || bits > 32) {
        return false;
    }

    bool negative = *text == '-';
    if (negative && format != 2) {
        return false;
    }

    if (negative) {
        ++text;
    }

    unsigned base = format == 0 ? 16 : 10;
    if (base == 16 && *text == '$') {
        ++text;
    } else if (base == 16 && text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
        text += 2;
    }

    if (!*text) {
        return false;
    }

    uint64_t range = UINT64_C(1) << bits;
    uint64_t limit = format == 2 ? (range / 2 - (negative ? 0 : 1)) : range - 1;
    uint64_t value = 0;
    for (const char *p = text; *p; ++p) {
        unsigned digit;
        if (*p >= '0' && *p <= '9') {
            digit = (unsigned)(*p - '0');
        } else if (base == 16 && *p >= 'A' && *p <= 'F') {
            digit = (unsigned)(*p - 'A') + 10;
        } else if (base == 16 && *p >= 'a' && *p <= 'f') {
            digit = (unsigned)(*p - 'a') + 10;
        } else {
            return false;
        }

        if (digit >= base || value > limit / base || value * base + digit > limit) {
            return false;
        }

        value = value * base + digit;
    }

    *out = (uint32_t)(negative ? (range - value) & (range - 1) : value);
    return true;
}

static bool selection(SearchPanel *panel, DebugSearchSpec *spec, DebugSearchRow *row) {
    return debug_search_spec(panel->search, spec) &&
           debug_search_row(panel->search, panel->selected_index, row) && row->address == panel->selected_address;
}

static void preserve_selection(SearchPanel *panel) {
    size_t count = debug_search_count(panel->search);
    if (panel->selected_address != UINT32_MAX) {
        for (size_t i = 0; i < count; ++i) {
            DebugSearchRow row;
            if (debug_search_row(panel->search, i, &row) && row.address == panel->selected_address) {
                panel->selected_index = i;
                panel->page = i / MEMORY_SEARCH_PAGE_SIZE;
                return;
            }
        }

        deselect(panel);
    }

    size_t last_page = count ? (count - 1) / MEMORY_SEARCH_PAGE_SIZE : 0;
    if (panel->page > last_page) {
        panel->page = last_page;
    }
}

static void format_value(char *out, size_t size, uint32_t value, uint8_t width, int format) {
    if (format == 0) {
        snprintf(out, size, "%0*" PRIX32, width * 2, value);
    } else {
        snprintf(out, size, "%" PRId64, debug_search_number(value, width, format == 2));
    }
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t error_size) {
    SearchPanel *panel = context;
    sync_session(panel);
    const char *spaces[DEBUG_MEMORY_SPACE_COUNT];
    /* The item pointers must outlive this snapshot. The space names themselves
     * are static; retain the pointer array in the process, too. */
    static const char *space_items[DEBUG_MEMORY_SPACE_COUNT];
    for (unsigned i = 0; i < DEBUG_MEMORY_SPACE_COUNT; ++i) {
        spaces[i] = debug_memory_name(i);
        space_items[i] = spaces[i];
    }

    size_t count = debug_search_count(panel->search);
    DebugSearchSpec spec = {0};
    bool active = debug_search_spec(panel->search, &spec);
    DebugSearchRow picked = {0};
    bool chosen = selection(panel, &spec, &picked);
    bool cpu_space = chosen && debug_memory_cpu_space(spec.space);
    bool can_test = cpu_space && panel->id == CHEAT_FINDER_PANEL && nes_execution_policy() == NES_EXECUTION_LIVE;
    if (active) {
        snprintf(panel->info, sizeof(panel->info), "%s %04" PRIX32 "-%04" PRIX32
                 " | %zu of %zu candidates | Page %zu of %zu", debug_memory_name(spec.space), spec.first, spec.last,
                 count, debug_search_total(panel->search), panel->page + 1,
                 count ? (count - 1) / MEMORY_SEARCH_PAGE_SIZE + 1 : 1);
    } else {
        snprintf(panel->info, sizeof(panel->info), "No snapshot. Set a range, width and format, then choose New search.");
    }

    if (chosen) {
        snprintf(panel->selection, sizeof(panel->selection), "%s $%04" PRIX32 ", %u byte%s%s",
                 debug_memory_name(spec.space), picked.address, spec.width, spec.width == 1 ? "" : "s",
                 cpu_space ? "" : " (cheats unavailable)");
    } else {
        snprintf(panel->selection, sizeof(panel->selection), "Select a result row for watches, copying or cheats.");
    }

    FrontendPanelControl controls[] = {
        {MEMORY_SEARCH_SPACE, FRONTEND_PANEL_CHOICE, "Memory space", NULL, space_items, DEBUG_MEMORY_SPACE_COUNT,
         (int)panel->space, true, false},
        {MEMORY_SEARCH_FIRST, FRONTEND_PANEL_TEXT, "First address (hex)", panel->first, NULL, 0, 0, true, false},
        {MEMORY_SEARCH_LAST, FRONTEND_PANEL_TEXT, "Last address (hex)", panel->last, NULL, 0, 0, true, false},
        {MEMORY_SEARCH_WIDTH, FRONTEND_PANEL_CHOICE, "Width (little endian)", NULL, sizes, 3, panel->width, true, false},
        {MEMORY_SEARCH_FORMAT, FRONTEND_PANEL_CHOICE, "Value format", NULL, formats, 3, panel->format, true, false},
        {MEMORY_SEARCH_ALIGNED, FRONTEND_PANEL_CHECKBOX, "Aligned addresses", NULL, NULL, 0, panel->aligned, true, false},
        {MEMORY_SEARCH_COMPARISON, FRONTEND_PANEL_CHOICE, "Comparison", NULL, comparisons, 6, panel->comparison, true, false},
        {MEMORY_SEARCH_REFERENCE, FRONTEND_PANEL_CHOICE, "Compare with", NULL, references, 5, panel->reference, true, false},
        {MEMORY_SEARCH_VALUE, FRONTEND_PANEL_TEXT, "Constant / address", panel->value, NULL, 0, 0, true, false},
        {MEMORY_SEARCH_START, FRONTEND_PANEL_ACTION, "New search", "", NULL, 0, 0, true, false},
        {MEMORY_SEARCH_FILTER, FRONTEND_PANEL_ACTION, "Filter", "", NULL, 0, 0, active, false},
        {MEMORY_SEARCH_REFRESH, FRONTEND_PANEL_ACTION, "Refresh", "", NULL, 0, 0, active, false},
        {MEMORY_SEARCH_UNDO, FRONTEND_PANEL_ACTION, "Undo", "", NULL, 0, 0, debug_search_can_undo(panel->search), false},
        {MEMORY_SEARCH_INFO, FRONTEND_PANEL_TEXT, "Results", panel->info, NULL, 0, 0, true, true},
        {MEMORY_SEARCH_PREV_PAGE, FRONTEND_PANEL_ACTION, "Previous page", "", NULL, 0, 0, panel->page > 0, false},
        {MEMORY_SEARCH_NEXT_PAGE, FRONTEND_PANEL_ACTION, "Next page", "", NULL, 0, 0,
         (panel->page + 1) * MEMORY_SEARCH_PAGE_SIZE < count, false},
        {MEMORY_SEARCH_SELECTION, FRONTEND_PANEL_TEXT, "Selection", panel->selection, NULL, 0, 0, true, true},
        {MEMORY_SEARCH_BYTE, FRONTEND_PANEL_CHOICE, "Cheat byte offset", NULL, byte_names, chosen ? spec.width : 1,
         panel->byte, cpu_space, false},
        {MEMORY_SEARCH_REPLACEMENT, FRONTEND_PANEL_TEXT, "Cheat value (hex)", panel->replacement, NULL, 0, 0, cpu_space, false},
        {MEMORY_SEARCH_COMPARE, FRONTEND_PANEL_TEXT, "Compare (hex, optional)", panel->compare, NULL, 0, 0, cpu_space, false},
        {MEMORY_SEARCH_DESCRIPTION, FRONTEND_PANEL_TEXT, "Cheat description", panel->description, NULL, 0, 0, true, false},
        {MEMORY_SEARCH_WATCH, FRONTEND_PANEL_ACTION, "Add watch", "", NULL, 0, 0, chosen, false},
        {MEMORY_SEARCH_SEND, FRONTEND_PANEL_ACTION, "Send to cheat editor", "", NULL, 0, 0, cpu_space, false},
        {MEMORY_SEARCH_TEST, FRONTEND_PANEL_ACTION, "Test as enabled cheat", "", NULL, 0, 0, can_test, false},
        {MEMORY_SEARCH_COPY, FRONTEND_PANEL_ACTION, "Copy result", "", NULL, 0, 0, chosen, false}
    };
    for (size_t i = 0; i < sizeof(controls) / sizeof(*controls); ++i) {
        if (!frontend_panel_add_control(model, &controls[i])) {
            return report(panel, error, error_size, "Memory Search panel is too small", false);
        }
    }

    for (unsigned col = 0; col < 5; ++col) {
        FrontendPanelControl header = {MEMORY_SEARCH_SORT + col, FRONTEND_PANEL_ACTION, columns[col],
                                       panel->sort == col ? (panel->descending ? "descending" : "ascending") : "",
                                       NULL, 0, 0, active, false};
        if (!frontend_panel_add_control(model, &header)) {
            return false;
        }
    }

    for (unsigned i = 0; i < MEMORY_SEARCH_PAGE_SIZE; ++i) {
        size_t index = panel->page * MEMORY_SEARCH_PAGE_SIZE + i;
        DebugSearchRow row;
        bool exists = debug_search_row(panel->search, index, &row);
        if (exists) {
            snprintf(panel->cells[i][0], sizeof(panel->cells[i][0]), "%04" PRIX32, row.address);
            format_value(panel->cells[i][1], sizeof(panel->cells[i][1]), row.current, spec.width, panel->format);
            format_value(panel->cells[i][2], sizeof(panel->cells[i][2]), row.previous, spec.width, panel->format);
            format_value(panel->cells[i][3], sizeof(panel->cells[i][3]), row.initial, spec.width, panel->format);
            snprintf(panel->cells[i][4], sizeof(panel->cells[i][4]), "%" PRIu32, row.changes);
        }

        for (unsigned col = 0; col < 5; ++col) {
            if (!exists) {
                panel->cells[i][col][0] = '\0';
            }

            panel->items[i][col] = panel->cells[i][col];
        }

        FrontendPanelControl item = {MEMORY_SEARCH_ROW + i, FRONTEND_PANEL_ACTION,
                                     exists && row.current != row.previous ? "Changed result" : "Result",
                                     panel->cells[i][0], panel->items[i], 5,
                                     exists && index == panel->selected_index, exists, false};
        if (!frontend_panel_add_control(model, &item)) {
            return false;
        }
    }

    model->status = panel->status;
    if (error && error_size) {
        error[0] = '\0';
    }

    return true;
}

static bool handoff(SearchPanel *panel, unsigned id, char *error, size_t error_size) {
    DebugSearchSpec spec;
    DebugSearchRow row;
    if (!selection(panel, &spec, &row)) {
        return report(panel, error, error_size, "Select a result from the current search", false);
    }

    if (id == MEMORY_SEARCH_COPY) {
        char text[256], current[32], previous[32], initial[32];
        format_value(current, sizeof(current), row.current, spec.width, panel->format);
        format_value(previous, sizeof(previous), row.previous, spec.width, panel->format);
        format_value(initial, sizeof(initial), row.initial, spec.width, panel->format);
        snprintf(text, sizeof(text), "%s\t%04" PRIX32 "\t%s\t%s\t%s\t%" PRIu32,
                 debug_memory_name(spec.space), row.address, current, previous, initial, row.changes);
        if (SDL_SetClipboardText(text) < 0) {
            return report(panel, error, error_size, SDL_GetError(), false);
        }

        return report(panel, error, error_size, "Selected result copied", true);
    }

    if (id == MEMORY_SEARCH_WATCH) {
        FrontendPanelInfo destination;
        if (!frontend_panel_get(WATCH_FRONTEND_PANEL, &destination) || !destination.enabled) {
            return report(panel, error, error_size, "Memory Watches is unavailable; no watch was added", false);
        }

        DebugWatchSpec watch = {.space = spec.space, .width = spec.width,
                               .format = (DebugWatchFormat)panel->format, .enabled = true};
        snprintf(watch.expression, sizeof(watch.expression), "$%04" PRIX32, row.address);
        snprintf(watch.label, sizeof(watch.label), "Search $%04" PRIX32, row.address);
        char message[128];
        uint32_t added = debug_watch_add(&watch, 1, message, sizeof(message));
        if (!added) {
            return report(panel, error, error_size, message, false);
        }

        if (!watch_frontend_select(added)) {
            (void)debugger_remove_watch(added);
            return report(panel, error, error_size, "Memory Watches could not select the new watch", false);
        }

        return report(panel, error, error_size, "Watch added to Memory Watches", true);
    }

    if (!debug_memory_cpu_space(spec.space)) {
        return report(panel, error, error_size, "Cheats cannot address this memory space", false);
    }

    if (id == MEMORY_SEARCH_TEST && (panel->id != CHEAT_FINDER_PANEL || nes_execution_policy() != NES_EXECUTION_LIVE)) {
        return report(panel, error, error_size, "Test candidates in Cheat Finder during a live game session", false);
    }

    uint32_t value, compare = 0;
    if (panel->byte < 0 || panel->byte >= spec.width || !number(panel->replacement, 8, 0, &value) ||
        (*panel->compare && !number(panel->compare, 8, 0, &compare))) {
        return report(panel, error, error_size, "Choose one byte and a 00-FF hexadecimal value; compare may be blank", false);
    }

    char code[16];
    if (*panel->compare) {
        snprintf(code, sizeof(code), "%04" PRIX32 ":%02" PRIX32 ":%02" PRIX32, row.address + panel->byte, value, compare);
    } else {
        snprintf(code, sizeof(code), "%04" PRIX32 ":%02" PRIX32, row.address + panel->byte, value);
    }

    if (!cheat_frontend_prepare(panel->editor, code, panel->description, error, error_size)) {
        return false;
    }

    if (id == MEMORY_SEARCH_TEST) {
        if (!frontend_panel_action(CHEATS_FRONTEND_PANEL, CHEAT_CONTROL_ADD, NULL, 0, error, error_size)) {
            return false;
        }

        return report(panel, error, error_size, "Enabled cheat added. Use Cheats to disable, edit or remove the test.", true);
    }

    return report(panel, error, error_size, "Draft sent to Cheats. Review it and choose Add new cheat.", true);
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t error_size) {
    SearchPanel *panel = context;
    sync_session(panel);
    uint32_t parsed;
    if (id == MEMORY_SEARCH_SPACE && selected >= 0 && selected < DEBUG_MEMORY_SPACE_COUNT) {
        panel->space = (unsigned)selected;
        uint32_t first, last;
        (void)debug_memory_bounds(panel->space, &first, &last);
        snprintf(panel->first, sizeof(panel->first), "%04" PRIX32, first);
        snprintf(panel->last, sizeof(panel->last), "%04" PRIX32, last);
        clear(panel);
        return true;
    }

    if ((id == MEMORY_SEARCH_WIDTH || id == MEMORY_SEARCH_FORMAT) && selected >= 0 && selected < 3) {
        if (id == MEMORY_SEARCH_WIDTH) {
            panel->width = selected;
        } else {
            panel->format = selected;
        }

        clear(panel);
        return true;
    }

    if (id == MEMORY_SEARCH_ALIGNED && (selected == 0 || selected == 1)) {
        panel->aligned = selected != 0;
        clear(panel);
        return true;
    }

    if (id == MEMORY_SEARCH_COMPARISON && selected >= 0 && selected < DEBUG_SEARCH_COMPARISON_COUNT) {
        panel->comparison = selected;
        return true;
    }

    if (id == MEMORY_SEARCH_REFERENCE && selected >= 0 && selected < DEBUG_SEARCH_REFERENCE_COUNT) {
        panel->reference = selected;
        return true;
    }

    if (id == MEMORY_SEARCH_FIRST || id == MEMORY_SEARCH_LAST) {
        uint32_t first, last;
        (void)debug_memory_bounds(panel->space, &first, &last);
        if (!number(value, 16, 0, &parsed) || parsed < first || parsed > last) {
            return report(panel, error, error_size, "Address is outside the selected memory space", false);
        }

        snprintf(id == MEMORY_SEARCH_FIRST ? panel->first : panel->last, 5, "%04" PRIX32, parsed);
        clear(panel);
        return true;
    }

    if (id == MEMORY_SEARCH_VALUE || id == MEMORY_SEARCH_DESCRIPTION) {
        char *destination = id == MEMORY_SEARCH_VALUE ? panel->value : panel->description;
        size_t capacity = id == MEMORY_SEARCH_VALUE ? sizeof(panel->value) : sizeof(panel->description);
        if (!value || strlen(value) >= capacity) {
            return report(panel, error, error_size, "The entered text is too long", false);
        }

        strcpy(destination, value);
        return true;
    }

    if (id == MEMORY_SEARCH_REPLACEMENT || id == MEMORY_SEARCH_COMPARE) {
        if (id == MEMORY_SEARCH_COMPARE && value && !*value) {
            panel->compare[0] = '\0';
            return true;
        }

        if (!number(value, 8, 0, &parsed)) {
            return report(panel, error, error_size, "Enter a hexadecimal byte from 00 to FF", false);
        }

        snprintf(id == MEMORY_SEARCH_COMPARE ? panel->compare : panel->replacement, 3, "%02" PRIX32, parsed);
        return true;
    }

    if (id == MEMORY_SEARCH_BYTE) {
        DebugSearchSpec spec;
        DebugSearchRow row;
        if (!selection(panel, &spec, &row) || selected < 0 || selected >= spec.width) {
            return report(panel, error, error_size, "Select a byte within the selected result", false);
        }

        panel->byte = selected;
        snprintf(panel->replacement, sizeof(panel->replacement), "%02X", (unsigned)((row.current >> (8 * selected)) & 255));
        return true;
    }

    if (id >= MEMORY_SEARCH_ROW && id < MEMORY_SEARCH_ROW + MEMORY_SEARCH_PAGE_SIZE) {
        size_t index = panel->page * MEMORY_SEARCH_PAGE_SIZE + id - MEMORY_SEARCH_ROW;
        DebugSearchRow row;
        if (!debug_search_row(panel->search, index, &row)) {
            return report(panel, error, error_size, "This result is no longer available", false);
        }

        panel->selected_index = index;
        panel->selected_address = row.address;
        panel->byte = 0;
        panel->compare[0] = '\0';
        snprintf(panel->replacement, sizeof(panel->replacement), "%02X", (unsigned)(row.current & 255));
        return true;
    }

    if (id == MEMORY_SEARCH_WATCH || id == MEMORY_SEARCH_SEND || id == MEMORY_SEARCH_TEST || id == MEMORY_SEARCH_COPY) {
        return handoff(panel, id, error, error_size);
    }

    if (id >= MEMORY_SEARCH_SORT && id < MEMORY_SEARCH_SORT + DEBUG_SEARCH_SORT_COUNT) {
        unsigned column = id - MEMORY_SEARCH_SORT;
        panel->descending = panel->sort == column ? !panel->descending : false;
        panel->sort = column;
        (void)debug_search_sort(panel->search, column, panel->descending);
        preserve_selection(panel);
        return true;
    }

    size_t count = debug_search_count(panel->search);
    if (id == MEMORY_SEARCH_PREV_PAGE && panel->page) {
        --panel->page;
        return true;
    }

    if (id == MEMORY_SEARCH_NEXT_PAGE && (panel->page + 1) * MEMORY_SEARCH_PAGE_SIZE < count) {
        ++panel->page;
        return true;
    }

    if (id == MEMORY_SEARCH_UNDO) {
        if (!debug_search_undo(panel->search)) {
            return report(panel, error, error_size, "There is no search step to undo", false);
        }

        preserve_selection(panel);
        return report(panel, error, error_size, "Previous search step restored", true);
    }

    DebugSearchResult result;
    if (id == MEMORY_SEARCH_START) {
        uint32_t first, last;
        if (!number(panel->first, 16, 0, &first) || !number(panel->last, 16, 0, &last)) {
            return report(panel, error, error_size, "Enter hexadecimal range endpoints", false);
        }

        DebugSearchSpec spec = {panel->space, first, last, widths[panel->width], panel->format == 2, panel->aligned};
        result = debug_search_start(panel->search, &spec);
        if (result == DEBUG_SEARCH_OK) {
            deselect(panel);
            panel->page = 0;
        }
    } else if (id == MEMORY_SEARCH_REFRESH) {
        result = debug_search_refresh(panel->search);
    } else if (id == MEMORY_SEARCH_FILTER) {
        parsed = 0;
        if ((panel->reference == DEBUG_SEARCH_CONSTANT || panel->reference == DEBUG_SEARCH_ADDRESS) &&
            !number(panel->value, panel->reference == DEBUG_SEARCH_ADDRESS ? 16 : 8 * widths[panel->width],
                    panel->reference == DEBUG_SEARCH_ADDRESS ? 0 : panel->format, &parsed)) {
            return report(panel, error, error_size, "The comparison value does not fit the selected format and width", false);
        }

        result = debug_search_filter(panel->search, (unsigned)panel->comparison, (unsigned)panel->reference, parsed);
    } else {
        return report(panel, error, error_size, "Invalid or unavailable Memory Search action", false);
    }

    if (result == DEBUG_SEARCH_OK) {
        preserve_selection(panel);
    }

    return report(panel, error, error_size, debug_search_result_message(result), result == DEBUG_SEARCH_OK);
}

bool memory_tools_panel(unsigned id) {
    return id == MEMORY_SEARCH_PANEL || id == CHEAT_FINDER_PANEL;
}

MemoryToolsFrontend *memory_tools_create(CheatFrontend *editor) {
    if (!editor) {
        return NULL;
    }

    MemoryToolsFrontend *frontend = calloc(1, sizeof(*frontend));
    if (!frontend) {
        return NULL;
    }

    for (unsigned i = 0; i < 2; ++i) {
        SearchPanel *panel = &frontend->panels[i];
        panel->editor = editor;
        panel->id = i ? CHEAT_FINDER_PANEL : MEMORY_SEARCH_PANEL;
        panel->search = debug_search_create(NULL);
        if (!panel->search) {
            memory_tools_destroy(frontend);
            return NULL;
        }

        strcpy(panel->first, "0000");
        strcpy(panel->last, "07FF");
        strcpy(panel->value, "0");
        strcpy(panel->description, "Memory search candidate");
        panel->reference = DEBUG_SEARCH_PREVIOUS;
        clear(panel);
    }

    return frontend;
}

bool memory_tools_register(MemoryToolsFrontend *frontend) {
    if (!frontend || frontend->panels[0].registered || frontend->panels[1].registered) {
        return false;
    }

    for (unsigned i = 0; i < 2; ++i) {
        SearchPanel *panel = &frontend->panels[i];
        FrontendPanelSpec spec = {panel->id, i ? "Cheat Finder" : "Memory Search", "Tools", FRONTEND_PANEL_NEEDS_SESSION,
                                  snapshot, action, panel};
        if (!frontend_panel_register(&spec)) {
            if (i) {
                (void)frontend_panel_unregister(frontend->panels[0].id);
                frontend->panels[0].registered = false;
            }

            return false;
        }

        panel->registered = true;
    }

    return true;
}

void memory_tools_image_changed(MemoryToolsFrontend *frontend) {
    if (frontend) {
        for (unsigned i = 0; i < 2; ++i) {
            clear(&frontend->panels[i]);
        }
    }
}

void memory_tools_destroy(MemoryToolsFrontend *frontend) {
    if (frontend) {
        for (unsigned i = 0; i < 2; ++i) {
            if (frontend->panels[i].registered) {
                (void)frontend_panel_unregister(frontend->panels[i].id);
            }

            debug_search_destroy(frontend->panels[i].search);
        }

        free(frontend);
    }
}
