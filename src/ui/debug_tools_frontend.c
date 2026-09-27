/*
 * debug_tools_frontend.c - Coverage, profiler, events, stack, symbols, source and capture panels
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "debug_tools_frontend.h"
#include "frontend_panels.h"
#include "platform_frontend.h"
#include "output_guard.h"
#include "../debugger/debug_analysis.h"
#include "../debugger/debug_capture.h"
#include "../debugger/debug_catalog.h"
#include "../debugger/expression.h"
#include "../util/file_io.h"
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { ROWS = 24, PROFILE_ROWS = 131072, REFERENCE_ROWS = 65536 };

enum { TOOL_CDL, TOOL_PROFILE, TOOL_EVENTS, TOOL_STACK, TOOL_SYMBOLS, TOOL_SOURCE, TOOL_REFS, TOOL_TEXT, TOOL_TRACE };

typedef struct {
    struct DebugToolsFrontend *owner;
    unsigned kind;
    size_t page, count, shown;
    int selected;
    bool follow, option, reads, writes, deduplicate;
    char status[256], info[256], preview[1536];
    char import_path[2048], export_path[2048], root[2048];
    char first[97], last[97], address[97], filter[97], physical[32];
    char name[64], span[32], file[256], line[32], comment[192], capacity[32], columns[32];
    char rows[ROWS][384];
    const char *items[ROWS];
    uint64_t keys[ROWS];
    uint16_t addresses[ROWS], returns[ROWS];
    DebugProfileRow *profile;
    DebugReference *references;
    size_t reference_count;
    uint32_t mask;
} ToolPanel;

struct DebugToolsFrontend {
    FrontendExecutionRuntime *execution;
    ToolPanel panels[DEBUG_TOOLS_PANEL_COUNT];
    unsigned registered;
};

static const char *const titles[] = {"Code/data logger", "CPU profiler",          "NES event viewer",
                                     "Call stack",       "Symbols and functions", "Source view",
                                     "Find references",  "Text hooker",           "Trace logger"};

static bool report(ToolPanel *p, char *error, size_t size, const char *message, bool ok) {
    snprintf(p->status, sizeof(p->status), "%s", message);
    if (error && size) {
        snprintf(error, size, "%s", ok ? "" : message);
    }
    return ok;
}

static bool expr16(const char *text, uint16_t *out) {
    uint32_t value;
    if (debug_symbol_resolve(text, out, NULL)) {
        return true;
    }
    if (!debug_expression_eval(text, &value, NULL, 0) || value > 65535) {
        return false;
    }
    *out = (uint16_t)value;
    return true;
}

static bool decimal(const char *text, uint64_t limit, uint64_t *value, int base) {
    if (!text || !*text || *text == '-' || *text == '+' || *text == ' ') {
        return false;
    }
    errno = 0;
    char *end;
    unsigned long long number = strtoull(text, &end, base);
    if (errno || *end || number > limit) {
        return false;
    }
    *value = number;
    return true;
}

static bool active(unsigned kind) {
    switch (kind) {
    case TOOL_CDL:
        return debug_cdl_enabled();
    case TOOL_PROFILE:
        return debug_profile_enabled();
    case TOOL_EVENTS:
        return debug_events_enabled();
    case TOOL_STACK:
        return debug_stack_enabled();
    case TOOL_TEXT:
        return debug_text_enabled();
    case TOOL_TRACE:
        return debug_log_enabled();
    default:
        return false;
    }
}

static void symbol_suffix(uint64_t key, char *out, size_t size) {
    DebugSymbol symbol;
    if (debug_symbol_find(key, &symbol)) {
        snprintf(out, size, "%s+%" PRIX64, symbol.name, key - symbol.key);
    } else if (size) {
        *out = 0;
    }
}

static size_t profile_collect(ToolPanel *p) {
    if (!p->profile) {
        p->profile = malloc(PROFILE_ROWS * sizeof(*p->profile));
    }
    if (!p->profile) {
        return 0;
    }
    return p->deduplicate ? debug_profile_function_rows(p->profile, PROFILE_ROWS, !p->option)
                          : debug_profile_rows(p->profile, PROFILE_ROWS, !p->option);
}

static bool filtered_symbol(ToolPanel *p, size_t ordinal, DebugSymbol *out) {
    for (size_t i = 0; i < debug_symbol_count(); ++i) {
        DebugSymbol s;
        debug_symbol_at(i, &s);
        if ((p->option && !s.function) || (*p->filter && !strstr(s.name, p->filter))) {
            continue;
        }
        if (!ordinal--) {
            *out = s;
            return true;
        }
    }
    return false;
}

static size_t text_lines(void) {
    size_t n = 0;
    const char *text = debug_text_output();
    while (*text) {
        if (*text++ == '\n') {
            ++n;
        }
    }
    return n;
}

static size_t panel_count(ToolPanel *p) {
    switch (p->kind) {
    case TOOL_CDL:
        return (debug_cdl_coverage().total + 15) / 16;
    case TOOL_PROFILE:
        return profile_collect(p);
    case TOOL_EVENTS:
        return debug_events_count(p->mask);
    case TOOL_STACK:
        return debug_stack_count();
    case TOOL_SYMBOLS: {
        size_t n = 0;
        for (size_t i = 0; i < debug_symbol_count(); ++i) {
            DebugSymbol symbol;
            debug_symbol_at(i, &symbol);
            if ((!p->option || symbol.function) && (!*p->filter || strstr(symbol.name, p->filter))) {
                ++n;
            }
        }
        return n;
    }
    case TOOL_SOURCE:
        return 1;
    case TOOL_REFS:
        return p->reference_count;
    case TOOL_TEXT:
        return text_lines();
    case TOOL_TRACE:
        return debug_log_count();
    default:
        return 0;
    }
}

static bool row(ToolPanel *p, size_t index, char *text, size_t size, uint16_t *address, uint64_t *key, uint16_t *ret) {
    *address = *ret = 0;
    *key = DEBUG_KEY_NONE;
    char label[96] = "";
    switch (p->kind) {
    case TOOL_CDL: {
        size_t offset = index * 16;
        char flags[17];
        DebugCoverage coverage = debug_cdl_coverage();
        if (offset >= coverage.total) {
            return false;
        }
        for (unsigned i = 0; i < 16; ++i) {
            uint8_t v = debug_cdl_at(offset + i);
            flags[i] = offset + i >= coverage.total ? ' '
                       : (v & 3) == 3               ? 'B'
                       : v & 1                      ? 'C'
                       : v & 2                      ? 'D'
                       : v & 4                      ? 'o'
                                                    : '.';
        }
        flags[16] = 0;
        *key = offset;
        snprintf(text, size, "PRG %08zX  %s", offset, flags);
        return true;
    }
    case TOOL_PROFILE: {
        if (index >= p->count || !p->profile) {
            return false;
        }
        const DebugProfileRow *r = &p->profile[index];
        *address = r->address;
        *key = r->key;
        symbol_suffix(*key, label, sizeof(label));
        uint64_t total = debug_profile_total_cycles();
        snprintf(text, size, "%04X PRG/CPU:%" PRIX64 "  %" PRIu64 " instructions  %" PRIu64 " cycles  %.2f%%  %s",
                 *address, *key, r->instructions, r->cycles, total ? 100.0 * (double)r->cycles / (double)total : 0,
                 label);
        return true;
    }
    case TOOL_EVENTS: {
        DebugNesEvent event;
        if (!debug_events_at(index, p->mask, &event)) {
            return false;
        }
        *address = event.pc;
        *key = event.pc_key;
        *ret = event.address;
        snprintf(text, size, "F:%" PRIu64 " %3d:%03d cycle:%" PRIu64 " cat:%03X PC:%04X bus:%04X=%02X key:%" PRIX64,
                 event.frame, event.scanline, event.dot, event.cycle, event.category, event.pc, event.address,
                 event.value, event.key);
        return true;
    }
    case TOOL_STACK: {
        DebugStackFrame frame;
        if (!debug_stack_at(index, &frame)) {
            return false;
        }
        *address = frame.target;
        *key = frame.target_key;
        *ret = frame.return_address;
        symbol_suffix(*key, label, sizeof(label));
        snprintf(text, size, "%s%s %04X -> %04X [%" PRIX64 "] return:%04X SP:%02X %s",
                 frame.interrupt ? "Interrupt" : "JSR", frame.uncertain ? " ? uncertain" : "", frame.source,
                 frame.target, frame.target_key, frame.return_address, frame.return_sp, label);
        return true;
    }
    case TOOL_SYMBOLS: {
        DebugSymbol symbol;
        if (!filtered_symbol(p, index, &symbol)) {
            return false;
        }
        *address = symbol.address;
        *key = symbol.key;
        snprintf(text, size, "%c %04X [%" PRIX64 "] %s (%u bytes) %.160s", symbol.function ? 'F' : 'L', *address, *key,
                 symbol.name, symbol.size, symbol.comment);
        return true;
    }
    case TOOL_SOURCE: {
        DebugCpuSnapshot state;
        debugger_get_cpu(&state);
        if (p->follow) {
            *address = state.cpu.pc;
        } else if (!expr16(p->address, address)) {
            return false;
        }
        *key = debug_analysis_key(*address);
        DebugSymbol symbol;
        if (debug_symbol_find(*key, &symbol) && symbol.line) {
            snprintf(text, size, "%.240s:%u %s", symbol.file, symbol.line, symbol.name);
            if (debug_source_line(p->root, &symbol, p->preview, sizeof(p->preview), p->status, sizeof(p->status))) {
                return true;
            }
        }
        DebugDisassembly instruction;
        debugger_disassemble(*address, &instruction);
        snprintf(text, size, "%04X %s (disassembly)", *address, instruction.text);
        snprintf(p->preview, sizeof(p->preview), "Source unavailable. %04X %s", *address, instruction.text);
        return true;
    }
    case TOOL_REFS: {
        if (!p->references || index >= p->reference_count) {
            return false;
        }
        DebugReference *r = &p->references[index];
        *address = r->address;
        *key = r->key;
        snprintf(text, size, "Static %04X [%" PRIX64 "] %s -> %04X", *address, *key, r->instruction, r->target);
        return true;
    }
    case TOOL_TEXT: {
        const char *start = debug_text_output();
        while (index-- && *start) {
            const char *end = strchr(start, '\n');
            start = end ? end + 1 : start + strlen(start);
        }
        const char *end = strchr(start, '\n');
        size_t len = end ? (size_t)(end - start) : strlen(start);
        if (len >= size) {
            len = size - 1;
        }
        memcpy(text, start, len);
        text[len] = 0;
        return true;
    }
    case TOOL_TRACE: {
        DebugLogEntry entry;
        if (!debug_log_at(index, &entry)) {
            return false;
        }
        *address = entry.cpu.pc;
        *key = entry.key;
        return debug_log_format(&entry, text, size);
    }
    default:
        return false;
    }
}

static bool control(FrontendPanelModel *model, unsigned id, FrontendPanelControlType type, const char *label,
                    const char *value, bool selected, bool readonly) {
    FrontendPanelControl c = {id, type, label, value, NULL, 0, selected, true, readonly};
    if (type == FRONTEND_PANEL_FILE_OPEN) {
        c.selected = FRONTEND_OPEN_MEMORY;
    }
    if (type == FRONTEND_PANEL_FILE_SAVE) {
        c.selected = FRONTEND_SAVE_MEMORY;
    }
    return frontend_panel_add_control(model, &c);
}

static bool snapshot_inner(ToolPanel *p, FrontendPanelModel *model) {
    p->count = panel_count(p);
    if (p->follow && (p->kind == TOOL_TEXT || p->kind == TOOL_TRACE)) {
        p->page = p->count > ROWS ? p->count - ROWS : 0;
    }
    if (p->page >= p->count) {
        p->page = p->count ? (p->count - 1) / ROWS * ROWS : 0;
    }
    p->shown = 0;
    for (size_t i = p->page; i < p->count && p->shown < ROWS; ++i) {
        size_t n = p->shown;
        if (!row(p, i, p->rows[n], sizeof(p->rows[n]), &p->addresses[n], &p->keys[n], &p->returns[n])) {
            break;
        }
        p->items[n] = p->rows[n];
        ++p->shown;
    }
    if (p->selected < 0 || p->selected >= (int)p->shown) {
        p->selected = 0;
    }
    snprintf(p->info, sizeof(p->info), "%zu results | rows %zu-%zu", p->count, p->count ? p->page + 1 : 0,
             p->page + p->shown);
    if (p->kind == TOOL_CDL) {
        DebugCoverage c = debug_cdl_coverage();
        snprintf(p->info, sizeof(p->info),
                 "%zu PRG bytes: code %zu, data %zu, operands %zu, untouched %zu (%.2f%% used)", c.total, c.code,
                 c.data, c.operand, c.untouched,
                 c.total ? 100.0 * (double)(c.total - c.untouched) / (double)c.total : 0);
    } else if (p->kind == TOOL_PROFILE) {
        snprintf(p->info, sizeof(p->info), "%zu locations | %" PRIu64 " cycles | %" PRIu64 " samples dropped", p->count,
                 debug_profile_total_cycles(), debug_profile_dropped());
    } else if (p->kind == TOOL_EVENTS) {
        snprintf(p->info, sizeof(p->info), "Completed frame: %zu filtered events | %" PRIu64 " dropped | scanline:dot",
                 p->count, debug_events_dropped());
    } else if (p->kind == TOOL_TRACE) {
        snprintf(p->info, sizeof(p->info), "%zu retained | %" PRIu64 " overwritten | rows %zu-%zu", p->count,
                 debug_log_overwritten(), p->count ? p->page + 1 : 0, p->page + p->shown);
    } else if (p->kind == TOOL_TEXT) {
        snprintf(p->info, sizeof(p->info), "%zu bytes | %" PRIu64 " lines dropped when full", debug_text_size(),
                 debug_text_dropped());
    }
    bool ok = control(model, DEBUG_TOOL_INFO, FRONTEND_PANEL_TEXT, "Summary", p->info, false, true);
    if (p->kind <= TOOL_STACK || p->kind >= TOOL_TEXT) {
        ok &= control(model, DEBUG_TOOL_START, FRONTEND_PANEL_ACTION,
                      active(p->kind) ? "Stop capture" : "Start capture", "", false, false);
        ok &= control(model, DEBUG_TOOL_CLEAR, FRONTEND_PANEL_ACTION, "Clear results", "", false, false);
    }
    if (p->kind == TOOL_PROFILE || p->kind == TOOL_SYMBOLS || p->kind == TOOL_REFS) {
        const char *name = p->kind == TOOL_PROFILE   ? "Sort by instructions (off: cycles)"
                           : p->kind == TOOL_SYMBOLS ? "Functions only"
                                                     : "Find constants (off: addresses)";
        ok &= control(model, DEBUG_TOOL_OPTION, FRONTEND_PANEL_CHECKBOX, name, NULL, p->option, false);
    }
    if (p->kind == TOOL_EVENTS || p->kind == TOOL_TRACE || p->kind == TOOL_SYMBOLS) {
        ok &= control(model, DEBUG_TOOL_FILTER, FRONTEND_PANEL_TEXT,
                      p->kind == TOOL_EVENTS  ? "Category mask (hex)"
                      : p->kind == TOOL_TRACE ? "Condition expression (nonzero)"
                                              : "Name contains",
                      p->filter, false, false);
    }
    if (p->kind == TOOL_REFS || p->kind == TOOL_TEXT || p->kind == TOOL_TRACE) {
        ok &= control(model, DEBUG_TOOL_FIRST, FRONTEND_PANEL_TEXT, "First CPU address", p->first, false, false);
        ok &= control(model, DEBUG_TOOL_LAST, FRONTEND_PANEL_TEXT, "Last CPU address", p->last, false, false);
    }
    if (p->kind == TOOL_TRACE) {
        ok &= control(model, DEBUG_TOOL_CAPACITY, FRONTEND_PANEL_TEXT, "History entries (1-1048576)", p->capacity,
                      false, false);
        ok &= control(model, DEBUG_TOOL_COLUMNS, FRONTEND_PANEL_TEXT, "Columns mask: regs=1 cycles=2 bytes=4 symbols=8",
                      p->columns, false, false);
    }
    if (p->kind == TOOL_TEXT) {
        ok &= control(model, DEBUG_TOOL_READS, FRONTEND_PANEL_CHECKBOX, "Observe reads", NULL, p->reads, false);
        ok &= control(model, DEBUG_TOOL_WRITES, FRONTEND_PANEL_CHECKBOX, "Observe writes", NULL, p->writes, false);
        ok &= control(model, DEBUG_TOOL_DEDUP, FRONTEND_PANEL_CHECKBOX, "Suppress repeated lines", NULL, p->deduplicate,
                      false);
    }
    if (p->kind == TOOL_PROFILE) {
        ok &= control(model, DEBUG_TOOL_DEDUP, FRONTEND_PANEL_CHECKBOX, "Group by function spans", NULL, p->deduplicate,
                      false);
    }
    if (p->kind == TOOL_SYMBOLS || p->kind == TOOL_SOURCE || p->kind == TOOL_REFS) {
        ok &= control(model, DEBUG_TOOL_ADDRESS, FRONTEND_PANEL_TEXT,
                      p->kind == TOOL_REFS ? "Target address/constant" : "CPU address or symbol", p->address, false,
                      false);
    }
    if (p->kind == TOOL_SYMBOLS || p->kind == TOOL_REFS) {
        ok &= control(model, DEBUG_TOOL_PHYSICAL, FRONTEND_PANEL_TEXT,
                      "Physical PRG offset (hex; blank=current mapping)", p->physical, false, false);
    }
    if (p->kind == TOOL_SYMBOLS) {
        ok &= control(model, DEBUG_TOOL_NAME, FRONTEND_PANEL_TEXT, "Symbol name", p->name, false, false);
        ok &= control(model, DEBUG_TOOL_SPAN, FRONTEND_PANEL_TEXT, "Span in bytes (decimal)", p->span, false, false);
        ok &=
            control(model, DEBUG_TOOL_FILE, FRONTEND_PANEL_TEXT, "Source file (relative path)", p->file, false, false);
        ok &= control(model, DEBUG_TOOL_LINE, FRONTEND_PANEL_TEXT, "Source line (decimal; 0=none)", p->line, false,
                      false);
        ok &= control(model, DEBUG_TOOL_COMMENT, FRONTEND_PANEL_TEXT, "Comment", p->comment, false, false);
        ok &= control(model, DEBUG_TOOL_ADD, FRONTEND_PANEL_ACTION, "Save label (Functions only marks a function)", "",
                      false, false);
        ok &= control(model, DEBUG_TOOL_REMOVE, FRONTEND_PANEL_ACTION, "Remove selected label", "", false, false);
    }
    if (p->kind == TOOL_SOURCE) {
        ok &= control(model, DEBUG_TOOL_ROOT, FRONTEND_PANEL_DIRECTORY, "Source root", p->root, false, false);
    }
    if (p->kind == TOOL_SOURCE || p->kind == TOOL_TRACE || p->kind == TOOL_TEXT) {
        ok &= control(model, DEBUG_TOOL_FOLLOW, FRONTEND_PANEL_CHECKBOX,
                      p->kind == TOOL_SOURCE ? "Follow PC" : "Follow last", NULL, p->follow, false);
    }
    if (p->kind == TOOL_EVENTS || p->kind == TOOL_TRACE || p->kind == TOOL_TEXT || p->kind == TOOL_REFS) {
        ok &= control(model, DEBUG_TOOL_APPLY, FRONTEND_PANEL_ACTION,
                      p->kind == TOOL_REFS ? "Find static references" : "Apply capture settings", "", false, false);
    }
    FrontendPanelControl list = {DEBUG_TOOL_ROWS, FRONTEND_PANEL_LIST, "Results", NULL, p->items,
                                 p->shown,        p->selected,         true,      false};
    ok &= frontend_panel_add_control(model, &list);
    ok &= control(model, DEBUG_TOOL_PREVIOUS, FRONTEND_PANEL_ACTION, "Previous page", "", false, false);
    ok &= control(model, DEBUG_TOOL_NEXT, FRONTEND_PANEL_ACTION, "Next page", "", false, false);
    if (p->kind != TOOL_TEXT && p->kind != TOOL_CDL) {
        ok &= control(model, DEBUG_TOOL_DISASSEMBLY, FRONTEND_PANEL_ACTION, "Inspect selected instruction", "", false,
                      false);
        ok &= control(model, DEBUG_TOOL_BREAK, FRONTEND_PANEL_ACTION, "Execute breakpoint at selection", "", false,
                      false);
    }
    if (p->kind == TOOL_STACK) {
        ok &= control(model, DEBUG_TOOL_RETURN, FRONTEND_PANEL_ACTION, "Inspect return address", "", false, false);
    }
    ok &= control(model, DEBUG_TOOL_PREVIEW, FRONTEND_PANEL_TEXT, "Selection / source", p->preview, false, true);
    ok &= control(model, DEBUG_TOOL_COPY, FRONTEND_PANEL_ACTION, "Copy selected row (text: all captured text)", "",
                  false, false);
    if (p->kind == TOOL_CDL || p->kind == TOOL_SYMBOLS || p->kind == TOOL_TEXT) {
        ok &= control(model, DEBUG_TOOL_IMPORT_PATH, FRONTEND_PANEL_FILE_OPEN,
                      p->kind == TOOL_TEXT ? "Encoding table" : "Import file", p->import_path, false, false);
        ok &= control(model, DEBUG_TOOL_IMPORT, FRONTEND_PANEL_ACTION, "Import", "", false, false);
    }
    if (p->kind != TOOL_SOURCE) {
        ok &= control(model, DEBUG_TOOL_EXPORT_PATH, FRONTEND_PANEL_FILE_SAVE, "Export file", p->export_path, false,
                      false);
        ok &= control(model, DEBUG_TOOL_EXPORT, FRONTEND_PANEL_ACTION,
                      p->kind == TOOL_CDL ? "Save coverage (.csv exports table)" : "Export", "", false, false);
    }
    model->status = p->status;
    return ok;
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    ToolPanel *p = context;
    frontend_execution_begin_machine_change(p->owner->execution);
    bool ok = snapshot_inner(p, model);
    frontend_execution_end_machine_change_preserving_audio(p->owner->execution);
    if (!ok) {
        return report(p, error, size, "Panel control capacity is too small", false);
    }
    if (error && size) {
        *error = 0;
    }
    return true;
}

static bool apply(ToolPanel *p, char *error, size_t size) {
    uint16_t first, last, target;
    uint64_t count, columns;
    if (p->kind == TOOL_EVENTS) {
        if (!decimal(p->filter, 0x3FF, &columns, 16)) {
            return report(p, error, size, "Category mask must be 000-3FF", false);
        }
        p->mask = (uint32_t)columns;
        return true;
    }
    if (!expr16(p->first, &first) || !expr16(p->last, &last) || first > last) {
        return report(p, error, size, "Invalid CPU address range", false);
    }
    if (p->kind == TOOL_TRACE) {
        if (!decimal(p->capacity, 1048576, &count, 10) || !decimal(p->columns, 15, &columns, 16)) {
            return report(p, error, size, "Invalid capacity or column mask", false);
        }
        DebugTraceFilter filter = {first, last, "", (unsigned)columns};
        strcpy(filter.condition, p->filter);
        return debug_log_configure((size_t)count, &filter, error, size);
    }
    if (p->kind == TOOL_TEXT) {
        debug_text_configure(first, last, p->reads, p->writes, p->deduplicate);
        return true;
    }
    if (p->kind == TOOL_REFS) {
        uint64_t physical = DEBUG_KEY_NONE;
        if (!expr16(p->address, &target) || (*p->physical && !decimal(p->physical, UINT32_MAX, &physical, 16))) {
            return report(p, error, size, "Invalid reference target or PRG offset", false);
        }
        if (!p->references) {
            p->references = malloc(REFERENCE_ROWS * sizeof(*p->references));
        }
        if (!p->references) {
            return report(p, error, size, "Cannot allocate references", false);
        }
        p->reference_count = debug_references(first, last, physical, target, p->option, p->references, REFERENCE_ROWS);
        p->page = 0;
        return true;
    }
    return false;
}

static bool export_rows(ToolPanel *p) {
    NesFileTransaction *tx = NULL;
    if (nes_file_transaction_begin(p->export_path, 128 * 1024 * 1024, &tx) != NES_FILE_OK) {
        return false;
    }
    p->count = panel_count(p);
    for (size_t i = 0; i < p->count; ++i) {
        char line[768];
        uint16_t address, ret;
        uint64_t key;
        if (!row(p, i, line, sizeof(line) - 2, &address, &key, &ret)) {
            nes_file_transaction_abort(&tx);
            return false;
        }
        strcat(line, "\n");
        if (nes_file_transaction_write(tx, line, strlen(line)) != NES_FILE_OK) {
            nes_file_transaction_abort(&tx);
            return false;
        }
    }
    return nes_file_transaction_commit(&tx) == NES_FILE_OK;
}

static bool action_inner(ToolPanel *p, unsigned id, const char *value, int selected, char *error, size_t size) {
    struct {
        unsigned id;
        char *text;
        size_t size;
    } fields[] = {{DEBUG_TOOL_FIRST, p->first, sizeof(p->first)},
                  {DEBUG_TOOL_LAST, p->last, sizeof(p->last)},
                  {DEBUG_TOOL_ADDRESS, p->address, sizeof(p->address)},
                  {DEBUG_TOOL_FILTER, p->filter, sizeof(p->filter)},
                  {DEBUG_TOOL_PHYSICAL, p->physical, sizeof(p->physical)},
                  {DEBUG_TOOL_NAME, p->name, sizeof(p->name)},
                  {DEBUG_TOOL_FILE, p->file, sizeof(p->file)},
                  {DEBUG_TOOL_SPAN, p->span, sizeof(p->span)},
                  {DEBUG_TOOL_LINE, p->line, sizeof(p->line)},
                  {DEBUG_TOOL_COMMENT, p->comment, sizeof(p->comment)},
                  {DEBUG_TOOL_ROOT, p->root, sizeof(p->root)},
                  {DEBUG_TOOL_IMPORT_PATH, p->import_path, sizeof(p->import_path)},
                  {DEBUG_TOOL_EXPORT_PATH, p->export_path, sizeof(p->export_path)},
                  {DEBUG_TOOL_CAPACITY, p->capacity, sizeof(p->capacity)},
                  {DEBUG_TOOL_COLUMNS, p->columns, sizeof(p->columns)}};

    for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i) {
        if (id == fields[i].id) {
            if (!value || strlen(value) >= fields[i].size) {
                return report(p, error, size, "Field is too long", false);
            }
            strcpy(fields[i].text, value);
            return true;
        }
    }
    if (id == DEBUG_TOOL_ROWS) {
        if (selected < 0 || selected >= (int)p->shown) {
            return false;
        }
        p->selected = selected;
        return true;
    }
    if (id == DEBUG_TOOL_PREVIOUS || id == DEBUG_TOOL_NEXT) {
        p->follow = false;
        if (id == DEBUG_TOOL_PREVIOUS) {
            p->page = p->page >= ROWS ? p->page - ROWS : 0;
        } else if (p->page + ROWS < p->count) {
            p->page += ROWS;
        }
        p->selected = 0;
        return true;
    }
    if (id == DEBUG_TOOL_FOLLOW) {
        p->follow = selected != 0;
        return true;
    }
    if (id == DEBUG_TOOL_OPTION) {
        p->option = selected != 0;
        return true;
    }
    if (id == DEBUG_TOOL_READS) {
        p->reads = selected != 0;
        return true;
    }
    if (id == DEBUG_TOOL_WRITES) {
        p->writes = selected != 0;
        return true;
    }
    if (id == DEBUG_TOOL_DEDUP) {
        p->deduplicate = selected != 0;
        return true;
    }
    if (id == DEBUG_TOOL_APPLY) {
        return apply(p, error, size);
    }
    if (id == DEBUG_TOOL_START) {
        bool on = !active(p->kind);
        if (on && (p->kind == TOOL_TEXT || p->kind == TOOL_TRACE) && !apply(p, error, size)) {
            return false;
        }
        switch (p->kind) {
        case TOOL_CDL:
            debug_cdl_enable(on);
            break;
        case TOOL_PROFILE:
            debug_profile_enable(on);
            break;
        case TOOL_EVENTS:
            debug_events_enable(on);
            break;
        case TOOL_STACK:
            debug_stack_enable(on);
            break;
        case TOOL_TEXT:
            debug_text_enable(on);
            break;
        case TOOL_TRACE:
            debug_log_enable(on);
            break;
        default:
            return false;
        }
        return active(p->kind) == on ||
               report(p, error, size, "Capture unavailable: image or allocation required", false);
    }
    if (id == DEBUG_TOOL_CLEAR) {
        switch (p->kind) {
        case TOOL_CDL:
            debug_cdl_clear();
            break;
        case TOOL_PROFILE:
            debug_profile_clear();
            break;
        case TOOL_EVENTS:
            debug_events_clear();
            break;
        case TOOL_STACK:
            debug_stack_clear();
            break;
        case TOOL_TEXT:
            debug_text_clear();
            break;
        case TOOL_TRACE:
            debug_log_clear();
            break;
        default:
            return false;
        }
        p->page = 0;
        return true;
    }
    if (id == DEBUG_TOOL_IMPORT) {
        if (p->kind == TOOL_CDL) {
            return debug_cdl_load(p->import_path) ||
                   report(p, error, size, "Invalid or wrong-image coverage file", false);
        }
        if (p->kind == TOOL_SYMBOLS) {
            return debug_symbol_import(p->import_path, error, size);
        }
        uint8_t *data = NULL;
        size_t bytes;
        if (p->kind != TOOL_TEXT || nes_file_read_all(p->import_path, 128 * 1024, &data, &bytes) != NES_FILE_OK) {
            return report(p, error, size, "Cannot read encoding table", false);
        }
        bool ok = debug_text_table((const char *)data, bytes, error, size);
        free(data);
        return ok;
    }
    if (id == DEBUG_TOOL_EXPORT) {
        const char *protected_paths[] = {p->import_path};
        if (!frontend_output_path_allowed(p->export_path, p->owner->execution, protected_paths, *p->import_path ? 1 : 0,
                                          error, size)) {
            return false;
        }
        bool ok;
        if (p->kind == TOOL_CDL) {
            const char *extension = strrchr(p->export_path, '.');
            ok = extension && !strcmp(extension, ".csv") ? debug_cdl_export(p->export_path)
                                                         : debug_cdl_save(p->export_path);
        } else if (p->kind == TOOL_SYMBOLS) {
            ok = debug_symbol_export(p->export_path);
        } else if (p->kind == TOOL_TRACE) {
            ok = debug_log_export(p->export_path);
        } else if (p->kind == TOOL_TEXT) {
            ok = debug_text_export(p->export_path);
        } else {
            ok = export_rows(p);
        }
        return ok || report(p, error, size, "Export failed; destination was not replaced", false);
    }
    if (id == DEBUG_TOOL_ADD && p->kind == TOOL_SYMBOLS) {
        DebugSymbol symbol = {0};
        uint64_t span, line;
        if (!expr16(p->address, &symbol.address) || !decimal(p->span, 65536, &span, 10) ||
            !decimal(p->line, UINT32_MAX, &line, 10)) {
            return report(p, error, size, "Invalid symbol address, span or line", false);
        }
        symbol.key = debug_analysis_key(symbol.address);
        if (*p->physical && !decimal(p->physical, UINT32_MAX, &symbol.key, 16)) {
            return false;
        }
        symbol.size = (uint32_t)span;
        symbol.line = (uint32_t)line;
        symbol.function = p->option;
        strcpy(symbol.name, p->name);
        strcpy(symbol.file, p->file);
        strcpy(symbol.comment, p->comment);
        return debug_symbol_set(&symbol) || report(p, error, size, "Invalid symbol or duplicate name", false);
    }
    if (!p->shown || p->selected < 0 || p->selected >= (int)p->shown) {
        return report(p, error, size, "Select a result first", false);
    }
    unsigned pick = (unsigned)p->selected;
    if (id == DEBUG_TOOL_COPY) {
        return SDL_SetClipboardText(p->kind == TOOL_TEXT ? debug_text_output() : p->rows[pick]) == 0;
    }
    if (id == DEBUG_TOOL_REMOVE && p->kind == TOOL_SYMBOLS) {
        return debug_symbol_remove(p->keys[pick]);
    }
    if (id == DEBUG_TOOL_BREAK) {
        if (p->kind == TOOL_SOURCE) {
            DebugSymbol symbol;
            return debug_symbol_find(p->keys[pick], &symbol) && debug_source_breakpoint(&symbol) != 0;
        }
        return debugger_add_mapped_breakpoint(DEBUG_BREAK_EXECUTE, p->addresses[pick], p->addresses[pick],
                                              p->keys[pick]) != 0;
    }
    if (id == DEBUG_TOOL_DISASSEMBLY || id == DEBUG_TOOL_RETURN) {
        uint16_t address = id == DEBUG_TOOL_RETURN ? p->returns[pick] : p->addresses[pick];
        if (id != DEBUG_TOOL_RETURN && debug_analysis_key(address) != p->keys[pick]) {
            return report(p, error, size, "This bank is not currently mapped at the saved CPU address", false);
        }
        size_t used = 0;
        for (unsigned i = 0; i < 8; ++i) {
            DebugDisassembly dis;
            debugger_disassemble(address, &dis);
            int n = snprintf(p->preview + used, sizeof(p->preview) - used, "%04X %s\n", address, dis.text);
            if (n < 0 || (size_t)n >= sizeof(p->preview) - used) {
                break;
            }
            used += (size_t)n;
            address = (uint16_t)(address + dis.length);
        }
        return true;
    }
    return report(p, error, size, "Unknown debugger tool control", false);
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    ToolPanel *p = context;
    frontend_execution_begin_machine_change(p->owner->execution);
    bool ok = action_inner(p, id, value, selected, error, size);
    frontend_execution_end_machine_change_preserving_audio(p->owner->execution);
    if (ok && error && size) {
        *error = 0;
    }
    return ok;
}

DebugToolsFrontend *debug_tools_frontend_create(FrontendExecutionRuntime *execution) {
    if (!execution) {
        return NULL;
    }
    DebugToolsFrontend *frontend = calloc(1, sizeof(*frontend));
    if (!frontend) {
        return NULL;
    }
    frontend->execution = execution;
    for (unsigned i = 0; i < DEBUG_TOOLS_PANEL_COUNT; ++i) {
        ToolPanel *p = &frontend->panels[i];
        p->owner = frontend;
        p->kind = i;
        p->follow = true;
        p->reads = true;
        p->deduplicate = true;
        p->mask = 0x3FF;
        strcpy(p->first, "$8000");
        strcpy(p->last, "$FFFF");
        strcpy(p->address, "$8000");
        strcpy(p->capacity, "65536");
        strcpy(p->columns, "F");
        strcpy(p->span, "1");
        strcpy(p->line, "0");
        strcpy(p->root, ".");
        if (i == TOOL_EVENTS) {
            strcpy(p->filter, "3FF");
        }
    }
    return frontend;
}

bool debug_tools_frontend_register(DebugToolsFrontend *frontend) {
    if (!frontend) {
        return false;
    }
    if (frontend->registered == DEBUG_TOOLS_PANEL_COUNT) {
        return true;
    }
    for (unsigned i = 0; i < DEBUG_TOOLS_PANEL_COUNT; ++i) {
        FrontendPanelSpec spec = {DEBUG_TOOLS_PANEL_FIRST + i,  titles[i], "Debug",
                                  FRONTEND_PANEL_NEEDS_SESSION, snapshot,  action,
                                  &frontend->panels[i]};
        if (!frontend_panel_register(&spec)) {
            while (frontend->registered) {
                frontend_panel_unregister(DEBUG_TOOLS_PANEL_FIRST + --frontend->registered);
            }
            return false;
        }
        ++frontend->registered;
    }
    return true;
}

void debug_tools_frontend_image_changed(DebugToolsFrontend *frontend) {
    if (!frontend) {
        return;
    }
    debugger_clear_mapped_breakpoints();
    (void)debug_analysis_image_changed();
    debug_catalog_image_changed();
    debug_capture_shutdown();
    for (unsigned i = 0; i < DEBUG_TOOLS_PANEL_COUNT; ++i) {
        ToolPanel *p = &frontend->panels[i];
        p->count = p->shown = p->page = p->reference_count = 0;
        p->selected = 0;
        *p->preview = *p->status = *p->import_path = *p->export_path = 0;
    }
}

void debug_tools_frontend_destroy(DebugToolsFrontend *frontend) {
    if (!frontend) {
        return;
    }
    debug_tools_frontend_unregister(frontend);
    for (unsigned i = 0; i < DEBUG_TOOLS_PANEL_COUNT; ++i) {
        free(frontend->panels[i].profile);
        free(frontend->panels[i].references);
    }
    free(frontend);
}

void debug_tools_frontend_unregister(DebugToolsFrontend *frontend) {
    if (!frontend) {
        return;
    }
    while (frontend->registered) {
        frontend_panel_unregister(DEBUG_TOOLS_PANEL_FIRST + --frontend->registered);
    }
}
