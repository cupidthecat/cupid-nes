/*
 * hex_frontend_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Hex editor selection, clipboard and write policies. SPDX-License-Identifier: GPL-3.0-or-later */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../debugger/debugger.h"
#include "../debugger/expression.h"
#include "../debugger/memory_editor.h"
#include "../debugger/memory_watch.h"
#include "../system/execution_policy.h"
#include "../system/vs_system.h"
#include "../ui/debug_frontend.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"
#include "../ui/hex_frontend.h"
#include "../ui/watch_frontend.h"
#include "../util/file_io.h"
#include "../../include/globals.h"

static char hex_error[256];

static bool act(unsigned id, const char *value, int selected) {
    return frontend_panel_action(HEX_FRONTEND_PANEL, id, value, selected, hex_error, sizeof(hex_error));
}

static bool control(unsigned id, FrontendPanelControl *out) {
    FrontendPanelControl controls[64];
    FrontendPanelModel model = {.controls = controls, .capacity = 64};
    if (!frontend_panel_snapshot(HEX_FRONTEND_PANEL, &model, NULL, 0)) return false;
    for (size_t i = 0; i < model.count; ++i) {
        if (controls[i].id != id) continue;
        *out = controls[i];
        return true;
    }
    return false;
}

static bool select_bytes(DebugMemorySpace space, unsigned address, size_t count) {
    char text[32];
    if (!act(HEX_SPACE, NULL, (int)space)) return false;
    snprintf(text, sizeof(text), "$%04X", address);
    if (!act(HEX_ADDRESS, text, 0)) return false;
    snprintf(text, sizeof(text), "%zu", count);
    return act(HEX_COUNT, text, 0) && act(HEX_SELECT, NULL, 0);
}

static bool clipboard_equals(const char *expected) {
    char *text = SDL_GetClipboardText();
    bool same = text && !strcmp(text, expected);
    SDL_free(text);
    return same;
}

static int selections(void) {
    FrontendPanelControl c;
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0, 1));
    cpu.pc = 0x20;
    BOARD_CHECK(act(HEX_ADDRESS, "PC + #3", 0) && act(HEX_COUNT, "3", 0) && act(HEX_SELECT, NULL, 0));
    BOARD_CHECK(control(HEX_SELECTION, &c) && c.selected == 0x23 && strstr(c.value, "3 bytes"));
    BOARD_CHECK(control(HEX_ROW + 2, &c) && c.selected == 0x38);
    BOARD_CHECK(act(HEX_EXTEND, NULL, 0x1F));
    BOARD_CHECK(control(HEX_SELECTION, &c) && c.selected == 0x1F && strstr(c.value, "5 bytes"));
    BOARD_CHECK(act(HEX_PICK, NULL, 0xFF) && act(HEX_EXTEND, NULL, 0x101));
    BOARD_CHECK(control(HEX_ROW, &c) && !strcmp(c.label, "$0100") && c.selected == 3);
    BOARD_CHECK(act(HEX_COPY, NULL, 0) && clipboard_equals("FF 00 01"));
    BOARD_CHECK(!act(HEX_PICK, NULL, 0x800));
    BOARD_CHECK(control(HEX_SELECTION, &c) && c.selected == 0x101);
    BOARD_CHECK(act(HEX_ADDRESS, "$07FF", 0) && act(HEX_COUNT, "2", 0) && !act(HEX_SELECT, NULL, 0));
    BOARD_CHECK(control(HEX_SELECTION, &c) && c.selected == 0x101);
    BOARD_CHECK(act(HEX_COUNT, "0", 0) && !act(HEX_SELECT, NULL, 0));
    BOARD_CHECK(act(HEX_COUNT, "65537", 0) && !act(HEX_SELECT, NULL, 0));
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_CPU, 0, 65536));
    BOARD_CHECK(control(HEX_COUNT, &c) && !strcmp(c.value, "65536"));
    BOARD_CHECK(control(HEX_WATCH, &c) && !c.enabled);
    BOARD_CHECK(!act(HEX_WATCH, NULL, 0));
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_PALETTE, 0x3F00, 32));
    BOARD_CHECK(control(HEX_ROW, &c) && !strcmp(c.label, "$3F00") && c.selected == 0xFFFF);
    BOARD_CHECK(control(HEX_ROW + 2, &c) && !c.enabled);
    BOARD_CHECK(!act(HEX_NEXT_PAGE, NULL, 0) && !act(HEX_PREV_PAGE, NULL, 0));
    return 0;
}

static int frozen_and_live(void) {
    FrontendPanelControl c;
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x20, 1) && act(HEX_BASELINE, NULL, 0));
    BOARD_CHECK(act(HEX_LIVE, NULL, 0));
    write_mem(0x20, 0xA5);
    BOARD_CHECK(control(HEX_ROW + 2, &c) && !strcmp(c.items[0], "20") && c.items[17][0] == ' ');
    BOARD_CHECK(act(HEX_COPY, NULL, 0) && clipboard_equals("20"));
    BOARD_CHECK(act(HEX_PATTERN, "21 22", 0) && act(HEX_FIND, NULL, 0));
    BOARD_CHECK(control(HEX_SELECTION, &c) && c.selected == 0x21);
    BOARD_CHECK(act(HEX_COPY, NULL, 0) && clipboard_equals("21 22"));
    BOARD_CHECK(act(HEX_PICK, NULL, 0x20) && act(HEX_REFRESH, NULL, 0));
    BOARD_CHECK(control(HEX_ROW + 2, &c) && !strcmp(c.items[0], "A5") && c.items[17][0] == '*');
    BOARD_CHECK(act(HEX_BASELINE, NULL, 0));
    BOARD_CHECK(control(HEX_ROW + 2, &c) && c.items[17][0] == ' ');
    BOARD_CHECK(act(HEX_PICK, NULL, 0xFF) && act(HEX_EXTEND, NULL, 0x100));
    BOARD_CHECK(SDL_SetClipboardText("Keep clipboard") == 0);
    BOARD_CHECK(!act(HEX_COPY, NULL, 0) && clipboard_equals("Keep clipboard"));
    BOARD_CHECK(act(HEX_LIVE, NULL, 1) && act(HEX_COPY, NULL, 0) && clipboard_equals("FF 00"));

    BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x7FE, 1));
    BOARD_CHECK(act(HEX_PATTERN, "A5 21", 0) && act(HEX_WRAP, NULL, 0));
    BOARD_CHECK(!act(HEX_FIND, NULL, 0));
    BOARD_CHECK(act(HEX_WRAP, NULL, 1) && act(HEX_FIND, NULL, 0));
    BOARD_CHECK(control(HEX_SELECTION, &c) && c.selected == 0x20);
    BOARD_CHECK(act(HEX_LIVE, NULL, 0));
    BOARD_CHECK(act(HEX_PATTERN, "30 31", 0));
    write_mem(0x30, 0x99);
    BOARD_CHECK(act(HEX_FIND, NULL, 0));
    BOARD_CHECK(act(HEX_COPY, NULL, 0) && clipboard_equals("30 31"));
    BOARD_CHECK(act(HEX_LIVE, NULL, 1) && act(HEX_COPY, NULL, 0) && clipboard_equals("99 31"));
    return 0;
}

static int writes(FrontendExecutionRuntime *execution, DebugFrontend *debug) {
    FrontendPanelControl c;
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x40, 1));
    BOARD_CHECK(act(HEX_BYTES, "C0 FF EE", 0) && act(HEX_APPLY, NULL, 0));
    BOARD_CHECK(read_mem(0x40) == 0xC0 && read_mem(0x41) == 0xFF && read_mem(0x42) == 0xEE);
    BOARD_CHECK(control(HEX_COUNT, &c) && !strcmp(c.value, "3"));
    BOARD_CHECK(act(HEX_APPLY, NULL, 0));
    BOARD_CHECK(act(HEX_BYTES, "12 GG", 0) && !act(HEX_APPLY, NULL, 0) && read_mem(0x40) == 0xC0);
    BOARD_CHECK(act(HEX_BYTES, "12", 0) && act(HEX_LIVE, NULL, 0));
    BOARD_CHECK(!act(HEX_APPLY, NULL, 0) && read_mem(0x40) == 0xC0);
    BOARD_CHECK(act(HEX_LIVE, NULL, 1));
    execution_control_set_paused(&execution->execution, false);
    debugger_resume();
    BOARD_CHECK(!act(HEX_APPLY, NULL, 0) && read_mem(0x40) == 0xC0);
    debugger_pause();
    execution_control_set_paused(&execution->execution, true);
    debug_frontend_image_changed(debug);
    BOARD_CHECK(!frontend_execution_paused(execution) && !debugger_is_paused());
    debugger_pause();
    frontend_execution_sync_debugger(execution);
    BOARD_CHECK(act(HEX_BYTES, "12", 0) && !act(HEX_APPLY, NULL, 0));
    BOARD_CHECK(!act(HEX_COPY, NULL, 0));
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x40, 1));
    BOARD_CHECK(SDL_SetClipboardText("11 22 33 44") == 0);
    BOARD_CHECK(act(HEX_PASTE, NULL, 0));
    BOARD_CHECK(read_mem(0x40) == 0x11 && read_mem(0x43) == 0x44);
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_CPU, 0x1FFF, 1));
    uint8_t before = read_mem(0x7FF);
    BOARD_CHECK(SDL_SetClipboardText("A5 5A") == 0 && !act(HEX_PASTE, NULL, 0));
    BOARD_CHECK(read_mem(0x7FF) == before);
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_CPU, 0x8000, 1));
    before = read_mem(0x8000);
    BOARD_CHECK(!act(HEX_PASTE, NULL, 0) && read_mem(0x8000) == before);
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x60, 1) && act(HEX_TEXT_MODE, NULL, 1));
    BOARD_CHECK(SDL_SetClipboardText("Cupid") == 0 && act(HEX_PASTE, NULL, 0));
    BOARD_CHECK(read_mem(0x60) == 'C' && read_mem(0x64) == 'd');
    BOARD_CHECK(act(HEX_COPY, NULL, 0) && clipboard_equals("Cupid"));
    const uint8_t invalid[][4] = {{0}, {0xC0, 0xAF}, {0xED, 0xA0, 0x80}, {0xF4, 0x90, 0x80, 0x80}, {0xE2, 0x82}};
    const size_t lengths[] = {1, 2, 3, 4, 2};
    for (size_t i = 0; i < sizeof(lengths) / sizeof(*lengths); ++i) {
        for (size_t j = 0; j < lengths[i]; ++j) write_mem((uint16_t)(0x60 + j), invalid[i][j]);
        BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x60, lengths[i]));
        BOARD_CHECK(!act(HEX_COPY, NULL, 0) && clipboard_equals("Cupid"));
    }
    BOARD_CHECK(act(HEX_TEXT_MODE, NULL, 0));
    return 0;
}

static int files_and_fields(FrontendExecutionRuntime *execution) {
    const char *path = "build/hex-editor-roundtrip.bin";
    const uint8_t data[] = {0, 0xA5, 0xFF, 0x10};
    BOARD_CHECK(nes_file_write_atomic(path, data, sizeof(data)) == NES_FILE_OK);
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x80, 1));
    BOARD_CHECK(act(HEX_IMPORT_PATH, path, 0));
    BOARD_CHECK(act(HEX_IMPORT, NULL, 0));
    BOARD_CHECK(act(HEX_EXPORT_PATH, path, 0) && act(HEX_EXPORT, NULL, 0));
    uint8_t *bytes = NULL;
    size_t count = 0;
    BOARD_CHECK(nes_file_read_all(path, 16, &bytes, &count) == NES_FILE_OK);
    BOARD_CHECK(count == sizeof(data) && !memcmp(bytes, data, count));
    free(bytes);
    execution->rom_path = path;
    write_mem(0x80, 0xCC);
    bool refused = !act(HEX_EXPORT, NULL, 0);
    execution->rom_path = NULL;
    BOARD_CHECK(refused);
    BOARD_CHECK(nes_file_read_all(path, 16, &bytes, &count) == NES_FILE_OK);
    BOARD_CHECK(count == sizeof(data) && !memcmp(bytes, data, count));
    free(bytes);
    BOARD_CHECK(nes_file_write_atomic(path, data, 0) == NES_FILE_OK);
    BOARD_CHECK(!act(HEX_IMPORT, NULL, 0) && read_mem(0x80) == 0xCC);
    uint8_t *oversized = calloc(DEBUG_MEMORY_EDIT_LIMIT + 1u, 1);
    BOARD_CHECK(oversized);
    NesFileResult written = nes_file_write_atomic(path, oversized, DEBUG_MEMORY_EDIT_LIMIT + 1u);
    free(oversized);
    BOARD_CHECK(written == NES_FILE_OK);
    BOARD_CHECK(!act(HEX_IMPORT, NULL, 0) && read_mem(0x80) == 0xCC);
    BOARD_CHECK(nes_file_write_atomic(path, data, sizeof(data)) == NES_FILE_OK);
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_CPU, 0x1FFF, 1));
    uint8_t last_ram = read_mem(0x7FF);
    BOARD_CHECK(!act(HEX_IMPORT, NULL, 0) && read_mem(0x7FF) == last_ram);
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x80, 1));
    const char *protected_paths[] = {path};
    frontend_execution_set_protected_paths(execution, protected_paths, 1);
    refused = !act(HEX_EXPORT, NULL, 0);
    frontend_execution_set_protected_paths(execution, NULL, 0);
    BOARD_CHECK(refused);
    BOARD_CHECK(nes_file_read_all(path, 16, &bytes, &count) == NES_FILE_OK);
    BOARD_CHECK(count == sizeof(data) && !memcmp(bytes, data, count));
    free(bytes);
    BOARD_CHECK(nes_file_remove(path) == NES_FILE_OK);

    char *large = malloc(NES_FILE_PATH_LIMIT + 1);
    BOARD_CHECK(large);
    memset(large, 'x', NES_FILE_PATH_LIMIT);
    large[NES_FILE_PATH_LIMIT] = '\0';
    const unsigned fields[] = {HEX_ADDRESS, HEX_COUNT, HEX_BYTES, HEX_PATTERN, HEX_IMPORT_PATH, HEX_EXPORT_PATH};
    for (size_t i = 0; i < sizeof(fields) / sizeof(*fields); ++i) {
        FrontendPanelControl c;
        BOARD_CHECK(act(fields[i], "retained", 0) && !act(fields[i], large, 0));
        BOARD_CHECK(control(fields[i], &c) && !strcmp(c.value, "retained"));
    }
    free(large);
    return 0;
}

static uint64_t hash_bytes(uint64_t hash, const void *data, size_t size) {
    const uint8_t *bytes = data;
    for (size_t i = 0; i < size; ++i) {
        hash ^= bytes[i];
        hash *= UINT64_C(1099511628211);
    }
    return hash;
}

static uint64_t history_hash(const NesRewindHistory *history) {
    uint64_t hash = UINT64_C(14695981039346656037);
    hash = hash_bytes(hash, &history->count, sizeof(history->count));
    hash = hash_bytes(hash, &history->head, sizeof(history->head));
    hash = hash_bytes(hash, &history->total_bytes, sizeof(history->total_bytes));
    for (size_t i = 0; i < history->capacity; ++i) {
        const NesStateBlob *entry = &history->entries[i];
        hash = hash_bytes(hash, &entry->size, sizeof(entry->size));
        hash = hash_bytes(hash, entry->data, entry->size);
    }
    return hash;
}

static bool timeline_matches(const FrontendExecutionRuntime *execution, uint64_t history,
                             const uint32_t *pixels, const uint32_t *saved, size_t bytes) {
    return frontend_execution_rewind_available(execution) == 2
        && history_hash(&execution->rewind) == history
        && nes_runahead_presented_frame(NULL, NULL) == pixels
        && !memcmp(pixels, saved, bytes);
}

static int replay_and_policies(FrontendExecutionRuntime *execution) {
    cpu.pc = 0x8000;
    cpu.halted = false;
    debugger_resume();
    execution_control_set_paused(&execution->execution, false);
    BOARD_CHECK(frontend_execution_set_rewind_seconds(execution, 1));
    BOARD_CHECK(frontend_execution_set_run_ahead(execution, 2));
    BOARD_CHECK(frontend_execution_run_frame(execution) && frontend_execution_run_frame(execution));
    debugger_pause();
    frontend_execution_sync_debugger(execution);
    BOARD_CHECK(frontend_execution_rewind_available(execution) == 2);
    BOARD_CHECK(nes_rewind_bytes(&execution->rewind) > 0);
    unsigned width = 0, height = 0;
    const uint32_t *pixels = nes_runahead_presented_frame(&width, &height);
    BOARD_CHECK(pixels && width == 256 && height == 240);
    size_t pixel_bytes = (size_t)width * height * sizeof(*pixels);
    uint32_t *saved = malloc(pixel_bytes);
    BOARD_CHECK(saved);
    memcpy(saved, pixels, pixel_bytes);
    uint64_t history = history_hash(&execution->rewind);
    uint8_t before = read_mem(0x120);
    char replacement[4];
    snprintf(replacement, sizeof(replacement), "%02X", before);
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x120, 1));
    BOARD_CHECK(act(HEX_BYTES, replacement, 0) && act(HEX_APPLY, NULL, 0));
    BOARD_CHECK(timeline_matches(execution, history, pixels, saved, pixel_bytes));
    BOARD_CHECK(act(HEX_BYTES, "A5 GG", 0) && !act(HEX_APPLY, NULL, 0));
    BOARD_CHECK(timeline_matches(execution, history, pixels, saved, pixel_bytes));
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_CPU, 0x1FFF, 1));
    uint8_t end = read_mem(0x7FF);
    BOARD_CHECK(act(HEX_BYTES, "A5 5A", 0) && !act(HEX_APPLY, NULL, 0));
    BOARD_CHECK(read_mem(0x7FF) == end && timeline_matches(execution, history, pixels, saved, pixel_bytes));
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x120, 1) && act(HEX_BYTES, "5A", 0));
    const uint32_t policies[] = {NES_EXECUTION_MOVIE_RECORDING, NES_EXECUTION_MOVIE_PLAYBACK,
        NES_EXECUTION_NETPLAY, NES_EXECUTION_SPECULATIVE, NES_EXECUTION_REWIND,
        NES_EXECUTION_MOVIE_PLAYBACK | NES_EXECUTION_SPECULATIVE};
    for (size_t i = 0; i < sizeof(policies) / sizeof(*policies); ++i) {
        BOARD_CHECK(nes_execution_set_policy(policies[i]));
        FrontendPanelControl c;
        bool disabled = control(HEX_APPLY, &c) && !c.enabled;
        bool refused = !act(HEX_APPLY, NULL, 0);
        BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
        BOARD_CHECK(disabled && refused && read_mem(0x120) == before);
        BOARD_CHECK(frontend_execution_paused(execution) && debugger_is_paused());
        BOARD_CHECK(timeline_matches(execution, history, pixels, saved, pixel_bytes));
    }
    BOARD_CHECK(act(HEX_LIVE, NULL, 0) && !act(HEX_APPLY, NULL, 0));
    BOARD_CHECK(timeline_matches(execution, history, pixels, saved, pixel_bytes));
    BOARD_CHECK(act(HEX_LIVE, NULL, 1));
    snprintf(replacement, sizeof(replacement), "%02X", before ^ 0xFFu);
    BOARD_CHECK(act(HEX_BYTES, replacement, 0) && act(HEX_APPLY, NULL, 0));
    BOARD_CHECK(read_mem(0x120) == (uint8_t)(before ^ 0xFFu));
    BOARD_CHECK(frontend_execution_rewind_available(execution) == 0 && nes_rewind_bytes(&execution->rewind) == 0);
    BOARD_CHECK(nes_runahead_presented_frame(NULL, NULL) == NULL);
    free(saved);
    BOARD_CHECK(frontend_execution_set_rewind_seconds(execution, 0));
    BOARD_CHECK(frontend_execution_set_run_ahead(execution, 0));
    return 0;
}

static int watch_handoff(void) {
    debug_watch_reset();
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x20, 3) && act(HEX_WATCH, NULL, 0));
    DebugWatchSample sample;
    BOARD_CHECK(debug_watch_at(0, &sample) && sample.address == 0x20 && sample.spec.width == 3);
    uint32_t original = sample.id;
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x100, 5) && act(HEX_WATCH, NULL, 0));
    BOARD_CHECK(debugger_watch_count() == 6);
    for (size_t i = 1; i < 6; ++i) {
        BOARD_CHECK(debug_watch_at(i, &sample) && sample.address == 0xFF + i && sample.spec.width == 1);
        BOARD_CHECK(sample.id != original);
    }
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x200, 59) && !act(HEX_WATCH, NULL, 0));
    BOARD_CHECK(debugger_watch_count() == 6 && debug_watch_at(0, &sample) && sample.id == original);
    BOARD_CHECK(act(HEX_TEXT_MODE, NULL, 1));
    BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x30, 4) && act(HEX_WATCH, NULL, 0));
    BOARD_CHECK(debug_watch_at(6, &sample) && sample.spec.width == 4 && sample.spec.format == DEBUG_WATCH_ASCII);
    BOARD_CHECK(watch_frontend_select(sample.id));
    return 0;
}

static bool foreign_watch_snapshot(void *context, FrontendPanelModel *model, char *error, size_t error_size) {
    (void)context; (void)error; (void)error_size;
    model->status = "Another owner";
    return true;
}

static bool foreign_watch_action(void *context, unsigned id, const char *value, int selected,
                                 char *error, size_t error_size) {
    (void)selected;
    unsigned *calls = context;
    if (id == WATCH_PICK && value && *value) ++*calls;
    if (error && error_size) snprintf(error, error_size, "Selection refused by the watch owner");
    return false;
}

static int watch_rollback(FrontendExecutionRuntime *execution) {
    debug_watch_reset();
    DebugWatchSpec original = {.space = DEBUG_MEMORY_RAM, .width = 2, .format = DEBUG_WATCH_HEX,
        .enabled = true, .expression = "$0070", .label = "Retained watch"};
    uint32_t first = debug_watch_add(&original, 2, hex_error, sizeof(hex_error));
    BOARD_CHECK(first && debugger_watch_count() == 2);
    HexFrontend *hex = hex_frontend_create(execution);
    BOARD_CHECK(hex && hex_frontend_register(hex));
    unsigned calls = 0;
    FrontendPanelSpec foreign = {WATCH_FRONTEND_PANEL, "Another owner", "Tools", FRONTEND_PANEL_NEEDS_SESSION,
        foreign_watch_snapshot, foreign_watch_action, &calls};
    BOARD_CHECK(frontend_panel_register(&foreign));
    const size_t counts[] = {3, 5};
    for (size_t i = 0; i < sizeof(counts) / sizeof(*counts); ++i) {
        BOARD_CHECK(select_bytes(DEBUG_MEMORY_RAM, 0x100, counts[i]) && !act(HEX_WATCH, NULL, 0));
        BOARD_CHECK(calls == i + 1 && debugger_watch_count() == 2);
        for (size_t row = 0; row < 2; ++row) {
            DebugWatchSample sample;
            BOARD_CHECK(debug_watch_at(row, &sample) && sample.id == first + row);
            BOARD_CHECK(sample.spec.width == 2 && sample.address == 0x70 + row * 2);
        }
        FrontendPanelInfo info;
        BOARD_CHECK(frontend_panel_get(WATCH_FRONTEND_PANEL, &info) && !strcmp(info.title, "Another owner"));
        BOARD_CHECK(frontend_panel_count() == 2);
    }
    hex_frontend_destroy(hex);
    BOARD_CHECK(frontend_panel_unregister(WATCH_FRONTEND_PANEL) && frontend_panel_count() == 0);
    return 0;
}

int test_hex_frontend_accuracy(void) {
    bool video_owned = (SDL_WasInit(SDL_INIT_VIDEO) & SDL_INIT_VIDEO) == 0;
    if (video_owned) {
        SDL_setenv("SDL_VIDEODRIVER", "dummy", 0);
        BOARD_CHECK(SDL_InitSubSystem(SDL_INIT_VIDEO) == 0);
    }
    frontend_commands_reset();
    frontend_panels_reset();
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    BoardImage image = {0};
    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0x2000, false));
    uint8_t *prg = image.data + sizeof(iNESHeader);
    prg[0] = 0x4C; prg[1] = 0; prg[2] = 0x80;
    for (unsigned vector = 0x7FFA; vector < 0x8000; vector += 2) {
        prg[vector] = 0; prg[vector + 1] = 0x80;
    }
    cpu_use_default_startup_alignment();
    cpu_set_test_mode(false);
    memset(&pad1, 0, sizeof(pad1));
    memset(&pad2, 0, sizeof(pad2));
    cpu_total_cycles = 0;
    apu_power_on(&apu);
    int loaded = board_image_load(&image);
    board_image_free(&image);
    BOARD_CHECK(loaded == 0);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    BOARD_CHECK(cpu_power_on(&cpu));
    vs_power_on_secondary();
    ppu_begin_frame_render(framebuffer);
    FrontendExecutionRuntime execution;
    frontend_execution_init(&execution, NULL, 44100, NULL, NULL, NULL, NULL);
    debugger_pause();
    execution_control_set_paused(&execution.execution, true);
    DebugFrontend *debug = debug_frontend_create(&execution);
    BOARD_CHECK(debug && debug_frontend_register_ui(debug));
    frontend_panel_set_session_active(true);
    for (uint16_t i = 0; i < 0x800; ++i) write_mem(i, (uint8_t)i);
    int failures = selections();
    failures += frozen_and_live();
    failures += writes(&execution, debug);
    failures += replay_and_policies(&execution);
    failures += files_and_fields(&execution);
    failures += watch_handoff();
    debug_frontend_destroy(debug);
    failures += watch_rollback(&execution);
    frontend_execution_shutdown(&execution);
    debugger_shutdown();
    (void)unload_rom();
    frontend_panels_reset();
    frontend_commands_reset();
    if (video_owned) SDL_QuitSubSystem(SDL_INIT_VIDEO);
    printf("Hex editor controls: %s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures;
}
