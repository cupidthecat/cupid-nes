/*
 * watch_frontend_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Watch editing, window handoff and stable selection. SPDX-License-Identifier: GPL-3.0-or-later */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../debugger/debugger.h"
#include "../debugger/memory_watch.h"
#include "../system/execution_policy.h"
#include "../ui/debug_frontend.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"
#include "../ui/hex_frontend.h"
#include "../ui/watch_frontend.h"
#include "../util/file_io.h"

static bool act(unsigned id, const char *value, int selected) {
    return frontend_panel_action(WATCH_FRONTEND_PANEL, id, value, selected, NULL, 0);
}

static bool control(unsigned panel, unsigned id, FrontendPanelControl *out) {
    FrontendPanelControl controls[64];
    FrontendPanelModel model = {.controls = controls, .capacity = 64};
    if (!frontend_panel_snapshot(panel, &model, NULL, 0)) return false;
    for (size_t i = 0; i < model.count; ++i) {
        if (controls[i].id != id) continue;
        *out = controls[i];
        return true;
    }
    return false;
}

static bool sample(uint32_t id, DebugWatchSample *out, size_t *index) {
    for (size_t i = 0; i < debugger_watch_count(); ++i) {
        if (!debug_watch_at(i, out) || out->id != id) continue;
        if (index) *index = i;
        return true;
    }
    return false;
}

static int workflows(FrontendExecutionRuntime *execution, DebugFrontend *debug) {
    for (uint16_t i = 0; i < 64; ++i) write_mem(i, (uint8_t)i);
    BOARD_CHECK(act(WATCH_EXPRESSION, "$0000", 0) && act(WATCH_LABEL, "Array", 0));
    BOARD_CHECK(act(WATCH_COUNT, "12", 0) && act(WATCH_ADD, NULL, 0));
    BOARD_CHECK(debugger_watch_count() == 12);
    FrontendPanelControl c;
    BOARD_CHECK(control(WATCH_FRONTEND_PANEL, WATCH_INFO, &c) && strstr(c.value, "Page 1 of 2"));
    BOARD_CHECK(act(WATCH_NEXT_PAGE, NULL, 0));
    BOARD_CHECK(control(WATCH_FRONTEND_PANEL, WATCH_ROW, &c) && !strcmp(c.value, "RAM:000A"));
    BOARD_CHECK(act(WATCH_ROW + 1, NULL, 0));
    DebugWatchSample row;
    BOARD_CHECK(debug_watch_at(11, &row));
    uint32_t id = row.id;
    BOARD_CHECK(act(WATCH_SORT + DEBUG_WATCH_SORT_CURRENT, NULL, 0));
    BOARD_CHECK(act(WATCH_SORT + DEBUG_WATCH_SORT_CURRENT, NULL, 0));
    BOARD_CHECK(control(WATCH_FRONTEND_PANEL, WATCH_ROW, &c) && c.selected && !strcmp(c.value, "RAM:000B"));
    BOARD_CHECK(act(WATCH_MOVE_DOWN, NULL, 0));
    size_t index;
    BOARD_CHECK(sample(id, &row, &index) && index == 1);
    BOARD_CHECK(act(WATCH_EXPRESSION, "$FFFF + #1", 0) && !act(WATCH_UPDATE, NULL, 0));
    BOARD_CHECK(sample(id, &row, NULL) && row.address == 11 && row.spec.width == 1);
    BOARD_CHECK(act(WATCH_EXPRESSION, "$0020", 0) && act(WATCH_WIDTH, NULL, 3));
    BOARD_CHECK(act(WATCH_FORMAT, NULL, DEBUG_WATCH_BINARY) && act(WATCH_LABEL, "Value, \"word\"", 0));
    write_mem(0x20, 4);
    write_mem(0x21, 3);
    write_mem(0x22, 2);
    write_mem(0x23, 1);
    BOARD_CHECK(act(WATCH_UPDATE, NULL, 0));
    BOARD_CHECK(control(WATCH_FRONTEND_PANEL, WATCH_VALUES, &c));
    BOARD_CHECK(strstr(c.value, "00000001000000100000001100000100"));
    BOARD_CHECK(act(WATCH_FREEZE, NULL, 1));
    write_mem(0x20, 5);
    BOARD_CHECK(control(WATCH_FRONTEND_PANEL, WATCH_ROW + 1, &c) && !strcmp(c.label, "Changed watch"));
    write_mem(0x20, 6);
    BOARD_CHECK(control(WATCH_FRONTEND_PANEL, WATCH_VALUES, &c));
    BOARD_CHECK(sample(id, &row, NULL) && row.previous == 0x01020304 && row.current == 0x01020306 && row.changes == 2);
    BOARD_CHECK(act(WATCH_PREVIOUS, NULL, 0));
    BOARD_CHECK(sample(id, &row, NULL) && row.previous == row.current && !row.changes);
    BOARD_CHECK(act(WATCH_TOGGLE, NULL, 0));
    BOARD_CHECK(control(WATCH_FRONTEND_PANEL, WATCH_ROW + 1, &c) && !strcmp(c.items[1], "Disabled"));
    BOARD_CHECK(act(WATCH_TOGGLE, NULL, 0));
    BOARD_CHECK(act(WATCH_EXPORT_PATH, "build/watch-table.csv", 0) && act(WATCH_EXPORT, NULL, 0));
    uint8_t *bytes = NULL;
    size_t size = 0;
    BOARD_CHECK(nes_file_read_all("build/watch-table.csv", DEBUG_WATCH_EXPORT_LIMIT, &bytes, &size) == NES_FILE_OK);
    char csv[DEBUG_WATCH_EXPORT_LIMIT + 1];
    memcpy(csv, bytes, size);
    csv[size] = '\0';
    free(bytes);
    BOARD_CHECK(strstr(csv, "id,space,address,expression,width,format,enabled") && strstr(csv, "\"Value, \"\"word\"\"\""));
    const char sentinel[] = "Keep the active image";
    const char *protected_path = "build/watch-protected-image.bin";
    BOARD_CHECK(nes_file_write_atomic(protected_path, sentinel, sizeof(sentinel)) == NES_FILE_OK);
    execution->rom_path = protected_path;
    BOARD_CHECK(act(WATCH_EXPORT_PATH, protected_path, 0) && !act(WATCH_EXPORT, NULL, 0));
    BOARD_CHECK(nes_file_read_all(protected_path, 128, &bytes, &size) == NES_FILE_OK);
    BOARD_CHECK(size == sizeof(sentinel) && !memcmp(bytes, sentinel, size));
    free(bytes);
    execution->rom_path = NULL;
    BOARD_CHECK(nes_file_remove(protected_path) == NES_FILE_OK);
    BOARD_CHECK(nes_file_remove("build/watch-table.csv") == NES_FILE_OK);

    cpu.pc = 0x20;
    BOARD_CHECK(act(WATCH_SPACE, NULL, DEBUG_MEMORY_CPU) && act(WATCH_EXPRESSION, "PC", 0));
    BOARD_CHECK(act(WATCH_UPDATE, NULL, 0));
    cpu.pc = 0x4018;
    BOARD_CHECK(control(WATCH_FRONTEND_PANEL, WATCH_SELECTION, &c) && strstr(c.value, "no readable mapped backing"));
    BOARD_CHECK(sample(id, &row, NULL) && !row.valid && !strcmp(row.spec.expression, "PC"));
    cpu.pc = 0x20;
    BOARD_CHECK(control(WATCH_FRONTEND_PANEL, WATCH_VALUES, &c) && strstr(c.value, "00000001000000100000001100000110"));
    BOARD_CHECK(!watch_frontend_select(UINT32_MAX));
    BOARD_CHECK(act(WATCH_REMOVE, NULL, 0) && debugger_watch_count() == 11);
    debug_frontend_image_changed(debug);
    BOARD_CHECK(control(WATCH_FRONTEND_PANEL, WATCH_UPDATE, &c) && !c.enabled);
    BOARD_CHECK(debugger_watch_count() == 11);
    return 0;
}

static int legacy_selection(void) {
    debug_watch_reset();
    cpu.pc = 0x20;
    DebugWatchSpec spec = {.space = DEBUG_MEMORY_CPU, .width = 2, .format = DEBUG_WATCH_SIGNED, .enabled = true};
    strcpy(spec.expression, "PC");
    uint32_t selected = debug_watch_add(&spec, 1, NULL, 0);
    strcpy(spec.expression, "$0010");
    uint32_t other = debug_watch_add(&spec, 1, NULL, 0);
    BOARD_CHECK(selected && other);
    BOARD_CHECK(frontend_panel_action(DEBUGGER_VIEWERS_FRONTEND_PANEL, VIEW_CONTROL_WATCH_LIST, NULL, 0, NULL, 0));
    /* A separate watch window can reorder the list before this action. */
    BOARD_CHECK(debug_watch_move(selected, 1));
    cpu.pc = 0x24;
    BOARD_CHECK(frontend_panel_action(DEBUGGER_VIEWERS_FRONTEND_PANEL, VIEW_CONTROL_WATCH_TOGGLE, NULL, 0, NULL, 0));
    DebugWatchSample row;
    BOARD_CHECK(sample(selected, &row, NULL) && !row.spec.enabled && !strcmp(row.spec.expression, "PC") &&
                row.spec.format == DEBUG_WATCH_SIGNED);
    BOARD_CHECK(sample(other, &row, NULL) && row.spec.enabled);
    /* The expression must survive another toggle without reordering, even if
     * the CPU has moved since the row was last sampled. */
    cpu.pc = 0x30;
    BOARD_CHECK(frontend_panel_action(DEBUGGER_VIEWERS_FRONTEND_PANEL, VIEW_CONTROL_WATCH_TOGGLE, NULL, 0, NULL, 0));
    BOARD_CHECK(sample(selected, &row, NULL) && row.spec.enabled && row.address == 0x30 &&
                !strcmp(row.spec.expression, "PC") && row.spec.format == DEBUG_WATCH_SIGNED);
    BOARD_CHECK(debugger_remove_watch(selected));
    BOARD_CHECK(!frontend_panel_action(DEBUGGER_VIEWERS_FRONTEND_PANEL, VIEW_CONTROL_WATCH_REMOVE, NULL, 0, NULL, 0));
    BOARD_CHECK(sample(other, &row, NULL) && row.spec.enabled && debugger_watch_count() == 1);
    FrontendPanelControl c;
    BOARD_CHECK(control(DEBUGGER_VIEWERS_FRONTEND_PANEL, VIEW_CONTROL_WATCH_LIST, &c) && c.selected < 0);
    BOARD_CHECK(control(DEBUGGER_VIEWERS_FRONTEND_PANEL, VIEW_CONTROL_WATCH_TOGGLE, &c) && !c.enabled);
    BOARD_CHECK(control(DEBUGGER_VIEWERS_FRONTEND_PANEL, VIEW_CONTROL_WATCH_REMOVE, &c) && !c.enabled);
    BOARD_CHECK(!frontend_panel_action(DEBUGGER_VIEWERS_FRONTEND_PANEL, VIEW_CONTROL_WATCH_REMOVE, NULL, 0, NULL, 0));
    BOARD_CHECK(!frontend_panel_action(DEBUGGER_VIEWERS_FRONTEND_PANEL, VIEW_CONTROL_WATCH_TOGGLE, NULL, 0, NULL, 0));
    BOARD_CHECK(sample(other, &row, NULL) && row.spec.enabled && debugger_watch_count() == 1);
    return 0;
}

static bool reserved(void *context, FrontendPanelModel *model, char *error, size_t size) {
    (void)context;
    (void)error;
    (void)size;
    model->status = "Another owner";
    return true;
}

static int registration(FrontendExecutionRuntime *execution) {
    const unsigned panels[] = {HEX_FRONTEND_PANEL, WATCH_FRONTEND_PANEL};
    for (size_t i = 0; i < sizeof(panels) / sizeof(*panels); ++i) {
        FrontendPanelSpec foreign = {panels[i], "Reserved panel", "Tools", 0, reserved, NULL, NULL};
        BOARD_CHECK(frontend_panel_register(&foreign));
        DebugFrontend *debug = debug_frontend_create(execution);
        BOARD_CHECK(debug && !debug_frontend_register_ui(debug));
        BOARD_CHECK(frontend_panel_count() == 1 && frontend_command_count() == 0);
        debug_frontend_destroy(debug);
        FrontendPanelInfo info;
        BOARD_CHECK(frontend_panel_get(panels[i], &info) && !strcmp(info.title, foreign.title));
        FrontendPanelControl storage[1];
        FrontendPanelModel model = {.controls = storage, .capacity = 1};
        BOARD_CHECK(frontend_panel_snapshot(panels[i], &model, NULL, 0) && !strcmp(model.status, "Another owner"));

        debug = debug_frontend_create(execution);
        BOARD_CHECK(debug && !debug_frontend_register_ui(debug));
        BOARD_CHECK(frontend_panel_unregister(panels[i]));
        BOARD_CHECK(debug_frontend_register_ui(debug) && !debug_frontend_register_ui(debug));
        debug_frontend_destroy(debug);
        BOARD_CHECK(frontend_panel_count() == 0 && frontend_command_count() == 0);
    }
    return 0;
}

int test_watch_frontend_accuracy(void) {
    frontend_commands_reset();
    frontend_panels_reset();
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    BoardImage image = {0};
    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0x2000, false));
    apu_power_on(&apu);
    int loaded = board_image_load(&image);
    board_image_free(&image);
    BOARD_CHECK(loaded == 0);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    BOARD_CHECK(cpu_power_on(&cpu));
    debugger_init();
    FrontendExecutionRuntime execution;
    frontend_execution_init(&execution, NULL, 44100, NULL, NULL, NULL, NULL);
    DebugFrontend *debug = debug_frontend_create(&execution);
    BOARD_CHECK(debug && debug_frontend_register_ui(debug));
    frontend_panel_set_session_active(true);
    int failures = workflows(&execution, debug) + legacy_selection();
    debug_frontend_destroy(debug);
    failures += registration(&execution);
    frontend_execution_shutdown(&execution);
    debugger_shutdown();
    (void)unload_rom();
    frontend_panels_reset();
    frontend_commands_reset();
    printf("Memory watch controls: %s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures;
}
