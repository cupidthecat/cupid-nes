/*
 * memory_tools_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Memory Search and Cheat Finder workflows. SPDX-License-Identifier: GPL-3.0-or-later */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../cheats/cheats.h"
#include "../debugger/debugger.h"
#include "../debugger/memory_search.h"
#include "../debugger/memory_watch.h"
#include "../replay/input_event.h"
#include "../system/execution_policy.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_execution.h"
#include "../ui/frontend_panels.h"
#include "../ui/machine_actions.h"
#include "../ui/memory_search_frontend.h"
#include "../ui/watch_frontend.h"

static unsigned panel_id;

static bool act(unsigned id, const char *value, int selected) {
    return frontend_panel_action(panel_id, id, value, selected, NULL, 0);
}

static bool get_control(unsigned id, FrontendPanelControl *out) {
    FrontendPanelControl controls[64];
    FrontendPanelModel model = {.controls = controls, .capacity = 64};
    if (!frontend_panel_snapshot(panel_id, &model, NULL, 0)) {
        return false;
    }

    for (size_t i = 0; i < model.count; ++i) {
        if (controls[i].id == id) {
            *out = controls[i];
            return true;
        }
    }

    return false;
}

static int workflows(void) {
    frontend_commands_reset();
    frontend_panels_reset();
    frontend_panel_set_session_active(true);
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    BOARD_CHECK(cheats_clear() == CHEAT_OK);
    BoardImage image = {0};
    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0x2000, false));
    apu_power_on(&apu);
    BOARD_CHECK(board_image_load(&image) == 0);
    board_image_free(&image);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    BOARD_CHECK(cpu_power_on(&cpu));
    debugger_init();
    FrontendExecutionRuntime runtime;
    frontend_execution_init(&runtime, NULL, 44100, NULL, NULL, NULL, NULL);
    WatchFrontend *watches = watch_frontend_create(&runtime);
    BOARD_CHECK(watches && watch_frontend_register(watches));
    CheatFrontend *editor = cheat_frontend_create("build/");
    BOARD_CHECK(editor && cheat_frontend_register_ui(editor));
    MemoryToolsFrontend *tools = memory_tools_create(editor);
    BOARD_CHECK(tools && memory_tools_register(tools) && !memory_tools_register(tools));
    BOARD_CHECK(!memory_tools_create(NULL));
    for (unsigned tool = 0; tool < 2; ++tool) {
        panel_id = tool ? CHEAT_FINDER_PANEL : MEMORY_SEARCH_PANEL;
        for (uint16_t i = 0; i < 32; ++i) {
            write_mem(i, (uint8_t)i);
        }

        FrontendPanelControl control;
        BOARD_CHECK(act(MEMORY_SEARCH_LAST, "001F", 0));
        BOARD_CHECK(act(MEMORY_SEARCH_START, NULL, 0));
        BOARD_CHECK(get_control(MEMORY_SEARCH_ROW, &control) && !strcmp(control.value, "0000"));
        BOARD_CHECK(act(MEMORY_SEARCH_NEXT_PAGE, NULL, 0));
        BOARD_CHECK(get_control(MEMORY_SEARCH_ROW, &control) && !strcmp(control.value, "000C"));
        BOARD_CHECK(act(MEMORY_SEARCH_ROW + 1, NULL, 0));
        BOARD_CHECK(act(MEMORY_SEARCH_SORT + DEBUG_SEARCH_SORT_CURRENT, NULL, 0));
        BOARD_CHECK(act(MEMORY_SEARCH_SORT + DEBUG_SEARCH_SORT_CURRENT, NULL, 0));
        BOARD_CHECK(get_control(MEMORY_SEARCH_SELECTION, &control) && strstr(control.value, "$000D"));
        size_t watch_index = debugger_watch_count();
        BOARD_CHECK(act(MEMORY_SEARCH_WATCH, NULL, 0));
        DebugWatch watch;
        uint32_t sampled;
        BOARD_CHECK(debugger_watch_at(watch_index, &watch, &sampled) && watch.address == 0xD && watch.size == 1 && sampled == 13);
        BOARD_CHECK(act(MEMORY_SEARCH_REFERENCE, NULL, DEBUG_SEARCH_PREVIOUS));
        BOARD_CHECK(act(MEMORY_SEARCH_COMPARISON, NULL, DEBUG_SEARCH_DIFFERENT));
        write_mem(13, 99);
        BOARD_CHECK(act(MEMORY_SEARCH_FILTER, NULL, 0));
        BOARD_CHECK(get_control(MEMORY_SEARCH_ROW, &control) && !strcmp(control.value, "000D") && control.selected);
        BOARD_CHECK(get_control(MEMORY_SEARCH_ROW + 1, &control) && !control.enabled);
        BOARD_CHECK(act(MEMORY_SEARCH_UNDO, NULL, 0));
        BOARD_CHECK(get_control(MEMORY_SEARCH_INFO, &control) && strstr(control.value, "32 of 32"));
        BOARD_CHECK(act(MEMORY_SEARCH_SPACE, NULL, DEBUG_MEMORY_PPU));
        BOARD_CHECK(act(MEMORY_SEARCH_START, NULL, 0));
        BOARD_CHECK(act(MEMORY_SEARCH_ROW, NULL, 0));
        BOARD_CHECK(act(MEMORY_SEARCH_WATCH, NULL, 0));
        DebugWatchSample ppu_watch;
        BOARD_CHECK(debug_watch_at(watch_index + 1, &ppu_watch) && ppu_watch.spec.space == DEBUG_MEMORY_PPU);
        BOARD_CHECK(!act(MEMORY_SEARCH_SEND, NULL, 0) && !act(MEMORY_SEARCH_TEST, NULL, 0));
        BOARD_CHECK(get_control(MEMORY_SEARCH_WATCH, &control) && control.enabled);
        BOARD_CHECK(act(MEMORY_SEARCH_SPACE, NULL, DEBUG_MEMORY_RAM));
        BOARD_CHECK(act(MEMORY_SEARCH_FIRST, "0010", 0));
        BOARD_CHECK(act(MEMORY_SEARCH_LAST, "0013", 0));
        BOARD_CHECK(act(MEMORY_SEARCH_WIDTH, NULL, 2));
        write_mem(0x10, 0x78);
        write_mem(0x11, 0x56);
        write_mem(0x12, 0x34);
        write_mem(0x13, 0x12);
        BOARD_CHECK(act(MEMORY_SEARCH_START, NULL, 0));
        BOARD_CHECK(act(MEMORY_SEARCH_ROW, NULL, 0));
        BOARD_CHECK(!act(MEMORY_SEARCH_BYTE, NULL, 4));
        BOARD_CHECK(act(MEMORY_SEARCH_BYTE, NULL, 2));
        BOARD_CHECK(get_control(MEMORY_SEARCH_REPLACEMENT, &control) && !strcmp(control.value, "34"));
        BOARD_CHECK(!act(MEMORY_SEARCH_REPLACEMENT, "100", 0));
        BOARD_CHECK(act(MEMORY_SEARCH_REPLACEMENT, "AB", 0));
        BOARD_CHECK(act(MEMORY_SEARCH_COMPARE, "34", 0));
        BOARD_CHECK(act(MEMORY_SEARCH_DESCRIPTION, "Test the selected high byte", 0));
        size_t before = cheats_count();
        BOARD_CHECK(act(MEMORY_SEARCH_SEND, NULL, 0) && cheats_count() == before);
        FrontendPanelControl editor_controls[16];
        FrontendPanelModel model = {.controls = editor_controls, .capacity = 16};
        BOARD_CHECK(frontend_panel_snapshot(CHEATS_FRONTEND_PANEL, &model, NULL, 0));
        BOARD_CHECK(!strcmp(editor_controls[1].value, "0012:AB:34"));
        if (!tool) {
            BOARD_CHECK(!act(MEMORY_SEARCH_TEST, NULL, 0));
        } else {
            for (unsigned policy = 1; policy <= NES_EXECUTION_REWIND; policy <<= 1) {
                BOARD_CHECK(nes_execution_set_policy(policy));
                BOARD_CHECK(!act(MEMORY_SEARCH_TEST, NULL, 0) && cheats_count() == before);
                BOARD_CHECK(get_control(MEMORY_SEARCH_TEST, &control) && !control.enabled);
            }

            BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
            BOARD_CHECK(act(MEMORY_SEARCH_TEST, NULL, 0) && cheats_count() == before + 1);
            CheatRecord cheat;
            BOARD_CHECK(cheats_at(before, &cheat) && cheat.enabled && cheat.address == 0x12);
            BOARD_CHECK(cheat.value == 0xAB && cheat.has_compare && cheat.compare == 0x34);
            write_mem(0x200, 0xA5); /* LDA $12 exercises the enabled read override. */
            write_mem(0x201, 0x12);
            cpu.pc = 0x200;
            BOARD_CHECK(cpu_step(&cpu) > 0 && cpu.a == 0xAB && debugger_peek_cpu(0x12) == 0x34);
            BOARD_CHECK(cheats_set_enabled(cheat.id, false) == CHEAT_OK);
            cpu.pc = 0x200;
            BOARD_CHECK(cpu_step(&cpu) > 0 && cpu.a == 0x34);
            BOARD_CHECK(cheats_remove(cheat.id) == CHEAT_OK);
        }

        BOARD_CHECK(act(MEMORY_SEARCH_FORMAT, NULL, 2));
        BOARD_CHECK(act(MEMORY_SEARCH_START, NULL, 0));
        BOARD_CHECK(act(MEMORY_SEARCH_ROW, NULL, 0));
        BOARD_CHECK(act(MEMORY_SEARCH_WATCH, NULL, 0));
        DebugWatchSample signed_watch;
        BOARD_CHECK(debug_watch_at(debugger_watch_count() - 1, &signed_watch));
        BOARD_CHECK(signed_watch.spec.space == DEBUG_MEMORY_RAM && signed_watch.spec.width == 4 &&
                    signed_watch.spec.format == DEBUG_WATCH_SIGNED);
        BOARD_CHECK(act(MEMORY_SEARCH_REFERENCE, NULL, DEBUG_SEARCH_CONSTANT));
        BOARD_CHECK(act(MEMORY_SEARCH_VALUE, "-2147483649", 0));
        BOARD_CHECK(!act(MEMORY_SEARCH_FILTER, NULL, 0));
        BOARD_CHECK(get_control(MEMORY_SEARCH_INFO, &control) && strstr(control.value, "1 of 1"));
        BOARD_CHECK(act(MEMORY_SEARCH_VALUE, "-2147483648", 0));
        BOARD_CHECK(act(MEMORY_SEARCH_FILTER, NULL, 0));
        memory_tools_image_changed(tools);
        BOARD_CHECK(get_control(MEMORY_SEARCH_FILTER, &control) && !control.enabled);
        BOARD_CHECK(!act(MEMORY_SEARCH_SEND, NULL, 0));
    }

    panel_id = MEMORY_SEARCH_PANEL;
    BOARD_CHECK(act(MEMORY_SEARCH_START, NULL, 0));
    panel_id = CHEAT_FINDER_PANEL;
    BOARD_CHECK(act(MEMORY_SEARCH_START, NULL, 0));
    debugger_shutdown();
    FrontendPanelControl control;
    BOARD_CHECK(get_control(MEMORY_SEARCH_FILTER, &control) && !control.enabled);
    panel_id = MEMORY_SEARCH_PANEL;
    BOARD_CHECK(get_control(MEMORY_SEARCH_FILTER, &control) && !control.enabled);
    memory_tools_destroy(tools);
    watch_frontend_destroy(watches);
    frontend_execution_shutdown(&runtime);
    FrontendPanelInfo info;
    BOARD_CHECK(!frontend_panel_get(MEMORY_SEARCH_PANEL, &info) && !frontend_panel_get(CHEAT_FINDER_PANEL, &info));
    BOARD_CHECK(frontend_panel_get(CHEATS_FRONTEND_PANEL, &info));
    cheat_frontend_destroy(editor);
    BOARD_CHECK(cheats_clear() == CHEAT_OK);
    (void)unload_rom();
    frontend_panels_reset();
    frontend_commands_reset();
    return 0;
}

static bool reject_machine_change(void *context, char *error, size_t size) {
    (void)context;
    (void)error;
    (void)size;
    return false;
}

static bool reject_input(const NesInputEvent *event, void *context) {
    (void)event;
    (void)context;
    return false;
}

static int lifecycle(void) {
    frontend_commands_reset();
    frontend_panels_reset();
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    BoardImage image = {0};
    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0x2000, false));
    apu_power_on(&apu);
    BOARD_CHECK(board_image_load(&image) == 0);
    board_image_free(&image);
    BOARD_CHECK(frontend_machine_power_cycle());
    debugger_init();
    FrontendExecutionRuntime runtime;
    frontend_execution_init(&runtime, NULL, 44100, NULL, NULL, NULL, NULL);
    BOARD_CHECK(frontend_execution_register_commands(&runtime));
    frontend_command_set_session_active(true);
    frontend_panel_set_session_active(true);
    CheatFrontend *editor = cheat_frontend_create("build/");
    MemoryToolsFrontend *tools = memory_tools_create(editor);
    BOARD_CHECK(editor && tools && cheat_frontend_register_ui(editor) && memory_tools_register(tools));
    const unsigned commands[] = {FRONTEND_COMMAND_SOFT_RESET, FRONTEND_COMMAND_POWER_CYCLE};
    for (size_t command = 0; command < sizeof(commands) / sizeof(*commands); ++command) {
        for (unsigned id = MEMORY_SEARCH_PANEL; id <= CHEAT_FINDER_PANEL; ++id) {
            panel_id = id;
            BOARD_CHECK(act(MEMORY_SEARCH_LAST, "001F", 0));
            BOARD_CHECK(act(MEMORY_SEARCH_START, NULL, 0));
        }

        debugger_pause();
        uint64_t revision = debugger_session_revision(), pause = debugger_pause_revision();
        runtime.before_machine_change = reject_machine_change;
        BOARD_CHECK(!frontend_command_invoke(commands[command], NULL, 0));
        BOARD_CHECK(debugger_session_revision() == revision && debugger_pause_revision() == pause);
        runtime.before_machine_change = NULL;
        nes_input_event_set_observer(reject_input, NULL);
        BOARD_CHECK(!frontend_command_invoke(commands[command], NULL, 0));
        BOARD_CHECK(debugger_session_revision() == revision && debugger_is_paused());
        nes_input_event_clear_observer();
        BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_MOVIE_PLAYBACK));
        BOARD_CHECK(!frontend_command_invoke(commands[command], NULL, 0));
        BOARD_CHECK(debugger_session_revision() == revision);
        BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
        for (unsigned id = MEMORY_SEARCH_PANEL; id <= CHEAT_FINDER_PANEL; ++id) {
            panel_id = id;
            FrontendPanelControl control;
            BOARD_CHECK(get_control(MEMORY_SEARCH_INFO, &control) && strstr(control.value, "32 of 32"));
        }

        BOARD_CHECK(frontend_command_invoke(commands[command], NULL, 0));
        BOARD_CHECK(debugger_session_revision() != revision && debugger_pause_revision() == pause);
        BOARD_CHECK(debugger_is_paused());
        for (unsigned id = MEMORY_SEARCH_PANEL; id <= CHEAT_FINDER_PANEL; ++id) {
            panel_id = id;
            FrontendPanelControl control;
            BOARD_CHECK(get_control(MEMORY_SEARCH_FILTER, &control) && !control.enabled);
            BOARD_CHECK(get_control(MEMORY_SEARCH_INFO, &control) && strstr(control.value, "No snapshot"));
        }
    }

    debugger_resume();
    panel_id = MEMORY_SEARCH_PANEL;
    BOARD_CHECK(act(MEMORY_SEARCH_START, NULL, 0));
    uint64_t revision = debugger_session_revision();
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_SPECULATIVE));
    BOARD_CHECK(frontend_machine_soft_reset() && frontend_machine_power_cycle());
    BOARD_CHECK(debugger_session_revision() == revision);
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    /* A real replayed reset invalidates visible snapshots as it is applied. */
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_MOVIE_PLAYBACK));
    NesInputEvent reset = {.type = NES_INPUT_EVENT_SOFT_RESET};
    BOARD_CHECK(nes_input_event_apply(&reset));
    BOARD_CHECK(debugger_session_revision() != revision);
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    memory_tools_destroy(tools);
    cheat_frontend_destroy(editor);
    frontend_execution_shutdown(&runtime);
    debugger_shutdown();
    BOARD_CHECK(unload_rom());
    frontend_panels_reset();
    frontend_commands_reset();
    return 0;
}

static bool occupied(void *context, FrontendPanelModel *model, char *error, size_t size) {
    (void)context;
    (void)error;
    (void)size;
    model->status = "Reserved by another owner";
    return true;
}

static int registration(void) {
    frontend_panels_reset();
    CheatFrontend *editor = cheat_frontend_create("build/");
    BOARD_CHECK(editor);
    for (unsigned collision = 0; collision < 2; ++collision) {
        unsigned id = collision ? CHEAT_FINDER_PANEL : MEMORY_SEARCH_PANEL;
        FrontendPanelSpec owner = {id, "Other panel", "Tools", 0, occupied, NULL, NULL};
        BOARD_CHECK(frontend_panel_register(&owner));
        MemoryToolsFrontend *tools = memory_tools_create(editor);
        BOARD_CHECK(tools && !memory_tools_register(tools));
        memory_tools_destroy(tools);
        FrontendPanelInfo info;
        BOARD_CHECK(frontend_panel_count() == 1 && frontend_panel_get(id, &info));
        BOARD_CHECK(!strcmp(info.title, "Other panel"));
        BOARD_CHECK(frontend_panel_unregister(id));
    }

    cheat_frontend_destroy(editor);
    frontend_panels_reset();
    return 0;
}

static bool refuse_watch(void *context, unsigned id, const char *value, int selected,
                         char *error, size_t error_size) {
    (void)selected;
    unsigned *calls = context;
    if (id == WATCH_PICK && value && *value) {
        ++*calls;
    }

    if (error && error_size) {
        snprintf(error, error_size, "Watch selection refused");
    }

    return false;
}

static int watch_handoff_failure(void) {
    frontend_panels_reset();
    frontend_panel_set_session_active(true);
    debugger_init();
    write_mem(0x10, 0x42);
    uint32_t retained = debugger_add_watch(0x10, 1, "Existing watch");
    BOARD_CHECK(retained);
    CheatFrontend *editor = cheat_frontend_create("build/");
    MemoryToolsFrontend *tools = memory_tools_create(editor);
    BOARD_CHECK(editor && tools && memory_tools_register(tools));
    unsigned calls = 0;
    FrontendPanelSpec owner = {WATCH_FRONTEND_PANEL, "Other watch owner", "Tools", 0,
                              occupied, refuse_watch, &calls};
    for (unsigned tool = 0; tool < 2; ++tool) {
        panel_id = tool ? CHEAT_FINDER_PANEL : MEMORY_SEARCH_PANEL;
        BOARD_CHECK(act(MEMORY_SEARCH_FIRST, "0010", 0));
        BOARD_CHECK(act(MEMORY_SEARCH_LAST, "0010", 0));
        BOARD_CHECK(act(MEMORY_SEARCH_START, NULL, 0));
        BOARD_CHECK(act(MEMORY_SEARCH_ROW, NULL, 0));
        BOARD_CHECK(!act(MEMORY_SEARCH_WATCH, NULL, 0));
        BOARD_CHECK(debugger_watch_count() == 1 && calls == tool);
        BOARD_CHECK(frontend_panel_register(&owner));
        BOARD_CHECK(!act(MEMORY_SEARCH_WATCH, NULL, 0));
        BOARD_CHECK(debugger_watch_count() == 1 && calls == tool + 1);
        DebugWatchSample row;
        BOARD_CHECK(debug_watch_at(0, &row) && row.id == retained && row.address == 0x10);
        BOARD_CHECK(row.current == 0x42 && !strcmp(row.spec.label, "Existing watch"));
        FrontendPanelControl selection;
        BOARD_CHECK(get_control(MEMORY_SEARCH_SELECTION, &selection) && strstr(selection.value, "$0010"));
        FrontendPanelInfo info;
        BOARD_CHECK(frontend_panel_get(WATCH_FRONTEND_PANEL, &info) && !strcmp(info.title, "Other watch owner"));
        BOARD_CHECK(frontend_panel_unregister(WATCH_FRONTEND_PANEL));
    }

    memory_tools_destroy(tools);
    cheat_frontend_destroy(editor);
    debugger_shutdown();
    frontend_panels_reset();
    return 0;
}

int test_memory_tools_accuracy(void) {
    int failures = workflows() + lifecycle() + registration() + watch_handoff_failure();
    printf("Memory Search and Cheat Finder controls: %s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures;
}
