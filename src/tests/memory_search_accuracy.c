/*
 * memory_search_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Memory search snapshots and observational reads. SPDX-License-Identifier: GPL-3.0-or-later */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../cheats/cheats.h"
#include "../debugger/debugger.h"
#include "../debugger/memory_search.h"
#include "../joypad/joypad.h"
#include "../state/state.h"
#include "../system/execution_policy.h"

static int machine(unsigned mapper) {
    BoardImage image = {0};
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    BOARD_CHECK(cheats_clear() == CHEAT_OK);
    BOARD_CHECK(board_image_create(&image, mapper, 0x10000, 0x8000, false));
    apu_power_on(&apu);
    int loaded = board_image_load(&image);
    board_image_free(&image);
    BOARD_CHECK(loaded == 0);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    BOARD_CHECK(cpu_power_on(&cpu));
    debugger_init();
    return 0;
}

static void put(uint16_t address, uint32_t value, unsigned width) {
    for (unsigned i = 0; i < width; ++i) {
        write_mem((uint16_t)(address + i), (uint8_t)(value >> (8 * i)));
    }
}

static int comparisons(void) {
    BOARD_CHECK(machine(0) == 0);
    DebugMemorySearch *search = debug_search_create(NULL);
    BOARD_CHECK(search);
    /* Three ordered values exercise all six predicates, including both signed
     * boundaries for each supported width. Expected masks are independent of
     * the search implementation. */
    const unsigned masks[] = {2, 5, 4, 1, 6, 3};
    for (unsigned width = 1; width <= 4; width *= 2) {
        for (unsigned sign = 0; sign < 2; ++sign) {
            uint32_t values[3] = {0, 1, (uint32_t)((UINT64_C(1) << (width * 8)) - 1)};
            if (sign) {
                values[0] = 1u << (width * 8 - 1);
                values[1] = 0;
                values[2] = (1u << (width * 8 - 1)) - 1;
            }

            for (unsigned i = 0; i < 3; ++i) {
                put((uint16_t)(0x10 + i * width), values[i], width);
            }

            DebugSearchSpec spec = {DEBUG_MEMORY_RAM, 0x10, 0x10 + 3 * width - 1, (uint8_t)width, sign != 0, true};
            for (unsigned comparison = 0; comparison < DEBUG_SEARCH_COMPARISON_COUNT; ++comparison) {
                BOARD_CHECK(debug_search_start(search, &spec) == DEBUG_SEARCH_OK);
                BOARD_CHECK(debug_search_filter(search, comparison, DEBUG_SEARCH_CONSTANT, values[1]) == DEBUG_SEARCH_OK);
                unsigned seen = 0;
                for (size_t i = 0; i < debug_search_count(search); ++i) {
                    DebugSearchRow row;
                    BOARD_CHECK(debug_search_row(search, i, &row));
                    seen |= 1u << ((row.address - 0x10) / width);
                }

                BOARD_CHECK(seen == masks[comparison] && debug_search_total(search) == 3);
                BOARD_CHECK(debug_search_undo(search) && debug_search_count(search) == 3);
                BOARD_CHECK(!debug_search_can_undo(search));
            }
        }
    }

    BOARD_CHECK(debug_search_number(0x80, 1, true) == -128);
    BOARD_CHECK(debug_search_number(0xFFFF8000, 2, true) == -32768);
    BOARD_CHECK(debug_search_number(0x80000000, 4, true) == INT64_C(-2147483648));
    BOARD_CHECK(debug_search_number(UINT32_MAX, 4, false) == INT64_C(4294967295));
    debug_search_destroy(search);
    return 0;
}

static int history(void) {
    BOARD_CHECK(machine(0) == 0);
    DebugMemorySearch *search = debug_search_create(NULL);
    DebugSearchSpec spec = {DEBUG_MEMORY_RAM, 0x10, 0x13, 1, false, false};
    BOARD_CHECK(search);
    for (uint16_t i = 0x10; i <= 0x13; ++i) {
        write_mem(i, 5);
    }

    BOARD_CHECK(debug_search_start(search, &spec) == DEBUG_SEARCH_OK);
    put(0x10, 0x06040505, 4);
    BOARD_CHECK(debug_search_filter(search, DEBUG_SEARCH_DIFFERENT, DEBUG_SEARCH_PREVIOUS, 0) == DEBUG_SEARCH_OK);
    BOARD_CHECK(debug_search_count(search) == 2);
    DebugSearchRow row;
    BOARD_CHECK(debug_search_row(search, 0, &row) && row.address == 0x12 && row.current == 4 && row.previous == 5);
    BOARD_CHECK(row.initial == 5 && row.changes == 1);
    BOARD_CHECK(debug_search_undo(search) && debug_search_count(search) == 4);
    BOARD_CHECK(debug_search_filter(search, DEBUG_SEARCH_GREATER, DEBUG_SEARCH_INITIAL, 0) == DEBUG_SEARCH_OK);
    BOARD_CHECK(debug_search_count(search) == 1);
    BOARD_CHECK(debug_search_row(search, 0, &row) && row.address == 0x13 && row.current == 6);
    write_mem(0x13, 7);
    BOARD_CHECK(debug_search_refresh(search) == DEBUG_SEARCH_OK);
    BOARD_CHECK(debug_search_row(search, 0, &row) && row.current == 7 && row.previous == 6 && row.changes == 2);
    /* Refresh must preserve the last filtering baseline (6). A previous-sample
     * comparison now sees 7, while a previous-search comparison still sees 6. */
    BOARD_CHECK(debug_search_filter(search, DEBUG_SEARCH_GREATER, DEBUG_SEARCH_PREVIOUS, 0) == DEBUG_SEARCH_OK);
    BOARD_CHECK(debug_search_count(search) == 0 && debug_search_total(search) == 4);
    BOARD_CHECK(debug_search_undo(search));
    BOARD_CHECK(debug_search_filter(search, DEBUG_SEARCH_GREATER, DEBUG_SEARCH_LAST_FILTER, 0) == DEBUG_SEARCH_OK);
    BOARD_CHECK(debug_search_count(search) == 1);
    BOARD_CHECK(debug_search_row(search, 0, &row) && row.changes == 2 && row.initial == 5);
    BOARD_CHECK(debug_search_filter(search, DEBUG_SEARCH_GREATER, DEBUG_SEARCH_LAST_FILTER, 0) == DEBUG_SEARCH_OK);
    BOARD_CHECK(debug_search_count(search) == 0);
    BOARD_CHECK(debug_search_undo(search));
    BOARD_CHECK(debug_search_filter(search, DEBUG_SEARCH_EQUAL, DEBUG_SEARCH_ADDRESS, 0x13) == DEBUG_SEARCH_OK);
    BOARD_CHECK(debug_search_count(search) == 1);
    BOARD_CHECK(debug_search_filter(search, DEBUG_SEARCH_EQUAL, DEBUG_SEARCH_ADDRESS, 0x14) == DEBUG_SEARCH_INVALID);
    BOARD_CHECK(debug_search_count(search) == 1);
    debug_search_destroy(search);
    return 0;
}

static int bounds_and_sort(void) {
    BOARD_CHECK(machine(0) == 0);
    DebugMemorySearch *search = debug_search_create(NULL);
    BOARD_CHECK(search);
    DebugSearchSpec spec = {DEBUG_MEMORY_RAM, 0x11, 0x1B, 4, false, true};
    BOARD_CHECK(debug_search_start(search, &spec) == DEBUG_SEARCH_OK && debug_search_count(search) == 2);
    DebugSearchRow row;
    BOARD_CHECK(debug_search_row(search, 0, &row) && row.address == 0x14);
    BOARD_CHECK(debug_search_row(search, 1, &row) && row.address == 0x18);
    spec.aligned = false;
    BOARD_CHECK(debug_search_start(search, &spec) == DEBUG_SEARCH_OK && debug_search_count(search) == 8);
    spec.first = 0x7FD;
    spec.last = 0x800;
    BOARD_CHECK(debug_search_start(search, &spec) == DEBUG_SEARCH_INVALID && debug_search_count(search) == 8);
    spec.last = 0x7FF;
    BOARD_CHECK(debug_search_start(search, &spec) == DEBUG_SEARCH_INVALID);
    spec = (DebugSearchSpec){DEBUG_MEMORY_RAM, 0x11, 0x14, 4, false, true};
    BOARD_CHECK(debug_search_start(search, &spec) == DEBUG_SEARCH_OK && debug_search_count(search) == 0);
    spec = (DebugSearchSpec){DEBUG_MEMORY_CPU, 0, 0xFFFF, 1, false, false};
    BOARD_CHECK(debug_search_start(search, &spec) == DEBUG_SEARCH_OK && debug_search_count(search) == 65536);
    BOARD_CHECK(debug_search_row(search, 65535, &row) && row.address == 0xFFFF);
    spec.width = 4;
    BOARD_CHECK(debug_search_start(search, &spec) == DEBUG_SEARCH_OK && debug_search_count(search) == 65533);
    BOARD_CHECK(debug_search_row(search, 65532, &row) && row.address == 0xFFFC);
    BOARD_CHECK(debug_search_filter(search, DEBUG_SEARCH_EQUAL, DEBUG_SEARCH_ADDRESS, 0xFFFD) == DEBUG_SEARCH_INVALID);

    spec = (DebugSearchSpec){DEBUG_MEMORY_RAM, 0x10, 0x13, 1, true, false};
    put(0x10, 0x01FF80FF, 4);
    BOARD_CHECK(debug_search_start(search, &spec) == DEBUG_SEARCH_OK);
    BOARD_CHECK(debug_search_sort(search, DEBUG_SEARCH_SORT_CURRENT, false));
    const uint32_t ascending[] = {0x11, 0x10, 0x12, 0x13};
    const uint32_t descending[] = {0x13, 0x10, 0x12, 0x11};
    for (size_t i = 0; i < 4; ++i) {
        BOARD_CHECK(debug_search_row(search, i, &row) && row.address == ascending[i]);
    }

    BOARD_CHECK(debug_search_sort(search, DEBUG_SEARCH_SORT_CURRENT, true));
    for (size_t i = 0; i < 4; ++i) {
        BOARD_CHECK(debug_search_row(search, i, &row) && row.address == descending[i]);
    }

    write_mem(0x11, 0);
    BOARD_CHECK(debug_search_refresh(search) == DEBUG_SEARCH_OK);
    BOARD_CHECK(debug_search_sort(search, DEBUG_SEARCH_SORT_CHANGES, true));
    BOARD_CHECK(debug_search_row(search, 0, &row) && row.address == 0x11 && row.changes == 1);
    BOARD_CHECK(debug_search_sort(search, DEBUG_SEARCH_SORT_ADDRESS, true));
    BOARD_CHECK(debug_search_undo(search));
    BOARD_CHECK(debug_search_row(search, 0, &row) && row.address == 0x13 && row.changes == 0);
    BOARD_CHECK(!debug_search_sort(search, DEBUG_SEARCH_SORT_COUNT, false));
    debug_search_destroy(search);
    return 0;
}

typedef struct {
    size_t live;
    bool fail;
} Allocations;

static void *allocate(void *context, size_t bytes) {
    Allocations *state = context;
    if (state->fail) {
        return NULL;
    }

    void *memory = malloc(bytes);
    if (memory) {
        ++state->live;
    }

    return memory;
}

static void release(void *context, void *memory) {
    Allocations *state = context;
    if (memory) {
        --state->live;
        free(memory);
    }
}

static int failures_and_sessions(void) {
    BOARD_CHECK(machine(0) == 0);
    Allocations state = {0};
    DebugSearchAllocator allocator = {&state, allocate, release};
    state.fail = true;
    BOARD_CHECK(!debug_search_create(&allocator) && state.live == 0);
    state.fail = false;
    DebugMemorySearch *search = debug_search_create(&allocator);
    BOARD_CHECK(search && state.live == 1);
    DebugSearchSpec spec = {DEBUG_MEMORY_RAM, 0x10, 0x12, 1, false, false};
    put(0x10, 0x030201, 3);
    BOARD_CHECK(debug_search_start(search, &spec) == DEBUG_SEARCH_OK);
    BOARD_CHECK(debug_search_filter(search, DEBUG_SEARCH_GREATER, DEBUG_SEARCH_CONSTANT, 1) == DEBUG_SEARCH_OK);
    BOARD_CHECK(debug_search_sort(search, DEBUG_SEARCH_SORT_CURRENT, true));
    DebugSearchRow before[2], after;
    for (size_t i = 0; i < 2; ++i) {
        BOARD_CHECK(debug_search_row(search, i, &before[i]));
    }

    state.fail = true;
    put(0x10, 0, 3);
    BOARD_CHECK(debug_search_start(search, &spec) == DEBUG_SEARCH_NO_MEMORY);
    BOARD_CHECK(debug_search_refresh(search) == DEBUG_SEARCH_NO_MEMORY);
    BOARD_CHECK(debug_search_filter(search, DEBUG_SEARCH_EQUAL, DEBUG_SEARCH_CONSTANT, 0) == DEBUG_SEARCH_NO_MEMORY);
    BOARD_CHECK(debug_search_filter(search, DEBUG_SEARCH_EQUAL, DEBUG_SEARCH_CONSTANT, 256) == DEBUG_SEARCH_INVALID);
    BOARD_CHECK(debug_search_count(search) == 2 && debug_search_total(search) == 3 && debug_search_can_undo(search));
    for (size_t i = 0; i < 2; ++i) {
        BOARD_CHECK(debug_search_row(search, i, &after) && !memcmp(&after, &before[i], sizeof(after)));
    }

    BOARD_CHECK(state.live == 3 && debug_search_undo(search));
    BOARD_CHECK(debug_search_count(search) == 3 && state.live == 2);
    BOARD_CHECK(debug_search_row(search, 0, &after) && after.current == 3 && after.address == 0x12);
    state.fail = false;
    for (unsigned lifecycle = 0; lifecycle < 3; ++lifecycle) {
        BOARD_CHECK(debug_search_start(search, &spec) == DEBUG_SEARCH_OK);
        BOARD_CHECK(debug_search_refresh(search) == DEBUG_SEARCH_OK);
        uint64_t session = debugger_session_revision();
        if (lifecycle == 0) {
            debugger_reset_session();
        } else if (lifecycle == 1) {
            debugger_shutdown();
        } else {
            debugger_init();
        }

        BOARD_CHECK(debugger_session_revision() != session);
        BOARD_CHECK(!debug_search_count(search) && !debug_search_total(search) && !debug_search_can_undo(search));
        BOARD_CHECK(!debug_search_spec(search, &spec) && !debug_search_row(search, 0, &after));
        BOARD_CHECK(debug_search_refresh(search) == DEBUG_SEARCH_NO_SNAPSHOT && state.live == 1);
    }

    debug_search_destroy(search);
    BOARD_CHECK(state.live == 0);
    return 0;
}

static int observational(void) {
    const unsigned mappers[] = {0, 4, 5, 9};
    uint8_t *sample = malloc(0x10000);
    BOARD_CHECK(sample);
    for (size_t board = 0; board < sizeof(mappers) / sizeof(*mappers); ++board) {
        BOARD_CHECK(machine(mappers[board]) == 0);
        BOARD_CHECK(joypad_set_adapter(NES_ADAPTER_NONE));
        BOARD_CHECK(joypad_set_port_device(0, NES_PORT_GAMEPAD));
        BOARD_CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
        pad1.strobe = 0;
        pad1.shift = 0x5A;
        ppu.status = 0xE0;
        ppu.w = 1;
        ppu.open_bus = 0x1F;
        apu.frame_irq = apu.frame_irq_source = true;
        apu.frame_irq_clear_delay = 0;
        NesStateBlob before = {0}, after = {0};
        BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
        for (unsigned space = 0; space < DEBUG_MEMORY_SPACE_COUNT; ++space) {
            uint32_t first, last;
            BOARD_CHECK(debug_memory_bounds(space, &first, &last));
            BOARD_CHECK(debug_memory_sample(space, first, last, sample, 0x10000));
            DebugMemorySearch *search = debug_search_create(NULL);
            DebugSearchSpec spec = {space, first, last, 1, false, false};
            BOARD_CHECK(search && debug_search_start(search, &spec) == DEBUG_SEARCH_OK);
            BOARD_CHECK(debug_search_refresh(search) == DEBUG_SEARCH_OK);
            BOARD_CHECK(debug_search_filter(search, DEBUG_SEARCH_EQUAL, DEBUG_SEARCH_PREVIOUS, 0) == DEBUG_SEARCH_OK);
            BOARD_CHECK(debug_search_count(search) == last - first + 1);
            debug_search_destroy(search);
        }

        BOARD_CHECK(nes_state_capture(&after) == NES_STATE_OK);
        BOARD_CHECK(before.size == after.size && !memcmp(before.data, after.data, before.size));
        BOARD_CHECK(ppu.status == 0xE0 && ppu.w == 1 && ppu.open_bus == 0x1F && pad1.shift == 0x5A);
        BOARD_CHECK(apu.frame_irq && apu.frame_irq_source && apu.frame_irq_clear_delay == 0);
        nes_state_blob_free(&before);
        nes_state_blob_free(&after);
    }

    memset(sample, 0xA5, 0x10000);
    BOARD_CHECK(!debug_memory_sample(DEBUG_MEMORY_RAM, 0, 0x800, sample, 0x10000));
    BOARD_CHECK(!debug_memory_sample(DEBUG_MEMORY_OAM, 0, 0xFF, sample, 0xFF));
    BOARD_CHECK(!debug_memory_sample(DEBUG_MEMORY_CPU, 0xFFFF, 0x10000, sample, 0x10000));
    BOARD_CHECK(!debug_memory_sample(DEBUG_MEMORY_CPU, 2, 1, sample, 0x10000));
    BOARD_CHECK(!debug_memory_sample(DEBUG_MEMORY_SPACE_COUNT, 0, 0, sample, 0x10000));
    for (size_t i = 0; i < 0x10000; ++i) {
        BOARD_CHECK(sample[i] == 0xA5);
    }

    BOARD_CHECK(machine(0) == 0);
    put(0x10, 0x42, 1);
    BOARD_CHECK(cheats_add("0010:FF", "Read override", true, NULL) == CHEAT_OK);
    write_mem(0x200, 0xA5); /* LDA $10 uses the production CPU read hook. */
    write_mem(0x201, 0x10);
    cpu.pc = 0x200;
    BOARD_CHECK(cpu_step(&cpu) > 0 && cpu.a == 0xFF);
    BOARD_CHECK(debug_memory_sample(DEBUG_MEMORY_RAM, 0x10, 0x10, sample, 1) && sample[0] == 0x42);
    BOARD_CHECK(cheats_clear() == CHEAT_OK);
    free(sample);
    return 0;
}

int test_memory_search_accuracy(void) {
    int failures = comparisons() + history() + bounds_and_sort() + failures_and_sessions() + observational();
    debugger_shutdown();
    (void)unload_rom();
    printf("Memory search snapshots: %s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures;
}
