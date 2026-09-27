/*
 * memory_editor_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Transactional memory editing and physical mappings. SPDX-License-Identifier: GPL-3.0-or-later */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../cheats/cheats.h"
#include "../debugger/debugger.h"
#include "../debugger/memory_editor.h"
#include "../joypad/joypad.h"
#include "../rom/game_db.h"
#include "../state/state.h"
#include "../system/execution_policy.h"
#include <time.h>

static int machine(unsigned mapper, size_t chr, uint8_t prg_ram, uint8_t chr_ram, unsigned submapper) {
    BoardImage image = {0};
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    BOARD_CHECK(cheats_clear() == CHEAT_OK);
    BOARD_CHECK(board_image_create(&image, mapper, 0x10000, chr, true));
    image.data[8] |= (uint8_t)(submapper << 4);
    image.data[10] = prg_ram;
    image.data[11] = chr_ram;
    if ((prg_ram | chr_ram) & 0xF0) image.data[6] |= 2;
    apu_power_on(&apu);
    int result = board_image_load(&image);
    board_image_free(&image);
    BOARD_CHECK(result == 0);
    apu_power_on(&apu);
    debugger_init();
    debugger_pause();
    ppu_set_oam_decay(false);
    return 0;
}

static bool edit(DebugMemorySpace space, uint32_t first, const uint8_t *data, size_t count, bool *changed) {
    return debug_memory_edit(space, first, data, count, debugger_session_revision(), changed, NULL, 0);
}

static int compare_state(NesStateBlob *before) {
    NesStateBlob after = {0};
    BOARD_CHECK(nes_state_capture(&after) == NES_STATE_OK);
    bool equal = before->size == after.size && !memcmp(before->data, after.data, before->size);
    nes_state_blob_free(&after);
    BOARD_CHECK(equal);
    return 0;
}

static int byte_sequences(void) {
    const struct { const char *text; bool ascii; uint8_t bytes[8]; size_t count; } valid[] = {
        {"00 7f FF", false, {0, 0x7F, 0xFF}, 3},
        {"$12,0x34\n0X56\t789a", false, {0x12, 0x34, 0x56, 0x78, 0x9A}, 5},
        {"NES \xC3\xA9", true, {'N', 'E', 'S', ' ', 0xC3, 0xA9}, 6}
    };
    char error[192], text[32];
    uint8_t bytes[8];
    size_t count;
    for (size_t i = 0; i < sizeof(valid) / sizeof(*valid); ++i) {
        BOARD_CHECK(debug_memory_parse_bytes(valid[i].text, valid[i].ascii, bytes, sizeof(bytes),
                                             &count, error, sizeof(error)));
        BOARD_CHECK(count == valid[i].count && !memcmp(bytes, valid[i].bytes, count) && !error[0]);
        BOARD_CHECK(debug_memory_format_hex(bytes, count, text, sizeof(text)));
        uint8_t roundtrip[8];
        size_t decoded;
        BOARD_CHECK(debug_memory_parse_bytes(text, false, roundtrip, sizeof(roundtrip), &decoded, NULL, 0));
        BOARD_CHECK(decoded == count && !memcmp(bytes, roundtrip, count));
    }
    const char *const invalid[] = {"", " ", "$,", "0x", "A", "012", "12 Q0", "GG", "1 23", "00 01 02 03 04 05 06 07 08"};
    for (size_t i = 0; i < sizeof(invalid) / sizeof(*invalid); ++i) {
        memset(bytes, 0xA5, sizeof(bytes));
        count = 79;
        BOARD_CHECK(!debug_memory_parse_bytes(invalid[i], false, bytes, sizeof(bytes), &count, error, sizeof(error)));
        BOARD_CHECK(count == 79 && error[0]);
        for (size_t j = 0; j < sizeof(bytes); ++j) BOARD_CHECK(bytes[j] == 0xA5);
    }
    char *bounded = malloc(DEBUG_MEMORY_TEXT_LIMIT);
    BOARD_CHECK(bounded);
    memset(bounded, 'A', DEBUG_MEMORY_TEXT_LIMIT);
    BOARD_CHECK(!debug_memory_parse_bytes(bounded, false, bytes, sizeof(bytes), &count, NULL, 0));
    free(bounded);
    strcpy(text, "unchanged");
    BOARD_CHECK(!debug_memory_format_hex(bytes, 8, text, 23) && !strcmp(text, "unchanged"));
    BOARD_CHECK(!debug_memory_parse_bytes("too long", true, bytes, 3, &count, NULL, 0));
    BOARD_CHECK(!debug_memory_parse_bytes(NULL, false, bytes, sizeof(bytes), &count, NULL, 0));
    return 0;
}

static int transactional_ram(void) {
    BOARD_CHECK(machine(0, 0, 7, 7, 0) == 0);
    const uint8_t value[] = {0x12, 0x34};
    bool changed = false;
    BOARD_CHECK(edit(DEBUG_MEMORY_RAM, 0x10, value, 2, &changed) && changed);
    BOARD_CHECK(cpu_peek_internal_ram(0x10) == 0x12 && cpu_peek_internal_ram(0x11) == 0x34);
    NesMemoryLocation base, mirror;
    BOARD_CHECK(cpu_debug_memory_location(0x10, &base) && cpu_debug_memory_location(0x810, &mirror));
    BOARD_CHECK(base.data == mirror.data && base.offset == 0x10 && !strcmp(base.backing, "CPU RAM"));
    NesStateBlob before = {0};
    BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
    BOARD_CHECK(edit(DEBUG_MEMORY_CPU, 0x810, value, 2, &changed) && !changed);
    BOARD_CHECK(!edit(DEBUG_MEMORY_CPU, 0x1FFF, value, 2, &changed) && !changed);
    BOARD_CHECK(!edit(DEBUG_MEMORY_RAM, 0x7FF, value, 2, &changed));
    BOARD_CHECK(!edit(DEBUG_MEMORY_CPU, 0x8000, value, 2, &changed));
    BOARD_CHECK(!edit(DEBUG_MEMORY_PPU, 0x4000, value, 1, &changed));
    BOARD_CHECK(!edit((DebugMemorySpace)-1, 0, value, 1, &changed));
    BOARD_CHECK(!edit(DEBUG_MEMORY_RAM, 0, value, SIZE_MAX, &changed));
    BOARD_CHECK(!debug_memory_edit(DEBUG_MEMORY_RAM, 0, value, 1, debugger_session_revision() - 1,
                                    &changed, NULL, 0));
    const uint32_t policies[] = {NES_EXECUTION_MOVIE_RECORDING, NES_EXECUTION_MOVIE_PLAYBACK,
                                NES_EXECUTION_REWIND, NES_EXECUTION_NETPLAY, NES_EXECUTION_SPECULATIVE};
    for (size_t i = 0; i < sizeof(policies) / sizeof(*policies); ++i) {
        BOARD_CHECK(nes_execution_set_policy(policies[i]));
        BOARD_CHECK(!edit(DEBUG_MEMORY_RAM, 0, value, 1, &changed) && !changed);
    }
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    debugger_resume();
    BOARD_CHECK(!edit(DEBUG_MEMORY_RAM, 0, value, 1, &changed));
    debugger_pause();
    BOARD_CHECK(compare_state(&before) == 0);
    nes_state_blob_free(&before);

    uint8_t aliases[0x801];
    memset(aliases, 0x6A, sizeof(aliases));
    aliases[0x800] = 0x6B;
    BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
    BOARD_CHECK(!edit(DEBUG_MEMORY_CPU, 0, aliases, sizeof(aliases), &changed) && !changed);
    BOARD_CHECK(compare_state(&before) == 0);
    nes_state_blob_free(&before);
    aliases[0x800] = 0x6A;
    BOARD_CHECK(edit(DEBUG_MEMORY_CPU, 0, aliases, sizeof(aliases), &changed) && changed);
    for (uint16_t i = 0; i < 0x800; ++i) BOARD_CHECK(cpu_peek_internal_ram(i) == 0x6A);
    return 0;
}

static int ppu_backing(void) {
    BOARD_CHECK(machine(0, 0, 7, 7, 0) == 0);
    bool changed;
    const uint8_t value[] = {0xFF, 0xBE, 0xAA, 0xEF};
    ppu_set_oam_decay(true);
    ppu.oam_decay_cycles[0] = 0;
    BOARD_CHECK(edit(DEBUG_MEMORY_OAM, 0, value, 4, &changed) && changed);
    BOARD_CHECK(ppu.oam[0] == 0xFF && ppu.oam[1] == 0xBE && ppu.oam[2] == 0xA2 && ppu.oam[3] == 0xEF);
    BOARD_CHECK(ppu.oam_decay_cycles[0] == cpu_get_bus_cycle());
    BOARD_CHECK(edit(DEBUG_MEMORY_PALETTE, 0x3F10, value, 1, &changed) && changed);
    BOARD_CHECK(ppu_palette[0] == 0x3F && ppu_palette[0x10] == 0x3F);
    uint8_t palette[32];
    for (unsigned i = 0; i < 32; ++i) palette[i] = (uint8_t)(i & 15);
    palette[16] = 1;
    NesStateBlob before = {0};
    BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
    BOARD_CHECK(!edit(DEBUG_MEMORY_PALETTE, 0x3F00, palette, 32, &changed) && !changed);
    BOARD_CHECK(compare_state(&before) == 0);
    nes_state_blob_free(&before);
    palette[16] = 0;
    BOARD_CHECK(edit(DEBUG_MEMORY_PALETTE, 0x3F00, palette, 32, &changed) && changed);
    BOARD_CHECK(edit(DEBUG_MEMORY_PPU, 0x0123, value, 1, &changed));
    BOARD_CHECK(debugger_peek_ppu(0x0123) == 0xFF);
    BOARD_CHECK(edit(DEBUG_MEMORY_PPU, 0x2123, value, 1, &changed));
    BOARD_CHECK(debugger_peek_ppu(0x2123) == 0xFF && debugger_peek_ppu(0x3123) == 0xFF);
    NesMemoryLocation a, b;
    BOARD_CHECK(ppu_debug_memory_location(0x2123, &a) && ppu_debug_memory_location(0x3123, &b));
    BOARD_CHECK(a.data == b.data && a.writable);
    ppu_set_oam_decay(false);
    BOARD_CHECK(machine(0, 0x2000, 7, 0, 0) == 0);
    BOARD_CHECK(ppu_debug_memory_location(0x123, &a) && !a.writable && !strcmp(a.backing, "CHR ROM"));
    BOARD_CHECK(!edit(DEBUG_MEMORY_PPU, 0x123, value, 1, &changed));
    return 0;
}

static int protection_and_banks(void) {
    BOARD_CHECK(machine(4, 0, 7, 7, 0) == 0);
    const uint8_t value = 0xA6;
    bool changed;
    NesMemoryLocation ram;
    write_mem(0xA001, 0x80);
    BOARD_CHECK(cpu_debug_memory_location(0x6000, &ram) && ram.writable);
    BOARD_CHECK(edit(DEBUG_MEMORY_CART_RAM, 0x6000, &value, 1, &changed) && changed);
    BOARD_CHECK(debugger_peek_cpu(0x6000) == value);
    write_mem(0xA001, 0xC0);
    BOARD_CHECK(cpu_debug_memory_location(0x6000, &ram) && !ram.writable);
    BOARD_CHECK(!edit(DEBUG_MEMORY_CART_RAM, 0x6000, &value, 1, &changed));
    write_mem(0xA001, 0);
    BOARD_CHECK(!cpu_debug_memory_location(0x6000, &ram));

    BOARD_CHECK(machine(4, 0, 4, 7, 1) == 0);
    write_mem(0x8000, 0x20);
    write_mem(0xA001, 0xE0);
    BOARD_CHECK(cpu_debug_memory_location(0x7000, &ram) && !ram.writable);
    BOARD_CHECK(cpu_debug_memory_location(0x7200, &ram) && ram.writable);
    BOARD_CHECK(edit(DEBUG_MEMORY_CPU, 0x7200, &value, 1, &changed));
    BOARD_CHECK(!edit(DEBUG_MEMORY_CPU, 0x7000, &value, 1, &changed));
    BOARD_CHECK(debugger_peek_cpu(0x7600) == value);

    BOARD_CHECK(machine(5, 0, 0x77, 7, 0) == 0);
    write_mem(0x5102, 2);
    write_mem(0x5103, 1);
    write_mem(0x5113, 0);
    BOARD_CHECK(cpu_debug_memory_location(0x6123, &ram) && ram.writable && !strcmp(ram.backing, "PRG NVRAM"));
    BOARD_CHECK(edit(DEBUG_MEMORY_CPU, 0x6123, &value, 1, &changed));
    write_mem(0x5113, 4);
    BOARD_CHECK(cpu_debug_memory_location(0x6123, &ram) && !strcmp(ram.backing, "PRG RAM"));
    uint8_t other = 0x35;
    BOARD_CHECK(edit(DEBUG_MEMORY_CPU, 0x6123, &other, 1, &changed));
    write_mem(0x5113, 0);
    BOARD_CHECK(debugger_peek_cpu(0x6123) == value);
    write_mem(0x5100, 3);
    write_mem(0x5114, 0);
    BOARD_CHECK(cpu_debug_memory_location(0x8123, &ram) && ram.writable && *ram.data == value);
    write_mem(0x5102, 0);
    BOARD_CHECK(cpu_debug_memory_location(0x8123, &ram) && !ram.writable);
    BOARD_CHECK(!edit(DEBUG_MEMORY_CPU, 0x8123, &other, 1, &changed));
    write_mem(0x5104, 2);
    BOARD_CHECK(edit(DEBUG_MEMORY_CPU, 0x5C10, &value, 1, &changed));
    write_mem(0x5104, 3);
    BOARD_CHECK(cpu_debug_memory_location(0x5C10, &ram) && !ram.writable && *ram.data == value);
    BOARD_CHECK(!edit(DEBUG_MEMORY_CPU, 0x5C10, &other, 1, &changed));
    return 0;
}

static int expanded_cpu_ram(void) {
    static const char board_name[] = "SSS-NROM-256";
    BoardImage image = {0};
    image.size = 32 + 8 + sizeof(board_name) + 8 + 0x8000 + 8 + 0x2000;
    image.data = calloc(1, image.size);
    BOARD_CHECK(image.data);
    memcpy(image.data, "UNIF", 4);
    image.data[4] = 7;
    uint8_t *chunk = image.data + 32;
    memcpy(chunk, "MAPR", 4);
    chunk[4] = sizeof(board_name);
    memcpy(chunk + 8, board_name, sizeof(board_name));
    chunk += 8 + sizeof(board_name);
    memcpy(chunk, "PRG0", 4);
    chunk[5] = 0x80;
    chunk += 8 + 0x8000;
    memcpy(chunk, "CHR0", 4);
    chunk[5] = 0x20;
    apu_power_on(&apu);
    int loaded = board_image_load(&image);
    board_image_free(&image);
    BOARD_CHECK(loaded == 0 && cart_cpu_ram_8k());
    debugger_init();
    debugger_pause();
    uint32_t first, last;
    BOARD_CHECK(debug_memory_bounds(DEBUG_MEMORY_RAM, &first, &last) && first == 0 && last == 0x1FFF);
    uint8_t bytes[0x2000], copied[0x2000];
    for (size_t i = 0; i < sizeof(bytes); ++i) bytes[i] = (uint8_t)(0x11 * (1 + i / 0x800));
    bool changed;
    BOARD_CHECK(edit(DEBUG_MEMORY_RAM, 0, bytes, sizeof(bytes), &changed) && changed);
    BOARD_CHECK(debug_memory_copy(DEBUG_MEMORY_CPU, 0, sizeof(copied), copied, sizeof(copied), NULL, 0));
    BOARD_CHECK(!memcmp(bytes, copied, sizeof(bytes)));
    for (unsigned bank = 0; bank < 4; ++bank) {
        unsigned address = bank * 0x800 + 0x33;
        NesMemoryLocation location;
        BOARD_CHECK(cpu_debug_memory_location((uint16_t)address, &location));
        BOARD_CHECK(location.writable && location.offset == address);
        BOARD_CHECK(debugger_peek_cpu((uint16_t)address) == bytes[address]);
    }
    NesStateBlob before = {0};
    BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
    bool refused = !edit(DEBUG_MEMORY_RAM, 0x1FFF, bytes, 2, &changed) && !changed;
    int preserved = compare_state(&before);
    nes_state_blob_free(&before);
    BOARD_CHECK(refused && preserved == 0);
    return 0;
}

static int selected_ppu_edits(PpuMachineContext *secondary) {
    const uint8_t value = 0xB7;
    bool changed;
    BOARD_CHECK(edit(DEBUG_MEMORY_PPU, 0x2123, &value, 1, &changed) && changed);
    BOARD_CHECK(secondary->vram[0x123] == value && debugger_peek_ppu(0x3123) == value);
    BOARD_CHECK(edit(DEBUG_MEMORY_PALETTE, 0x3F10, &value, 1, &changed) && changed);
    BOARD_CHECK(secondary->palette[0] == 0x37 && secondary->palette[0x10] == 0x37);
    BOARD_CHECK(edit(DEBUG_MEMORY_OAM, 2, &value, 1, &changed) && changed);
    BOARD_CHECK(secondary->state.oam[2] == 0xA3);
    BOARD_CHECK(secondary->state.oam_decay_cycles[0] == cpu_get_bus_cycle());
    uint8_t sampled[4] = {0}, copied[4] = {0}, snapshot[256];
    DebugMemoryPage page = {0};
    BOARD_CHECK(debug_memory_capture(&page, DEBUG_MEMORY_OAM, 0, false));
    BOARD_CHECK(page.bytes[0].value == 0x22 && page.bytes[2].value == 0xA3);
    BOARD_CHECK(debug_memory_copy(DEBUG_MEMORY_OAM, 0, sizeof(copied), copied, sizeof(copied), NULL, 0));
    BOARD_CHECK(debug_memory_sample(DEBUG_MEMORY_OAM, 0, 3, sampled, sizeof(sampled)));
    BOARD_CHECK(!memcmp(copied, sampled, sizeof(copied)));
    debugger_copy_oam(snapshot);
    BOARD_CHECK(!memcmp(snapshot, secondary->state.oam, sizeof(snapshot)));
    uint64_t refresh = secondary->state.oam_decay_cycles[0];
    BOARD_CHECK(edit(DEBUG_MEMORY_OAM, 2, &value, 1, &changed) && !changed);
    BOARD_CHECK(secondary->state.oam_decay_cycles[0] == refresh);
    return 0;
}

static int selected_ppu_context(void) {
    BOARD_CHECK(machine(0, 0, 7, 7, 0) == 0);
    PpuMachineContext *secondary = calloc(1, sizeof(*secondary));
    BOARD_CHECK(secondary);
    memset(ppu.oam, 0x11, sizeof(ppu.oam));
    memset(ppu_vram, 0x44, sizeof(ppu_vram));
    memset(ppu_palette, 0x15, sizeof(ppu_palette));
    memset(secondary->state.oam, 0x22, sizeof(secondary->state.oam));
    memset(secondary->vram, 0x66, sizeof(secondary->vram));
    memset(secondary->palette, 0x27, sizeof(secondary->palette));
    NesStateBlob before = {0};
    if (nes_state_capture(&before) != NES_STATE_OK) {
        free(secondary);
        BOARD_CHECK(false);
    }
    ppu_set_oam_decay(true);
    ppu_select_machine(secondary, NULL);
    int failures = selected_ppu_edits(secondary);
    ppu_select_machine(NULL, NULL);
    ppu_set_oam_decay(false);
    free(secondary);
    failures += compare_state(&before);
    nes_state_blob_free(&before);
    return failures;
}

static int custom_view_check(DebugMemorySpace space, uint32_t address) {
    DebugMemoryByte byte;
    DebugMemoryPage page = {0};
    uint8_t copied = 0, replacement = 0xB6;
    bool changed;
    BOARD_CHECK(debug_memory_inspect(space, address, &byte));
    if (!byte.readable || byte.physical || byte.writable)
        fprintf(stderr, "custom view space=%u address=$%04X readable=%u physical=%u writable=%u\n",
                (unsigned)space, (unsigned)address, byte.readable, byte.physical, byte.writable);
    BOARD_CHECK(byte.readable && !byte.physical && !byte.writable);
    BOARD_CHECK(debug_memory_capture(&page, space, address, false));
    BOARD_CHECK(debug_memory_copy(space, address, 1, &copied, 1, NULL, 0) && copied == byte.value);
    BOARD_CHECK(!edit(space, address, &replacement, 1, &changed) && !changed);
    return 0;
}

static int custom_mapper_views(void) {
    BOARD_CHECK(machine(284, 0, 7, 7, 0) == 0);
    write_mem(0xC123, 1);
    ppu_write(0x23D2, 0x1B);
    write_mem(0x800A, 4);
    cart_set_ppu_fetch_source(CART_PPU_FETCH_BG);
    (void)cart_ppu_read(0x2123);
    cart_set_ppu_fetch_source(CART_PPU_FETCH_CPU);
    write_mem(0x8002, 0);
    write_mem(0x8003, 0x10);
    write_mem(0x8000, 0);
    write_mem(0x8001, 0xC0);
    write_mem(0x8001, 0x40);
    NesStateBlob before = {0};
    BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
    BOARD_CHECK(cart_cpu_peek_bus(0x5000, 0) == 0 && cart_cpu_peek_bus(0x5000, 0xFF) == 0);
    BOARD_CHECK(debugger_peek_cpu(0x5800) == 0x40);
    BOARD_CHECK((debugger_peek_cpu(0x4800) & 0x7F) == 0x64);
    int failures = custom_view_check(DEBUG_MEMORY_CPU, 0x5000);
    failures += custom_view_check(DEBUG_MEMORY_PPU, 0x23D2);
    failures += compare_state(&before);
    nes_state_blob_free(&before);

    BOARD_CHECK(machine(682, 0x2000, 9, 7, 0) == 0);
    write_mem(0x4100, 0x84);
    write_mem(0x4106, 0xC0);
    write_mem(0x4116, 0);
    write_mem(0x4107, 0xC0);
    write_mem(0x4117, 1);
    write_mem(0x415C, 0x1F);
    write_mem(0x415D, 0xFF);
    write_mem(0x415E, 2);
    write_mem(0x415F, 0x87);
    write_mem(0x415F, 0x98);
    write_mem(0x415C, 0x1F);
    write_mem(0x415D, 0xFF);
    BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
    BOARD_CHECK(cart_cpu_peek_bus(0x415F, 0) == 0x87 && cart_cpu_peek_bus(0x415F, 0xFF) == 0x87);
    BOARD_CHECK(debugger_peek_cpu(0x4160) == 0x21);
    BOARD_CHECK(debugger_peek_cpu(0x4100) == 0x84);
    (void)debugger_peek_cpu(0x4151);
    (void)debugger_peek_cpu(0x4280);
    (void)debugger_peek_cpu(0x4282);
    (void)debugger_peek_cpu(0xFFFA);
    (void)debugger_peek_cpu(0xFFFB);
    failures += custom_view_check(DEBUG_MEMORY_CPU, 0x415F);
    failures += custom_view_check(DEBUG_MEMORY_PPU, 0x0123);
    failures += custom_view_check(DEBUG_MEMORY_PPU, 0x2123);
    failures += compare_state(&before);
    nes_state_blob_free(&before);
    BOARD_CHECK(read_mem(0x415F) == 0x87 && read_mem(0x415F) == 0x98);
    return failures;
}

static int unequal_ram_chips(void) {
    BOARD_CHECK(machine(0, 0x2000, 0x65, 0, 0) == 0);
    NesMemoryLocation first, mirror;
    BOARD_CHECK(cpu_debug_memory_location(0x6001, &first) && cpu_debug_memory_location(0x7001, &mirror));
    BOARD_CHECK(first.data == mirror.data && first.offset == 1 && !strcmp(first.backing, "PRG NVRAM"));
    uint8_t value = 0xA3;
    bool changed;
    BOARD_CHECK(edit(DEBUG_MEMORY_CART_RAM, 0x6001, &value, 1, &changed) && changed);
    BOARD_CHECK(debugger_peek_cpu(0x7001) == value);
    uint8_t bytes[0x1001];
    memset(bytes, 0x6A, sizeof(bytes));
    bytes[0x1000] = 0x6B;
    NesStateBlob before = {0};
    BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
    bool refused = !edit(DEBUG_MEMORY_CPU, 0x6000, bytes, sizeof(bytes), &changed) && !changed;
    int preserved = compare_state(&before);
    nes_state_blob_free(&before);
    BOARD_CHECK(refused && preserved == 0);
    bytes[0x1000] = 0x6A;
    BOARD_CHECK(edit(DEBUG_MEMORY_CPU, 0x6000, bytes, sizeof(bytes), &changed) && changed);
    BOARD_CHECK(debugger_peek_cpu(0x6001) == 0x6A && debugger_peek_cpu(0x7001) == 0x6A);
    return 0;
}

static int database_ram_layout(unsigned work, unsigned save) {
    BoardImage image = {0};
    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0x2000, false));
    uint32_t crc = game_db_crc32(image.data + 16, image.size - 16);
    char row[256];
    int length = snprintf(row, sizeof(row),
        "%08X,NesNtsc,TEST,,,0,b32768,b8192,0,b%u,b%u,%u,h,1,N,0,0,0\n",
        (unsigned)crc, work, save, save ? 1u : 0u);
    bool configured = length > 0 && (size_t)length < sizeof(row)
        && rom_database_load_memory(row, (size_t)length);
    apu_power_on(&apu);
    int loaded = configured ? board_image_load(&image) : -1;
    board_image_free(&image);
    BOARD_CHECK(loaded == 0 && rom_metadata_source() == ROM_METADATA_DATABASE);
    debugger_init();
    debugger_pause();
    uint8_t value = 0x83;
    bool changed;
    DebugMemoryByte byte;
    if (work == 0x80 || save == 0x80) {
        BOARD_CHECK(debug_memory_inspect(DEBUG_MEMORY_CPU, 0x6001, &byte) && !byte.readable && !byte.physical);
        BOARD_CHECK(!edit(DEBUG_MEMORY_CPU, 0x6001, &value, 1, &changed) && !changed);
    } else {
        BOARD_CHECK(debug_memory_inspect(DEBUG_MEMORY_CPU, 0x6001, &byte) && byte.writable && byte.offset == 1);
        BOARD_CHECK(!strcmp(byte.backing, save ? "PRG NVRAM" : "PRG RAM"));
        BOARD_CHECK(edit(DEBUG_MEMORY_CPU, 0x6001, &value, 1, &changed) && changed);
        BOARD_CHECK(debugger_peek_cpu(0x6C01) == (save ? 0 : value));
        BOARD_CHECK(debug_memory_inspect(DEBUG_MEMORY_CPU, 0x7801, &byte) && !byte.readable && !byte.physical);
        BOARD_CHECK(!edit(DEBUG_MEMORY_CPU, 0x7801, &value, 1, &changed) && !changed);
        uint8_t crossing[] = {0x35, 0x46};
        NesStateBlob before = {0};
        BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
        bool refused = !edit(DEBUG_MEMORY_CPU, 0x77FF, crossing, sizeof(crossing), &changed) && !changed;
        int preserved = compare_state(&before);
        nes_state_blob_free(&before);
        BOARD_CHECK(refused && preserved == 0);
    }
    BOARD_CHECK(unload_rom());
    return 0;
}

static int partial_ram_pages(void) {
    bool previous = rom_database_overrides_enabled();
    rom_database_set_overrides(true);
    int failures = database_ram_layout(0x0C00, 0) + database_ram_layout(0x2000, 0x1800)
        + database_ram_layout(0x80, 0) + database_ram_layout(0, 0x80);
    (void)unload_rom();
    rom_database_clear();
    rom_database_set_overrides(previous);
    return failures;
}

static bool write_editor_file(const char *path, const uint8_t *bytes, size_t size) {
    FILE *file = fopen(path, "wb");
    if (!file) return false;
    bool written = fwrite(bytes, 1, size, file) == size;
    return fclose(file) == 0 && written;
}

static bool read_editor_file(const char *path, uint8_t *bytes, size_t size) {
    FILE *file = fopen(path, "rb");
    if (!file) return false;
    bool read = fread(bytes, 1, size, file) == size;
    int extra = fgetc(file);
    return fclose(file) == 0 && read && extra == EOF;
}

static int load_editor_file(const char *path) {
    BOARD_CHECK(load_rom(path) == 0);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    BOARD_CHECK(cpu_power_on(&cpu));
    debugger_init();
    debugger_pause();
    return 0;
}

static int prg_save_roundtrip(unsigned mapper, const char *path, const char *save, uint8_t expected[0x4000]) {
    BOARD_CHECK(load_editor_file(path) == 0);
    bool changed;
    uint8_t first = 0xD2, last = 0x88, persisted[0x4000];
    BOARD_CHECK(edit(DEBUG_MEMORY_CPU, 0x6001, &first, 1, &changed) && changed);
    BOARD_CHECK(edit(DEBUG_MEMORY_CPU, 0x7FFF, &last, 1, &changed) && changed);
    expected[1] = first;
    expected[0x1FFF] = last;
    cart_battery_flush();
    BOARD_CHECK(read_editor_file(save, persisted, sizeof(persisted)));
    BOARD_CHECK(!memcmp(expected, persisted, sizeof(persisted)));
    if (mapper == 284) {
        write_mem(0x800A, 8);
        BOARD_CHECK(debugger_peek_cpu(0x7000) == 0xC1);
        uint8_t work_value = 0xE9;
        BOARD_CHECK(edit(DEBUG_MEMORY_CPU, 0x7000, &work_value, 1, &changed) && changed);
        cart_battery_flush();
        BOARD_CHECK(read_editor_file(save, persisted, sizeof(persisted)));
        BOARD_CHECK(!memcmp(expected, persisted, sizeof(persisted)));
    }
    BOARD_CHECK(unload_rom() && load_editor_file(path) == 0);
    BOARD_CHECK(debugger_peek_cpu(0x6001) == first && debugger_peek_cpu(0x7FFF) == last);
    BOARD_CHECK(debugger_peek_cpu(0x7000) == 0x44);
    return 0;
}

static void select_saved_chr(unsigned mapper) {
    if (mapper == 4) {
        write_mem(0x8000, 0);
        write_mem(0x8001, 8);
    } else write_mem(0x8000, 1);
}

static int chr_save_roundtrip(unsigned mapper, const char *path, const char *save, uint8_t expected[0x2000]) {
    BOARD_CHECK(load_editor_file(path) == 0);
    uint8_t original = debugger_peek_ppu(0x123), volatile_value = 0x2A, saved_value = 0xE5;
    bool changed;
    BOARD_CHECK(edit(DEBUG_MEMORY_PPU, 0x123, &volatile_value, 1, &changed) && changed);
    select_saved_chr(mapper);
    DebugMemoryByte byte;
    BOARD_CHECK(debug_memory_inspect(DEBUG_MEMORY_PPU, 0x123, &byte));
    BOARD_CHECK(byte.physical && byte.writable && byte.offset == 0x123 && !strcmp(byte.backing, "CHR NVRAM"));
    BOARD_CHECK(byte.value == 0xA6);
    BOARD_CHECK(edit(DEBUG_MEMORY_PPU, 0x123, &saved_value, 1, &changed) && changed);
    expected[0x123] = saved_value;
    cart_battery_flush();
    uint8_t persisted[0x2000];
    BOARD_CHECK(read_editor_file(save, persisted, sizeof(persisted)));
    BOARD_CHECK(!memcmp(expected, persisted, sizeof(persisted)));
    BOARD_CHECK(unload_rom() && load_editor_file(path) == 0);
    BOARD_CHECK(debugger_peek_ppu(0x123) == original);
    select_saved_chr(mapper);
    BOARD_CHECK(debugger_peek_ppu(0x123) == saved_value);
    return 0;
}

static int editor_battery_case(unsigned mapper, bool chr) {
    BoardImage image = {0};
    BOARD_CHECK(board_image_create(&image, mapper, 0x8000, chr ? 0 : 0x2000, true));
    image.data[6] |= 2;
    image.data[10] = chr ? 7 : 0x87;
    image.data[11] = chr ? 0x77 : 0;
    if (!chr && !board_image_add_trainer(&image, 0xC1)) {
        board_image_free(&image);
        BOARD_CHECK(false);
    }
    char path[160], save[168];
    snprintf(path, sizeof(path), "build/memory-editor-%u-%lu-%lu.nes", mapper,
             (unsigned long)time(NULL), (unsigned long)clock());
    strcpy(save, path);
    strcpy(strrchr(save, '.'), chr ? ".chr.sav" : ".sav");
    uint8_t expected[0x4000];
    memset(expected, chr ? 0xA6 : 0xA5, 0x2000);
    memset(expected + 0x2000, 0xB7, 0x2000);
    if (!chr) expected[0x1000] = 0x44;
    bool written = write_editor_file(path, image.data, image.size)
        && write_editor_file(save, expected, chr ? 0x2000 : sizeof(expected));
    board_image_free(&image);
    int failures = !written ? 1 : chr ? chr_save_roundtrip(mapper, path, save, expected)
                                    : prg_save_roundtrip(mapper, path, save, expected);
    if (!unload_rom()) ++failures;
    if (remove(path) != 0 || remove(save) != 0) ++failures;
    return failures;
}

static int observational_views(void) {
    const unsigned boards[] = {0, 1, 4, 5, 9, 70};
    for (size_t n = 0; n < sizeof(boards) / sizeof(*boards); ++n) {
        BOARD_CHECK(machine(boards[n], 0x8000, 7, 0, 0) == 0);
        BOARD_CHECK(joypad_set_adapter(NES_ADAPTER_NONE));
        BOARD_CHECK(joypad_set_port_device(0, NES_PORT_GAMEPAD));
        BOARD_CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
        pad1.strobe = 0;
        pad1.shift = 0x5A;
        ppu.status = 0xE0;
        ppu.w = 1;
        apu.frame_irq = apu.frame_irq_source = true;
        apu.frame_irq_clear_delay = 0;
        NesStateBlob before = {0};
        BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
        DebugMemoryPage page = {0};
        BOARD_CHECK(debug_memory_capture(&page, DEBUG_MEMORY_CPU, 0x2000, false));
        BOARD_CHECK(!page.bytes[0].writable && !page.bytes[0].readable && page.bytes[2].readable);
        BOARD_CHECK(debug_memory_capture(&page, DEBUG_MEMORY_CPU, 0x4000, false));
        BOARD_CHECK(debug_memory_capture(&page, DEBUG_MEMORY_CPU, 0xFF00, false));
        BOARD_CHECK(debug_memory_capture(&page, DEBUG_MEMORY_PPU, 0xFD0, false));
        uint8_t bytes[32];
        memset(bytes, 0xC4, sizeof(bytes));
        BOARD_CHECK(!debug_memory_copy(DEBUG_MEMORY_CPU, 0x1FFF, 2, bytes, sizeof(bytes), NULL, 0));
        BOARD_CHECK(bytes[0] == 0xC4 && bytes[1] == 0xC4);
        BOARD_CHECK(debug_memory_copy(DEBUG_MEMORY_CPU, 0x8000, sizeof(bytes), bytes, sizeof(bytes), NULL, 0));
        for (unsigned i = 0; i < sizeof(bytes); ++i) BOARD_CHECK(bytes[i] == debugger_peek_cpu((uint16_t)(0x8000 + i)));
        const uint8_t pattern[] = {0xCB, 0xDA, 0xEC};
        uint32_t found = 0xBEEF;
        BOARD_CHECK(!debug_memory_find(DEBUG_MEMORY_CPU, 0x4018, pattern, 3, false, &found, NULL, 0));
        BOARD_CHECK(found == 0xBEEF);
        BOARD_CHECK(compare_state(&before) == 0);
        nes_state_blob_free(&before);
    }
    BOARD_CHECK(machine(0, 0, 7, 7, 0) == 0);
    const uint8_t pattern[] = {0xC2, 0xD3, 0xE4};
    bool changed;
    BOARD_CHECK(edit(DEBUG_MEMORY_RAM, 0x10, pattern, 3, &changed));
    BOARD_CHECK(cheats_add("0010:FF", "Read override", true, NULL) == CHEAT_OK);
    DebugMemoryPage page = {0};
    BOARD_CHECK(debug_memory_capture(&page, DEBUG_MEMORY_RAM, 0, false));
    BOARD_CHECK(page.bytes[0x10].value == 0xC2 && !page.bytes[0x10].changed);
    uint32_t found;
    BOARD_CHECK(debug_memory_find(DEBUG_MEMORY_RAM, 0x11, pattern, 3, true, &found, NULL, 0) && found == 0x10);
    uint8_t newer = 0xF5;
    BOARD_CHECK(edit(DEBUG_MEMORY_RAM, 0x10, &newer, 1, &changed));
    BOARD_CHECK(debug_memory_capture(&page, DEBUG_MEMORY_RAM, 0, false) && page.bytes[0x10].changed);
    BOARD_CHECK(debug_memory_capture(&page, DEBUG_MEMORY_RAM, 0, true) && !page.bytes[0x10].changed);
    BOARD_CHECK(edit(DEBUG_MEMORY_RAM, 0x10, pattern, 1, &changed));
    debugger_invalidate_memory();
    BOARD_CHECK(debug_memory_capture(&page, DEBUG_MEMORY_RAM, 0, false) && !page.bytes[0x10].changed);
    BOARD_CHECK(cheats_clear() == CHEAT_OK);
    return 0;
}

int test_memory_editor_accuracy(void) {
    int failures = byte_sequences() + transactional_ram() + ppu_backing() + protection_and_banks() + observational_views();
    failures += expanded_cpu_ram() + selected_ppu_context();
    failures += custom_mapper_views() + unequal_ram_chips() + partial_ram_pages();
    failures += editor_battery_case(0, false) + editor_battery_case(284, false)
        + editor_battery_case(4, true) + editor_battery_case(70, true);
    ppu_set_oam_decay(false);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    debugger_shutdown();
    (void)cheats_clear();
    (void)unload_rom();
    printf("Memory editor: %s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures;
}
