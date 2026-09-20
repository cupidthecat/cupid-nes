/*
 * state_accuracy.c - Full-machine save-state regressions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../joypad/joypad.h"
#include "../ppu/ppu.h"
#include "../rom/fds.h"
#include "../rom/mapper.h"
#include "../rom/rom.h"
#include "../state/state.h"
#include "../state/state_alloc.h"
#include "../system/timing.h"
#include "../system/vs_system.h"
#include "../../include/globals.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

extern uint8_t ram[0x800];
extern uint64_t cpu_total_cycles;

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static bool blobs_equal(const NesStateBlob *a, const NesStateBlob *b) {
    return a && b && a->size == b->size
        && (!a->size || memcmp(a->data, b->data, a->size) == 0);
}

static uint32_t state_test_u32(const uint8_t *data) {
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8)
        | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void report_blob_difference(const char *label, const NesStateBlob *expected,
                                   const NesStateBlob *actual) {
    if (!expected || !actual) return;
    size_t limit = expected->size < actual->size ? expected->size : actual->size;
    size_t offset = limit >= 20 ? 20 : 0;
    while (offset < limit && expected->data[offset] == actual->data[offset]) ++offset;
    fprintf(stderr, "%s: state sizes %zu/%zu, first difference %zu",
            label, expected->size, actual->size, offset);
    if (offset < limit && expected->size >= 20 + 49 && actual->size >= 20 + 49) {
        size_t cursor = 20 + 45 + 4;
        while (cursor + 8 <= expected->size) {
            uint32_t tag = state_test_u32(expected->data + cursor);
            uint32_t length = state_test_u32(expected->data + cursor + 4);
            size_t start = cursor + 8;
            if (start > expected->size || length > expected->size - start) break;
            if (offset >= start && offset < start + length) {
                fprintf(stderr, " in chunk %c%c%c%c +%zu",
                        (char)tag, (char)(tag >> 8), (char)(tag >> 16), (char)(tag >> 24),
                        offset - start);
                break;
            }
            cursor = start + length;
        }
    }
    if (offset < limit)
        fprintf(stderr, " (%02X/%02X)", expected->data[offset], actual->data[offset]);
    fputc('\n', stderr);
}

static bool restore_ok(const char *label, const NesStateBlob *blob) {
    NesStateResult result = nes_state_restore(blob->data, blob->size);
    if (result != NES_STATE_OK)
        fprintf(stderr, "%s: restore result %d (%s)\n", label, result,
                nes_state_result_string(result));
    return result == NES_STATE_OK;
}

static uint8_t *make_ines(unsigned mapper, size_t prg_bytes, size_t chr_bytes,
                          bool chr_ram, size_t *image_size) {
    if (!image_size || !prg_bytes || (prg_bytes & 0x3FFFu)
        || (!chr_ram && (!chr_bytes || (chr_bytes & 0x1FFFu)))) return NULL;
    size_t payload = prg_bytes + (chr_ram ? 0 : chr_bytes);
    if (payload > SIZE_MAX - sizeof(iNESHeader)) return NULL;
    *image_size = sizeof(iNESHeader) + payload;
    uint8_t *image = (uint8_t *)calloc(1, *image_size);
    if (!image) return NULL;
    iNESHeader *header = (iNESHeader *)image;
    memcpy(header->signature, "NES\x1A", 4);
    header->prg_rom_chunks = (uint8_t)(prg_bytes / 0x4000u);
    header->chr_rom_chunks = chr_ram ? 0 : (uint8_t)(chr_bytes / 0x2000u);
    header->flags6 = (uint8_t)((mapper & 0x0Fu) << 4);
    header->flags7 = (uint8_t)(mapper & 0xF0u);
    uint8_t *prg = image + sizeof(*header);
    for (size_t i = 0; i < prg_bytes; ++i) prg[i] = (uint8_t)(i / 0x2000u);
    if (!chr_ram) {
        uint8_t *chr = prg + prg_bytes;
        for (size_t i = 0; i < chr_bytes; ++i) chr[i] = (uint8_t)(i / 0x0400u);
    }
    return image;
}

static void set_vectors(uint8_t *prg, size_t prg_bytes, uint16_t vector) {
    prg[prg_bytes - 6] = (uint8_t)vector;
    prg[prg_bytes - 5] = (uint8_t)(vector >> 8);
    prg[prg_bytes - 4] = (uint8_t)vector;
    prg[prg_bytes - 3] = (uint8_t)(vector >> 8);
    prg[prg_bytes - 2] = (uint8_t)vector;
    prg[prg_bytes - 1] = (uint8_t)(vector >> 8);
}

static bool power_machine(void) {
    cpu_use_default_startup_alignment();
    cpu_set_test_mode(false);
    memset(ram, 0, sizeof(ram));
    memset(&pad1, 0, sizeof(pad1));
    memset(&pad2, 0, sizeof(pad2));
    cpu_total_cycles = 0;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    if (!cpu_power_on(&cpu)) return false;
    ppu_begin_frame_render(framebuffer);
    return true;
}

static bool run_cpu_steps(unsigned steps) {
    for (unsigned i = 0; i < steps; ++i)
        if (cpu_step(&cpu) <= 0 || cpu.halted) return false;
    return true;
}

static uint8_t *make_replay_nrom(size_t *image_size, uint8_t marker) {
    uint8_t *image = make_ines(0, 0x8000, 0x2000, false, image_size);
    if (!image) return NULL;
    uint8_t *prg = image + sizeof(iNESHeader);
    static const uint8_t program[] = {
        0xE6,0x00,             /* INC $00 */
        0xA5,0x00,             /* LDA $00 */
        0x8D,0x00,0x20,        /* STA $2000 */
        0x8D,0x00,0x40,        /* STA $4000 */
        0x8D,0x16,0x40,        /* STA $4016 */
        0x4C,0x00,0x80         /* JMP $8000 */
    };
    memcpy(prg, program, sizeof(program));
    prg[0x100] = marker;
    set_vectors(prg, 0x8000, 0x8000);
    return image;
}

static int test_state_replay_and_failure_atomicity(void) {
    size_t image_size;
    uint8_t *image = make_replay_nrom(&image_size, 0x31);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    CHECK(power_machine());
    CHECK(joypad_set_player(0, BTN_A, true));
    CHECK(run_cpu_steps(90));

    NesStateBlob start = {0}, expected = {0}, actual = {0}, before_failure = {0};
    CHECK(nes_state_capture(&start) == NES_STATE_OK);
    CHECK(run_cpu_steps(160));
    CHECK(nes_state_capture(&expected) == NES_STATE_OK);
    CHECK(nes_state_restore(start.data, start.size) == NES_STATE_OK);
    CHECK(run_cpu_steps(160));
    CHECK(nes_state_capture(&actual) == NES_STATE_OK);
    CHECK(blobs_equal(&expected, &actual));

    CHECK(nes_state_capture(&before_failure) == NES_STATE_OK);
    nes_state_test_fail_allocation_after(0);
    CHECK(nes_state_restore(start.data, start.size) == NES_STATE_ERROR_OUT_OF_MEMORY);
    nes_state_test_fail_allocation_after(-1);
    NesStateBlob after_failure = {0};
    CHECK(nes_state_capture(&after_failure) == NES_STATE_OK);
    CHECK(blobs_equal(&before_failure, &after_failure));
    nes_state_blob_free(&after_failure);

    /* Mapper preparation consumes two deterministic allocations.  Failing the
       next one exercises a late Family BASIC prepare without mutating CPU/APU. */
    nes_state_test_fail_allocation_after(2);
    CHECK(nes_state_restore(start.data, start.size) == NES_STATE_ERROR_OUT_OF_MEMORY);
    nes_state_test_fail_allocation_after(-1);
    CHECK(nes_state_capture(&after_failure) == NES_STATE_OK);
    CHECK(blobs_equal(&before_failure, &after_failure));

    nes_state_blob_free(&after_failure);
    nes_state_blob_free(&before_failure);
    nes_state_blob_free(&actual);
    nes_state_blob_free(&expected);
    nes_state_blob_free(&start);
    CHECK(unload_rom());
    return 0;
}

static int test_state_format_identity_files_and_slots(void) {
    size_t image_size;
    uint8_t *image = make_replay_nrom(&image_size, 0x41);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    CHECK(power_machine() && run_cpu_steps(40));
    NesStateBlob saved = {0}, before = {0}, after = {0};
    CHECK(nes_state_capture(&saved) == NES_STATE_OK);

    uint8_t *broken = (uint8_t *)malloc(saved.size);
    CHECK(broken != NULL);
    memcpy(broken, saved.data, saved.size);
    broken[saved.size - 1] ^= 0x80;
    CHECK(nes_state_restore(broken, saved.size) == NES_STATE_ERROR_CORRUPT);
    memcpy(broken, saved.data, saved.size);
    broken[8] ^= 1;
    CHECK(nes_state_restore(broken, saved.size) == NES_STATE_ERROR_VERSION);
    CHECK(nes_state_restore(saved.data, saved.size - 1) == NES_STATE_ERROR_CORRUPT);
    free(broken);

    CHECK(nes_state_capture(&before) == NES_STATE_OK);
    image[sizeof(iNESHeader) + 0x100] ^= 0x7F;
    CHECK(load_rom_memory(image, image_size) == 0 && power_machine());
    CHECK(nes_state_capture(&after) == NES_STATE_OK);
    CHECK(nes_state_restore(saved.data, saved.size) == NES_STATE_ERROR_INCOMPATIBLE);
    NesStateBlob incompatible_after = {0};
    CHECK(nes_state_capture(&incompatible_after) == NES_STATE_OK);
    CHECK(blobs_equal(&after, &incompatible_after));
    nes_state_blob_free(&incompatible_after);
    nes_state_blob_free(&after);
    nes_state_blob_free(&before);
    free(image);

    image = make_replay_nrom(&image_size, 0x41);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    CHECK(power_machine() && run_cpu_steps(40));
    NesStateBlob file_expected = {0};
    CHECK(nes_state_capture(&file_expected) == NES_STATE_OK);
    char directory[96];
    snprintf(directory, sizeof(directory), "build/state-accuracy-%lu", (unsigned long)time(NULL));
#ifdef _WIN32
    CHECK(_mkdir(directory) == 0);
#else
    CHECK(mkdir(directory, 0700) == 0);
#endif
    char file_path[128];
    snprintf(file_path, sizeof(file_path), "%s/manual.cstate", directory);
    CHECK(nes_state_save_file(file_path) == NES_STATE_OK);
    CHECK(nes_state_save_slot(directory, 7) == NES_STATE_OK);
    CHECK(run_cpu_steps(70));
    CHECK(nes_state_load_file(file_path) == NES_STATE_OK);
    NesStateBlob loaded = {0};
    CHECK(nes_state_capture(&loaded) == NES_STATE_OK && blobs_equal(&file_expected, &loaded));
    nes_state_blob_free(&loaded);
    CHECK(run_cpu_steps(30));
    CHECK(nes_state_load_slot(directory, 7) == NES_STATE_OK);
    CHECK(nes_state_capture(&loaded) == NES_STATE_OK && blobs_equal(&file_expected, &loaded));
    nes_state_blob_free(&loaded);
    nes_state_blob_free(&file_expected);
    remove(file_path);
    snprintf(file_path, sizeof(file_path), "%s/slot-7.cstate", directory);
    remove(file_path);
#ifdef _WIN32
    _rmdir(directory);
#else
    rmdir(directory);
#endif
    nes_state_blob_free(&saved);
    CHECK(unload_rom());
    return 0;
}

static void mmc3_a12_pulse(uint64_t cycle) {
    cart_notify_ppu_address(0x2000, cycle);
    cart_notify_ppu_address(0x1000, cycle + 9);
}

static int test_state_native_mapper_irq_and_audio(void) {
    size_t image_size;
    uint8_t *image = make_ines(4, 0x20000, 0x2000, false, &image_size);
    CHECK(image != NULL);
    ((iNESHeader *)image)->flags6 |= 0x08;
    set_vectors(image + sizeof(iNESHeader), 0x20000, 0xE000);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    CHECK(power_machine());
    cart_cpu_write(0x8000, 6);
    cart_cpu_write(0x8001, 3);
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 5);
    cart_cpu_write(0xA001, 0x80);
    cart_cpu_write(0x6000, 0xAB);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    mmc3_a12_pulse(30);
    CHECK(cart_irq_pending());
    NesStateBlob mapper_state = {0};
    CHECK(nes_state_capture(&mapper_state) == NES_STATE_OK);
    cart_cpu_write(0x8000, 6);
    cart_cpu_write(0x8001, 9);
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0x6000, 0x55);
    CHECK(!cart_irq_pending());
    CHECK(nes_state_restore(mapper_state.data, mapper_state.size) == NES_STATE_OK);
    CHECK(cart_irq_pending() && cart_cpu_read(0x6000) == 0xAB);
    CHECK(cart_ppu_read(0) == 4 && cart_ppu_read(0x0400) == 5);
    nes_state_blob_free(&mapper_state);
    CHECK(unload_rom());

    image = make_ines(85, 0x40000, 0x2000, false, &image_size);
    CHECK(image != NULL);
    uint8_t *prg = image + sizeof(iNESHeader);
    prg[0x3E000] = 0x4C; prg[0x3E001] = 0x00; prg[0x3E002] = 0xE0;
    set_vectors(prg, 0x40000, 0xE000);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    CHECK(power_machine());
    cart_cpu_write(0x9010, 0x30);
    cart_cpu_write(0x9030, 0x10);
    for (unsigned i = 0; i < 600; ++i) cart_clock_cpu_cycle(false);
    NesStateBlob fm_state = {0}, fm_restored = {0};
    CHECK(nes_state_capture(&fm_state) == NES_STATE_OK);
    cart_cpu_write(0x9010, 0x30);
    cart_cpu_write(0x9030, 0x00);
    for (unsigned i = 0; i < 100; ++i) cart_clock_cpu_cycle(false);
    CHECK(restore_ok("VRC7", &fm_state));
    CHECK(nes_state_capture(&fm_restored) == NES_STATE_OK);
    if (!blobs_equal(&fm_state, &fm_restored))
        report_blob_difference("VRC7", &fm_state, &fm_restored);
    CHECK(blobs_equal(&fm_state, &fm_restored));
    nes_state_blob_free(&fm_restored);
    nes_state_blob_free(&fm_state);
    CHECK(unload_rom());
    return 0;
}

static int test_state_board_restore(void) {
    size_t image_size;
    uint8_t *image = make_ines(77, 0x10000, 0x4000, false, &image_size);
    CHECK(image != NULL);
    uint8_t *prg = image + sizeof(iNESHeader);
    prg[0x100] = 0xFF;
    set_vectors(prg, 0x10000, 0xC000);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    CHECK(power_machine());
    cart_cpu_write(0x8100, 0x21);
    cart_cpu_write(0x6123, 0xA6);
    NesStateBlob saved = {0}, restored = {0};
    CHECK(nes_state_capture(&saved) == NES_STATE_OK);
    cart_cpu_write(0x8100, 0x00);
    cart_cpu_write(0x6123, 0x11);
    CHECK(restore_ok("board77", &saved));
    CHECK(nes_state_capture(&restored) == NES_STATE_OK);
    if (!blobs_equal(&saved, &restored))
        report_blob_difference("board77", &saved, &restored);
    CHECK(blobs_equal(&saved, &restored));
    CHECK(cart_cpu_read(0x6123) == 0xA6);
    nes_state_blob_free(&restored);
    nes_state_blob_free(&saved);
    CHECK(unload_rom());
    return 0;
}

static void put16(uint8_t *data, uint16_t value) {
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static size_t make_nsf(uint8_t *image, size_t capacity) {
    enum { PAYLOAD = 0x100 };
    if (!image || capacity < 0x80 + PAYLOAD) return 0;
    memset(image, 0, 0x80 + PAYLOAD);
    memcpy(image, "NESM\x1A", 5);
    image[5] = 1;
    image[6] = 2;
    image[7] = 1;
    put16(image + 8, 0x8000);
    put16(image + 10, 0x8000);
    put16(image + 12, 0x8010);
    put16(image + 110, 1000);
    put16(image + 120, 1000);
    image[123] = NSF_SOUND_VRC6;
    uint8_t *payload = image + 0x80;
    memset(payload, 0xEA, PAYLOAD);
    payload[0] = 0x60;
    payload[0x10] = 0x60;
    return 0x80 + PAYLOAD;
}

static int test_state_nsf_expansion_replay(void) {
    uint8_t image[0x180];
    size_t size = make_nsf(image, sizeof(image));
    CHECK(size && load_rom_memory(image, size) == 0);
    CHECK(power_machine());
    cart_cpu_write(0x9000, 0x1F);
    cart_cpu_write(0x9001, 15);
    cart_cpu_write(0x9002, 0x80);
    CHECK(run_cpu_steps(60));
    NesStateBlob start = {0}, expected = {0}, actual = {0};
    CHECK(nes_state_capture(&start) == NES_STATE_OK);
    CHECK(run_cpu_steps(180));
    CHECK(nes_state_capture(&expected) == NES_STATE_OK);
    CHECK(restore_ok("NSF", &start));
    CHECK(run_cpu_steps(180));
    CHECK(nes_state_capture(&actual) == NES_STATE_OK && blobs_equal(&expected, &actual));
    nes_state_blob_free(&actual);
    nes_state_blob_free(&expected);
    nes_state_blob_free(&start);
    CHECK(unload_rom());
    return 0;
}

enum { STATE_FDS_SIDE_SIZE = 65500 };

static void make_fds_bios(uint8_t bios[0x2000]) {
    memset(bios, 0xEA, 0x2000);
    bios[0x1FFA] = 0; bios[0x1FFB] = 0xE0;
    bios[0x1FFC] = 0; bios[0x1FFD] = 0xE0;
    bios[0x1FFE] = 0; bios[0x1FFF] = 0xE0;
}

static uint8_t *make_fds_disk(size_t sides, size_t *size) {
    *size = 16 + sides * STATE_FDS_SIDE_SIZE;
    uint8_t *disk = (uint8_t *)calloc(1, *size);
    if (!disk) return NULL;
    memcpy(disk, "FDS\x1A", 4);
    disk[4] = (uint8_t)sides;
    for (size_t side = 0; side < sides; ++side) {
        uint8_t *raw = disk + 16 + side * STATE_FDS_SIDE_SIZE;
        raw[0] = 1;
        raw[1] = 0x2A;
        for (unsigned i = 0; i < 10; ++i) raw[14 + i] = (uint8_t)(side * 16 + i);
        raw[55] = (uint8_t)(0x40 + side);
        raw[56] = 2;
    }
    return disk;
}

static bool fds_wait_transfer(void) {
    for (unsigned cycle = 0; cycle < 700000; ++cycle) {
        if (fds_irq_pending()) return true;
        fds_clock_cpu(1);
    }
    return false;
}

static bool fds_dirty_header(uint8_t value) {
    if (!fds_insert_disk(0)) return false;
    cart_cpu_write(0x4025, 0xC5);
    if (!fds_wait_transfer() || cart_cpu_read_bus(0x4031, 0) != 1) return false;
    cart_cpu_write(0x4024, value);
    cart_cpu_write(0x4025, 0x41);
    fds_clock_cpu(150);
    return fds_disk_dirty();
}

static int test_state_fds_media_and_automation(void) {
    uint8_t bios[0x2000];
    make_fds_bios(bios);
    size_t disk_size;
    uint8_t *disk = make_fds_disk(2, &disk_size);
    CHECK(disk != NULL);
    const char *path = "build/state-accuracy-fds.fds";
    remove(path);
    CHECK(nes_set_region_mode(NES_REGION_MODE_AUTO));
    CHECK(load_fds_memory(disk, disk_size, bios, sizeof(bios), path, false) == 0);
    free(disk);
    CHECK(power_machine());
    FdsAutomationOptions automation = {true, true};
    fds_set_automation_options(automation);
    CHECK(fds_insert_disk(1));
    cart_cpu_write(0x6000, 0xA5);
    fds_automation_frame(10);
    NesStateBlob clean = {0}, restored = {0};
    CHECK(nes_state_capture(&clean) == NES_STATE_OK);

    fds_eject_disk();
    cart_cpu_write(0x6000, 0x11);
    fds_automation_frame(100);
    fds_set_write_protected(true);
    CHECK(restore_ok("FDS clean", &clean));
    CHECK(fds_disk_inserted() && fds_current_side() == 1);
    CHECK(cart_cpu_read(0x6000) == 0xA5);
    CHECK(fds_write_protected()); /* Host write permission is not snapshot state. */
    CHECK(nes_state_capture(&restored) == NES_STATE_OK && blobs_equal(&clean, &restored));
    nes_state_blob_free(&restored);

    fds_set_write_protected(false);
    CHECK(fds_dirty_header(0xA5));
    CHECK(fds_disk_dirty());
    CHECK(restore_ok("FDS dirty", &clean));
    CHECK(fds_disk_dirty()); /* Loading a clean snapshot cannot clear unsaved media. */
    CHECK(fds_flush() && !fds_disk_dirty());
    nes_state_blob_free(&clean);
    CHECK(unload_rom());
    remove(path);
    return 0;
}

static uint8_t *make_dual_vs(size_t *image_size) {
    iNESHeader header = {0};
    memcpy(header.signature, "NES\x1A", 4);
    header.prg_rom_chunks = 4;
    header.chr_rom_chunks = 4;
    header.flags6 = 0x30;
    header.flags7 = 0x69;
    header.zero[2] = (uint8_t)((VS_TYPE_DUAL << 4) | 1);
    header.zero[4] = VS_INPUT_STANDARD;
    *image_size = sizeof(header) + 0x10000 + 0x8000;
    uint8_t *image = (uint8_t *)calloc(1, *image_size);
    if (!image) return NULL;
    memcpy(image, &header, sizeof(header));
    uint8_t *prg = image + sizeof(header);
    static const uint8_t program[] = {0xE6,0x00,0x4C,0x00,0x80};
    for (size_t side = 0; side < 2; ++side) {
        size_t base = side * 0x8000;
        memcpy(prg + base, program, sizeof(program));
        set_vectors(prg + base, 0x8000, 0x8000);
    }
    return image;
}

static int test_state_dual_vs_replay(void) {
    size_t image_size;
    uint8_t *image = make_dual_vs(&image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    CHECK(vs_dual_system());
    CHECK(power_machine());
    vs_power_on_secondary();
    for (unsigned i = 0; i < 80; ++i) CHECK(vs_cpu_step() > 0);
    CHECK(vs_active_side() == 0);
    NesStateBlob start = {0}, expected = {0}, actual = {0};
    CHECK(nes_state_capture(&start) == NES_STATE_OK);
    for (unsigned i = 0; i < 160; ++i) CHECK(vs_cpu_step() > 0);
    CHECK(nes_state_capture(&expected) == NES_STATE_OK);
    CHECK(nes_state_restore(start.data, start.size) == NES_STATE_OK);
    for (unsigned i = 0; i < 160; ++i) CHECK(vs_cpu_step() > 0);
    CHECK(nes_state_capture(&actual) == NES_STATE_OK && blobs_equal(&expected, &actual));
    nes_state_blob_free(&actual);
    nes_state_blob_free(&expected);
    nes_state_blob_free(&start);
    CHECK(unload_rom());
    return 0;
}

int test_state_accuracy(void) {
    static int (*const tests[])(void) = {
        test_state_replay_and_failure_atomicity,
        test_state_format_identity_files_and_slots,
        test_state_native_mapper_irq_and_audio,
        test_state_board_restore,
        test_state_nsf_expansion_replay,
        test_state_fds_media_and_automation,
        test_state_dual_vs_replay
    };
    int failures = 0;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) failures += tests[i]();
    nes_state_test_fail_allocation_after(-1);
    unload_rom();
    printf("Save states: %zu groups, %d failures\n", sizeof(tests) / sizeof(tests[0]), failures);
    return failures;
}
