/*
 * persistence_accuracy.c - Save failures retain the running cartridge
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License, version 3 or later.
 * This program is distributed without any warranty.
 */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../util/file_io.h"
#ifdef _WIN32
#include <direct.h>
#include <process.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

static int make_directory(const char *path) {
#ifdef _WIN32
    return _mkdir(path);
#else
    return mkdir(path, 0700);
#endif
}

static int remove_directory(const char *path) {
#ifdef _WIN32
    return _rmdir(path);
#else
    return rmdir(path);
#endif
}

static bool store_ram(uint16_t address, uint8_t value) {
    write_mem(0x0200, 0xa9);
    write_mem(0x0201, value);
    write_mem(0x0202, 0x8d);
    write_mem(0x0203, (uint8_t)address);
    write_mem(0x0204, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4;
}

static int test_failed_save_retains_image(const char *directory, unsigned mapper) {
    char rom_path[256], save_path[256], child[280];
    BOARD_CHECK(snprintf(rom_path, sizeof(rom_path), "%s/cart-%u.nes", directory, mapper) > 0);
    BOARD_CHECK(snprintf(save_path, sizeof(save_path), "%s/cart-%u.sav", directory, mapper) > 0);
    BOARD_CHECK(snprintf(child, sizeof(child), "%s/retained.bin", save_path) > 0);

    BoardImage original, replacement;
    BOARD_CHECK(board_image_create(&original, mapper, 0x8000, 0x2000, true));
    original.data[6] |= 2;
    original.data[10] = 0x70; /* 8 KiB battery RAM without work RAM. */
    BOARD_CHECK(board_image_create(&replacement, 0, 0x4000, 0x2000, true));
    BOARD_CHECK(nes_file_write_atomic(rom_path, original.data, original.size) == NES_FILE_OK);
    BOARD_CHECK(load_rom(rom_path) == 0);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    BOARD_CHECK(cpu_power_on(&cpu));
    BOARD_CHECK(store_ram(0x6007, 0x91));
    BOARD_CHECK(read_mem(0x6007) == 0x91);

    /* A nonempty directory at the save destination makes the final atomic
     * replacement fail even when creating and writing its sibling succeeds. */
    BOARD_CHECK(make_directory(save_path) == 0);
    BOARD_CHECK(nes_file_write_atomic(child, "unchanged", 9) == NES_FILE_OK);
    uint8_t *previous_prg = prg_rom;
    uint32_t previous_crc = rom_file_crc32();
    uint16_t previous_pc = cpu.pc;
    BOARD_CHECK(load_rom_memory(replacement.data, replacement.size) == -1);
    BOARD_CHECK(prg_rom == previous_prg && rom_file_crc32() == previous_crc);
    BOARD_CHECK(cpu.pc == previous_pc && read_mem(0x6007) == 0x91);
    BOARD_CHECK(!unload_rom());
    BOARD_CHECK(prg_rom == previous_prg && read_mem(0x6007) == 0x91);
    uint8_t *retained = NULL;
    size_t retained_size = 0;
    BOARD_CHECK(nes_file_read_all(child, 9, &retained, &retained_size) == NES_FILE_OK);
    BOARD_CHECK(retained_size == 9 && !memcmp(retained, "unchanged", 9));
    free(retained);

    BOARD_CHECK(nes_file_remove(child) == NES_FILE_OK);
    BOARD_CHECK(remove_directory(save_path) == 0);
    BOARD_CHECK(load_rom_memory(replacement.data, replacement.size) == 0);
    BOARD_CHECK(load_rom(rom_path) == 0);
    BOARD_CHECK(read_mem(0x6007) == 0x91);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(nes_file_remove(save_path) == NES_FILE_OK);
    BOARD_CHECK(nes_file_remove(rom_path) == NES_FILE_OK);
    board_image_free(&original);
    board_image_free(&replacement);
    return 0;
}

int test_persistence_accuracy(void) {
    char directory[128];
#ifdef _WIN32
    unsigned process = (unsigned)_getpid();
#else
    unsigned process = (unsigned)getpid();
#endif
    BOARD_CHECK(snprintf(directory, sizeof(directory), "build/persistence-%u", process) > 0);
    BOARD_CHECK(make_directory(directory) == 0);
    int failures = test_failed_save_retains_image(directory, 0)
                 + test_failed_save_retains_image(directory, 31);
    if (remove_directory(directory) != 0) ++failures;
    printf("Persistence: 2 groups, %d failures\n", failures);
    return failures;
}
