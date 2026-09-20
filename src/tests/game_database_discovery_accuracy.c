/*
 * game_database_discovery_accuracy.c - Default database loading and retention
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#include "board_tests.h"
#include "../ui/game_database.h"
#include "../rom/game_db.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include "../joypad/joypad.h"
#ifdef _WIN32
#include <direct.h>
#include <process.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

static int database_row(char *row, size_t capacity, const BoardImage *image, const char *system) {
    uint32_t crc = game_db_crc32(image->data + 16, image->size - 16);
    int length = snprintf(row, capacity,
        "%08X,%s,TEST,,,0,b32768,b8192,0,8,0,0,h,8,N,,0,0\n", (unsigned)crc, system);
    return length > 0 && (size_t)length < capacity ? length : -1;
}

static int test_default_database_paths(const char *directory, const BoardImage *image) {
    FrontendDatabaseStatus status;
    BOARD_CHECK(frontend_database_load(NULL, directory, &status));
    BOARD_CHECK(status.outcome == FRONTEND_DATABASE_ABSENT && !status.explicit_path);
    BOARD_CHECK(status.entry_count == 0 && status.file_result == NES_FILE_NOT_FOUND);
    char expected_path[256];
    BOARD_CHECK(snprintf(expected_path, sizeof(expected_path), "%s/NesDB.txt", directory) > 0);
    BOARD_CHECK(strcmp(status.path, expected_path) == 0);

    char row[512];
    int length = database_row(row, sizeof(row), image, "NesPal");
    BOARD_CHECK(length > 0 && nes_file_write_atomic(expected_path, row, (size_t)length) == NES_FILE_OK);
    BOARD_CHECK(frontend_database_load(NULL, directory, &status));
    BOARD_CHECK(status.outcome == FRONTEND_DATABASE_LOADED && status.entry_count == 1);
    BOARD_CHECK(strcmp(status.path, expected_path) == 0 && !status.explicit_path);
    BOARD_CHECK(board_image_load(image) == 0 && rom_metadata_source() == ROM_METADATA_DATABASE);
    BOARD_CHECK(nes_timing()->region == NES_REGION_PAL && joypad_port_device(1) == NES_PORT_ZAPPER);
    uint32_t crc = game_db_crc32(image->data + 16, image->size - 16);
    BOARD_CHECK(rom_prg_chr_crc32() == crc);
    BOARD_CHECK(rom_file_crc32() == game_db_crc32(image->data, image->size));

    char explicit_path[256];
    BOARD_CHECK(snprintf(explicit_path, sizeof(explicit_path), "%s/custom-\xC3\xA9.txt", directory) > 0);
    length = database_row(row, sizeof(row), image, "NesNtsc");
    BOARD_CHECK(length > 0 && nes_file_write_atomic(explicit_path, row, (size_t)length) == NES_FILE_OK);
    uint8_t *active_prg = prg_rom;
    uint64_t cycles = cpu_total_cycles;
    BOARD_CHECK(frontend_database_load(explicit_path, directory, &status));
    BOARD_CHECK(status.explicit_path && status.outcome == FRONTEND_DATABASE_LOADED);
    BOARD_CHECK(strcmp(status.path, explicit_path) == 0 && status.entry_count == 1);
    BOARD_CHECK(prg_rom == active_prg && cpu_total_cycles == cycles && nes_timing()->region == NES_REGION_PAL);
    BOARD_CHECK(board_image_load(image) == 0 && nes_timing()->region == NES_REGION_NTSC);

    // A malformed selected file cannot fall back to the valid default silently.
    BOARD_CHECK(nes_file_write_atomic(explicit_path, "invalid,row\n", 12) == NES_FILE_OK);
    active_prg = prg_rom;
    cycles = cpu_total_cycles;
    write_mem(6, 0xA5);
    BOARD_CHECK(!frontend_database_load(explicit_path, directory, &status));
    BOARD_CHECK(status.explicit_path && status.outcome == FRONTEND_DATABASE_ERROR);
    BOARD_CHECK(status.entry_count == 1 && status.message[0]);
    GameDbEntry entry;
    BOARD_CHECK(game_db_lookup(crc, &entry) && strcmp(entry.system, "NesNtsc") == 0);
    BOARD_CHECK(prg_rom == active_prg && cpu_total_cycles == cycles && read_mem(6) == 0xA5);
    BOARD_CHECK(nes_timing()->region == NES_REGION_NTSC);
    BOARD_CHECK(nes_file_remove(explicit_path) == NES_FILE_OK);
    BOARD_CHECK(!frontend_database_load(explicit_path, directory, &status));
    BOARD_CHECK(status.file_result == NES_FILE_NOT_FOUND && status.entry_count == 1);

    BOARD_CHECK(frontend_database_load(NULL, directory, &status));
    BOARD_CHECK(game_db_lookup(crc, &entry) && strcmp(entry.system, "NesPal") == 0);
    BOARD_CHECK(load_rom_memory(image->data, image->size - 1) < 0);
    BOARD_CHECK(prg_rom == active_prg && cpu_total_cycles == cycles && read_mem(6) == 0xA5);
    BOARD_CHECK(nes_timing()->region == NES_REGION_NTSC);

    BOARD_CHECK(nes_file_write_atomic(expected_path, "bad", 3) == NES_FILE_OK);
    BOARD_CHECK(!frontend_database_load(NULL, directory, &status));
    BOARD_CHECK(!status.explicit_path && status.outcome == FRONTEND_DATABASE_ERROR);
    BOARD_CHECK(game_db_lookup(crc, &entry) && strcmp(entry.system, "NesPal") == 0);
    BOARD_CHECK(prg_rom == active_prg && read_mem(6) == 0xA5);
    BOARD_CHECK(nes_file_remove(expected_path) == NES_FILE_OK);
    BOARD_CHECK(frontend_database_load(NULL, directory, &status));
    BOARD_CHECK(status.outcome == FRONTEND_DATABASE_ABSENT && status.entry_count == 1);
    return 0;
}

static int test_default_database_metadata(const char *directory, BoardImage *image) {
    char path[256], row[512];
    BOARD_CHECK(snprintf(path, sizeof(path), "%s/NesDB.txt", directory) > 0);
    int length = database_row(row, sizeof(row), image, "NesPal");
    BOARD_CHECK(length > 0 && nes_file_write_atomic(path, row, (size_t)length) == NES_FILE_OK);
    FrontendDatabaseStatus status;
    BOARD_CHECK(frontend_database_load(NULL, directory, &status));

    joypad_set_configuration_overrides(NES_INPUT_OVERRIDE_PORT2);
    BOARD_CHECK(joypad_set_port_device(1, NES_PORT_GAMEPAD));
    BOARD_CHECK(board_image_load(image) == 0 && nes_timing()->region == NES_REGION_PAL);
    BOARD_CHECK(joypad_port_device(1) == NES_PORT_GAMEPAD);
    joypad_set_configuration_overrides(0);

    rom_database_set_overrides(false);
    BOARD_CHECK(board_image_load(image) == 0 && rom_metadata_source() == ROM_METADATA_INES);
    BOARD_CHECK(nes_timing()->region == NES_REGION_NTSC && joypad_port_device(1) == NES_PORT_GAMEPAD);
    BOARD_CHECK(load_rom_memory(image->data + 16, image->size - 16) == 0);
    BOARD_CHECK(rom_metadata_source() == ROM_METADATA_DATABASE_HEADERLESS);
    BOARD_CHECK(nes_timing()->region == NES_REGION_PAL && joypad_port_device(1) == NES_PORT_ZAPPER);

    rom_database_set_overrides(true);
    image->data[7] = 0x08;
    image->data[12] = 0;
    image->data[15] = 0x01;
    BOARD_CHECK(board_image_load(image) == 0 && rom_metadata_source() == ROM_METADATA_NES20);
    BOARD_CHECK(nes_timing()->region == NES_REGION_NTSC && joypad_port_device(1) == NES_PORT_GAMEPAD);
    image->data[7] = image->data[12] = image->data[15] = 0;
    BOARD_CHECK(board_image_load(image) == 0 && rom_metadata_source() == ROM_METADATA_DATABASE);
    BOARD_CHECK(nes_timing()->region == NES_REGION_PAL && joypad_port_device(1) == NES_PORT_ZAPPER);
    BOARD_CHECK(nes_file_remove(path) == NES_FILE_OK);
    return 0;
}

int test_game_database_discovery_accuracy(void) {
    char directory[128];
#ifdef _WIN32
    unsigned process = (unsigned)_getpid();
#else
    unsigned process = (unsigned)getpid();
#endif
    BOARD_CHECK(snprintf(directory, sizeof(directory), "build/database-discovery-%u", process) > 0);
#ifdef _WIN32
    BOARD_CHECK(_mkdir(directory) == 0);
#else
    BOARD_CHECK(mkdir(directory, 0700) == 0);
#endif
    NesRegionMode region = nes_region_mode();
    NesConsoleModel console = nes_console_model();
    uint8_t overrides = joypad_configuration_overrides();
    NesPortDevice port2 = joypad_port_device(1);
    NesExpansionDevice expansion = joypad_expansion_device();
    rom_database_clear();
    rom_database_set_overrides(true);
    BOARD_CHECK(nes_set_region_mode(NES_REGION_MODE_AUTO));
    BOARD_CHECK(nes_set_console_model(NES_CONSOLE_NES001));
    joypad_set_configuration_overrides(0);
    BOARD_CHECK(joypad_set_port_device(1, NES_PORT_GAMEPAD));
    BOARD_CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0x2000, false));
    int failures = test_default_database_paths(directory, &image);
    failures += test_default_database_metadata(directory, &image);
    board_image_free(&image);
    BOARD_CHECK(unload_rom());
    rom_database_clear();
    rom_database_set_overrides(true);
    BOARD_CHECK(nes_set_region_mode(region) && nes_set_console_model(console));
    joypad_set_configuration_overrides(0);
    BOARD_CHECK(joypad_set_port_device(1, port2) && joypad_set_expansion_device(expansion));
    joypad_set_configuration_overrides(overrides);
#ifdef _WIN32
    if (_rmdir(directory) != 0) ++failures;
#else
    if (rmdir(directory) != 0) ++failures;
#endif
    printf("Game database discovery: 2 groups, %d failures\n", failures);
    return failures;
}
