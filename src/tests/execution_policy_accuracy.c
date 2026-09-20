/*
 * execution_policy_accuracy.c - Isolated modes retain pending persistent data
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License, version 3 or later.
 * This program is distributed without any warranty.
 */
#include "board_tests.h"
#include "../joypad/family_basic.h"
#include "../joypad/special_peripherals.h"
#include "../system/execution_policy.h"
#include "../system/hardware.h"
#include "../util/file_io.h"
#ifdef _WIN32
#include <direct.h>
#include <process.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

static const uint32_t isolated_modes[] = {
    NES_EXECUTION_MOVIE_PLAYBACK, NES_EXECUTION_NETPLAY,
    NES_EXECUTION_SPECULATIVE, NES_EXECUTION_REWIND,
    NES_EXECUTION_MOVIE_RECORDING | NES_EXECUTION_SPECULATIVE
};

static int read_saved_byte(const char *path, size_t offset) {
    FILE *file = nes_file_open(path, "rb");
    if (!file) return -1;
    int byte = fseek(file, (long)offset, SEEK_SET) == 0 ? fgetc(file) : -1;
    if (fclose(file) != 0) return -1;
    return byte;
}

static int policy_cart(const char *directory, unsigned mapper) {
    char rom_path[256], save_path[256];
    snprintf(rom_path, sizeof(rom_path), "%s/cart-%u.nes", directory, mapper);
    snprintf(save_path, sizeof(save_path), "%s/cart-%u.sav", directory, mapper);
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, mapper, 0x8000, 0x2000, true));
    image.data[6] |= 2;
    image.data[10] = 0x70;
    BOARD_CHECK(nes_file_write_atomic(rom_path, image.data, image.size) == NES_FILE_OK);
    BOARD_CHECK(load_rom(rom_path) == 0);
    cart_cpu_write(0x6009, 0xA1);
    BOARD_CHECK(cart_battery_flush());
    BOARD_CHECK(read_saved_byte(save_path, 9) == 0xA1);
    uint8_t previous = 0xA1;
    for (size_t i = 0; i < sizeof(isolated_modes) / sizeof(isolated_modes[0]); i++) {
        uint8_t next = (uint8_t)(0xB0 + i);
        // The byte is already dirty when a nested execution mode starts.
        cart_cpu_write(0x6009, next);
        BOARD_CHECK(nes_execution_set_policy(isolated_modes[i]));
        BOARD_CHECK(!nes_execution_allows_persistence());
        BOARD_CHECK(cart_battery_flush());
        BOARD_CHECK(cart_cpu_read(0x6009) == next);
        BOARD_CHECK(read_saved_byte(save_path, 9) == previous);
        BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
        BOARD_CHECK(cart_battery_flush());
        BOARD_CHECK(read_saved_byte(save_path, 9) == next);
        previous = next;
    }

    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_MOVIE_RECORDING));
    cart_cpu_write(0x6009, 0xE2);
    BOARD_CHECK(nes_execution_allows_persistence() && cart_battery_flush());
    BOARD_CHECK(read_saved_byte(save_path, 9) == 0xE2);
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(nes_file_remove(save_path) == NES_FILE_OK);
    BOARD_CHECK(nes_file_remove(rom_path) == NES_FILE_OK);
    board_image_free(&image);
    return 0;
}

static void set_turbo_byte(uint8_t value) {
    turbo_file_reset_protocol();
    turbo_file_write(0);
    for (unsigned bit = 0; bit < 8; bit++) {
        uint8_t wire = (uint8_t)((value >> bit) & 1u);
        turbo_file_write((uint8_t)(0x06 | wire));
        turbo_file_write((uint8_t)(0x02 | wire));
    }
}

static int policy_turbo(const char *directory) {
    char identity[256], save_path[256];
    snprintf(identity, sizeof(identity), "%s/turbo.nes", directory);
    snprintf(save_path, sizeof(save_path), "%s/turbo.turbofile.sav", directory);
    BOARD_CHECK(turbo_file_configure(identity));
    set_turbo_byte(0x6D);
    BOARD_CHECK(turbo_file_flush());
    uint8_t previous = 0x6D;
    for (size_t i = 0; i < sizeof(isolated_modes) / sizeof(isolated_modes[0]); i++) {
        uint8_t next = (uint8_t)(0xA0 + i);
        set_turbo_byte(next);
        BOARD_CHECK(nes_execution_set_policy(isolated_modes[i]));
        BOARD_CHECK(turbo_file_flush() && read_saved_byte(save_path, 0) == previous);
        BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
        BOARD_CHECK(turbo_file_flush() && read_saved_byte(save_path, 0) == next);
        previous = next;
    }

    BOARD_CHECK(turbo_file_shutdown());
    BOARD_CHECK(nes_file_remove(save_path) == NES_FILE_OK);
    return 0;
}

static void battle_send_value(uint16_t value) {
    for (unsigned bit = 0; bit < 16; bit++) {
        unsigned output = (battle_box_read(1) >> 4) & 1u;
        if (output != ((value >> bit) & 1u)) (void)battle_box_read(1);
        battle_box_write(1);
        battle_box_write(0);
    }
}

static void battle_command_word(unsigned command, unsigned address) {
    battle_send_value((uint16_t)((((command ^ 0x7Fu) & 0x7Fu) << 8) | (address & 0x7Fu)));
}

static void set_battle_word(uint16_t word) {
    battle_box_reset_protocol();
    battle_command_word(0x09, 0);
    battle_command_word(0x06, 3);
    battle_send_value(word);
}

static int policy_battle(const char *directory) {
    char identity[256], save_path[256];
    snprintf(identity, sizeof(identity), "%s/battle.nes", directory);
    snprintf(save_path, sizeof(save_path), "%s/battle.battlebox.sav", directory);
    BOARD_CHECK(battle_box_configure(identity));
    set_battle_word(0x3456);
    BOARD_CHECK(battle_box_flush());
    uint16_t previous = 0x3456;
    for (size_t i = 0; i < sizeof(isolated_modes) / sizeof(isolated_modes[0]); i++) {
        uint16_t next = (uint16_t)(0x9180 + i);
        set_battle_word(next);
        BOARD_CHECK(nes_execution_set_policy(isolated_modes[i]));
        BOARD_CHECK(battle_box_flush());
        BOARD_CHECK(read_saved_byte(save_path, 6) == (previous & 0xFF));
        BOARD_CHECK(read_saved_byte(save_path, 7) == (previous >> 8));
        BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
        BOARD_CHECK(battle_box_flush());
        BOARD_CHECK(read_saved_byte(save_path, 6) == (next & 0xFF));
        BOARD_CHECK(read_saved_byte(save_path, 7) == (next >> 8));
        previous = next;
    }

    BOARD_CHECK(battle_box_shutdown());
    BOARD_CHECK(nes_file_remove(save_path) == NES_FILE_OK);
    return 0;
}

static int check_tape_samples(const uint8_t *expected, size_t size) {
    family_basic_write(0x04, 0);
    BOARD_CHECK(family_basic_tape_play(0));
    for (size_t sample = 0; sample < size * 8; sample++) {
        unsigned bit = (family_basic_read(0, sample * FAMILY_BASIC_TAPE_SAMPLE_CYCLES) >> 1) & 1u;
        BOARD_CHECK(bit == ((expected[sample / 8] >> (sample & 7u)) & 1u));
    }

    (void)family_basic_read(0, size * 8 * FAMILY_BASIC_TAPE_SAMPLE_CYCLES);
    BOARD_CHECK(family_basic_tape_mode() == FB_TAPE_STOPPED);
    return 0;
}

static int policy_tape(const char *directory) {
    char path[256];
    snprintf(path, sizeof(path), "%s/caf\xC3\xA9-\xE7\x8C\xAB-\xF0\x9F\x92\xBE.tap", directory);
    const uint8_t original[] = {0xA1, 0x02, 0xE3};
    const uint8_t recording[] = {0x39, 0xC2, 0x2F};
    BOARD_CHECK(nes_file_write_atomic(path, original, sizeof(original)) == NES_FILE_OK);
    BOARD_CHECK(family_basic_tape_load_file(path));
    BOARD_CHECK(check_tape_samples(original, sizeof(original)) == 0);
    BOARD_CHECK(family_basic_tape_load(recording, sizeof(recording)));
    for (size_t i = 0; i < sizeof(isolated_modes) / sizeof(isolated_modes[0]); i++) {
        BOARD_CHECK(nes_execution_set_policy(isolated_modes[i]));
        BOARD_CHECK(!family_basic_tape_save_file(path));
        BOARD_CHECK(read_saved_byte(path, 0) == original[0]);
        BOARD_CHECK(check_tape_samples(recording, sizeof(recording)) == 0);
    }

    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    BOARD_CHECK(family_basic_tape_save_file(path));
    BOARD_CHECK(read_saved_byte(path, 0) == recording[0]);
    family_basic_shutdown();
    BOARD_CHECK(nes_file_remove(path) == NES_FILE_OK);
    return 0;
}

int test_execution_policy_accuracy(void) {
    char directory[128];
#ifdef _WIN32
    unsigned process = (unsigned)_getpid();
#else
    unsigned process = (unsigned)getpid();
#endif
    BOARD_CHECK(snprintf(directory, sizeof(directory), "build/execution-policy-%u", process) > 0);
#ifdef _WIN32
    BOARD_CHECK(_mkdir(directory) == 0);
#else
    BOARD_CHECK(mkdir(directory, 0700) == 0);
#endif
    uint32_t previous_policy = nes_execution_policy();
    NesRamPowerOnState previous_ram = nes_ram_power_on_state();
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    BOARD_CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_ZERO));
    int failures = policy_cart(directory, 0) + policy_cart(directory, 31)
                 + policy_turbo(directory) + policy_battle(directory) + policy_tape(directory);
    if (!nes_execution_set_policy(previous_policy)) ++failures;
    if (!nes_set_ram_power_on_state(previous_ram)) ++failures;
#ifdef _WIN32
    if (_rmdir(directory) != 0) ++failures;
#else
    if (rmdir(directory) != 0) ++failures;
#endif
    printf("Execution persistence policy: 5 groups, %d failures\n", failures);
    return failures;
}
