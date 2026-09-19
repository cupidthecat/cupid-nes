/*
 * input_accuracy_devices.h - Expansion input and storage regression tests
 *
 * Author: @frankischilling
 *
 * This private header contains regression tests for expansion input devices and
 * their persistent storage.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef INPUT_ACCURACY_DEVICES_H
#define INPUT_ACCURACY_DEVICES_H

typedef struct {
    char directory[96];
    char rom[128];
    char turbo[128];
    char battle[128];
} InputStorageFixture;

static int input_storage_begin(InputStorageFixture *paths) {
    for (unsigned attempt = 0; attempt < 1000; ++attempt) {
        snprintf(paths->directory, sizeof(paths->directory), ".input-storage-test-%llu-%u",
                 (unsigned long long)time(NULL), attempt);
#ifdef _WIN32
        int result = _mkdir(paths->directory);
#else
        int result = mkdir(paths->directory, 0700);
#endif
        if (result == 0) {
            snprintf(paths->rom, sizeof(paths->rom), "%s/cart.nes", paths->directory);
            snprintf(paths->turbo, sizeof(paths->turbo), "%s/cart.turbofile.sav", paths->directory);
            snprintf(paths->battle, sizeof(paths->battle), "%s/cart.battlebox.sav", paths->directory);
            return 0;
        }
        if (errno != EEXIST) return -1;
    }
    return -1;
}

static int input_storage_end(const InputStorageFixture *paths) {
    int result = 0;
    if (remove(paths->turbo) != 0 && errno != ENOENT) result = 1;
    if (remove(paths->battle) != 0 && errno != ENOENT) result = 1;
#ifdef _WIN32
    if (_rmdir(paths->directory) != 0) result = 1;
#else
    if (rmdir(paths->directory) != 0) result = 1;
#endif
    return result;
}

static void turbo_clock_bit(unsigned bit) {
    joypad_write_ports((uint8_t)(0x06 | (bit & 1u)));
    joypad_write_ports((uint8_t)(0x02 | (bit & 1u)));
}

static int turbo_file_protocol(void) {
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device_name("turbo-file"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_TURBO_FILE);
    pad2.buttons = 1;
    joypad_write_ports(0);
    for (unsigned bit = 0; bit < 8; ++bit) turbo_clock_bit(0xA5u >> bit);
    joypad_write_ports(0);
    latch_controllers();
    write_mem(0x4018, 0);
    CHECK((read_mem(0x4017) & 5u) == 5u);
    joypad_write_ports(0);
    for (unsigned bit = 0; bit < 8; ++bit) {
        write_mem(0x4018, 0);
        CHECK((read_mem(0x4017) & 4u) == (uint8_t)(((0xA5u >> bit) & 1u) << 2));
        turbo_clock_bit(0xA5u >> bit);
    }

    joypad_write_ports(0);
    joypad_write_ports(7);
    joypad_write_ports(7);
    CHECK((turbo_file_read(1) & 4u) == 4u);
    joypad_write_ports(3);
    CHECK((turbo_file_read(1) & 4u) == 0u);

    joypad_write_ports(0);
    for (unsigned bit = 0; bit < 65536; ++bit) turbo_clock_bit(bit == 0);
    CHECK((turbo_file_read(1) & 4u) == 4u);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    CHECK((read_mem(0x4017) & 4u) == 0);
    CHECK(joypad_persistent_shutdown());
    return 0;
}

static int turbo_file_persistence(void) {
    InputStorageFixture paths;
    CHECK(input_storage_begin(&paths) == 0);
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_TURBO_FILE));
    CHECK(joypad_persistent_configure(paths.rom));
    joypad_write_ports(0);
    for (unsigned byte = 0; byte < 0x2000; ++byte) {
        uint8_t value = (uint8_t)(byte ^ (byte >> 5) ^ 0xA5);
        for (unsigned bit = 0; bit < 8; ++bit) turbo_clock_bit(value >> bit);
    }
    CHECK(joypad_persistent_flush());
    FILE *saved = fopen(paths.turbo, "rb");
    CHECK(saved != NULL);
    for (unsigned byte = 0; byte < 0x2000; ++byte)
        CHECK(fgetc(saved) == (uint8_t)(byte ^ (byte >> 5) ^ 0xA5));
    CHECK(fgetc(saved) == EOF);
    CHECK(fclose(saved) == 0);

    CHECK(joypad_persistent_shutdown());
    CHECK(joypad_set_expansion_device(NES_EXPANSION_TURBO_FILE));
    CHECK(joypad_persistent_configure(paths.rom));
    joypad_write_ports(0);
    for (unsigned bit = 0; bit < 8; ++bit) {
        CHECK((turbo_file_read(1) >> 2) == ((0xA5u >> bit) & 1u));
        turbo_clock_bit(0xA5u >> bit);
    }
    CHECK(joypad_persistent_shutdown());
    CHECK(input_storage_end(&paths) == 0);
    return 0;
}

static int turbo_file_failed_save(void) {
    InputStorageFixture paths;
    CHECK(input_storage_begin(&paths) == 0);
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_TURBO_FILE));
    CHECK(joypad_persistent_configure(paths.rom));
    joypad_write_ports(0);
    turbo_clock_bit(1);
#ifdef _WIN32
    CHECK(_mkdir(paths.turbo) == 0);
#else
    CHECK(mkdir(paths.turbo, 0700) == 0);
#endif
    CHECK(!joypad_persistent_shutdown());
    joypad_write_ports(0);
    CHECK((turbo_file_read(1) & 4u) == 4u);
#ifdef _WIN32
    CHECK(_rmdir(paths.turbo) == 0);
#else
    CHECK(rmdir(paths.turbo) == 0);
#endif
    CHECK(joypad_persistent_flush());
    FILE *saved = fopen(paths.turbo, "rb");
    CHECK(saved != NULL && fgetc(saved) == 1);
    CHECK(fclose(saved) == 0);
    CHECK(joypad_persistent_shutdown());
    CHECK(input_storage_end(&paths) == 0);
    return 0;
}

static unsigned battle_read_output(void) {
    write_mem(0x4018, 0);
    return (read_mem(0x4017) >> 4) & 1u;
}

static void battle_send_bit(unsigned bit) {
    unsigned output = battle_read_output();
    if (output != (bit & 1u)) output = battle_read_output();
    (void)output;
    write_mem(0x4016, 1);
    write_mem(0x4016, 0);
}

static void battle_send_word(uint16_t value) {
    for (unsigned bit = 0; bit < 16; ++bit) battle_send_bit(value >> bit);
}

static void battle_command(unsigned command, unsigned address) {
    battle_send_word((uint16_t)((((command ^ 0x7Fu) & 0x7Fu) << 8) | (address & 0x7Fu)));
}

static uint16_t battle_read_word(unsigned address) {
    battle_command(0x01, address);
    uint16_t value = 0;
    for (unsigned bit = 0; bit < 16; ++bit) {
        write_mem(0x4018, 0);
        uint8_t port = read_mem(0x4017);
        value |= (uint16_t)(((port >> 3) & 1u) << bit);
        write_mem(0x4016, 1);
        write_mem(0x4016, 0);
    }
    return value;
}

static int battle_box_protocol(void) {
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device_name("battle-box"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_BATTLE_BOX);
    pad2.buttons = 1;
    latch_controllers();
    uint8_t first = read_mem(0x4017);
    uint8_t second = read_mem(0x4017);
    CHECK((first & 1u) == 1u);
    CHECK(((first ^ second) & 0x10u) == 0x10u);

    battle_box_reset_protocol();
    battle_command(0x09, 0);
    battle_command(0x06, 0x2A);
    battle_send_word(0xA55A);
    battle_box_reset_protocol();
    battle_command(0x09, 0);
    CHECK(battle_read_word(0x2A) == 0xA55A);

    battle_box_reset_protocol();
    write_mem(0x4016, 1);
    (void)read_mem(0x4017);
    write_mem(0x4016, 0);
    battle_command(0x09, 0);
    battle_command(0x06, 0x2A);
    battle_send_word(0x5AA5);
    battle_box_reset_protocol();
    write_mem(0x4016, 1);
    (void)read_mem(0x4017);
    write_mem(0x4016, 0);
    CHECK(battle_read_word(0x2A) == 0x5AA5);

    battle_box_reset_protocol();
    battle_command(0x09, 0);
    battle_command(0x0C, 0);
    battle_box_reset_protocol();
    CHECK(battle_read_word(0x2A) == 0);
    write_mem(0x4016, 1);
    (void)read_mem(0x4017);
    write_mem(0x4016, 0);
    battle_box_reset_protocol();
    write_mem(0x4016, 1);
    (void)read_mem(0x4017);
    write_mem(0x4016, 0);
    CHECK(battle_read_word(0x2A) == 0);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    CHECK(joypad_persistent_shutdown());
    return 0;
}

static int battle_box_persistence(void) {
    InputStorageFixture paths;
    CHECK(input_storage_begin(&paths) == 0);
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_BATTLE_BOX));
    CHECK(joypad_persistent_configure(paths.rom));
    battle_command(0x09, 0);
    for (unsigned chip = 0; chip < 2; ++chip) {
        if (chip) {
            write_mem(0x4016, 1);
            (void)read_mem(0x4017);
            write_mem(0x4016, 0);
        }
        for (unsigned address = 0; address < 128; ++address) {
            battle_command(0x06, address);
            battle_send_word((uint16_t)(0x5A00u ^ (chip << 15) ^ address));
        }
    }
    CHECK(joypad_persistent_flush());
    FILE *saved = fopen(paths.battle, "rb");
    CHECK(saved != NULL);
    for (unsigned chip = 0; chip < 2; ++chip) {
        for (unsigned address = 0; address < 128; ++address) {
            uint16_t expected = (uint16_t)(0x5A00u ^ (chip << 15) ^ address);
            CHECK(fgetc(saved) == (expected & 0xFF));
            CHECK(fgetc(saved) == (expected >> 8));
        }
    }
    CHECK(fgetc(saved) == EOF && fclose(saved) == 0);

    CHECK(joypad_persistent_shutdown());
    CHECK(joypad_set_expansion_device(NES_EXPANSION_BATTLE_BOX));
    CHECK(joypad_persistent_configure(paths.rom));
    battle_box_reset_protocol();
    CHECK(battle_read_word(0x7F) == (uint16_t)(0x5A00u ^ 0x7Fu));
    CHECK(joypad_persistent_shutdown());
    CHECK(input_storage_end(&paths) == 0);
    return 0;
}

static int battle_box_failed_save(void) {
    InputStorageFixture paths;
    CHECK(input_storage_begin(&paths) == 0);
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_BATTLE_BOX));
    CHECK(joypad_persistent_configure(paths.rom));
    battle_command(0x09, 0);
    battle_command(0x06, 3);
    battle_send_word(0x1234);
#ifdef _WIN32
    CHECK(_mkdir(paths.battle) == 0);
#else
    CHECK(mkdir(paths.battle, 0700) == 0);
#endif
    CHECK(!joypad_persistent_shutdown());
    battle_box_reset_protocol();
    CHECK(battle_read_word(3) == 0x1234);
#ifdef _WIN32
    CHECK(_rmdir(paths.battle) == 0);
#else
    CHECK(rmdir(paths.battle) == 0);
#endif
    CHECK(joypad_persistent_flush());
    CHECK(joypad_persistent_shutdown());
    CHECK(input_storage_end(&paths) == 0);
    return 0;
}

static void subor_select_row(unsigned row, bool upper) {
    write_mem(0x4016, 5);
    write_mem(0x4016, 4);
    for (unsigned i = 0; i < row; ++i) {
        write_mem(0x4016, 6);
        write_mem(0x4016, 4);
    }
    if (upper) write_mem(0x4016, 6);
}

static int subor_keyboard_matrix(void) {
    typedef struct {
        SuborKey key;
        uint8_t bit;
    } KeyCase;
    static const KeyCase cases[13][2] = {
        {{SUBOR_KEY_4, 0}, {SUBOR_KEY_F2, 0}},
        {{SUBOR_KEY_2, 0}, {SUBOR_KEY_F1, 0}},
        {{SUBOR_KEY_INSERT, 0}, {SUBOR_KEY_F8, 0}},
        {{SUBOR_KEY_9, 0}, {SUBOR_KEY_F5, 0}},
        {{SUBOR_KEY_RIGHT_BRACKET, 0}, {SUBOR_KEY_F7, 0}},
        {{SUBOR_KEY_Q, 0}, {SUBOR_KEY_ESCAPE, 0}},
        {{SUBOR_KEY_7, 0}, {SUBOR_KEY_F4, 0}},
        {{SUBOR_KEY_MINUS, 0}, {SUBOR_KEY_F6, 0}},
        {{SUBOR_KEY_T, 0}, {SUBOR_KEY_F3, 0}},
        {{SUBOR_KEY_KP6, 0}, {SUBOR_KEY_UNKNOWN1, 1}},
        {{SUBOR_KEY_ALT, 0}, {SUBOR_KEY_F12, 0}},
        {{SUBOR_KEY_KP_MINUS, 0}, {SUBOR_KEY_F10, 0}},
        {{SUBOR_KEY_GRAVE, 0}, {SUBOR_KEY_F9, 0}}
    };

    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device_name("subor-keyboard"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_SUBOR_KEYBOARD);
    write_mem(0x4016, 0);
    CHECK((read_mem(0x4017) & 0x1E) == 0x1E);
    for (unsigned row = 0; row < 13; ++row) {
        for (unsigned upper = 0; upper < 2; ++upper) {
            KeyCase test = cases[row][upper];
            CHECK(joypad_set_subor_key(test.key, true));
            subor_select_row(row, upper != 0);
            uint8_t expected = (uint8_t)(0x1E & ~(1u << (test.bit + 1u)));
            if (row == 9 && upper) expected &= (uint8_t)~0x02u;
            CHECK((read_mem(0x4017) & 0x1E) == expected);
            CHECK(joypad_set_subor_key(test.key, false));
        }
    }

    CHECK(joypad_set_subor_key(SUBOR_KEY_A, true));
    subor_select_row(5, true);
    CHECK((read_mem(0x4017) & 0x1E) == 0x1A);
    write_mem(0x4016, 6);
    CHECK((read_mem(0x4017) & 0x1E) == 0x1A);
    write_mem(0x4016, 4);
    CHECK((read_mem(0x4017) & 0x1E) == 0x1E);
    CHECK(joypad_set_subor_key(SUBOR_KEY_A, false));
    CHECK(!joypad_set_subor_key(SUBOR_KEY_COUNT, true));
    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    CHECK((read_mem(0x4017) & 0x1E) == 0);
    return 0;
}

static uint8_t subor_mouse_read_byte(void) {
    uint8_t value = 0;
    for (unsigned bit = 0; bit < 8; ++bit) {
        write_mem(0x4018, 0);
        value = (uint8_t)((value << 1) | (read_mem(0x4017) & 1u));
    }
    return value;
}

static void subor_mouse_latch(void) {
    write_mem(0x4016, 1);
    write_mem(0x4016, 0);
}

static int subor_mouse_packets(void) {
    input_fixture(NES_CONSOLE_NES001, NES_REGION_NTSC);
    CHECK(!joypad_set_port_device_name(0, "subor-mouse"));
    CHECK(joypad_set_port_device_name(1, "subor-mouse"));
    CHECK(joypad_port_device(1) == NES_PORT_SUBOR_MOUSE);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_SUBOR_KEYBOARD));
    CHECK(joypad_configuration_valid());

    CHECK(joypad_add_subor_mouse_motion(1, -1));
    CHECK(joypad_set_subor_mouse_buttons(true, false));
    subor_mouse_latch();
    CHECK(subor_mouse_read_byte() == 0x9C);

    CHECK(joypad_add_subor_mouse_motion(-40, 20));
    CHECK(joypad_set_subor_mouse_buttons(true, true));
    subor_mouse_latch();
    CHECK(subor_mouse_read_byte() == 0xF5);
    subor_mouse_latch();
    CHECK(subor_mouse_read_byte() == 0x3E);
    subor_mouse_latch();
    CHECK(subor_mouse_read_byte() == 0x13);

    CHECK(joypad_add_subor_mouse_motion(5, -6));
    CHECK(joypad_set_subor_mouse_buttons(false, false));
    subor_mouse_latch();
    uint8_t partial = 0;
    for (unsigned bit = 0; bit < 4; ++bit)
        partial = (uint8_t)((partial << 1) | (read_mem(0x4017) & 1u));
    CHECK(partial == 0);
    CHECK(joypad_add_subor_mouse_motion(1, 1));
    subor_mouse_latch();
    CHECK(subor_mouse_read_byte() == 0x16);
    subor_mouse_latch();
    CHECK(subor_mouse_read_byte() == 0x1B);
    subor_mouse_latch();
    CHECK(subor_mouse_read_byte() == 0x14);

    CHECK(joypad_set_port_device(1, NES_PORT_GAMEPAD));
    CHECK(!joypad_add_subor_mouse_motion(1, 1));
    CHECK(!joypad_set_subor_mouse_buttons(true, true));
    return 0;
}

static uint32_t hori_track_report(void) {
    latch_controllers();
    uint32_t value = 0;
    for (unsigned bit = 0; bit < 24; ++bit) {
        write_mem(0x4018, 0);
        value |= (uint32_t)(((read_mem(0x4016) >> 1) & 1u) << bit);
    }
    return value;
}

static int hori_track_reports(void) {
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device_name("hori-track"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_HORI_TRACK);
    pad1.buttons = 0xA5;
    CHECK(hori_track_report() == 0x09FFA5u);

    CHECK(joypad_add_hori_track_motion(1, -1));
    CHECK(hori_track_report() == 0x0970A5u);
    CHECK(joypad_add_hori_track_motion(-99, 99));
    CHECK(hori_track_report() == 0x09E1A5u);

    CHECK(joypad_add_hori_track_motion(2, 3));
    latch_controllers();
    uint32_t report = 0;
    for (unsigned bit = 0; bit < 8; ++bit) {
        write_mem(0x4018, 0);
        report |= (uint32_t)(((read_mem(0x4016) >> 1) & 1u) << bit);
    }
    CHECK(joypad_add_hori_track_motion(7, 7));
    for (unsigned bit = 8; bit < 24; ++bit) {
        write_mem(0x4018, 0);
        report |= (uint32_t)(((read_mem(0x4016) >> 1) & 1u) << bit);
    }
    CHECK(report == 0x09B3A5u);
    CHECK(hori_track_report() == 0x0911A5u);

    write_mem(0x4016, 1);
    write_mem(0x4018, 0);
    uint8_t first = read_mem(0x4016) & 2u;
    write_mem(0x4018, 0);
    CHECK((read_mem(0x4016) & 2u) == first);
    write_mem(0x4016, 0);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    CHECK(!joypad_add_hori_track_motion(1, 1));
    CHECK((read_mem(0x4016) & 2u) == 0);
    return 0;
}

static int konami_hyper_shot_signals(void) {
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device_name("konami-hyper-shot"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_KONAMI_HYPER_SHOT);
    pad1.buttons = (1u << BTN_A) | (1u << BTN_B);
    pad2.buttons = (1u << BTN_A) | (1u << BTN_B);

    write_mem(0x4016, 0);
    CHECK((read_mem(0x4017) & 0x1E) == 0x1E);
    write_mem(0x4016, 4);
    CHECK((read_mem(0x4017) & 0x1E) == 0x18);
    write_mem(0x4016, 2);
    CHECK((read_mem(0x4017) & 0x1E) == 0x06);
    write_mem(0x4016, 6);
    CHECK((read_mem(0x4017) & 0x1E) == 0);

    pad1.buttons = 1u << BTN_A;
    pad2.buttons = 1u << BTN_B;
    write_mem(0x4016, 0);
    CHECK((read_mem(0x4017) & 0x1E) == 0x12);
    pad1.buttons = 1u << BTN_B;
    pad2.buttons = 1u << BTN_A;
    CHECK((read_mem(0x4017) & 0x1E) == 0x0C);

    pad2.buttons |= 1u << BTN_START;
    latch_controllers();
    CHECK((read_mem(0x4017) & 1u) == 1u);
    (void)read_mem(0x4017);
    (void)read_mem(0x4017);
    CHECK((read_mem(0x4017) & 1u) == 1u);

    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    CHECK((read_mem(0x4017) & 0x1E) == 0);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_KONAMI_HYPER_SHOT));
    write_mem(0x4016, 0);
    CHECK((read_mem(0x4017) & 0x1E) == 0x0C);
    CHECK(!joypad_set_expansion_device((NesExpansionDevice)999));
    return 0;
}

static int bandai_hyper_shot_signals(void) {
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device_name("bandai-hyper-shot"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_BANDAI_HYPER_SHOT);
    pad1.buttons = 0xA5;
    latch_controllers();
    for (unsigned bit = 0; bit < 8; ++bit) {
        write_mem(0x4018, 0);
        uint8_t value = read_mem(0x4016);
        uint8_t expected = (uint8_t)((0xA5u >> bit) & 1u);
        CHECK((value & 1u) == expected);
        CHECK(((value >> 1) & 1u) == expected);
    }
    write_mem(0x4018, 0);
    CHECK((read_mem(0x4016) & 3u) == 1u);

    write_mem(0x4016, 1);
    pad1.buttons = 1u << BTN_A;
    write_mem(0x4018, 0);
    CHECK((read_mem(0x4016) & 3u) == 3u);
    pad1.buttons = 0;
    write_mem(0x4018, 0);
    CHECK((read_mem(0x4016) & 3u) == 0);
    write_mem(0x4016, 0);

    CHECK(joypad_set_zapper(2, 32, 20, false));
    ppu.scanline = 20;
    ppu.dot = 33;
    write_mem(0x4018, 0);
    CHECK((read_mem(0x4017) & 0x18) == 0x08);
    sensor_pixel(32, 20, 0x20);
    write_mem(0x4018, 0);
    CHECK((read_mem(0x4017) & 0x18) == 0);
    CHECK(joypad_set_zapper(2, 32, 20, true));
    write_mem(0x4018, 0);
    CHECK((read_mem(0x4017) & 0x18) == 0x10);
    CHECK(joypad_set_zapper(2, -1, -1, true));
    write_mem(0x4018, 0);
    CHECK((read_mem(0x4017) & 0x18) == 0x18);

    CHECK(joypad_set_zapper(2, 40, 30, false));
    sensor_pixel(40, 30, 0x20);
    ppu.scanline = 30;
    ppu.dot = 40;
    CHECK((read_mem(0x4017) & 0x08) == 0x08);
    ppu.dot = 42;
    CHECK((read_mem(0x4017) & 0x08) == 0);

    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    write_mem(0x4018, 0);
    CHECK((read_mem(0x4017) & 0x18) == 0);
    CHECK((read_mem(0x4016) & 2u) == 0);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_BANDAI_HYPER_SHOT));
    pad1.buttons = 1;
    latch_controllers();
    CHECK((read_mem(0x4016) & 3u) == 3u);
    return 0;
}

static int party_tap_reports(void) {
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device_name("party-tap"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_PARTY_TAP);
    pad2.buttons = 1;
    CHECK(joypad_set_party_tap_button(0, true));
    CHECK(joypad_set_party_tap_button(2, true));
    CHECK(joypad_set_party_tap_button(4, true));
    CHECK(!joypad_set_party_tap_button(6, true));
    latch_controllers();
    CHECK((read_mem(0x4017) & 0x1D) == 0x15);
    CHECK((read_mem(0x4017) & 0x1C) == 0x08);
    CHECK((read_mem(0x4017) & 0x1C) == 0x14);
    CHECK((read_mem(0x4017) & 0x1C) == 0x14);

    CHECK(joypad_set_party_tap_button(0, false));
    CHECK(joypad_set_party_tap_button(2, false));
    CHECK(joypad_set_party_tap_button(4, false));
    CHECK(joypad_set_party_tap_button(1, true));
    CHECK(joypad_set_party_tap_button(3, true));
    CHECK(joypad_set_party_tap_button(5, true));
    CHECK((read_mem(0x4017) & 0x1C) == 0x14);
    latch_controllers();
    CHECK((read_mem(0x4017) & 0x1C) == 0x08);
    CHECK((read_mem(0x4017) & 0x1C) == 0x14);

    write_mem(0x4016, 1);
    CHECK((read_mem(0x4017) & 0x1C) == 0x08);
    CHECK((read_mem(0x4017) & 0x1C) == 0x08);
    CHECK(joypad_set_party_tap_button(1, false));
    CHECK(joypad_set_party_tap_button(0, true));
    CHECK((read_mem(0x4017) & 0x1C) == 0x04);
    write_mem(0x4016, 0);
    CHECK((read_mem(0x4017) & 0x1C) == 0x04);

    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    CHECK(!joypad_set_party_tap_button(0, true));
    CHECK((read_mem(0x4017) & 0x1C) == 0);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_PARTY_TAP));
    latch_controllers();
    CHECK((read_mem(0x4017) & 0x1C) == 0x04);
    return 0;
}

static uint16_t pachinko_report(void) {
    latch_controllers();
    uint16_t value = 0;
    for (unsigned bit = 0; bit < 16; ++bit) {
        write_mem(0x4018, 0);
        value |= (uint16_t)(((read_mem(0x4016) >> 1) & 1u) << bit);
    }
    return value;
}

static int pachinko_reports(void) {
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device_name("pachinko"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_PACHINKO);
    pad1.buttons = 0xA5;
    CHECK(pachinko_report() == 0xFFA5);
    CHECK(joypad_set_pachinko_controls(true, false));
    CHECK(pachinko_report() == 0x7FA5);
    for (unsigned i = 1; i < 0x32; ++i) (void)pachinko_report();
    CHECK(joypad_set_pachinko_controls(false, false));
    CHECK(pachinko_report() == 0xB3A5);
    CHECK(joypad_set_pachinko_controls(true, false));
    for (unsigned i = 0x32; i < 0x63; ++i) (void)pachinko_report();
    CHECK(joypad_set_pachinko_controls(false, false));
    CHECK(pachinko_report() == 0x39A5);
    CHECK(joypad_set_pachinko_controls(true, false));
    CHECK(pachinko_report() == 0x39A5);
    CHECK(joypad_set_pachinko_controls(false, true));
    CHECK(pachinko_report() == 0xB9A5);

    CHECK(joypad_set_pachinko_controls(false, false));
    pad1.buttons = 0x5A;
    latch_controllers();
    for (unsigned bit = 0; bit < 4; ++bit) {
        write_mem(0x4018, 0);
        uint8_t value = read_mem(0x4016);
        CHECK((value & 1u) == ((0x5Au >> bit) & 1u));
        CHECK(((value >> 1) & 1u) == ((0x5Au >> bit) & 1u));
    }
    pad1.buttons = 0xA5;
    for (unsigned bit = 4; bit < 8; ++bit) {
        write_mem(0x4018, 0);
        CHECK(((read_mem(0x4016) >> 1) & 1u) == ((0x5Au >> bit) & 1u));
    }

    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    CHECK(!joypad_set_pachinko_controls(true, false));
    CHECK((read_mem(0x4016) & 2u) == 0);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_PACHINKO));
    pad1.buttons = 0;
    CHECK(pachinko_report() == 0xFF00);
    return 0;
}

static int exciting_boxing_signals(void) {
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device_name("exciting-boxing"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_EXCITING_BOXING);
    pad2.buttons = 1;

    latch_controllers();
    CHECK((read_mem(0x4017) & 0x1F) == 0x1F);
    for (unsigned sensor = 0; sensor < 4; ++sensor) {
        CHECK(joypad_set_boxing_sensor(sensor, true));
        uint8_t expected = (uint8_t)(0x1E & ~(1u << (sensor + 1u)));
        CHECK((read_mem(0x4017) & 0x1E) == expected);
        CHECK((read_mem(0x4017) & 0x1E) == expected);
        CHECK(joypad_set_boxing_sensor(sensor, false));
    }
    CHECK(joypad_set_boxing_sensor(0, true));
    CHECK(joypad_set_boxing_sensor(2, true));
    CHECK((read_mem(0x4017) & 0x1E) == 0x14);

    write_mem(0x4016, 2);
    CHECK((read_mem(0x4017) & 0x1E) == 0x1E);
    for (unsigned sensor = 4; sensor < 8; ++sensor) {
        CHECK(joypad_set_boxing_sensor(sensor, true));
        uint8_t expected = (uint8_t)(0x1E & ~(1u << (sensor - 3u)));
        CHECK((read_mem(0x4017) & 0x1E) == expected);
        CHECK(joypad_set_boxing_sensor(sensor, false));
    }
    CHECK(joypad_set_boxing_sensor(4, true));
    CHECK(joypad_set_boxing_sensor(7, true));
    CHECK((read_mem(0x4017) & 0x1E) == 0x0C);
    write_mem(0x4016, 0);
    CHECK((read_mem(0x4017) & 0x1E) == 0x14);
    CHECK((read_mem(0x4017) & 1u) == 1u);

    CHECK(!joypad_set_boxing_sensor(8, true));
    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    CHECK(!joypad_set_boxing_sensor(0, true));
    CHECK((read_mem(0x4017) & 0x1E) == 0);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_EXCITING_BOXING));
    write_mem(0x4016, 0);
    CHECK((read_mem(0x4017) & 0x1E) == 0x14);
    return 0;
}

static uint8_t jissen_read_row(unsigned row) {
    write_mem(0x4016, (uint8_t)((row << 1) | 1u));
    write_mem(0x4016, (uint8_t)(row << 1));
    uint8_t value = 0;
    for (unsigned bit = 0; bit < 8; ++bit) {
        write_mem(0x4018, 0);
        value |= (uint8_t)(((read_mem(0x4017) >> 1) & 1u) << bit);
    }
    return value;
}

static int jissen_mahjong_rows(void) {
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device_name("jissen-mahjong"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_JISSEN_MAHJONG);
    pad2.buttons = 1;
    CHECK(jissen_read_row(0) == 0);

    CHECK(joypad_set_jissen_key(JISSEN_KEY_N, true));
    CHECK(joypad_set_jissen_key(JISSEN_KEY_L, true));
    CHECK(joypad_set_jissen_key(JISSEN_KEY_J, true));
    CHECK(jissen_read_row(1) == 0x54);

    CHECK(joypad_set_jissen_key(JISSEN_KEY_H, true));
    CHECK(joypad_set_jissen_key(JISSEN_KEY_F, true));
    CHECK(joypad_set_jissen_key(JISSEN_KEY_D, true));
    CHECK(joypad_set_jissen_key(JISSEN_KEY_B, true));
    CHECK(jissen_read_row(2) == 0x55);

    CHECK(joypad_set_jissen_key(JISSEN_KEY_RON, true));
    CHECK(joypad_set_jissen_key(JISSEN_KEY_CHII, true));
    CHECK(joypad_set_jissen_key(JISSEN_KEY_KAN, true));
    CHECK(joypad_set_jissen_key(JISSEN_KEY_SELECT, true));
    CHECK(jissen_read_row(3) == 0xAA);

    write_mem(0x4016, 5);
    write_mem(0x4016, 4);
    uint8_t partial = 0;
    for (unsigned bit = 0; bit < 4; ++bit) {
        write_mem(0x4018, 0);
        partial |= (uint8_t)(((read_mem(0x4017) >> 1) & 1u) << bit);
    }
    CHECK(partial == 0x05);
    write_mem(0x4016, 6);
    uint8_t upper = 0;
    for (unsigned bit = 4; bit < 8; ++bit) {
        write_mem(0x4018, 0);
        upper |= (uint8_t)(((read_mem(0x4017) >> 1) & 1u) << bit);
    }
    CHECK(upper == 0x50);
    CHECK(jissen_read_row(3) == 0xAA);

    write_mem(0x4016, 7);
    CHECK((read_mem(0x4017) & 2u) == 0);
    CHECK((read_mem(0x4017) & 2u) == 0);
    write_mem(0x4016, 6);

    CHECK(!joypad_set_jissen_key(JISSEN_KEY_COUNT, true));
    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    CHECK(!joypad_set_jissen_key(JISSEN_KEY_A, true));
    CHECK((read_mem(0x4017) & 2u) == 0);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_JISSEN_MAHJONG));
    CHECK(jissen_read_row(0) == 0);
    return 0;
}

static uint8_t barcode_battler_expected_bit(const char record[20], unsigned position) {
    unsigned frame_bit = position % 10u;
    if (frame_bit == 0) return 1;
    if (frame_bit == 9) return 0;
    unsigned character = position / 10u;
    return (uint8_t)(1u ^ (((uint8_t)record[character] >> (frame_bit - 1u)) & 1u));
}

static int barcode_battler_stream(void) {
    static const NesRegion regions[] = {NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_DENDY};
    static const char record[21] = "0123456789012EPOCH\r\n";

    for (unsigned region = 0; region < sizeof(regions) / sizeof(regions[0]); ++region) {
        input_fixture(NES_CONSOLE_HVC001, regions[region]);
        CHECK(joypad_set_expansion_device_name("barcode-battler"));
        CHECK(joypad_expansion_device() == NES_EXPANSION_BARCODE_BATTLER);
        pad2.buttons = 1;
        write_mem(0x4016, 1);
        CHECK(joypad_scan_barcode_battler("0123456789012"));

        uint64_t cycles_per_bit = (uint32_t)nes_timing()->cpu_hz / 1200u;
        CHECK(cycles_per_bit != 0);
        input_program(0xAD, 0x4017);
        for (unsigned position = 0; position < 200; ++position) {
            uint64_t read_cycle = position ? (uint64_t)position * cycles_per_bit : 4u;
            cpu_total_cycles = read_cycle - 4u;
            cpu.pc = 0x8000;
            CHECK(cpu_step(&cpu) == 4);
            uint8_t expected = barcode_battler_expected_bit(record, position);
            CHECK((cpu.a & 0x05u) == (uint8_t)(1u | (expected << 2)));
        }

        cpu_total_cycles = 200u * cycles_per_bit - 4u;
        cpu.pc = 0x8000;
        CHECK(cpu_step(&cpu) == 4);
        CHECK((cpu.a & 0x05u) == 1u);
    }
    return 0;
}

static int barcode_battler_lifetime(void) {
    static const char first_record[21] = "     12345678EPOCH\r\n";
    static const char second_record[21] = "     87654321EPOCH\r\n";

    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_BARCODE_BATTLER));
    CHECK((read_mem(0x4017) & 4u) == 0);
    CHECK(!joypad_scan_barcode_battler(NULL));
    CHECK(!joypad_scan_barcode_battler("1234567"));
    CHECK(!joypad_scan_barcode_battler("12345678901234"));
    CHECK(!joypad_scan_barcode_battler("1234X678"));
    CHECK((read_mem(0x4017) & 4u) == 0);

    cpu_total_cycles = 100;
    CHECK(joypad_scan_barcode_battler("12345678"));
    uint64_t first_insert = cpu_total_cycles;
    uint64_t cycles_per_bit = (uint32_t)nes_timing()->cpu_hz / 1200u;
    cpu_total_cycles = first_insert + 73u * cycles_per_bit;
    CHECK(!joypad_scan_barcode_battler("bad"));
    CHECK((read_mem(0x4017) & 4u) ==
          (uint8_t)(barcode_battler_expected_bit(first_record, 73) << 2));

    cpu_total_cycles = first_insert + 90u * cycles_per_bit;
    CHECK(joypad_scan_barcode_battler("87654321"));
    uint64_t second_insert = cpu_total_cycles;
    CHECK((read_mem(0x4017) & 4u) ==
          (uint8_t)(barcode_battler_expected_bit(second_record, 0) << 2));
    cpu_total_cycles = second_insert + 37u * cycles_per_bit;
    CHECK((read_mem(0x4017) & 4u) ==
          (uint8_t)(barcode_battler_expected_bit(second_record, 37) << 2));

    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    CHECK(!joypad_scan_barcode_battler("12345678"));
    CHECK((read_mem(0x4017) & 4u) == 0);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_BARCODE_BATTLER));
    CHECK((read_mem(0x4017) & 4u) == 0);
    CHECK(joypad_scan_barcode_battler("12345678"));
    CHECK(joypad_set_expansion_device(NES_EXPANSION_BARCODE_BATTLER));
    CHECK((read_mem(0x4017) & 4u) == 0);
    return 0;
}

static uint32_t oeka_kids_expected_state(int x, int y, bool touch, bool click) {
    if (x < -1) x = -1;
    if (x > 255) x = 255;
    if (y < -1) y = -1;
    if (y > 239) y = 239;
    x += 8;
    y -= 14;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    uint8_t tablet_x = (uint8_t)((unsigned)x * 240u / 256u);
    uint8_t tablet_y = (uint8_t)((unsigned)y * 256u / 240u);
    return ((uint32_t)tablet_x << 10) | ((uint32_t)tablet_y << 2)
        | (touch ? 2u : 0u) | (click ? 1u : 0u);
}

static int oeka_kids_read_report(uint32_t expected) {
    write_mem(0x4016, 1);
    write_mem(0x4016, 0);
    write_mem(0x4016, 1);
    input_program(0xAD, 0x4017);
    cpu.pc = 0x8000;
    CHECK(cpu_step(&cpu) == 4);
    CHECK((cpu.a & 0x0Du) == 0x05u);

    for (int bit = 17; bit >= 0; --bit) {
        write_mem(0x4016, 3);
        cpu.pc = 0x8000;
        CHECK(cpu_step(&cpu) == 4);
        uint8_t tablet = (expected & (1u << bit)) ? 0 : 0x08;
        CHECK((cpu.a & 0x0Du) == (uint8_t)(1u | tablet));
        write_mem(0x4016, 1);
    }

    write_mem(0x4016, 3);
    cpu.pc = 0x8000;
    CHECK(cpu_step(&cpu) == 4);
    CHECK((cpu.a & 0x0Du) == 0x09u);
    write_mem(0x4016, 0);
    cpu.pc = 0x8000;
    CHECK(cpu_step(&cpu) == 4);
    CHECK((cpu.a & 0x0Cu) == 0);
    return 0;
}

static int oeka_kids_tablet_reports(void) {
    static const struct {
        int x;
        int y;
        bool touch;
        bool click;
    } cases[] = {
        {0, 0, false, false},
        {255, 239, true, true},
        {-1, -1, false, true},
        {999, 999, true, false},
        {-999, -999, false, false}
    };

    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device_name("oeka-kids-tablet"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_OEKA_KIDS_TABLET);
    pad2.buttons = 1;
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        CHECK(joypad_set_oeka_kids_tablet(cases[i].x, cases[i].y,
                                          cases[i].touch, cases[i].click));
        CHECK(oeka_kids_read_report(oeka_kids_expected_state(cases[i].x, cases[i].y,
                                                              cases[i].touch, cases[i].click)) == 0);
    }

    CHECK(joypad_set_oeka_kids_tablet(40, 80, true, false));
    uint32_t old_report = oeka_kids_expected_state(40, 80, true, false);
    write_mem(0x4016, 0);
    write_mem(0x4016, 1);
    CHECK(joypad_set_oeka_kids_tablet(200, 180, false, true));
    for (int bit = 17; bit >= 0; --bit) {
        write_mem(0x4016, 3);
        uint8_t tablet = (old_report & (1u << bit)) ? 0 : 0x08;
        CHECK((read_mem(0x4017) & 0x0Cu) == tablet);
        write_mem(0x4016, 1);
    }
    write_mem(0x4016, 0);
    CHECK(oeka_kids_read_report(oeka_kids_expected_state(200, 180, false, true)) == 0);

    CHECK(joypad_set_adapter(NES_ADAPTER_FAMICOM_TWO));
    CHECK(!joypad_configuration_valid());
    CHECK(joypad_set_adapter(NES_ADAPTER_NONE));
    CHECK(joypad_configuration_valid());
    CHECK(!joypad_set_expansion_device_name("oeka-tablet"));
    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    CHECK(!joypad_set_oeka_kids_tablet(0, 48, true, false));
    CHECK((read_mem(0x4017) & 0x0Cu) == 0);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_OEKA_KIDS_TABLET));
    write_mem(0x4016, 1);
    CHECK((read_mem(0x4017) & 0x0Cu) == 0x04);
    write_mem(0x4016, 3);
    CHECK((read_mem(0x4017) & 0x0Cu) == 0x08);
    return 0;
}

static int oeka_kids_cpu_store(uint16_t address, uint8_t value) {
    ram[0x0200] = 0x8D;
    ram[0x0201] = (uint8_t)address;
    ram[0x0202] = (uint8_t)(address >> 8);
    cpu.pc = 0x0200;
    cpu.a = value;
    CHECK(cpu_step(&cpu) == 4);
    return 0;
}

static int oeka_kids_cartridge_and_tablet(void) {
    static uint8_t image[sizeof(iNESHeader) + 0x20000];
    iNESHeader header = {0};
    memcpy(header.signature, "NES\x1A", 4);
    header.prg_rom_chunks = 8;
    header.flags6 = 1;
    header.flags7 = 0x68;
    header.zero[0] = 9;
    memset(image, 0xFF, sizeof(image));
    memcpy(image, &header, sizeof(header));
    for (unsigned bank = 0; bank < 4; ++bank) {
        size_t base = sizeof(header) + bank * 0x8000u;
        image[base + 0x0100] = (uint8_t)(0x50u + bank);
        image[base + 0x7FFC] = 0x00;
        image[base + 0x7FFD] = 0x02;
    }

    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(load_rom_memory(image, sizeof(image)) == 0);
    CHECK(rom_mapper_number(&ines_header) == 96 && chr_size == 0x8000);
    CHECK(joypad_set_expansion_device_name("oeka-kids-tablet"));
    CHECK(joypad_configuration_valid());
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    pad2.buttons = 1;
    ram[0x0210] = 0xAD;
    ram[0x0211] = 0x17;
    ram[0x0212] = 0x40;

    CHECK(oeka_kids_cpu_store(0x8FFE, 5) == 0);
    CHECK(read_mem(0x8100) == 0x51);
    (void)ppu_read(0x0000);
    (void)ppu_read(0x2100);
    ppu_write(0x0123, 0xA1);
    (void)ppu_read(0x2200);
    ppu_write(0x0123, 0xA2);
    (void)ppu_read(0x2100);
    (void)ppu_read(0x2200);
    CHECK(ppu_read(0x0123) == 0xA1);
    (void)ppu_read(0x2200);
    ppu_write(0x1123, 0xA3);
    CHECK(ppu_read(0x0123) == 0xA2);

    // These fixed reports encode tablet coordinates (120,128) and (7,0).
    // The CPU program uses the loaded board while clocking the expansion port.
    static const uint32_t reports[] = {0x1E203, 0x01C01};
    for (unsigned sample = 0; sample < 2; ++sample) {
        CHECK(joypad_set_oeka_kids_tablet(sample ? 0 : 120, sample ? 0 : 134,
                                          sample == 0, true));
        CHECK(oeka_kids_cpu_store(0x4016, 0) == 0);
        CHECK(oeka_kids_cpu_store(0x4016, 1) == 0);
        cpu.pc = 0x0210;
        CHECK(cpu_step(&cpu) == 4 && (cpu.a & 0x0Du) == 0x05u);
        for (int bit = 17; bit >= 0; --bit) {
            CHECK(oeka_kids_cpu_store(0x4016, 3) == 0);
            cpu.pc = 0x0210;
            CHECK(cpu_step(&cpu) == 4);
            uint8_t expected = (reports[sample] & (1u << bit)) ? 1 : 9;
            CHECK((cpu.a & 0x0Du) == expected);
            CHECK(oeka_kids_cpu_store(0x4016, 1) == 0);
        }
        CHECK(read_mem(0x8100) == 0x51);
        CHECK(ppu_read(0x0123) == 0xA2 && ppu_read(0x1123) == 0xA3);
        ppu_soft_reset(&ppu);
        apu_soft_reset(&apu);
        cpu_soft_reset(&cpu);
        CHECK(read_mem(0x8100) == 0x51 && ppu_read(0x0123) == 0xA2);
    }

    image[11] = 8;
    CHECK(load_rom_memory(image, sizeof(image)) == 0);
    CHECK(rom_mapper_number(&ines_header) == 96 && chr_size == 0x4000);
    CHECK(joypad_expansion_device() == NES_EXPANSION_OEKA_KIDS_TABLET);
    ppu_write(0x0123, 0xB4);
    CHECK(ppu_read(0x0123) == 0xB4);
    CHECK(read_mem(0x8100) == 0x50);
    CHECK(joypad_set_adapter(NES_ADAPTER_FAMICOM_TWO));
    CHECK(!joypad_configuration_valid());
    CHECK(joypad_set_adapter(NES_ADAPTER_NONE) && joypad_configuration_valid());
    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    CHECK(unload_rom());
    return 0;
}

#endif // INPUT_ACCURACY_DEVICES_H

