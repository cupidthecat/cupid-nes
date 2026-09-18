/*
 * bandai_accuracy.c - Bandai cartridge and serial-device regressions
 *
 * Author: @frankischilling
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */
#include "../rom/rom.h"
#include "../rom/mapper.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../apu/apu.h"
#include "../system/timing.h"
#include <errno.h>
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

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); return 1; \
} } while (0)

static iNESHeader board_header(unsigned mapper, unsigned submapper, bool legacy,
                               size_t prg_bytes, size_t chr_bytes, unsigned save_shift) {
    iNESHeader h = {0};
    memcpy(h.signature, "NES\x1A", 4);
    h.flags6 = (uint8_t)((mapper << 4) | (save_shift ? 2 : 0));
    h.flags7 = (uint8_t)((mapper & 0xF0) | (legacy ? 0 : 8));
    h.prg_rom_chunks = (uint8_t)(prg_bytes / 0x4000);
    h.chr_rom_chunks = (uint8_t)(chr_bytes / 0x2000);
    if (!legacy) {
        h.prg_ram_size = (uint8_t)(submapper << 4);
        h.flags10 = (uint8_t)(save_shift << 4);
        h.zero[0] = chr_bytes ? 0 : 7;
    }
    return h;
}

static uint8_t *board_image(const iNESHeader *h, size_t *bytes) {
    size_t prg_bytes = (size_t)h->prg_rom_chunks * 0x4000;
    size_t chr_bytes = (size_t)h->chr_rom_chunks * 0x2000;
    *bytes = sizeof(*h) + prg_bytes + chr_bytes;
    uint8_t *image = malloc(*bytes);
    if (!image) return NULL;
    memcpy(image, h, sizeof(*h));
    for (size_t i = 0; i < prg_bytes; ++i) image[sizeof(*h) + i] = (uint8_t)(i / 0x4000);
    for (size_t i = 0; i < chr_bytes; ++i) image[sizeof(*h) + prg_bytes + i] = (uint8_t)(i / 0x0400);
    return image;
}

static int load_board(const iNESHeader *h) {
    size_t bytes;
    uint8_t *image = board_image(h, &bytes);
    if (!image) return -1;
    int result = load_rom_memory(image, bytes);
    free(image);
    return result;
}

static int test_bandai_mapping(void) {
    static const unsigned variants[][3] = {
        {16, 0, 1}, {16, 0, 0}, {16, 4, 0}, {16, 5, 0},
        {153, 0, 1}, {153, 0, 0}, {157, 0, 1}, {157, 0, 0},
        {159, 0, 1}, {159, 0, 0}
    };
    for (size_t variant = 0; variant < sizeof(variants) / sizeof(variants[0]); ++variant) {
        unsigned mapper = variants[variant][0], sub = variants[variant][1];
        bool chr_ram = mapper == 153 || mapper == 157;
        unsigned nv_shift = mapper == 153 ? 7 : mapper == 159 ? 1 : mapper == 16 && sub == 5 ? 2 : 0;
        iNESHeader h = board_header(mapper, sub, variants[variant][2] != 0,
                                   0x40000, chr_ram ? 0 : 0x20000, nv_shift);
        CHECK(load_board(&h) == 0);
        CHECK(rom_mapper_number(&ines_header) == (int)mapper);
        CHECK(cart && cart->clock && !cart->reset);
        uint16_t reg = sub == 4 ? 0x6000 : 0x8000;
        CHECK(cart_cpu_read_bus(0x8123, 0x56) == 0x56);
        CHECK(cart_cpu_read(0xC000) == 15 && cart_cpu_read(0xFFFF) == 15);
        cart_cpu_write((uint16_t)(reg + 0x0188), 0xF9);
        CHECK(cart_cpu_read(0x8000) == 9 && cart_cpu_read(0xC000) == 15);
        cart_cpu_write(0x6128, 3);
        CHECK(cart_cpu_read(0x8000) == (mapper == 16 && sub != 5 ? 3 : 9));
        cart_cpu_write(0xFFF8, 7);
        CHECK(cart_cpu_read(0x8000) == (sub == 4 ? 3 : 7));
        static const Mirroring mirrors[] = {
            MIRROR_VERTICAL, MIRROR_HORIZONTAL, MIRROR_SINGLE0, MIRROR_SINGLE1
        };
        for (unsigned mode = 0; mode < 4; ++mode) {
            cart_cpu_write((uint16_t)(reg + 0x0139), (uint8_t)(0xFC | mode));
            CHECK(cart_get_mirroring() == mirrors[mode]);
        }
        if (chr_ram) {
            cart_ppu_write(0x0012, 0xA6);
            cart_ppu_write(0x1FFF, 0x69);
            for (unsigned slot = 0; slot < 8; ++slot)
                cart_cpu_write((uint16_t)(reg + slot), (uint8_t)(8 + slot));
            CHECK(cart_ppu_read(0x0012) == 0xA6 && cart_ppu_read(0x1FFF) == 0x69);
        } else {
            CHECK(cart_ppu_read(0x0012) == 0x12);
            for (unsigned slot = 0; slot < 8; ++slot) {
                cart_cpu_write((uint16_t)(reg + 0x0120 + slot), (uint8_t)(8 + slot));
                CHECK(cart_ppu_read((uint16_t)(slot * 0x0400)) == 8 + slot);
                cart_ppu_write((uint16_t)(slot * 0x0400), 0xFF);
                CHECK(cart_ppu_read((uint16_t)(slot * 0x0400)) == 8 + slot);
            }
        }
        CHECK(cart_cpu_read_bus(0x5FFF, 0x96) == 0x96);
        if (mapper != 153) {
            CHECK((cart_cpu_read_bus(0x6ABC, 0xFF) & 0xE7) == 0xE7);
            CHECK((cart_cpu_read_bus(0x7FFF, 0) & 8) == 0);
        }
    }
    return 0;
}

static int test_bandai_sram_and_outer_bank(void) {
    iNESHeader h = board_header(153, 0, false, 0x80000, 0, 7);
    CHECK(load_board(&h) == 0);
    cart_cpu_write(0x6000, 0x96);
    cart_cpu_write(0x7FFF, 0x69);
    cart_cpu_write(0x8008, 3);
    cart_cpu_write(0x8000, 1);
    CHECK(cart_cpu_read(0x8000) == 19 && cart_cpu_read(0xC000) == 31);
    cart_cpu_write(0x8007, 1);
    cart_cpu_write(0x8000, 0);
    CHECK(cart_cpu_read(0x8000) == 19 && cart_cpu_read(0xC000) == 31);
    cart_cpu_write(0x8007, 0);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xC000) == 15);
    cart_cpu_write(0x800D, 0xDF);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    cart_cpu_write(0x6000, 0xFF);
    cart_cpu_write(0x800D, 0x20);
    CHECK(cart_cpu_read(0x6000) == 0x96 && cart_cpu_read(0x7FFF) == 0x69);
    h = board_header(16, 5, false, 0x80000, 0, 0);
    CHECK(load_board(&h) == 0);
    cart_cpu_write(0x8008, 2);
    cart_cpu_write(0x8003, 1);
    CHECK(cart_cpu_read(0x8000) == 18 && cart_cpu_read(0xC000) == 31);
    cart_cpu_write(0x8003, 2);
    CHECK(cart_cpu_read(0x8000) == 2 && cart_cpu_read(0xC000) == 15);
    return 0;
}

static int test_bandai_irq(void) {
    const unsigned variants[][2] = {{16, 0}, {16, 4}, {16, 5}, {153, 0}, {157, 0}, {159, 0}};
    for (size_t i = 0; i < sizeof(variants) / sizeof(variants[0]); ++i) {
        unsigned mapper = variants[i][0], sub = variants[i][1];
        iNESHeader h = board_header(mapper, sub, false, 0x40000, 0, 0);
        CHECK(load_board(&h) == 0);
        uint16_t reg = sub == 4 ? 0x6000 : 0x8000;
        cart_cpu_write((uint16_t)(reg + 11), 1);
        cart_cpu_write((uint16_t)(reg + 12), 0);
        cart_cpu_write((uint16_t)(reg + 10), 1);
        cart->clock(1);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
        (void)cart_cpu_read(0x6000);
        cart_cpu_write((uint16_t)(reg + 11), 4);
        CHECK(cart_irq_pending());
        cart_cpu_write((uint16_t)(reg + 10), 0);
        CHECK(!cart_irq_pending());
        cart->clock(65536);
        CHECK(!cart_irq_pending());
        cart_cpu_write((uint16_t)(reg + 12), 0);
        cart_cpu_write((uint16_t)(reg + 10), 1);
        cart->clock(1);
        cart_cpu_write((uint16_t)(reg + 11), 0);
        cart->clock(1);
        CHECK(cart_irq_pending() == (sub == 4));
        if (sub != 4) {
            cart->clock(2);
            CHECK(!cart_irq_pending());
            cart->clock(1);
            CHECK(cart_irq_pending());
        }
        cart_cpu_write((uint16_t)(reg + 10), 0);
        cart_cpu_write((uint16_t)(reg + 11), 0xFF);
        cart_cpu_write((uint16_t)(reg + 12), 0xFF);
        cart_cpu_write((uint16_t)(reg + 10), 1);
        cart->clock(65535);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
    }
    return 0;
}

static int test_bandai_cpu_irq(void) {
    iNESHeader h = board_header(16, 5, false, 0x40000, 0, 0);
    CHECK(load_board(&h) == 0);
    prg_rom[prg_size - 4] = 0;
    prg_rom[prg_size - 3] = 2;
    prg_rom[prg_size - 2] = 0;
    prg_rom[prg_size - 1] = 3;
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    for (unsigned i = 0; i < 32; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    write_mem(0x800B, 7);
    write_mem(0x800C, 0);
    write_mem(0x800A, 1);
    for (unsigned step = 0; step < 3; ++step) {
        CHECK(cpu_step(&cpu) == 2);
        CHECK(!cart_irq_pending());
    }
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
    for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
    CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG));
    CHECK(cart_irq_pending());
    write_mem(0x800A, 0);
    CHECK(!cart_irq_pending());
    write_mem(0x8008, 3);
    cpu_soft_reset(&cpu);
    CHECK(cart_cpu_read(0x8000) == 3);
    return 0;
}

static bool extra_eeprom;

static void serial_pins(bool scl, bool sda) {
    if (extra_eeprom) {
        if (!scl) cart_cpu_write(0x8000, 0);
        cart_cpu_write(0x800D, sda ? 0x40 : 0);
        if (scl) cart_cpu_write(0x8000, 8);
    } else {
        cart_cpu_write(0x800D, (uint8_t)((scl ? 0x20 : 0) | (sda ? 0x40 : 0)));
    }
}

static bool serial_output(void) { return (cart_cpu_read_bus(0x6000, 0xE7) & 0x10) != 0; }

static void serial_start(void) {
    serial_pins(false, true);
    serial_pins(true, true);
    serial_pins(true, false);
    serial_pins(false, false);
}

static void serial_stop(void) {
    serial_pins(false, false);
    serial_pins(true, false);
    serial_pins(true, true);
    serial_pins(false, true);
}

static void serial_select(bool extra) {
    extra_eeprom = true;
    serial_stop();
    extra_eeprom = false;
    serial_stop();
    extra_eeprom = extra;
}

static bool serial_send(uint8_t value, bool lsb_first) {
    for (unsigned i = 0; i < 8; ++i) {
        bool bit = ((value >> (lsb_first ? i : 7 - i)) & 1) != 0;
        serial_pins(false, bit);
        serial_pins(true, bit);
        serial_pins(false, bit);
    }
    serial_pins(false, true);
    serial_pins(true, true);
    bool acknowledged = !serial_output();
    serial_pins(false, true);
    return acknowledged;
}

static uint8_t serial_receive(bool lsb_first, bool acknowledge) {
    uint8_t value = 0;
    for (unsigned i = 0; i < 8; ++i) {
        serial_pins(false, true);
        serial_pins(true, true);
        if (serial_output()) value |= (uint8_t)(1u << (lsb_first ? i : 7 - i));
        serial_pins(false, true);
    }
    serial_pins(false, !acknowledge);
    serial_pins(true, !acknowledge);
    serial_pins(false, !acknowledge);
    return value;
}

static int eeprom02_address(uint8_t address) {
    serial_start();
    CHECK(serial_send(0xA0, false));
    CHECK(serial_send(address, false));
    return 0;
}

static int eeprom02_read(uint8_t address, uint8_t expected) {
    CHECK(eeprom02_address(address) == 0);
    serial_start();
    CHECK(serial_send(0xA1, false));
    CHECK(serial_receive(false, false) == expected);
    serial_stop();
    return 0;
}

static int eeprom01_read(uint8_t address, uint8_t expected) {
    serial_start();
    CHECK(serial_send((uint8_t)(address | 0x80), true));
    CHECK(serial_receive(true, false) == expected);
    serial_stop();
    return 0;
}

static int test_bandai_eeprom02(void) {
    iNESHeader h = board_header(16, 5, false, 0x40000, 0x20000, 2);
    CHECK(load_board(&h) == 0);
    serial_select(false);
    CHECK(eeprom02_address(0x26) == 0);
    CHECK(serial_send(0x96, false));
    CHECK(serial_send(0x69, false));
    serial_stop();
    CHECK(eeprom02_read(0x26, 0x96) == 0);
    CHECK(eeprom02_read(0x27, 0x69) == 0);
    CHECK(eeprom02_address(0xFF) == 0);
    CHECK(serial_send(0xA5, false));
    CHECK(serial_send(0x5A, false));
    serial_stop();
    CHECK(eeprom02_address(0xFF) == 0);
    serial_start();
    CHECK(serial_send(0xA1, false));
    CHECK(serial_receive(false, true) == 0xA5);
    CHECK(serial_receive(false, false) == 0x5A);
    serial_stop();
    serial_start();
    CHECK(!serial_send(0x20, false));
    CHECK(!serial_send(0x26, false));
    CHECK(!serial_send(0xFF, false));
    serial_stop();
    CHECK(eeprom02_read(0x26, 0x96) == 0);
    CHECK(eeprom02_address(0x26) == 0);
    for (unsigned bit = 0; bit < 3; ++bit) {
        serial_pins(false, true);
        serial_pins(true, true);
        serial_pins(false, true);
    }
    serial_stop();
    CHECK(eeprom02_read(0x26, 0x96) == 0);
    cart_cpu_write(0x600D, 0x60); // Submapper 5 does not clock the chip through lower aliases.
    CHECK(serial_output());
    h.flags10 = 0;
    h.flags6 &= (uint8_t)~2u;
    CHECK(load_board(&h) == 0);
    serial_select(false);
    CHECK(!serial_output()); // No EEPROM was declared on this variant.
    return 0;
}

static int test_bandai_eeprom01(void) {
    iNESHeader h = board_header(159, 0, false, 0x40000, 0x20000, 1);
    CHECK(load_board(&h) == 0);
    serial_select(false);
    serial_start();
    CHECK(serial_send(0x26, true));
    CHECK(serial_send(0x96, true));
    CHECK(!serial_send(0x69, true)); // 24C01 writes one byte per address transaction.
    serial_stop();
    CHECK(eeprom01_read(0x26, 0x96) == 0);
    CHECK(eeprom01_read(0x27, 0) == 0);
    serial_start();
    CHECK(serial_send(0x7F, true));
    CHECK(serial_send(0x69, true));
    serial_stop();
    CHECK(eeprom01_read(0x7F, 0x69) == 0);
    CHECK(eeprom01_read(0, 0) == 0);
    serial_pins(false, true);
    serial_pins(true, false); // Simultaneous SDA/SCL changes do not constitute START.
    serial_pins(false, false);
    CHECK(!serial_send(0x26, true));
    CHECK(!serial_send(0xFF, true));
    serial_stop();
    CHECK(eeprom01_read(0x26, 0x96) == 0);
    return 0;
}

static int test_datach_dual_eeprom(void) {
    iNESHeader h = board_header(157, 0, false, 0x40000, 0, 1);
    CHECK(load_board(&h) == 0);
    serial_select(false);
    CHECK(eeprom02_address(0x26) == 0);
    CHECK(serial_send(0x96, false));
    serial_stop();
    serial_select(true);
    serial_start();
    CHECK(serial_send(0x26, true));
    CHECK(serial_send(0x69, true));
    serial_stop();
    CHECK(eeprom01_read(0x26, 0x69) == 0);
    serial_select(false);
    CHECK(eeprom02_read(0x26, 0x96) == 0);
    serial_select(true);
    serial_start();
    CHECK(serial_send(0xA6, true));
    serial_pins(false, true);
    serial_pins(true, true); // First LSB of $69 is high.
    CHECK(serial_output());
    serial_pins(false, true);
    serial_pins(true, true); // Second bit is low and holds the shared SDA line down.
    CHECK(!serial_output());
    cart_cpu_write(0x800D, 0x40);
    CHECK(!serial_output());
    serial_stop();
    h.flags10 = 0;
    h.flags6 &= (uint8_t)~2u;
    CHECK(load_board(&h) == 0);
    serial_select(false);
    CHECK(eeprom02_address(0x26) == 0);
    CHECK(serial_send(0xA5, false));
    serial_stop();
    CHECK(eeprom02_read(0x26, 0xA5) == 0);
    serial_select(true);
    serial_start();
    CHECK(!serial_send(0x26, true));
    serial_stop();
    return 0;
}

static int barcode_waveform(const char *code, const char *bars) {
    CHECK(cart_set_barcode(code));
    size_t barcode_bits = strlen(bars);
    size_t total = 33 + barcode_bits + 32;
    for (size_t bit = 0; bit < total; ++bit) {
        uint8_t expected = bit < 33 || bit >= 33 + barcode_bits || bars[bit - 33] == '0' ? 8 : 0;
        CHECK((cart_cpu_read_bus(0x6000, 0xE7) & 8) == expected);
        CHECK((cart_cpu_read_bus(0x7FFF, 0) & 8) == expected);
        cart->clock(999);
        CHECK((cart_cpu_read_bus(0x6123, 0) & 8) == expected);
        cart->clock(1);
    }
    CHECK((cart_cpu_read_bus(0x6000, 0xFF) & 8) == 0);
    return 0;
}

static int test_datach_barcode(void) {
    iNESHeader h = board_header(157, 0, false, 0x40000, 0, 0);
    CHECK(load_board(&h) == 0);
    static const char ean8[] = "101" "0011001" "0010011" "0111101" "0100011"
        "01010" "1001110" "1010000" "1000100" "1110010" "101";
    static const char ean13[] = "101" "0001101" "0100111" "0101111" "0111101" "0001001" "0110011"
        "01010" "1000010" "1000010" "1000010" "1110100" "1000010" "1100110" "101";
    CHECK(barcode_waveform("12345670", ean8) == 0);
    CHECK(barcode_waveform("4006381333931", ean13) == 0);
    CHECK(barcode_waveform("12345679", ean8) == 0); // The reader generates the checksum digit.
    CHECK(cart_set_barcode("12345670"));
    cart->clock(33000);
    CHECK((cart_cpu_read(0x6000) & 8) == 0);
    CHECK(!cart_set_barcode(NULL));
    CHECK(!cart_set_barcode("1234567"));
    CHECK(!cart_set_barcode("1234567x"));
    CHECK(!cart_set_barcode("12345678901234"));
    CHECK((cart_cpu_read(0x6000) & 8) == 0);
    cart->clock(1000);
    CHECK((cart_cpu_read(0x6000) & 8) == 8);

    prg_rom[prg_size - 4] = 0;
    prg_rom[prg_size - 3] = 2;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    write_mem(0x0200, 0xAD);
    write_mem(0x0201, 0x00);
    write_mem(0x0202, 0x60);
    for (unsigned alignment = 0; alignment < 2; ++alignment) {
        CHECK(cart_set_barcode("12345670"));
        cart->clock((int)(32995 + alignment));
        cpu.pc = 0x0200;
        CHECK(cpu_step(&cpu) == 4);
        CHECK((cpu.a & 8) == (alignment ? 0 : 8));
    }
    h = board_header(159, 0, false, 0x40000, 0, 1);
    CHECK(load_board(&h) == 0);
    CHECK(!cart_set_barcode("12345670"));
    return 0;
}

static int test_bandai_rejected_loads(void) {
    iNESHeader h = board_header(16, 5, false, 0x40000, 0x20000, 2);
    CHECK(load_board(&h) == 0);
    serial_select(false);
    CHECK(eeprom02_address(0x26) == 0);
    CHECK(serial_send(0x96, false));
    serial_stop();
    cart_cpu_write(0x8008, 3);
    cart_cpu_write(0x800A, 1);
    cart->clock(1);
    CHECK(cart_irq_pending());
    uint8_t *previous_prg = prg_rom, *previous_chr = chr_rom;
    iNESHeader rejected[] = {h, h, h, h, h};
    rejected[0].prg_ram_size = 0x20;
    rejected[1].flags10 = 0x30;
    rejected[2].flags10 = 0x27;
    rejected[3].flags6 &= (uint8_t)~2u;
    rejected[4].zero[0] = 7;
    for (size_t i = 0; i < sizeof(rejected) / sizeof(rejected[0]); ++i) {
        CHECK(load_board(&rejected[i]) == -1);
        CHECK(prg_rom == previous_prg && chr_rom == previous_chr && cart_irq_pending());
        CHECK(cart_cpu_read(0x8000) == 3);
        CHECK(eeprom02_read(0x26, 0x96) == 0);
    }
    size_t bytes;
    uint8_t *image = board_image(&h, &bytes);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, bytes - 1);
    free(image);
    CHECK(loaded == -1 && prg_rom == previous_prg && cart_irq_pending());
    CHECK(eeprom02_read(0x26, 0x96) == 0);
    return 0;
}

typedef struct {
    char directory[96], rom[128], save[128], small[128], large[128];
} BandaiSavePaths;

static int save_paths(BandaiSavePaths *paths) {
    for (unsigned attempt = 0; attempt < 1000; ++attempt) {
        snprintf(paths->directory, sizeof(paths->directory), ".bandai-save-test-%llu-%u",
                 (unsigned long long)time(NULL), attempt);
#ifdef _WIN32
        int result = _mkdir(paths->directory);
#else
        int result = mkdir(paths->directory, 0700);
#endif
        if (result == 0) {
            snprintf(paths->rom, sizeof(paths->rom), "%s/cart.nes", paths->directory);
            snprintf(paths->save, sizeof(paths->save), "%s/cart.sav", paths->directory);
            snprintf(paths->small, sizeof(paths->small), "%s/cart.eeprom128", paths->directory);
            snprintf(paths->large, sizeof(paths->large), "%s/cart.eeprom256", paths->directory);
            return 0;
        }
        if (errno != EEXIST) return -1;
    }
    return -1;
}

static int write_image(const char *path, const iNESHeader *header) {
    size_t bytes;
    uint8_t *image = board_image(header, &bytes);
    if (!image) return -1;
    FILE *file = fopen(path, "wb");
    if (!file) { free(image); return -1; }
    size_t written = fwrite(image, 1, bytes, file);
    int closed = fclose(file);
    free(image);
    return written == bytes && closed == 0 ? 0 : -1;
}

static long file_length(const char *path) {
    FILE *file = fopen(path, "rb");
    if (!file) return -1;
    long bytes = fseek(file, 0, SEEK_END) == 0 ? ftell(file) : -1;
    fclose(file);
    return bytes;
}

static int file_byte(const char *path, long offset) {
    FILE *file = fopen(path, "rb");
    if (!file) return -1;
    int byte = fseek(file, offset, SEEK_SET) == 0 ? fgetc(file) : -1;
    fclose(file);
    return byte;
}

static int persistence_cases(const BandaiSavePaths *paths) {
    iNESHeader h = board_header(157, 0, false, 0x40000, 0, 1);
    CHECK(write_image(paths->rom, &h) == 0 && load_rom(paths->rom) == 0);
    serial_select(false);
    CHECK(eeprom02_address(0x26) == 0);
    CHECK(serial_send(0x96, false));
    serial_stop();
    serial_select(true);
    serial_start();
    CHECK(serial_send(0x26, true));
    CHECK(serial_send(0x69, true));
    serial_stop();
    cart_battery_flush();
    CHECK(file_length(paths->large) == 256 && file_length(paths->small) == 128);
    CHECK(file_byte(paths->large, 0x26) == 0x96 && file_byte(paths->small, 0x26) == 0x69);
    CHECK(file_length(paths->save) == -1);
    unload_rom();
    CHECK(load_rom(paths->rom) == 0);
    serial_select(false);
    CHECK(eeprom02_read(0x26, 0x96) == 0);
    serial_select(true);
    CHECK(eeprom01_read(0x26, 0x69) == 0);
    unload_rom();

    FILE *file = fopen(paths->large, "wb");
    CHECK(file != NULL);
    int written = fputc(0xC7, file);
    int closed = fclose(file);
    CHECK(written == 0xC7 && closed == 0);
    file = fopen(paths->save, "wb");
    CHECK(file != NULL);
    written = fputs("AB", file);
    closed = fclose(file);
    CHECK(written >= 0 && closed == 0);
    CHECK(load_rom(paths->rom) == 0);
    serial_select(false);
    CHECK(eeprom02_read(0, 0xC7) == 0);
    CHECK(eeprom02_read(0x26, 0) == 0);
    serial_select(true);
    CHECK(eeprom01_read(0x26, 0x69) == 0);
    cart_battery_flush();
    CHECK(file_length(paths->save) == 2 && file_byte(paths->save, 0) == 'A');
    unload_rom();

    h = board_header(153, 0, false, 0x80000, 0, 7);
    CHECK(write_image(paths->rom, &h) == 0 && load_rom(paths->rom) == 0);
    CHECK(cart_cpu_read(0x6000) == 'A' && cart_cpu_read(0x7FFF) == 0);
    cart_cpu_write(0x6000, 0xA6);
    cart_cpu_write(0x7FFF, 0x6A);
    unload_rom();
    CHECK(file_length(paths->save) == 0x2000);
    CHECK(file_byte(paths->save, 0) == 0xA6 && file_byte(paths->save, 0x1FFF) == 0x6A);
    CHECK(file_byte(paths->small, 0x26) == 0x69 && file_byte(paths->large, 0) == 0xC7);
    CHECK(load_rom(paths->rom) == 0);
    CHECK(cart_cpu_read(0x6000) == 0xA6 && cart_cpu_read(0x7FFF) == 0x6A);
    unload_rom();

    // Legacy mapper 159 supplies EEPROM hardware even without a battery flag.
    h = board_header(159, 0, true, 0x40000, 0x20000, 0);
    CHECK(write_image(paths->rom, &h) == 0 && load_rom(paths->rom) == 0);
    serial_select(false);
    serial_start();
    CHECK(serial_send(0x26, true));
    CHECK(serial_send(0x35, true));
    serial_stop();
    unload_rom();
    CHECK(file_byte(paths->small, 0x26) == 0x35);
    CHECK(file_length(paths->save) == 0x2000 && file_byte(paths->save, 0) == 0xA6);
    CHECK(load_rom(paths->rom) == 0);
    serial_select(false);
    CHECK(eeprom01_read(0x26, 0x35) == 0);
    return 0;
}

static int test_bandai_persistence(void) {
    BandaiSavePaths paths;
    CHECK(save_paths(&paths) == 0);
    int result = persistence_cases(&paths);
    unload_rom();
    const char *files[] = {paths.rom, paths.save, paths.small, paths.large};
    for (unsigned i = 0; i < sizeof(files) / sizeof(files[0]); ++i)
        if (remove(files[i]) != 0 && errno != ENOENT) result = 1;
#ifdef _WIN32
    if (_rmdir(paths.directory) != 0) result = 1;
#else
    if (rmdir(paths.directory) != 0) result = 1;
#endif
    return result;
}

int test_bandai_accuracy(void) {
    int (*tests[])(void) = {
        test_bandai_mapping, test_bandai_sram_and_outer_bank, test_bandai_irq,
        test_bandai_cpu_irq, test_bandai_eeprom02, test_bandai_eeprom01,
        test_datach_dual_eeprom, test_datach_barcode, test_bandai_rejected_loads,
        test_bandai_persistence
    };
    int failures = 0;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) failures += tests[i]();
    unload_rom();
    printf("Bandai accuracy: %zu groups, %d failures\n", sizeof(tests) / sizeof(tests[0]), failures);
    return failures;
}
