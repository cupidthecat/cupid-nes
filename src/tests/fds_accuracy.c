/*
 * fds_accuracy.c - Famicom Disk System hardware regressions
 *
 * Author: @frankischilling
 *
 * This file exercises the production FDS loader, mapped memory, media controls, disk and
 * timer IRQs, persistence, CPU integration, and wavetable/modulation audio behavior.
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

#include <errno.h>
#include <stdint.h>
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
#include "../rom/fds.h"
#include "../rom/mapper.h"
#include "../rom/rom.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../apu/apu.h"
#include "../system/timing.h"
#include "../system/vs_system.h"
#include "../../include/globals.h"

#define FDS_SIDE_SIZE 65500u
#define QD_SIDE_SIZE 65536u
#define FDS_INITIAL_GAP_BYTES (28300u / 8u)
#define FDS_FIRST_DATA_IRQ_CYCLES (50002u + 150u * (FDS_INITIAL_GAP_BYTES + 1u))

extern uint8_t ram[0x0800];

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static void make_bios(uint8_t bios[0x2000]) {
    memset(bios, 0xEA, 0x2000);
    bios[0x1FFA] = 0x00; bios[0x1FFB] = 0xE0;
    bios[0x1FFC] = 0x00; bios[0x1FFD] = 0xE0;
    bios[0x1FFE] = 0x00; bios[0x1FFF] = 0xE1;
}

static uint8_t *make_disk(size_t sides, bool headered, size_t *size) {
    size_t prefix = headered ? 16 : 0;
    *size = prefix + sides * FDS_SIDE_SIZE;
    uint8_t *disk = (uint8_t *)calloc(1, *size);
    if (!disk) return NULL;
    if (headered) {
        memcpy(disk, "FDS\x1A", 4);
        disk[4] = (uint8_t)sides;
    }
    for (size_t side = 0; side < sides; ++side) {
        uint8_t *raw = disk + prefix + side * FDS_SIDE_SIZE;
        raw[0] = 1;
        raw[1] = 0x2A;
        raw[55] = (uint8_t)(0x40 + side);
        raw[56] = 2;
        raw[57] = 0;
    }
    return disk;
}

static uint16_t test_crc_update(uint16_t crc, uint8_t value) {
    crc ^= value;
    for (unsigned bit = 0; bit < 8; ++bit) {
        bool carry = (crc & 1u) != 0;
        crc >>= 1;
        if (carry) crc ^= 0x8408;
    }
    return crc;
}

static void append_block_crc(uint8_t *block, size_t data_size) {
    uint16_t crc = 0;
    for (size_t i = 0; i < data_size; ++i) crc = test_crc_update(crc, block[i]);
    block[data_size] = (uint8_t)crc;
    block[data_size + 1] = (uint8_t)(crc >> 8);
}

static uint8_t *make_qd_disk(size_t sides, bool headered, bool valid_crc, size_t *size) {
    size_t prefix = headered ? 16 : 0;
    *size = prefix + sides * QD_SIDE_SIZE;
    uint8_t *disk = (uint8_t *)calloc(1, *size);
    if (!disk) return NULL;
    if (headered) {
        memcpy(disk, "FDS\x1A", 4);
        disk[4] = (uint8_t)sides;
    }
    for (size_t side = 0; side < sides; ++side) {
        uint8_t *raw = disk + prefix + side * QD_SIDE_SIZE;
        raw[0] = 1;
        raw[1] = 0x2A;
        raw[55] = (uint8_t)(0x40 + side);
        append_block_crc(raw, 56);
        raw[58] = 2;
        raw[59] = 0;
        append_block_crc(raw + 58, 2);
        if (!valid_crc) raw[57] ^= 0x80;
    }
    return disk;
}

static int load_fixture(size_t sides, bool headered, const char *path, bool protected_media) {
    uint8_t bios[0x2000];
    make_bios(bios);
    size_t disk_size;
    uint8_t *disk = make_disk(sides, headered, &disk_size);
    if (!disk) return -1;
    int result = load_fds_memory(disk, disk_size, bios, sizeof(bios), path, protected_media);
    free(disk);
    return result;
}

static uint8_t *make_nrom(size_t *size) {
    *size = sizeof(iNESHeader) + 0x4000 + 0x2000;
    uint8_t *image = (uint8_t *)calloc(1, *size);
    if (!image) return NULL;
    iNESHeader *h = (iNESHeader *)image;
    memcpy(h->signature, "NES\x1A", 4);
    h->prg_rom_chunks = 1;
    h->chr_rom_chunks = 1;
    memset(image + sizeof(*h), 0x5C, 0x4000);
    return image;
}

static uint8_t *make_unsupported_mapper(size_t *size) {
    uint8_t *image = make_nrom(size);
    if (image) {
        iNESHeader *header = (iNESHeader *)image;
        header->flags6 = 0xF0;
        header->flags7 = 0xF8;
        header->prg_ram_size = 0x0F; // Synthetic unsupported mapper 4095.
    }
    return image;
}

static int test_fds_loader_and_memory(void) {
    size_t nrom_size;
    uint8_t *nrom = make_nrom(&nrom_size);
    CHECK(nrom != NULL);
    ((iNESHeader *)nrom)->flags7 = 0x09; // A single VS cabinet also uses an NROM board.
    CHECK(nrom != NULL && load_rom_memory(nrom, nrom_size) == 0);
    CHECK(vs_enabled());
    free(nrom);
    Mapper *previous = cart;
    CHECK(cart_cpu_read(0x8000) == 0x5C);

    uint8_t bios[0x2000] = {0};
    size_t disk_size;
    uint8_t *disk = make_disk(2, true, &disk_size);
    CHECK(disk != NULL);
    CHECK(load_fds_memory(disk, disk_size, bios, sizeof(bios) - 1, NULL, false) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0x5C);
    CHECK(vs_enabled());
    CHECK(load_fds_memory(disk, disk_size - 1, bios, sizeof(bios), NULL, false) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0x5C);
    disk[4] = 3;
    CHECK(load_fds_memory(disk, disk_size, bios, sizeof(bios), NULL, false) == -1);
    CHECK(cart == previous);
    disk[4] = 2;

    for (size_t i = 0; i < sizeof(bios); ++i) bios[i] = (uint8_t)(i ^ 0xA5);
    CHECK(load_fds_memory(disk, disk_size, bios, sizeof(bios), NULL, false) == 0);
    CHECK(!vs_enabled());
    free(disk);
    CHECK(rom_is_fds() && fds_side_count() == 2 && fds_current_side() == 0);
    CHECK(cart_cpu_read(0xE000) == 0xA5 && cart_cpu_read(0xFFFF) == (uint8_t)(0x1FFF ^ 0xA5));
    cart_cpu_write(0x6123, 0x6C);
    cart_cpu_write(0xDFFF, 0x7D);
    CHECK(cart_cpu_read(0x6123) == 0x6C && cart_cpu_read(0xDFFF) == 0x7D);
    cart_ppu_write(0x0123, 0x9E);
    CHECK(cart_ppu_read(0x0123) == 0x9E);
    cart_cpu_write(0x4025, 0x08);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cart_cpu_write(0x4025, 0x00);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);

    CHECK(fds_insert_disk(1) && fds_current_side() == 1);
    CHECK(!fds_insert_disk(2));
    fds_eject_disk();
    CHECK(!fds_disk_inserted() && (cart_cpu_read_bus(0x4032, 0) & 7) == 7);
    CHECK(fds_insert_disk(0));
    fds_set_write_protected(true);
    CHECK(fds_write_protected() && (cart_cpu_read_bus(0x4032, 0) & 4));
    fds_set_write_protected(false);
    CHECK(!fds_write_protected());

    size_t qd_size;
    uint8_t *qd = make_qd_disk(1, false, true, &qd_size);
    CHECK(qd != NULL && qd_size == QD_SIDE_SIZE);
    CHECK(load_fds_memory(qd, qd_size, bios, sizeof(bios), NULL, false) == 0);
    CHECK(rom_is_fds() && fds_side_count() == 1);
    free(qd);
    qd = make_qd_disk(2, true, true, &qd_size);
    CHECK(qd != NULL && qd_size == 16 + 2 * QD_SIDE_SIZE);
    CHECK(load_fds_memory(qd, qd_size, bios, sizeof(bios), NULL, false) == 0);
    CHECK(fds_side_count() == 2 && fds_current_side() == 0);
    free(qd);

    // A fully parsed cartridge that fails mapper validation must leave FDS active.
    size_t unsupported_size;
    uint8_t *unsupported = make_unsupported_mapper(&unsupported_size);
    CHECK(unsupported != NULL);
    Mapper *fds_mapper = cart;
    CHECK(load_rom_memory(unsupported, unsupported_size) == -1);
    CHECK(rom_is_fds() && cart == fds_mapper && fds_side_count() == 2);
    free(unsupported);
    return 0;
}

static int test_fds_timer_irq(void) {
    CHECK(load_fixture(1, true, NULL, false) == 0);
    cart_cpu_write(0x4020, 2);
    cart_cpu_write(0x4021, 0);
    cart_cpu_write(0x4022, 2);
    CHECK(!fds_irq_pending());
    fds_clock_cpu(2);
    CHECK(!fds_irq_pending());
    fds_clock_cpu(1);
    CHECK(fds_irq_pending());
    CHECK(cart_cpu_read_bus(0x4030, 0) & 1);
    CHECK(!fds_irq_pending());
    fds_clock_cpu(4);
    CHECK(!fds_irq_pending()); // One-shot timer disabled itself.

    cart_cpu_write(0x4020, 1);
    cart_cpu_write(0x4022, 3);
    fds_clock_cpu(2);
    CHECK(fds_irq_pending());
    (void)cart_cpu_read_bus(0x4030, 0);
    fds_clock_cpu(2);
    CHECK(fds_irq_pending());
    cart_cpu_write(0x4023, 0);
    CHECK(!fds_irq_pending());
    cart_cpu_write(0x4022, 3);
    fds_clock_cpu(4);
    CHECK(!fds_irq_pending());

    cart_cpu_write(0x4023, 3);
    cart_cpu_write(0x4025, 0x08); // Horizontal mirroring survives console reset.
    cart_cpu_write(0x4020, 1);
    cart_cpu_write(0x4022, 3);
    fds_clock_cpu(2);
    CHECK(fds_irq_pending());
    cart->reset();
    CHECK(fds_irq_pending() && cart_get_mirroring() == MIRROR_HORIZONTAL);
    (void)cart_cpu_read_bus(0x4030, 0);
    fds_clock_cpu(2);
    CHECK(fds_irq_pending());
    return 0;
}

static bool wait_for_disk_irq(unsigned max_cycles) {
    for (unsigned cycle = 0; cycle < max_cycles; ++cycle) {
        fds_clock_cpu(1);
        if (fds_irq_pending()) return true;
    }
    return false;
}

static bool next_disk_transfer(uint8_t *value) {
    fds_clock_cpu(149);
    if (fds_irq_pending()) return false;
    fds_clock_cpu(1);
    if (!fds_irq_pending()) return false;
    *value = cart_cpu_read_bus(0x4031, 0);
    return true;
}

static bool seek_first_block_with_crc_reset(void) {
    // Scan the initial gap with disk-ready clear so the sync marker resets CRC.
    cart_cpu_write(0x4025, 0x85);
    fds_clock_cpu(50002u + 150u * FDS_INITIAL_GAP_BYTES);
    cart_cpu_write(0x4025, 0xC5);
    // With ready asserted after the sync marker, the first block byte ends the gap.
    // That transfer completes without an IRQ; following bytes use the normal IRQ cadence.
    fds_clock_cpu(150);
    if (fds_irq_pending()) return false;
    return cart_cpu_read_bus(0x4031, 0) == 1;
}

static int test_fds_disk_transfer(void) {
    CHECK(load_fixture(1, true, NULL, false) == 0);
    // Scan + motor on + read mode + ready + transfer IRQ.
    cart_cpu_write(0x4025, 0xC5);
    fds_clock_cpu(FDS_FIRST_DATA_IRQ_CYCLES - 1u);
    CHECK(!fds_irq_pending());
    fds_clock_cpu(1);
    CHECK(fds_irq_pending());
    CHECK(cart_cpu_read_bus(0x4033, 0) & 0x80);
    fds_eject_disk();
    CHECK(!fds_disk_inserted());
    CHECK(cart_cpu_read_bus(0x4033, 0) & 0x80);
    CHECK(fds_irq_pending());
    fds_clock_cpu(1);
    CHECK(cart_cpu_read_bus(0x4033, 0) & 0x80);
    CHECK(fds_irq_pending());
    (void)cart_cpu_read_bus(0x4031, 0);
    CHECK(!fds_irq_pending());
    CHECK(fds_insert_disk(0));

    // Restart the transfer from the beginning for the byte-cadence checks below.
    cart_cpu_write(0x4025, 0xC5);
    fds_clock_cpu(FDS_FIRST_DATA_IRQ_CYCLES - 1u);
    CHECK(!fds_irq_pending());
    fds_clock_cpu(1);
    CHECK(fds_irq_pending());
    uint8_t value = cart_cpu_read_bus(0x4031, 0);
    CHECK(value == 1);
    CHECK(!fds_irq_pending());
    CHECK((cart_cpu_read_bus(0x4030, 0) & 0x80) == 0);
    fds_clock_cpu(149);
    CHECK(!fds_irq_pending());
    fds_clock_cpu(1);
    CHECK(fds_irq_pending());
    CHECK(cart_cpu_read_bus(0x4031, 0) == 0x2A);

    // A control write acknowledges a pending transfer IRQ without consuming data.
    fds_clock_cpu(150);
    CHECK(fds_irq_pending());
    cart_cpu_write(0x4025, 0xC5);
    CHECK(!fds_irq_pending());

    // FDS files use synthetic drive CRC bytes and never expose CRC-error status.
    CHECK(load_fixture(1, true, NULL, false) == 0);
    CHECK(seek_first_block_with_crc_reset());
    uint8_t transfer;
    for (unsigned byte = 0; byte < 56; ++byte) CHECK(next_disk_transfer(&transfer));
    cart_cpu_write(0x4025, 0xD5);
    CHECK(next_disk_transfer(&transfer));
    CHECK((cart_cpu_read_bus(0x4030, 0) & 0x10) == 0);

    uint8_t bios[0x2000];
    make_bios(bios);
    size_t qd_size;
    uint8_t *qd = make_qd_disk(1, true, true, &qd_size);
    CHECK(qd != NULL && load_fds_memory(qd, qd_size, bios, sizeof(bios), NULL, false) == 0);
    CHECK(seek_first_block_with_crc_reset());
    for (unsigned byte = 0; byte < 56; ++byte) CHECK(next_disk_transfer(&transfer));
    cart_cpu_write(0x4025, 0xD5);
    CHECK(next_disk_transfer(&transfer));
    CHECK((cart_cpu_read_bus(0x4030, 0) & 0x10) == 0);
    free(qd);

    qd = make_qd_disk(1, true, false, &qd_size);
    CHECK(qd != NULL && load_fds_memory(qd, qd_size, bios, sizeof(bios), NULL, false) == 0);
    CHECK(seek_first_block_with_crc_reset());
    for (unsigned byte = 0; byte < 56; ++byte) CHECK(next_disk_transfer(&transfer));
    cart_cpu_write(0x4025, 0xD5);
    CHECK(next_disk_transfer(&transfer));
    CHECK(cart_cpu_read_bus(0x4030, 0) & 0x10);
    free(qd);

    // Clearing disk-ready resets the CRC accumulator and error state on transfer.
    cart_cpu_write(0x4025, 0x85);
    for (unsigned cycle = 0; cycle < 1000; ++cycle) fds_clock_cpu(1);
    CHECK((cart_cpu_read_bus(0x4030, 0) & 0x10) == 0);

    fds_eject_disk();
    fds_clock_cpu(1000);
    CHECK(!fds_irq_pending());
    CHECK((cart_cpu_read_bus(0x4032, 0) & 3) == 3);

    // Reaching the physical end of a side stops the motor and returns to not-ready.
    CHECK(fds_insert_disk(0));
    cart_cpu_write(0x4025, 0xC5);
    fds_clock_cpu(11000000);
    CHECK((cart_cpu_read_bus(0x4033, 0) & 0x80) == 0);
    CHECK(cart_cpu_read_bus(0x4032, 0) & 0x02);
    return 0;
}

typedef struct {
    char directory[96];
    char disk[128];
    char bios[128];
} FdsTemp;

static int temp_begin(FdsTemp *paths) {
    for (unsigned attempt = 0; attempt < 1000; ++attempt) {
        snprintf(paths->directory, sizeof(paths->directory), ".fds-test-%llu-%u",
                 (unsigned long long)time(NULL), attempt);
#ifdef _WIN32
        int result = _mkdir(paths->directory);
#else
        int result = mkdir(paths->directory, 0700);
#endif
        if (result == 0) {
            snprintf(paths->disk, sizeof(paths->disk), "%s/disk.fds", paths->directory);
            snprintf(paths->bios, sizeof(paths->bios), "%s/bios.bin", paths->directory);
            return 0;
        }
        if (errno != EEXIST) return -1;
    }
    return -1;
}

static int temp_end(const FdsTemp *paths) {
    unload_rom();
    int result = 0;
    if (remove(paths->disk) != 0 && errno != ENOENT) result = 1;
    if (remove(paths->bios) != 0 && errno != ENOENT) result = 1;
#ifdef _WIN32
    if (_rmdir(paths->directory) != 0) result = 1;
#else
    if (rmdir(paths->directory) != 0) result = 1;
#endif
    return result;
}

static int write_disk_file(const char *path, const uint8_t *disk, size_t size) {
    FILE *fp = fopen(path, "wb");
    if (!fp) return -1;
    size_t written = fwrite(disk, 1, size, fp);
    int closed = fclose(fp);
    return written == size && closed == 0 ? 0 : -1;
}

static int read_file_byte(const char *path, long offset) {
    FILE *fp = fopen(path, "rb");
    if (!fp) return -1;
    int value = fseek(fp, offset, SEEK_SET) == 0 ? fgetc(fp) : -1;
    fclose(fp);
    return value;
}

static int read_file_exact(const char *path, uint8_t *data, size_t size) {
    FILE *fp = fopen(path, "rb");
    if (!fp) return -1;
    size_t count = fread(data, 1, size, fp);
    int closed = fclose(fp);
    return count == size && closed == 0 ? 0 : -1;
}

static int test_fds_persistence(void) {
    FdsTemp paths;
    CHECK(temp_begin(&paths) == 0);
    uint8_t bios[0x2000];
    make_bios(bios);
    size_t disk_size;
    uint8_t *disk = make_disk(2, true, &disk_size);
    CHECK(disk != NULL && write_disk_file(paths.disk, disk, disk_size) == 0);
    CHECK(write_disk_file(paths.bios, bios, sizeof(bios)) == 0);

    // The normal file loader reads both user-supplied media and BIOS before activation.
    CHECK(load_fds(paths.disk, paths.bios, false) == 0);
    CHECK(rom_is_fds() && fds_side_count() == 2 && cart_cpu_read(0xE000) == bios[0]);

    // Read the sync marker and block type first, then overwrite the following byte.
    cart_cpu_write(0x4025, 0xC5);
    CHECK(wait_for_disk_irq(700000));
    CHECK(cart_cpu_read_bus(0x4031, 0) == 1);
    cart_cpu_write(0x4024, 0xA5);
    cart_cpu_write(0x4025, 0x41);
    fds_clock_cpu(149);
    CHECK(!fds_disk_dirty());
    fds_clock_cpu(1);
    CHECK(fds_disk_dirty());
    CHECK(fds_flush() && !fds_disk_dirty());
    CHECK(read_file_byte(paths.disk, 17) == 0xA5);

    // Rebuilding a modified side must leave every unmodified side byte-for-byte exact.
    uint8_t *persisted = (uint8_t *)malloc(disk_size);
    CHECK(persisted != NULL && read_file_exact(paths.disk, persisted, disk_size) == 0);
    CHECK(memcmp(persisted + 16 + FDS_SIDE_SIZE,
                 disk + 16 + FDS_SIDE_SIZE, FDS_SIDE_SIZE) == 0);
    memcpy(disk, persisted, disk_size);
    free(persisted);
    CHECK(load_fds_memory(disk, disk_size, bios, sizeof(bios), paths.disk, false) == 0);
    CHECK(disk[17] == 0xA5);

    // Protected media ignores writes and remains byte-for-byte unchanged.
    CHECK(load_fds_memory(disk, disk_size, bios, sizeof(bios), paths.disk, true) == 0);
    cart_cpu_write(0x4025, 0xC5);
    CHECK(wait_for_disk_irq(700000));
    CHECK(cart_cpu_read_bus(0x4031, 0) == 1);
    cart_cpu_write(0x4024, 0x11);
    cart_cpu_write(0x4025, 0x41);
    fds_clock_cpu(150);
    CHECK(!fds_disk_dirty() && read_file_byte(paths.disk, 17) == 0xA5);

    // Exclusive temp creation must preserve a stale temp file and the dirty active disk.
    CHECK(load_fds_memory(disk, disk_size, bios, sizeof(bios), paths.disk, false) == 0);
    cart_cpu_write(0x4025, 0xC5);
    CHECK(wait_for_disk_irq(700000));
    CHECK(cart_cpu_read_bus(0x4031, 0) == 1);
    cart_cpu_write(0x4024, 0x33);
    cart_cpu_write(0x4025, 0x41);
    fds_clock_cpu(150);
    CHECK(fds_disk_dirty());
    char stale_temp[160];
    snprintf(stale_temp, sizeof(stale_temp), "%s.cupid-fds.tmp", paths.disk);
    const uint8_t sentinel = 0x7B;
    CHECK(write_disk_file(stale_temp, &sentinel, 1) == 0);
    CHECK(!fds_flush() && fds_disk_dirty() && read_file_byte(stale_temp, 0) == sentinel);

    // Unload is transactional too: failed persistence keeps the active machine intact.
    CHECK(!unload_rom());
    CHECK(rom_is_fds() && fds_disk_dirty() && cart_cpu_read(0xE000) == bios[0]);

    // A replacement load must fail without discarding the dirty FDS machine.
    size_t nrom_size;
    uint8_t *nrom = make_nrom(&nrom_size);
    CHECK(nrom != NULL);
    CHECK(load_rom_memory(nrom, nrom_size) == -1);
    CHECK(rom_is_fds() && fds_disk_dirty() && cart_cpu_read(0xE000) == bios[0]);
    CHECK(read_file_byte(stale_temp, 0) == sentinel);
    CHECK(remove(stale_temp) == 0);
    CHECK(fds_flush() && !fds_disk_dirty());
    CHECK(load_rom_memory(nrom, nrom_size) == 0);
    free(nrom);
    free(disk);
    return temp_end(&paths);
}

static int test_fds_audio(void) {
    CHECK(load_fixture(1, false, NULL, false) == 0);
    cart_cpu_write(0x4089, 0x80);
    cart_cpu_write(0x4040, 0x3F);
    CHECK((cart_cpu_read_bus(0x4040, 0) & 0x3F) == 0x3F);
    cart_cpu_write(0x4089, 0x00);
    cart_cpu_write(0x4080, 0x9F); // Fixed volume gain 31.
    cart_cpu_write(0x4082, 0xFF);
    cart_cpu_write(0x4083, 0x00);
    fds_clock_cpu(1);
    CHECK((cart_cpu_read_bus(0x4090, 0) & 0x3F) == 31);
    CHECK(fds_expansion_audio() < 0.0f);

    cart_cpu_write(0x4087, 0x80); // Disable modulator before table writes.
    for (unsigned i = 0; i < 32; ++i) cart_cpu_write(0x4088, 3);
    CHECK((cart_cpu_read_bus(0x4095, 0) & 0x0F) == 4);
    cart_cpu_write(0x4085, 0x7F);
    CHECK((cart_cpu_read_bus(0x4097, 0) & 0x7F) == 0x7F);

    // Envelope period is deterministic and CPU-clocked.
    cart_cpu_write(0x408A, 1);
    cart_cpu_write(0x4080, 0x80); // Fix gain at zero before enabling the envelope.
    cart_cpu_write(0x4080, 0x40); // Speed 0, increase, envelope enabled.
    fds_clock_cpu(15);
    CHECK((cart_cpu_read_bus(0x4090, 0) & 0x3F) == 0);
    fds_clock_cpu(1);
    CHECK((cart_cpu_read_bus(0x4090, 0) & 0x3F) == 1);

    // Debug accumulators expose the same internal bits used by the hardware counters.
    CHECK(load_fixture(1, false, NULL, false) == 0);
    cart_cpu_write(0x4087, 0x80);
    cart_cpu_write(0x4082, 0x01);
    cart_cpu_write(0x4083, 0x00);
    fds_clock_cpu(4096);
    CHECK(cart_cpu_read_bus(0x4091, 0) == 0x01);

    cart_cpu_write(0x4086, 0x01);
    cart_cpu_write(0x4087, 0x00);
    fds_clock_cpu(32);
    CHECK((cart_cpu_read_bus(0x4093, 0) & 0x7F) == 0x01);

    // Audio reset clears the modulation counter and cached pitch contribution.
    CHECK(load_fixture(1, false, NULL, false) == 0);
    cart_cpu_write(0x4082, 0x00);
    cart_cpu_write(0x4083, 0x04); // Wave pitch = $400.
    cart_cpu_write(0x4084, 0xA0); // Fixed modulation gain = 32.
    cart_cpu_write(0x4085, 0x20); // Counter = 32, yielding +$400 modulation.
    cart_cpu_write(0x4087, 0x80); // Freeze the modulator with its current output.
    cart_cpu_write(0x4023, 0x01); // Keep disk registers enabled, reset audio registers.
    CHECK((cart_cpu_read_bus(0x4097, 0) & 0x7F) == 0);
    fds_clock_cpu(4);
    CHECK(cart_cpu_read_bus(0x4091, 0) == 0x01);
    return 0;
}

static int test_fds_cpu_bios_irq(void) {
    uint8_t bios[0x2000];
    make_bios(bios);
    const uint8_t program[] = {
        0xA9, 0x5A,       // LDA #$5A
        0x8D, 0x00, 0x60, // STA $6000
        0xA9, 0x02,       // LDA #$02
        0x8D, 0x20, 0x40, // STA $4020
        0xA9, 0x00,       // LDA #$00
        0x8D, 0x21, 0x40, // STA $4021
        0xA9, 0x02,       // LDA #$02
        0x8D, 0x22, 0x40, // STA $4022
        0x58,             // CLI
        0xEA,             // NOP
        0x4C, 0x16, 0xE0  // JMP $E016
    };
    memcpy(bios, program, sizeof(program));
    const uint8_t handler[] = {
        0xAD, 0x30, 0x40, // LDA $4030 (ack timer IRQ)
        0xEE, 0x01, 0x60, // INC $6001
        0x40              // RTI
    };
    memcpy(bios + 0x100, handler, sizeof(handler));

    size_t disk_size;
    uint8_t *disk = make_disk(1, true, &disk_size);
    CHECK(disk != NULL && load_fds_memory(disk, disk_size, bios, sizeof(bios), NULL, false) == 0);
    free(disk);

    memset(ram, 0, 0x800);
    memset(&cpu, 0, sizeof(cpu));
    memset(&pad1, 0, sizeof(pad1));
    memset(&pad2, 0, sizeof(pad2));
    nes_set_region(NES_REGION_NTSC);
    cpu_total_cycles = 0;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    for (unsigned step = 0; step < 100 && cart_cpu_read(0x6001) == 0; ++step)
        CHECK(cpu_step(&cpu) > 0);
    CHECK(cart_cpu_read(0x6000) == 0x5A);
    CHECK(cart_cpu_read(0x6001) == 1);
    CHECK(!fds_irq_pending());
    return 0;
}

int test_fds_accuracy(void) {
    static int (*const tests[])(void) = {
        test_fds_loader_and_memory,
        test_fds_timer_irq,
        test_fds_disk_transfer,
        test_fds_persistence,
        test_fds_audio,
        test_fds_cpu_bios_irq
    };
    int failures = 0;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) failures += tests[i]();
    unload_rom();
    printf("FDS accuracy: %zu groups, %d failures\n", sizeof(tests) / sizeof(tests[0]), failures);
    return failures;
}
