/*
 * fds_options_accuracy.c - Disk overlays, source retention, and reloads
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License, version 3 or later.
 * This program is distributed without any warranty.
 */
#include "board_tests.h"
#include "../media/image_source.h"
#include "../media/patch.h"
#include "../rom/fds.h"
#include "../rom/game_db.h"
#include "../third_party/miniz/miniz.h"
#include "../util/file_io.h"
#include <sys/stat.h>
#ifdef _WIN32
#include <direct.h>
#include <io.h>
#include <process.h>
#else
#include <unistd.h>
#endif

static int directory_create(const char *path) {
#ifdef _WIN32
    return _mkdir(path);
#else
    return mkdir(path, 0700);
#endif
}

static int directory_remove(const char *path) {
#ifdef _WIN32
    return _rmdir(path);
#else
    return rmdir(path);
#endif
}

static int file_read_only(const char *path, bool read_only) {
#ifdef _WIN32
    return _chmod(path, read_only ? _S_IREAD : _S_IREAD | _S_IWRITE);
#else
    return chmod(path, read_only ? 0400 : 0600);
#endif
}

static void firmware(uint8_t bios[0x2000]) {
    memset(bios, 0xEA, 0x2000);
    for (unsigned vector = 0x1FFA; vector <= 0x1FFE; vector += 2) {
        bios[vector] = 0;
        bios[vector + 1] = 0xE0;
    }
}

static uint8_t *disk_image(bool headered, bool qd, size_t *size) {
    size_t prefix = headered ? 16 : 0;
    size_t capacity = qd ? 65536 : 65500;
    *size = prefix + 2 * capacity;
    uint8_t *disk = (uint8_t *)calloc(*size, 1);
    if (!disk) return NULL;
    if (headered) {
        memcpy(disk, "FDS\x1A", 4);
        disk[4] = 2;
    }

    for (size_t side = 0; side < 2; side++) {
        uint8_t *raw = disk + prefix + side * capacity;
        raw[0] = 1;
        raw[1] = 0x2A;
        raw[55] = (uint8_t)(0x40 + side);
        raw[qd ? 58 : 56] = 2;
    }

    return disk;
}

static bool wait_transfer(void) {
    for (unsigned cycle = 0; cycle < 700000; cycle++) {
        if (fds_irq_pending()) return true;
        fds_clock_cpu(1);
    }

    return false;
}

static bool write_header_byte(size_t side, uint8_t value) {
    if (!fds_insert_disk(side)) return false;
    cart_cpu_write(0x4025, 0xC5);
    if (!wait_transfer() || cart_cpu_read_bus(0x4031, 0) != 1) return false;
    cart_cpu_write(0x4024, value);
    cart_cpu_write(0x4025, 0x41);
    fds_clock_cpu(150);
    return true;
}

static int read_header_byte(size_t side) {
    if (!fds_insert_disk(side)) return -1;
    cart_cpu_write(0x4025, 0xC5);
    if (!wait_transfer() || cart_cpu_read_bus(0x4031, 0) != 1) return -1;
    if (!wait_transfer()) return -1;
    return cart_cpu_read_bus(0x4031, 0);
}

static int file_matches(const char *path, const uint8_t *expected, size_t expected_size) {
    uint8_t *data = NULL;
    size_t size = 0;
    BOARD_CHECK(nes_file_read_all(path, expected_size, &data, &size) == NES_FILE_OK);
    bool matches = size == expected_size && !memcmp(data, expected, size);
    free(data);
    BOARD_CHECK(matches);
    return 0;
}

static int check_overlay_geometry(const char *directory, bool headered, bool qd, unsigned id) {
    char disk_path[256], bios_path[256], overlay_path[512];
    snprintf(disk_path, sizeof(disk_path), "%s/disk-%u.fds", directory, id);
    snprintf(bios_path, sizeof(bios_path), "%s/bios-%u.bin", directory, id);
    size_t size = 0;
    uint8_t *disk = disk_image(headered, qd, &size);
    uint8_t bios[0x2000];
    firmware(bios);
    BOARD_CHECK(disk && nes_file_write_atomic(disk_path, disk, size) == NES_FILE_OK);
    BOARD_CHECK(nes_file_write_atomic(bios_path, bios, sizeof(bios)) == NES_FILE_OK);
    FdsLoadOptions options = {FDS_SAVE_OVERLAY, NULL, false};
    BOARD_CHECK(load_fds_with_options(disk_path, bios_path, &options) == 0);
    BOARD_CHECK(fds_save_mode() == FDS_SAVE_OVERLAY && fds_save_path());
    BOARD_CHECK(strlen(fds_save_path()) < sizeof(overlay_path));
    strcpy(overlay_path, fds_save_path());
    BOARD_CHECK(strcmp(disk_path, overlay_path));
    BOARD_CHECK(write_header_byte(1, 0xA5) && fds_disk_dirty());
    BOARD_CHECK(fds_flush() && !fds_disk_dirty());
    BOARD_CHECK(file_matches(disk_path, disk, size) == 0);

    uint8_t *patch = NULL, *restored = NULL;
    size_t patch_size = 0, restored_size = 0;
    BOARD_CHECK(nes_file_read_all(overlay_path, 32u * 1024u * 1024u, &patch, &patch_size) == NES_FILE_OK);
    BOARD_CHECK(patch_size >= 8 && !memcmp(patch, "PATCH", 5));
    BOARD_CHECK(nes_patch_apply(disk, size, patch, patch_size, size, &restored, &restored_size) == NES_PATCH_OK);
    free(patch);
    BOARD_CHECK(restored_size == size);
    size_t changed = (headered ? 16u : 0u) + (qd ? 65536u : 65500u) + 1u;
    for (size_t i = 0; i < size; i++) {
        BOARD_CHECK(restored[i] == (i == changed ? 0xA5 : disk[i]));
    }

    free(restored);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_fds_with_options(disk_path, bios_path, &options) == 0);
    BOARD_CHECK(read_header_byte(0) == 0x2A && read_header_byte(1) == 0xA5);

    // Reload performs its persistence check before reading the current overlay.
    BOARD_CHECK(write_header_byte(0, 0x5A) && fds_disk_dirty());
    BOARD_CHECK(load_fds_with_options(disk_path, bios_path, &options) == 0);
    BOARD_CHECK(!fds_disk_dirty() && read_header_byte(0) == 0x5A && read_header_byte(1) == 0xA5);
    BOARD_CHECK(file_matches(disk_path, disk, size) == 0);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(nes_file_remove(overlay_path) == NES_FILE_OK);
    BOARD_CHECK(nes_file_remove(disk_path) == NES_FILE_OK);
    BOARD_CHECK(nes_file_remove(bios_path) == NES_FILE_OK);
    free(disk);
    return 0;
}

static int test_overlay_geometry(const char *directory) {
    return check_overlay_geometry(directory, true, false, 0)
         + check_overlay_geometry(directory, false, false, 1)
         + check_overlay_geometry(directory, true, true, 2)
         + check_overlay_geometry(directory, false, true, 3);
}

static int test_overlay_failure_and_protection(const char *directory) {
    char disk_path[256], overlay_path[256], blocker[280], alias[280];
    snprintf(disk_path, sizeof(disk_path), "%s/retained.fds", directory);
    snprintf(overlay_path, sizeof(overlay_path), "%s/retained.ips", directory);
    snprintf(blocker, sizeof(blocker), "%s/kept.bin", overlay_path);
    snprintf(alias, sizeof(alias), "./%s", disk_path);
    uint8_t bios[0x2000];
    firmware(bios);
    size_t size = 0;
    uint8_t *disk = disk_image(true, false, &size);
    BOARD_CHECK(disk && nes_file_write_atomic(disk_path, disk, size) == NES_FILE_OK);

    FdsLoadOptions options = {FDS_SAVE_OVERLAY, disk_path, false};
    BOARD_CHECK(load_fds_memory_options(disk, size, bios, sizeof(bios), disk_path, &options) == -1);
    options.overlay_path = alias;
    BOARD_CHECK(load_fds_memory_options(disk, size, bios, sizeof(bios), disk_path, &options) == -1);
    options.overlay_path = overlay_path;
    BOARD_CHECK(load_fds_memory_options(disk, size, bios, sizeof(bios), disk_path, &options) == 0);
    BOARD_CHECK(file_read_only(disk_path, true) == 0);
    cart_cpu_write(0x6123, 0x76);
    BOARD_CHECK(write_header_byte(0, 0x99) && fds_disk_dirty());
    BOARD_CHECK(directory_create(overlay_path) == 0);
    BOARD_CHECK(nes_file_write_atomic(blocker, "retained", 8) == NES_FILE_OK);
    BOARD_CHECK(!fds_flush() && fds_disk_dirty());
    BOARD_CHECK(!unload_rom() && rom_is_fds() && cart_cpu_read(0x6123) == 0x76);
    BoardImage replacement;
    BOARD_CHECK(board_image_create(&replacement, 0, 0x4000, 0x2000, true));
    BOARD_CHECK(load_rom_memory(replacement.data, replacement.size) == -1);
    BOARD_CHECK(rom_is_fds() && fds_disk_dirty() && cart_cpu_read(0x6123) == 0x76);
    board_image_free(&replacement);
    BOARD_CHECK(file_matches(disk_path, disk, size) == 0);
    BOARD_CHECK(file_matches(blocker, (const uint8_t *)"retained", 8) == 0);
    BOARD_CHECK(nes_file_remove(blocker) == NES_FILE_OK);
    BOARD_CHECK(directory_remove(overlay_path) == 0);
    BOARD_CHECK(fds_flush() && !fds_disk_dirty());
    BOARD_CHECK(unload_rom());

    options.write_protected = true;
    BOARD_CHECK(load_fds_memory_options(disk, size, bios, sizeof(bios), disk_path, &options) == 0);
    BOARD_CHECK(fds_write_protected() && read_header_byte(0) == 0x99);
    BOARD_CHECK(write_header_byte(0, 0x42) && !fds_disk_dirty());
    BOARD_CHECK(read_header_byte(0) == 0x99);
    BOARD_CHECK(file_matches(disk_path, disk, size) == 0);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(file_read_only(disk_path, false) == 0);
    BOARD_CHECK(nes_file_remove(overlay_path) == NES_FILE_OK);
    BOARD_CHECK(nes_file_remove(disk_path) == NES_FILE_OK);
    free(disk);
    return 0;
}

static int test_rejected_overlays(const char *directory) {
    char disk_path[256], overlay_path[256];
    snprintf(disk_path, sizeof(disk_path), "%s/invalid.fds", directory);
    snprintf(overlay_path, sizeof(overlay_path), "%s/invalid.ips", directory);
    uint8_t bios[0x2000];
    firmware(bios);
    size_t size = 0;
    uint8_t *disk = disk_image(true, false, &size);
    BOARD_CHECK(disk && nes_file_write_atomic(disk_path, disk, size) == NES_FILE_OK);
    BoardImage current;
    BOARD_CHECK(board_image_create(&current, 0, 0x4000, 0x2000, true));
    BOARD_CHECK(load_rom_memory(current.data, current.size) == 0);
    uint8_t *original_rom = prg_rom;
    uint32_t original_crc = rom_file_crc32();
    const uint8_t changed_header[] = {'P','A','T','C','H', 0,0,4, 0,1, 3, 'E','O','F'};
    const uint8_t truncated_size[] = {'P','A','T','C','H','E','O','F', 0,0,1};
    const uint8_t incomplete[] = {'P','A','T','C','H',0,0};
    const uint8_t *patches[] = {changed_header, truncated_size, incomplete};
    const size_t sizes[] = {sizeof(changed_header), sizeof(truncated_size), sizeof(incomplete)};
    FdsLoadOptions options = {FDS_SAVE_OVERLAY, overlay_path, false};
    for (size_t i = 0; i < sizeof(patches) / sizeof(patches[0]); i++) {
        BOARD_CHECK(nes_file_write_atomic(overlay_path, patches[i], sizes[i]) == NES_FILE_OK);
        BOARD_CHECK(load_fds_memory_options(disk, size, bios, sizeof(bios), disk_path, &options) == -1);
        BOARD_CHECK(prg_rom == original_rom && rom_file_crc32() == original_crc && !rom_is_fds());
    }

    BOARD_CHECK(file_matches(disk_path, disk, size) == 0);
    BOARD_CHECK(unload_rom());
    board_image_free(&current);
    BOARD_CHECK(nes_file_remove(overlay_path) == NES_FILE_OK);
    BOARD_CHECK(nes_file_remove(disk_path) == NES_FILE_OK);
    free(disk);
    return 0;
}

static int test_archived_disk_overlay(const char *directory) {
    char archive_path[256], bios_path[256], overlay_path[512], error[256];
    snprintf(archive_path, sizeof(archive_path), "%s/disk.zip", directory);
    snprintf(bios_path, sizeof(bios_path), "%s/disk-bios.bin", directory);
    uint8_t bios[0x2000];
    firmware(bios);
    BOARD_CHECK(nes_file_write_atomic(bios_path, bios, sizeof(bios)) == NES_FILE_OK);
    size_t disk_size = 0;
    uint8_t *disk = disk_image(true, false, &disk_size);
    BOARD_CHECK(disk);
    mz_zip_archive zip = {0};
    BOARD_CHECK(mz_zip_writer_init_heap(&zip, 0, 0));
    BOARD_CHECK(mz_zip_writer_add_mem(&zip, "game.fds", disk, disk_size, 9));
    void *archive = NULL;
    size_t archive_size = 0;
    BOARD_CHECK(mz_zip_writer_finalize_heap_archive(&zip, &archive, &archive_size));
    BOARD_CHECK(mz_zip_writer_end(&zip));
    BOARD_CHECK(nes_file_write_atomic(archive_path, archive, archive_size) == NES_FILE_OK);
    BOARD_CHECK(file_read_only(archive_path, true) == 0);
    NesImageRequest request = {archive_path, NULL, NULL};
    NesImageSource source = {0};
    BOARD_CHECK(nes_image_prepare(&request, &source, error, sizeof(error)) == NES_MEDIA_OK);
    BOARD_CHECK(source.archived && !source.patched);
    BOARD_CHECK(nes_image_load(&source, bios_path, NULL, false, error, sizeof(error)) == NES_MEDIA_OK);
    BOARD_CHECK(rom_is_fds() && fds_save_mode() == FDS_SAVE_OVERLAY);
    BOARD_CHECK(fds_save_path() && strlen(fds_save_path()) < sizeof(overlay_path));
    strcpy(overlay_path, fds_save_path());
    BOARD_CHECK(write_header_byte(0, 0x93) && fds_disk_dirty());
    BOARD_CHECK(nes_image_load(&source, bios_path, NULL, false, error, sizeof(error)) == NES_MEDIA_OK);
    BOARD_CHECK(read_header_byte(0) == 0x93 && !fds_disk_dirty());
    BOARD_CHECK(file_matches(archive_path, (const uint8_t *)archive, archive_size) == 0);
    BOARD_CHECK(unload_rom());
    nes_image_source_free(&source);
    BOARD_CHECK(file_read_only(archive_path, false) == 0);
    BOARD_CHECK(nes_file_remove(overlay_path) == NES_FILE_OK);
    BOARD_CHECK(nes_file_remove(archive_path) == NES_FILE_OK);
    BOARD_CHECK(nes_file_remove(bios_path) == NES_FILE_OK);
    mz_free(archive);
    free(disk);
    return 0;
}

int test_fds_options_accuracy(void) {
    char directory[128];
#ifdef _WIN32
    unsigned process = (unsigned)_getpid();
#else
    unsigned process = (unsigned)getpid();
#endif
    BOARD_CHECK(snprintf(directory, sizeof(directory), "build/fds-options-%u", process) > 0);
    BOARD_CHECK(directory_create(directory) == 0);
    int failures = test_overlay_geometry(directory) + test_overlay_failure_and_protection(directory)
                 + test_rejected_overlays(directory) + test_archived_disk_overlay(directory);
    if (directory_remove(directory) != 0) ++failures;
    printf("FDS save options: 4 groups, %d failures\n", failures);
    return failures;
}
