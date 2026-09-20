/*
 * media_accuracy.c - Archive selection, image patches, and save identities
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License, version 3 or later.
 * This program is distributed without any warranty.
 */
#include "board_tests.h"
#include "media_fixtures.h"
#include "../media/image_source.h"
#include "../rom/game_db.h"
#include "../system/timing.h"
#include "../third_party/miniz/miniz.h"
#include "../util/file_io.h"
#ifdef _WIN32
#include <direct.h>
#include <process.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

static const char unicode_member[] = "caf\xC3\xA9/\xE3\x82\xB2\xE3\x83\xBC\xE3\x83\xA0\xF0\x9F\x98\x80.nes";
static const uint8_t change_immediate[] = {'P','A','T','C','H', 0,0,17, 0,1, 0x7E, 'E','O','F'};

static int media_path(char path[512], const char *directory, const char *name) {
    int size = snprintf(path, 512, "%s/%s", directory, name);
    return size > 0 && size < 512 ? 0 : -1;
}

static int save_file_path(char path[512], const NesImageSource *source) {
    const char *extension = strrchr(source->save_path, '.');
    size_t stem = extension ? (size_t)(extension - source->save_path) : strlen(source->save_path);
    if (stem > 507) return -1;
    memcpy(path, source->save_path, stem);
    memcpy(path + stem, ".sav", 5);
    return 0;
}

static uint8_t *fixture_image(uint8_t tag, size_t *size) {
    *size = 16 + 0x4000 + 0x2000;
    uint8_t *data = (uint8_t *)calloc(*size, 1);
    if (!data) return NULL;
    memcpy(data, "NES\x1A", 4);
    data[4] = 1;
    data[5] = 1;
    data[6] = 2;
    memset(data + 16, 0xEA, 0x4000);
    const uint8_t program[] = {0xA9, tag, 0x85, 0, 0x4C, 4, 0x80};
    memcpy(data + 16, program, sizeof(program));
    for (size_t vector = 0x3FFA; vector <= 0x3FFE; vector += 2) {
        data[16 + vector] = 0;
        data[16 + vector + 1] = 0x80;
    }

    return data;
}

static int write_fixture(const char *path, const uint8_t *data, size_t size) {
    BOARD_CHECK(nes_file_write_atomic(path, data, size) == NES_FILE_OK);
    return 0;
}

static int check_single_archive(const char *directory, const uint8_t *fixture, size_t fixture_size,
                                 const char *filename) {
    char path[512], error[256];
    BOARD_CHECK(media_path(path, directory, filename) == 0);
    BOARD_CHECK(write_fixture(path, fixture, fixture_size) == 0);
    NesArchiveList list = {0};
    BOARD_CHECK(nes_image_list(path, &list, error, sizeof(error)) == NES_MEDIA_OK);
    BOARD_CHECK(list.count == 1 && !strcmp(list.entries[0].name, unicode_member));
    BOARD_CHECK(list.entries[0].size == 16 + 0x6000);
    nes_archive_list_free(&list);
    BOARD_CHECK(!list.entries && !list.count);

    NesImageSource source = {0};
    NesImageRequest request = {path, NULL, NULL};
    BOARD_CHECK(nes_image_prepare(&request, &source, error, sizeof(error)) == NES_MEDIA_OK);
    BOARD_CHECK(source.archived && !source.patched && !strcmp(source.member, unicode_member));
    BOARD_CHECK(source.source_crc32 == MEDIA_FIXTURE_FIRST_CRC);
    size_t expected_size = 0;
    uint8_t *expected = fixture_image(0x11, &expected_size);
    BOARD_CHECK(expected && source.size == expected_size && !memcmp(source.data, expected, expected_size));
    BOARD_CHECK(nes_image_load(&source, NULL, NULL, false, error, sizeof(error)) == NES_MEDIA_OK);
    BOARD_CHECK(rom_file_crc32() == MEDIA_FIXTURE_FIRST_CRC && cart_cpu_read(0x8001) == 0x11);
    BOARD_CHECK(unload_rom());
    free(expected);
    nes_image_source_free(&source);
    BOARD_CHECK(!source.data && !source.path && !source.member && !source.save_path && !source.size);
    BOARD_CHECK(nes_file_remove(path) == NES_FILE_OK);
    return 0;
}

static int test_single_archives(const char *directory) {
    int failures = check_single_archive(directory, media_fixture_zip_single,
                                         sizeof(media_fixture_zip_single), "caf\xC3\xA9-\xE7\x8C\xAB-\xF0\x9F\x92\xBE.zip");
    failures += check_single_archive(directory, media_fixture_seven_single,
                                      sizeof(media_fixture_seven_single), "unicode.7z");
    return failures;
}

static int check_member_selection(const char *directory, const uint8_t *fixture,
                                   size_t fixture_size, const char *filename,
                                   const char *second_name) {
    char path[512], error[256];
    BOARD_CHECK(media_path(path, directory, filename) == 0);
    BOARD_CHECK(write_fixture(path, fixture, fixture_size) == 0);
    NesArchiveList list = {0};
    BOARD_CHECK(nes_image_list(path, &list, error, sizeof(error)) == NES_MEDIA_OK);
    BOARD_CHECK(list.count == 2);
    bool found_first = false, found_second = false;
    for (size_t i = 0; i < list.count; i++) {
        found_first |= !strcmp(list.entries[i].name, unicode_member);
        found_second |= !strcmp(list.entries[i].name, second_name);
    }

    BOARD_CHECK(found_first && found_second);
    nes_archive_list_free(&list);
    NesImageRequest request = {path, NULL, NULL};
    NesImageSource source = {0};
    BOARD_CHECK(nes_image_prepare(&request, &source, error, sizeof(error)) == NES_MEDIA_NEEDS_SELECTION);
    BOARD_CHECK(!source.data && !source.path && error[0]);
    request.member = "missing.nes";
    BOARD_CHECK(nes_image_prepare(&request, &source, error, sizeof(error)) == NES_MEDIA_MEMBER_NOT_FOUND);
    BOARD_CHECK(!source.data && !source.path);
    request.member = second_name;
    BOARD_CHECK(nes_image_prepare(&request, &source, error, sizeof(error)) == NES_MEDIA_OK);
    BOARD_CHECK(source.source_crc32 == MEDIA_FIXTURE_SECOND_CRC);
    BOARD_CHECK(nes_image_load(&source, NULL, NULL, false, error, sizeof(error)) == NES_MEDIA_OK);
    BOARD_CHECK(cart_cpu_read(0x8001) == 0x22);
    BOARD_CHECK(unload_rom());
    nes_image_source_free(&source);
    BOARD_CHECK(nes_file_remove(path) == NES_FILE_OK);
    return 0;
}

static int test_member_selection(const char *directory) {
    return check_member_selection(directory, media_fixture_zip_multiple,
                                    sizeof(media_fixture_zip_multiple), "multiple.zip", "second/game.nes")
         + check_member_selection(directory, media_fixture_seven_solid,
                                    sizeof(media_fixture_seven_solid), "solid.7z", "game.nes");
}

static int check_rejected_archive(const char *path, const uint8_t *data, size_t size,
                                   NesMediaResult expected, uint8_t *previous_rom,
                                   uint32_t previous_crc) {
    BOARD_CHECK(write_fixture(path, data, size) == 0);
    char error[256];
    NesImageSource source = {0};
    NesImageRequest request = {path, NULL, NULL};
    NesMediaResult result = nes_image_prepare(&request, &source, error, sizeof(error));
    BOARD_CHECK(result == expected);
    BOARD_CHECK(!source.data && !source.path && !source.member && !source.save_path && !source.size);
    BOARD_CHECK(error[0] && prg_rom == previous_rom && rom_file_crc32() == previous_crc);
    return 0;
}

static int test_rejected_archives(const char *directory) {
    char path[512];
    BOARD_CHECK(media_path(path, directory, "rejected.zip") == 0);
    size_t image_size = 0;
    uint8_t *image = fixture_image(0x11, &image_size);
    BOARD_CHECK(image && load_rom_memory(image, image_size) == 0);
    free(image);
    uint8_t *previous = prg_rom;
    uint32_t previous_crc = rom_file_crc32();
    BOARD_CHECK(check_rejected_archive(path, media_fixture_zip_traversal, sizeof(media_fixture_zip_traversal),
                                       NES_MEDIA_INVALID, previous, previous_crc) == 0);
    BOARD_CHECK(check_rejected_archive(path, media_fixture_zip_absolute, sizeof(media_fixture_zip_absolute),
                                       NES_MEDIA_INVALID, previous, previous_crc) == 0);
    BOARD_CHECK(check_rejected_archive(path, media_fixture_zip_duplicate, sizeof(media_fixture_zip_duplicate),
                                       NES_MEDIA_INVALID, previous, previous_crc) == 0);
    BOARD_CHECK(check_rejected_archive(path, media_fixture_zip_no_image, sizeof(media_fixture_zip_no_image),
                                       NES_MEDIA_MEMBER_NOT_FOUND, previous, previous_crc) == 0);
    BOARD_CHECK(check_rejected_archive(path, media_fixture_zip_unsupported, sizeof(media_fixture_zip_unsupported),
                                       NES_MEDIA_UNSUPPORTED, previous, previous_crc) == 0);
    BOARD_CHECK(check_rejected_archive(path, media_fixture_seven_encrypted, sizeof(media_fixture_seven_encrypted),
                                       NES_MEDIA_UNSUPPORTED, previous, previous_crc) == 0);
    BOARD_CHECK(check_rejected_archive(path, media_fixture_zip_single, sizeof(media_fixture_zip_single) - 9,
                                       NES_MEDIA_INVALID, previous, previous_crc) == 0);
    BOARD_CHECK(check_rejected_archive(path, media_fixture_seven_single, sizeof(media_fixture_seven_single) - 9,
                                       NES_MEDIA_INVALID, previous, previous_crc) == 0);

    uint8_t corrupted[sizeof(media_fixture_zip_single)];
    memcpy(corrupted, media_fixture_zip_single, sizeof(corrupted));
    size_t central = 0;
    for (size_t i = 0; i + 4 < sizeof(corrupted); i++) {
        if (!memcmp(corrupted + i, "PK\x01\x02", 4)) { central = i; break; }
    }

    BOARD_CHECK(central && central + 28 <= sizeof(corrupted));
    corrupted[central + 16] ^= 1; /* Incorrect member CRC in the central directory. */
    BOARD_CHECK(check_rejected_archive(path, corrupted, sizeof(corrupted), NES_MEDIA_INVALID,
                                       previous, previous_crc) == 0);
    memcpy(corrupted, media_fixture_zip_single, sizeof(corrupted));
    corrupted[central + 24] = 1;
    corrupted[central + 25] = 0;
    corrupted[central + 26] = 0;
    corrupted[central + 27] = 0x10; /* 256 MiB + 1 declared output bytes. */
    BOARD_CHECK(check_rejected_archive(path, corrupted, sizeof(corrupted), NES_MEDIA_TOO_LARGE,
                                       previous, previous_crc) == 0);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(nes_file_remove(path) == NES_FILE_OK);
    return 0;
}

static int test_member_and_patch_save_identity(const char *directory) {
    char path[512], patch_path[512], error[256];
    BOARD_CHECK(media_path(path, directory, "save-identity.zip") == 0);
    BOARD_CHECK(media_path(patch_path, directory, "change.ips") == 0);
    BOARD_CHECK(write_fixture(path, media_fixture_zip_multiple, sizeof(media_fixture_zip_multiple)) == 0);
    BOARD_CHECK(write_fixture(patch_path, change_immediate, sizeof(change_immediate)) == 0);
    NesImageRequest request = {path, unicode_member, NULL};
    NesImageSource first = {0}, second = {0}, patched = {0};
    BOARD_CHECK(nes_image_prepare(&request, &first, error, sizeof(error)) == NES_MEDIA_OK);
    request.member = "second/game.nes";
    BOARD_CHECK(nes_image_prepare(&request, &second, error, sizeof(error)) == NES_MEDIA_OK);
    request.member = unicode_member;
    request.patch_path = patch_path;
    BOARD_CHECK(nes_image_prepare(&request, &patched, error, sizeof(error)) == NES_MEDIA_OK);
    BOARD_CHECK(patched.patched && patched.source_crc32 == first.source_crc32);
    BOARD_CHECK(patched.size == first.size && patched.data[17] == 0x7E && first.data[17] == 0x11);
    BOARD_CHECK(strcmp(first.save_path, second.save_path) && strcmp(first.save_path, patched.save_path)
                && strcmp(second.save_path, patched.save_path));
    NesImageSource *sources[] = {&first, &second, &patched};
    const uint8_t saved_values[] = {0x19, 0xA5, 0x7B};
    char save_paths[3][512];
    for (size_t i = 0; i < 3; i++) {
        BOARD_CHECK(save_file_path(save_paths[i], sources[i]) == 0);
        BOARD_CHECK(nes_image_load(sources[i], NULL, NULL, false, error, sizeof(error)) == NES_MEDIA_OK);
        BOARD_CHECK(cart_cpu_read(0x6003) == 0);
        cart_cpu_write(0x6003, saved_values[i]);
        BOARD_CHECK(cart_cpu_read(0x6003) == saved_values[i]);
        BOARD_CHECK(rom_flush_persistent());
    }

    for (size_t i = 0; i < 3; i++) {
        BOARD_CHECK(nes_image_load(sources[i], NULL, NULL, false, error, sizeof(error)) == NES_MEDIA_OK);
        BOARD_CHECK(cart_cpu_read(0x6003) == saved_values[i]);
    }

    BOARD_CHECK(unload_rom());
    uint8_t *archive = NULL;
    size_t archive_size = 0;
    BOARD_CHECK(nes_file_read_all(path, 4096, &archive, &archive_size) == NES_FILE_OK);
    BOARD_CHECK(archive_size == sizeof(media_fixture_zip_multiple)
                && !memcmp(archive, media_fixture_zip_multiple, archive_size));
    free(archive);
    for (size_t i = 0; i < 3; i++) {
        BOARD_CHECK(nes_file_remove(save_paths[i]) == NES_FILE_OK);
        nes_image_source_free(sources[i]);
    }

    BOARD_CHECK(nes_file_remove(path) == NES_FILE_OK);
    BOARD_CHECK(nes_file_remove(patch_path) == NES_FILE_OK);
    return 0;
}

static int test_post_patch_database_lookup(const char *directory) {
    char path[512], patch_path[512], error[256], row[512];
    BOARD_CHECK(media_path(path, directory, "database.zip") == 0);
    BOARD_CHECK(media_path(patch_path, directory, "database.ips") == 0);
    BOARD_CHECK(write_fixture(path, media_fixture_zip_single, sizeof(media_fixture_zip_single)) == 0);
    BOARD_CHECK(write_fixture(patch_path, change_immediate, sizeof(change_immediate)) == 0);
    NesImageRequest request = {path, NULL, patch_path};
    NesImageSource source = {0};
    BOARD_CHECK(nes_image_prepare(&request, &source, error, sizeof(error)) == NES_MEDIA_OK);
    uint32_t crc = game_db_crc32(source.data + 16, source.size - 16);
    int length = snprintf(row, sizeof(row), "%08X,NesPal,TEST,,,0,b16384,b8192,0,0,b8192,1,h,1,N,0,0,0\n", (unsigned)crc);
    BOARD_CHECK(length > 0 && (size_t)length < sizeof(row));
    BOARD_CHECK(rom_database_load_memory(row, (size_t)length));
    bool previous_overrides = rom_database_overrides_enabled();
    NesRegionMode previous_region = nes_region_mode();
    rom_database_set_overrides(true);
    BOARD_CHECK(nes_set_region_mode(NES_REGION_MODE_AUTO));
    BOARD_CHECK(nes_image_load(&source, NULL, NULL, false, error, sizeof(error)) == NES_MEDIA_OK);
    BOARD_CHECK(rom_metadata_source() == ROM_METADATA_DATABASE && nes_timing()->region == NES_REGION_PAL);
    BOARD_CHECK(rom_file_crc32() == game_db_crc32(source.data, source.size));
    BOARD_CHECK(cart_cpu_read(0x8001) == 0x7E);
    BOARD_CHECK(unload_rom());
    rom_database_clear();
    rom_database_set_overrides(previous_overrides);
    BOARD_CHECK(nes_set_region_mode(previous_region));
    nes_image_source_free(&source);
    BOARD_CHECK(nes_file_remove(path) == NES_FILE_OK);
    BOARD_CHECK(nes_file_remove(patch_path) == NES_FILE_OK);
    return 0;
}

static int test_plain_and_invalid_patch(const char *directory) {
    char path[512], patch_path[512], error[256];
    BOARD_CHECK(media_path(path, directory, "plain.nes") == 0);
    BOARD_CHECK(media_path(patch_path, directory, "invalid.ips") == 0);
    size_t image_size;
    uint8_t *image = fixture_image(0x11, &image_size);
    BOARD_CHECK(image && write_fixture(path, image, image_size) == 0);
    BOARD_CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    uint8_t *previous = prg_rom;
    NesImageRequest request = {path, NULL, NULL};
    NesImageSource source = {0};
    BOARD_CHECK(nes_image_prepare(&request, &source, error, sizeof(error)) == NES_MEDIA_OK);
    BOARD_CHECK(!source.archived && !source.patched && !strcmp(source.save_path, path));
    nes_image_source_free(&source);
    request.member = "not-an-archive.nes";
    BOARD_CHECK(nes_image_prepare(&request, &source, error, sizeof(error)) == NES_MEDIA_MEMBER_NOT_FOUND);
    BOARD_CHECK(prg_rom == previous && !source.data);
    request.member = NULL;
    request.patch_path = patch_path;
    BOARD_CHECK(write_fixture(patch_path, change_immediate, sizeof(change_immediate) - 1) == 0);
    BOARD_CHECK(nes_image_prepare(&request, &source, error, sizeof(error)) == NES_MEDIA_PATCH_ERROR);
    BOARD_CHECK(error[0] && prg_rom == previous && !source.data && !source.path);
    BOARD_CHECK(nes_file_remove(patch_path) == NES_FILE_OK);
    BOARD_CHECK(nes_image_prepare(&request, &source, error, sizeof(error)) == NES_MEDIA_NOT_FOUND);
    BOARD_CHECK(prg_rom == previous && !source.data);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(nes_file_remove(path) == NES_FILE_OK);
    return 0;
}

static int test_archive_entry_limit(const char *directory) {
    mz_zip_archive zip = {0};
    BOARD_CHECK(mz_zip_writer_init_heap(&zip, 0, 0));
    for (unsigned i = 0; i <= NES_ARCHIVE_ENTRY_LIMIT; i++) {
        char name[40];
        snprintf(name, sizeof(name), "entry-%u.txt", i);
        BOARD_CHECK(mz_zip_writer_add_mem(&zip, name, NULL, 0, 0));
    }

    void *archive = NULL;
    size_t archive_size = 0;
    BOARD_CHECK(mz_zip_writer_finalize_heap_archive(&zip, &archive, &archive_size));
    BOARD_CHECK(mz_zip_writer_end(&zip));
    char path[512], error[256];
    BOARD_CHECK(media_path(path, directory, "too-many.zip") == 0);
    BOARD_CHECK(write_fixture(path, (const uint8_t *)archive, archive_size) == 0);
    mz_free(archive);
    NesArchiveList list = {0};
    BOARD_CHECK(nes_image_list(path, &list, error, sizeof(error)) == NES_MEDIA_TOO_LARGE);
    BOARD_CHECK(!list.entries && !list.count && error[0]);
    BOARD_CHECK(nes_file_remove(path) == NES_FILE_OK);
    return 0;
}

int test_media_accuracy(void) {
    char directory[128];
#ifdef _WIN32
    unsigned process = (unsigned)_getpid();
#else
    unsigned process = (unsigned)getpid();
#endif
    BOARD_CHECK(snprintf(directory, sizeof(directory), "build/media-%u", process) > 0);
#ifdef _WIN32
    BOARD_CHECK(_mkdir(directory) == 0);
#else
    BOARD_CHECK(mkdir(directory, 0700) == 0);
#endif
    int failures = test_single_archives(directory) + test_member_selection(directory)
                 + test_rejected_archives(directory) + test_member_and_patch_save_identity(directory)
                 + test_post_patch_database_lookup(directory) + test_plain_and_invalid_patch(directory)
                 + test_archive_entry_limit(directory);
#ifdef _WIN32
    if (_rmdir(directory) != 0) ++failures;
#else
    if (rmdir(directory) != 0) ++failures;
#endif
    printf("Image sources: 7 groups, %d failures\n", failures);
    return failures;
}
