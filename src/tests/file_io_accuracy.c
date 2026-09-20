/*
 * file_io_accuracy.c - File limits, Unicode paths, and replacement failures
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
#include "../util/file_io.h"
#ifdef _WIN32
#include <direct.h>
#include <process.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

static int test_file_roundtrip(const char *directory) {
    char path[256];
    int length = snprintf(path, sizeof(path), "%s/\xC3\xA9-\xE7\x8C\xAB-\xF0\x9F\x92\xBE.bin", directory);
    BOARD_CHECK(length > 0 && (size_t)length < sizeof(path));
    const uint8_t original[] = {0x00, 0x19, 0x80, 0xFF, 0x00, 0x31};
    BOARD_CHECK(nes_file_write_atomic(path, original, sizeof(original)) == NES_FILE_OK);
    uint8_t *loaded = NULL;
    size_t size = 0;
    BOARD_CHECK(nes_file_read_all(path, sizeof(original), &loaded, &size) == NES_FILE_OK);
    BOARD_CHECK(size == sizeof(original) && memcmp(loaded, original, size) == 0);
    free(loaded);
    BOARD_CHECK(nes_file_read_all(path, sizeof(original) - 1, &loaded, &size) == NES_FILE_TOO_LARGE);
    BOARD_CHECK(!loaded && size == 0);

    BOARD_CHECK(nes_file_write_atomic(path, NULL, sizeof(original)) == NES_FILE_INVALID_ARGUMENT);
    BOARD_CHECK(nes_file_read_all(path, sizeof(original), &loaded, &size) == NES_FILE_OK);
    BOARD_CHECK(size == sizeof(original) && memcmp(loaded, original, size) == 0);
    free(loaded);

    const uint8_t replacement[] = {0x72, 0x00, 0x85};
    BOARD_CHECK(nes_file_write_atomic(path, replacement, sizeof(replacement)) == NES_FILE_OK);
    BOARD_CHECK(nes_file_read_all(path, sizeof(original), &loaded, &size) == NES_FILE_OK);
    BOARD_CHECK(size == sizeof(replacement) && memcmp(loaded, replacement, size) == 0);
    free(loaded);
    BOARD_CHECK(nes_file_write_atomic(path, NULL, 0) == NES_FILE_OK);
    BOARD_CHECK(nes_file_read_all(path, 0, &loaded, &size) == NES_FILE_OK && loaded && size == 0);
    free(loaded);
    FILE *stream = nes_file_open(path, "ab");
    BOARD_CHECK(stream && fwrite(original, 1, sizeof(original), stream) == sizeof(original));
    BOARD_CHECK(fclose(stream) == 0);
    BOARD_CHECK(nes_file_read_all(path, sizeof(original), &loaded, &size) == NES_FILE_OK);
    BOARD_CHECK(size == sizeof(original) && memcmp(loaded, original, size) == 0);
    free(loaded);
    BOARD_CHECK(nes_file_remove(path) == NES_FILE_OK);
    BOARD_CHECK(nes_file_read_all(path, 32, &loaded, &size) == NES_FILE_NOT_FOUND && !loaded && !size);
    return 0;
}

static int test_file_rejected_operations(const char *directory) {
    uint8_t *loaded = NULL;
    size_t size = 7;
    BOARD_CHECK(nes_file_read_all(NULL, 32, &loaded, &size) == NES_FILE_INVALID_ARGUMENT);
    BOARD_CHECK(!loaded && !size);
    BOARD_CHECK(nes_file_open("\xC0\xAF", "rb") == NULL);
    BOARD_CHECK(nes_file_open(directory, "invalid") == NULL);
    BOARD_CHECK(nes_file_write_atomic("\xED\xA0\x80", "x", 1) == NES_FILE_INVALID_ARGUMENT);
    BOARD_CHECK(nes_file_read_all(directory, 32, &loaded, &size) == NES_FILE_IO_ERROR);
    BOARD_CHECK(!loaded && !size);
    char child[256];
    BOARD_CHECK(snprintf(child, sizeof(child), "%s/kept.bin", directory) > 0);
    BOARD_CHECK(nes_file_write_atomic(child, "kept", 4) == NES_FILE_OK);
    // Replacing a non-empty directory fails after the temporary file was written.
    BOARD_CHECK(nes_file_write_atomic(directory, "rejected", 8) == NES_FILE_IO_ERROR);
    BOARD_CHECK(nes_file_read_all(child, 4, &loaded, &size) == NES_FILE_OK);
    BOARD_CHECK(size == 4 && memcmp(loaded, "kept", 4) == 0);
    free(loaded);
    BOARD_CHECK(nes_file_remove(child) == NES_FILE_OK);
    return 0;
}

int test_file_io_accuracy(void) {
    char directory[128];
#ifdef _WIN32
    unsigned process = (unsigned)_getpid();
#else
    unsigned process = (unsigned)getpid();
#endif
    int length = snprintf(directory, sizeof(directory), "build/file-io-%u", process);
    BOARD_CHECK(length > 0 && (size_t)length < sizeof(directory));
#ifdef _WIN32
    BOARD_CHECK(_mkdir(directory) == 0);
#else
    BOARD_CHECK(mkdir(directory, 0700) == 0);
#endif
    int failures = test_file_roundtrip(directory) + test_file_rejected_operations(directory);
#ifdef _WIN32
    if (_rmdir(directory) != 0) ++failures;
#else
    if (rmdir(directory) != 0) ++failures;
#endif
    printf("File I/O: 2 groups, %d failures\n", failures);
    return failures;
}
