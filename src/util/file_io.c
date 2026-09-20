/*
 * file_io.c - UTF-8 file paths, bounded reads, and atomic replacement
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#define _FILE_OFFSET_BITS 64
#endif
#include "file_io.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifdef _WIN32
#include <io.h>
#include <process.h>
#include <windows.h>
#else
#include <unistd.h>
#endif

static atomic_uint_fast64_t temporary_sequence;

static bool valid_path(const char *path) {
    if (!path || !*path) return false;
    size_t length = 0;
    while (path[length]) {
        if (length == NES_FILE_PATH_LIMIT) return false;
        ++length;
    }
    const unsigned char *bytes = (const unsigned char *)path;
    for (size_t i = 0; i < length;) {
        uint32_t value = bytes[i++];
        if (value < 0x80u) continue;
        unsigned continuation;
        uint32_t minimum;
        if (value >= 0xC2u && value <= 0xDFu) {
            continuation = 1; minimum = 0x80u; value &= 0x1Fu;
        } else if (value >= 0xE0u && value <= 0xEFu) {
            continuation = 2; minimum = 0x800u; value &= 0x0Fu;
        } else if (value >= 0xF0u && value <= 0xF4u) {
            continuation = 3; minimum = 0x10000u; value &= 0x07u;
        } else return false;
        if (continuation > length - i) return false;
        for (unsigned j = 0; j < continuation; ++j) {
            unsigned next = bytes[i++];
            if ((next & 0xC0u) != 0x80u) return false;
            value = (value << 6) | (next & 0x3Fu);
        }
        if (value < minimum || value > 0x10FFFFu || (value >= 0xD800u && value <= 0xDFFFu))
            return false;
    }
    return true;
}

static NesFileResult file_error(void) {
    switch (errno) {
        case ENOENT: return NES_FILE_NOT_FOUND;
        case EINVAL: return NES_FILE_INVALID_ARGUMENT;
        case ENOMEM: return NES_FILE_OUT_OF_MEMORY;
        default: return NES_FILE_IO_ERROR;
    }
}

static bool valid_mode(const char *mode) {
    static const char *const modes[] = {
        "r", "rb", "r+", "rb+", "r+b", "w", "wb", "w+", "wb+", "w+b",
        "a", "ab", "a+", "ab+", "a+b"
    };
    if (!mode) return false;
    for (size_t i = 0; i < sizeof(modes) / sizeof(modes[0]); ++i)
        if (strcmp(mode, modes[i]) == 0) return true;
    return false;
}

#ifdef _WIN32
static wchar_t *wide_path(const char *path) {
    int count = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path, -1, NULL, 0);
    if (count <= 0) { errno = EINVAL; return NULL; }
    wchar_t *wide = malloc((size_t)count * sizeof(*wide));
    if (!wide) { errno = ENOMEM; return NULL; }
    if (!MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path, -1, wide, count)) {
        free(wide);
        errno = EINVAL;
        return NULL;
    }
    return wide;
}
#endif

FILE *nes_file_open(const char *path, const char *mode) {
    if (!valid_path(path) || !valid_mode(mode)) {
        errno = EINVAL;
        return NULL;
    }
#ifdef _WIN32
    wchar_t *wide = wide_path(path);
    if (!wide) return NULL;
    wchar_t *wide_mode = wide_path(mode);
    if (!wide_mode) { free(wide); return NULL; }
    FILE *file = _wfopen(wide, wide_mode);
    free(wide_mode);
    free(wide);
    return file;
#else
    return fopen(path, mode);
#endif
}

NesFileResult nes_file_read_all(const char *path, size_t limit, uint8_t **data, size_t *size) {
    if (data) *data = NULL;
    if (size) *size = 0;
    if (!data || !size || !valid_path(path)) return NES_FILE_INVALID_ARGUMENT;
    FILE *file = nes_file_open(path, "rb");
    if (!file) return file_error();
#ifdef _WIN32
    struct _stat64 info;
    bool regular = _fstat64(_fileno(file), &info) == 0 && (info.st_mode & _S_IFMT) == _S_IFREG;
#else
    struct stat info;
    bool regular = fstat(fileno(file), &info) == 0 && S_ISREG(info.st_mode);
#endif
    NesFileResult result = NES_FILE_OK;
    if (!regular || info.st_size < 0) result = NES_FILE_IO_ERROR;
    else if ((uintmax_t)info.st_size > limit)
        result = NES_FILE_TOO_LARGE;
    uint8_t *buffer = NULL;
    size_t length = result == NES_FILE_OK ? (size_t)info.st_size : 0;
    if (result == NES_FILE_OK) {
        buffer = malloc(length ? length : 1);
        if (!buffer) result = NES_FILE_OUT_OF_MEMORY;
        else if (fread(buffer, 1, length, file) != length || fgetc(file) != EOF || ferror(file))
            result = NES_FILE_IO_ERROR;
    }
    if (fclose(file) != 0 && result == NES_FILE_OK) result = NES_FILE_IO_ERROR;
    if (result != NES_FILE_OK) { free(buffer); return result; }
    *data = buffer;
    *size = length;
    return NES_FILE_OK;
}

NesFileResult nes_file_remove(const char *path) {
    if (!valid_path(path)) return NES_FILE_INVALID_ARGUMENT;
#ifdef _WIN32
    wchar_t *wide = wide_path(path);
    if (!wide) return file_error();
    int removed = _wremove(wide);
    free(wide);
#else
    int removed = remove(path);
#endif
    return removed == 0 ? NES_FILE_OK : file_error();
}

NesFileResult nes_file_same(const char *left, const char *right, bool *same) {
    if (same) *same = false;
    if (!same || !valid_path(left) || !valid_path(right)) return NES_FILE_INVALID_ARGUMENT;
    if (!strcmp(left, right)) {
        *same = true;
        return NES_FILE_OK;
    }

    FILE *first = nes_file_open(left, "rb");
    if (!first) return file_error();
    FILE *second = nes_file_open(right, "rb");
    if (!second) {
        NesFileResult result = file_error();
        fclose(first);
        return result;
    }

    NesFileResult result = NES_FILE_OK;
#ifdef _WIN32
    BY_HANDLE_FILE_INFORMATION a, b;
    HANDLE first_handle = (HANDLE)_get_osfhandle(_fileno(first));
    HANDLE second_handle = (HANDLE)_get_osfhandle(_fileno(second));
    if (!GetFileInformationByHandle(first_handle, &a) || !GetFileInformationByHandle(second_handle, &b)) {
        result = NES_FILE_IO_ERROR;
    } else {
        *same = a.dwVolumeSerialNumber == b.dwVolumeSerialNumber
             && a.nFileIndexHigh == b.nFileIndexHigh && a.nFileIndexLow == b.nFileIndexLow;
    }
#else
    struct stat a, b;
    if (fstat(fileno(first), &a) != 0 || fstat(fileno(second), &b) != 0) {
        result = NES_FILE_IO_ERROR;
    } else {
        *same = a.st_dev == b.st_dev && a.st_ino == b.st_ino;
    }
#endif
    if (fclose(first) != 0) result = NES_FILE_IO_ERROR;
    if (fclose(second) != 0) result = NES_FILE_IO_ERROR;
    return result;
}

static FILE *create_exclusive(const char *path) {
#ifdef _WIN32
    wchar_t *wide = wide_path(path);
    if (!wide) return NULL;
    int descriptor = _wopen(wide, _O_WRONLY | _O_CREAT | _O_EXCL | _O_BINARY, _S_IREAD | _S_IWRITE);
    free(wide);
    if (descriptor < 0) return NULL;
    FILE *file = _fdopen(descriptor, "wb");
    if (!file) { _close(descriptor); (void)nes_file_remove(path); }
#else
    int descriptor = open(path, O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (descriptor < 0) return NULL;
    FILE *file = fdopen(descriptor, "wb");
    if (!file) { close(descriptor); (void)nes_file_remove(path); }
#endif
    return file;
}

static bool flush_file(FILE *file) {
    if (fflush(file) != 0) return false;
#ifdef _WIN32
    return _commit(_fileno(file)) == 0;
#else
    return fsync(fileno(file)) == 0;
#endif
}

static bool replace_file(const char *temporary, const char *destination) {
#ifdef _WIN32
    wchar_t *source = wide_path(temporary);
    if (!source) return false;
    wchar_t *target = wide_path(destination);
    if (!target) { free(source); return false; }
    bool result = MoveFileExW(source, target, MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0;
    free(source);
    free(target);
    return result;
#else
    return rename(temporary, destination) == 0;
#endif
}

NesFileResult nes_file_write_atomic(const char *path, const void *data, size_t size) {
    if (!valid_path(path) || (!data && size)) return NES_FILE_INVALID_ARGUMENT;
    size_t path_length = strlen(path);
    if (path_length > NES_FILE_PATH_LIMIT - 64) return NES_FILE_INVALID_ARGUMENT;
    char *temporary = malloc(path_length + 64);
    if (!temporary) return NES_FILE_OUT_OF_MEMORY;
#ifdef _WIN32
    unsigned long process_id = (unsigned long)_getpid();
#else
    unsigned long process_id = (unsigned long)getpid();
#endif
    FILE *file = NULL;
    for (unsigned attempt = 0; attempt < 16; ++attempt) {
        uint64_t sequence = atomic_fetch_add_explicit(&temporary_sequence, 1, memory_order_relaxed);
        snprintf(temporary, path_length + 64, "%s.tmp-%lu-%" PRIu64, path, process_id, sequence);
        file = create_exclusive(temporary);
        if (file || errno != EEXIST) break;
    }
    if (!file) { NesFileResult result = file_error(); free(temporary); return result; }
    bool complete = (!size || fwrite(data, 1, size, file) == size) && flush_file(file);
    if (fclose(file) != 0) complete = false;
    if (complete) complete = replace_file(temporary, path);
    if (!complete) (void)nes_file_remove(temporary);
    free(temporary);
    return complete ? NES_FILE_OK : NES_FILE_IO_ERROR;
}

const char *nes_file_result_message(NesFileResult result) {
    switch (result) {
        case NES_FILE_OK: return "File operation completed";
        case NES_FILE_NOT_FOUND: return "File not found";
        case NES_FILE_INVALID_ARGUMENT: return "Invalid file path or argument";
        case NES_FILE_TOO_LARGE: return "File exceeds the supported size";
        case NES_FILE_OUT_OF_MEMORY: return "Not enough memory for the file operation";
        case NES_FILE_IO_ERROR: return "Could not read or write the file";
        default: return "Unknown file error";
    }
}
