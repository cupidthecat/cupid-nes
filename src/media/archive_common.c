/*
 * archive_common.c - Bounded archive memory and member names
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "archive_internal.h"
#include "../util/file_io.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

typedef union {
    max_align_t alignment;
    size_t size;
} AllocationHeader;

void *archive_allocate(ArchiveBudget *budget, size_t size) {
    if (!size) {
        size = 1;
    }

    if (size > budget->limit - budget->used || size > SIZE_MAX - sizeof(AllocationHeader)) {
        budget->exhausted = true;
        return NULL;
    }

    AllocationHeader *allocation = (AllocationHeader *)malloc(sizeof(*allocation) + size);
    if (!allocation) {
        budget->allocation_failed = true;
        return NULL;
    }

    allocation->size = size;
    budget->used += size;
    return allocation + 1;
}

void archive_deallocate(ArchiveBudget *budget, void *address) {
    if (address) {
        AllocationHeader *allocation = (AllocationHeader *)address - 1;
        budget->used -= allocation->size;
        free(allocation);
    }
}

void *archive_reallocate(ArchiveBudget *budget, void *address, size_t size) {
    if (!address) {
        return archive_allocate(budget, size);
    }

    if (!size) {
        archive_deallocate(budget, address);
        return NULL;
    }

    AllocationHeader *allocation = (AllocationHeader *)address - 1;
    size_t old_size = allocation->size;
    if (size > budget->limit - (budget->used - old_size)
        || size > SIZE_MAX - sizeof(*allocation)) {
        budget->exhausted = true;
        return NULL;
    }

    AllocationHeader *replacement = (AllocationHeader *)realloc(allocation, sizeof(*allocation) + size);
    if (!replacement) {
        budget->allocation_failed = true;
        return NULL;
    }

    replacement->size = size;
    budget->used = budget->used - old_size + size;
    return replacement + 1;
}

NesMediaResult archive_budget_result(const ArchiveBudget *budget) {
    return budget->exhausted ? NES_MEDIA_TOO_LARGE
         : budget->allocation_failed ? NES_MEDIA_OUT_OF_MEMORY : NES_MEDIA_INVALID;
}

bool archive_supported_name(const char *name) {
    static const char *const extensions[] = {".nes", ".unf", ".unif", ".nsf", ".nsfe", ".fds", ".qd", ".stbx", ".bin"};
    const char *extension = strrchr(name, '.');
    if (!extension) {
        return false;
    }

    for (size_t i = 0; i < sizeof(extensions) / sizeof(extensions[0]); i++) {
        size_t j = 0;
        while (extension[j] && extensions[i][j]
               && tolower((unsigned char)extension[j]) == extensions[i][j]) {
            j++;
        }

        if (!extension[j] && !extensions[i][j]) {
            return true;
        }
    }

    return false;
}

bool archive_valid_utf8(const char *name) {
    const unsigned char *next = (const unsigned char *)name;
    while (*next) {
        uint32_t code;
        unsigned trailing;
        unsigned first = *next++;
        if (first < 0x80) {
            if (first < 0x20 || first == 0x7f) {
                return false;
            }

            continue;
        }

        if (first >= 0xc2 && first <= 0xdf) {
            code = first & 0x1fu;
            trailing = 1;
        } else if (first >= 0xe0 && first <= 0xef) {
            code = first & 0x0fu;
            trailing = 2;
        } else if (first >= 0xf0 && first <= 0xf4) {
            code = first & 0x07u;
            trailing = 3;
        } else {
            return false;
        }

        for (unsigned i = 0; i < trailing; i++) {
            if ((*next & 0xc0u) != 0x80u) {
                return false;
            }

            code = (code << 6) | (*next++ & 0x3fu);
        }

        if ((trailing == 2 && code < 0x800) || (trailing == 3 && code < 0x10000)
            || (code >= 0xd800 && code <= 0xdfff) || code > 0x10ffff) {
            return false;
        }
    }

    return true;
}

static size_t encode_utf8(char *target, uint32_t code) {
    if (code < 0x80) {
        target[0] = (char)code;
        return 1;
    }

    if (code < 0x800) {
        target[0] = (char)(0xc0 | (code >> 6));
        target[1] = (char)(0x80 | (code & 0x3f));
        return 2;
    }

    if (code < 0x10000) {
        target[0] = (char)(0xe0 | (code >> 12));
        target[1] = (char)(0x80 | ((code >> 6) & 0x3f));
        target[2] = (char)(0x80 | (code & 0x3f));
        return 3;
    }

    target[0] = (char)(0xf0 | (code >> 18));
    target[1] = (char)(0x80 | ((code >> 12) & 0x3f));
    target[2] = (char)(0x80 | ((code >> 6) & 0x3f));
    target[3] = (char)(0x80 | (code & 0x3f));
    return 4;
}

char *archive_utf16_name(const uint16_t *name, size_t units) {
    if (!units || units > NES_FILE_PATH_LIMIT || name[units - 1] != 0) {
        return NULL;
    }

    char *result = (char *)malloc(units * 3);
    if (!result) {
        return NULL;
    }

    size_t length = 0;
    for (size_t i = 0; i + 1 < units; i++) {
        uint32_t code = name[i];
        if (code >= 0xd800 && code <= 0xdbff) {
            if (i + 2 >= units || name[i + 1] < 0xdc00 || name[i + 1] > 0xdfff) {
                free(result);
                return NULL;
            }

            code = 0x10000u + ((code - 0xd800u) << 10) + (name[++i] - 0xdc00u);
        } else if (!code || (code >= 0xdc00 && code <= 0xdfff)) {
            free(result);
            return NULL;
        }

        length += encode_utf8(result + length, code);
    }

    result[length] = 0;
    return result;
}

char *archive_cp437_name(const uint8_t *name, size_t size) {
    static const uint16_t upper[128] = {
        0x00c7,0x00fc,0x00e9,0x00e2,0x00e4,0x00e0,0x00e5,0x00e7,
        0x00ea,0x00eb,0x00e8,0x00ef,0x00ee,0x00ec,0x00c4,0x00c5,
        0x00c9,0x00e6,0x00c6,0x00f4,0x00f6,0x00f2,0x00fb,0x00f9,
        0x00ff,0x00d6,0x00dc,0x00a2,0x00a3,0x00a5,0x20a7,0x0192,
        0x00e1,0x00ed,0x00f3,0x00fa,0x00f1,0x00d1,0x00aa,0x00ba,
        0x00bf,0x2310,0x00ac,0x00bd,0x00bc,0x00a1,0x00ab,0x00bb,
        0x2591,0x2592,0x2593,0x2502,0x2524,0x2561,0x2562,0x2556,
        0x2555,0x2563,0x2551,0x2557,0x255d,0x255c,0x255b,0x2510,
        0x2514,0x2534,0x252c,0x251c,0x2500,0x253c,0x255e,0x255f,
        0x255a,0x2554,0x2569,0x2566,0x2560,0x2550,0x256c,0x2567,
        0x2568,0x2564,0x2565,0x2559,0x2558,0x2552,0x2553,0x256b,
        0x256a,0x2518,0x250c,0x2588,0x2584,0x258c,0x2590,0x2580,
        0x03b1,0x00df,0x0393,0x03c0,0x03a3,0x03c3,0x00b5,0x03c4,
        0x03a6,0x0398,0x03a9,0x03b4,0x221e,0x03c6,0x03b5,0x2229,
        0x2261,0x00b1,0x2265,0x2264,0x2320,0x2321,0x00f7,0x2248,
        0x00b0,0x2219,0x00b7,0x221a,0x207f,0x00b2,0x25a0,0x00a0
    };
    if (size > NES_FILE_PATH_LIMIT) {
        return NULL;
    }

    char *result = (char *)malloc(size * 3 + 1);
    if (!result) {
        return NULL;
    }

    size_t length = 0;
    for (size_t i = 0; i < size; i++) {
        if (!name[i]) {
            free(result);
            return NULL;
        }

        length += encode_utf8(result + length, name[i] < 128 ? name[i] : upper[name[i] - 128]);
    }

    result[length] = 0;
    return result;
}

NesMediaResult archive_add_entry(ArchiveReader *reader, const char *name, size_t size, unsigned index) {
    if (!archive_supported_name(name)) {
        return NES_MEDIA_OK;
    }

    size_t length = strlen(name);
    if (!length || length >= NES_FILE_PATH_LIMIT || size > NES_IMAGE_SIZE_LIMIT
        || reader->list.count >= NES_ARCHIVE_ENTRY_LIMIT) {
        return NES_MEDIA_TOO_LARGE;
    }

    if (!archive_valid_utf8(name) || name[0] == '/' || name[0] == '\\' || strchr(name, ':')) {
        return NES_MEDIA_INVALID;
    }

    char *normalized = (char *)malloc(length + 1);
    if (!normalized) {
        return NES_MEDIA_OUT_OF_MEMORY;
    }

    memcpy(normalized, name, length + 1);
    for (size_t i = 0; i < length; i++) {
        if (normalized[i] == '\\') {
            normalized[i] = '/';
        }
    }

    const char *part = normalized;
    for (size_t i = 0; i <= length; i++) {
        if (!normalized[i] || normalized[i] == '/') {
            size_t part_size = (size_t)(normalized + i - part);
            if (!part_size || (part_size == 1 && part[0] == '.')
                || (part_size == 2 && part[0] == '.' && part[1] == '.')) {
                free(normalized);
                return NES_MEDIA_INVALID;
            }

            part = normalized + i + 1;
        }
    }

    for (size_t i = 0; i < reader->list.count; i++) {
        if (!strcmp(reader->list.entries[i].name, normalized)) {
            free(normalized);
            return NES_MEDIA_INVALID;
        }
    }

    size_t count = reader->list.count;
    NesArchiveEntry *entries = (NesArchiveEntry *)realloc(reader->list.entries, (count + 1) * sizeof(*entries));
    if (!entries) {
        free(normalized);
        return NES_MEDIA_OUT_OF_MEMORY;
    }

    reader->list.entries = entries;
    unsigned *indices = (unsigned *)realloc(reader->indices, (count + 1) * sizeof(*indices));
    if (!indices) {
        free(normalized);
        return NES_MEDIA_OUT_OF_MEMORY;
    }

    reader->indices = indices;
    entries[count] = (NesArchiveEntry){normalized, size};
    indices[count] = index;
    reader->list.count++;
    return NES_MEDIA_OK;
}

NesMediaResult archive_open(const uint8_t *data, size_t size, ArchiveReader *reader) {
    memset(reader, 0, sizeof(*reader));
    NesMediaResult result;
    if (size >= 4 && !memcmp(data, "PK", 2)) {
        result = archive_open_zip(data, size, reader);
    } else if (size >= 6 && !memcmp(data, "7z\xbc\xaf\x27\x1c", 6)) {
        result = archive_open_7z(data, size, reader);
    } else {
        return NES_MEDIA_UNSUPPORTED;
    }

    if (result == NES_MEDIA_OK && !reader->list.count) {
        result = NES_MEDIA_MEMBER_NOT_FOUND;
    }

    if (result != NES_MEDIA_OK) {
        archive_close(reader);
    }

    return result;
}

void nes_archive_list_free(NesArchiveList *list) {
    if (list) {
        for (size_t i = 0; i < list->count; i++) {
            free(list->entries[i].name);
        }

        free(list->entries);
        memset(list, 0, sizeof(*list));
    }
}

void archive_close(ArchiveReader *reader) {
    if (reader) {
        if (reader->close) {
            reader->close(reader->backend);
        }

        nes_archive_list_free(&reader->list);
        free(reader->indices);
        memset(reader, 0, sizeof(*reader));
    }
}
