/*
 * archive_7z.c - 7z image extraction from bounded memory streams
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "archive_internal.h"
#include "../third_party/lzma/7z.h"
#include "../third_party/lzma/7zCrc.h"
#include "../util/file_io.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
    ILookInStream interface;
    const uint8_t *data;
    size_t size;
    size_t position;
} ArchiveStream;

typedef struct {
    ISzAlloc interface;
    ArchiveBudget budget;
} SevenZipAllocator;

typedef struct {
    CSzArEx archive;
    ArchiveStream stream;
    SevenZipAllocator allocator;
} SevenZipReader;

static SRes stream_look(ILookInStreamPtr interface, const void **buffer, size_t *size) {
    ArchiveStream *stream = (ArchiveStream *)interface;
    size_t remaining = stream->size - stream->position;
    if (*size > remaining) {
        *size = remaining;
    }

    *buffer = stream->data + stream->position;
    return SZ_OK;
}

static SRes stream_skip(ILookInStreamPtr interface, size_t size) {
    ArchiveStream *stream = (ArchiveStream *)interface;
    if (size > stream->size - stream->position) {
        return SZ_ERROR_INPUT_EOF;
    }

    stream->position += size;
    return SZ_OK;
}

static SRes stream_read(ILookInStreamPtr interface, void *buffer, size_t *size) {
    const void *source;
    SRes result = stream_look(interface, &source, size);
    if (result != SZ_OK) {
        return result;
    }

    if (*size) {
        memcpy(buffer, source, *size);
    }

    return stream_skip(interface, *size);
}

static SRes stream_seek(ILookInStreamPtr interface, Int64 *position, ESzSeek origin) {
    ArchiveStream *stream = (ArchiveStream *)interface;
    size_t base;
    switch (origin) {
        case SZ_SEEK_SET: base = 0; break;
        case SZ_SEEK_CUR: base = stream->position; break;
        case SZ_SEEK_END: base = stream->size; break;
        default: return SZ_ERROR_PARAM;
    }

    size_t target;
    if (*position < 0) {
        UInt64 distance = (UInt64)(-(*position + 1)) + 1;
        if (distance > base) {
            return SZ_ERROR_INPUT_EOF;
        }

        target = base - (size_t)distance;
    } else {
        UInt64 distance = (UInt64)*position;
        if (distance > stream->size - base) {
            return SZ_ERROR_INPUT_EOF;
        }

        target = base + (size_t)distance;
    }

    stream->position = target;
    *position = (Int64)target;
    return SZ_OK;
}

static void *seven_allocate(ISzAllocPtr interface, size_t size) {
    SevenZipAllocator *allocator = (SevenZipAllocator *)interface;
    return archive_allocate(&allocator->budget, size);
}

static void seven_deallocate(ISzAllocPtr interface, void *address) {
    SevenZipAllocator *allocator = (SevenZipAllocator *)interface;
    archive_deallocate(&allocator->budget, address);
}

static void seven_close(void *opaque) {
    SevenZipReader *reader = (SevenZipReader *)opaque;
    SzArEx_Free(&reader->archive, &reader->allocator.interface);
    free(reader);
}

static NesMediaResult seven_result(SevenZipReader *reader, SRes result) {
    if (result == SZ_ERROR_UNSUPPORTED) {
        return NES_MEDIA_UNSUPPORTED;
    }

    return archive_budget_result(&reader->allocator.budget);
}

static NesMediaResult seven_extract(void *opaque, unsigned index, uint8_t **data, size_t *size) {
    SevenZipReader *reader = (SevenZipReader *)opaque;
    *data = NULL;
    *size = 0;
    if (index >= reader->archive.NumFiles || SzArEx_IsDir(&reader->archive, index)) {
        return NES_MEDIA_INVALID;
    }

    UInt64 bytes = SzArEx_GetFileSize(&reader->archive, index);
    UInt32 folder = reader->archive.FileToFolder[index];
    if (bytes > NES_IMAGE_SIZE_LIMIT
        || (folder != (UInt32)-1
            && SzAr_GetFolderUnpackSize(&reader->archive.db, folder) > NES_IMAGE_SIZE_LIMIT)) {
        return NES_MEDIA_TOO_LARGE;
    }

    UInt32 block_index = (UInt32)-1;
    Byte *block = NULL;
    size_t block_size = 0, offset = 0, written = 0;
    SRes result = SzArEx_Extract(&reader->archive, &reader->stream.interface, index,
                                &block_index, &block, &block_size, &offset, &written,
                                &reader->allocator.interface, &reader->allocator.interface);
    NesMediaResult status = NES_MEDIA_OK;
    if (result != SZ_OK) {
        status = seven_result(reader, result);
    } else if (written != bytes || offset > block_size || written > block_size - offset) {
        status = NES_MEDIA_INVALID;
    } else {
        uint8_t *output = (uint8_t *)malloc(written ? written : 1);
        if (!output) {
            status = NES_MEDIA_OUT_OF_MEMORY;
        } else {
            if (written) {
                memcpy(output, block + offset, written);
            }

            *data = output;
            *size = written;
        }
    }

    seven_deallocate(&reader->allocator.interface, block);
    return status;
}

NesMediaResult archive_open_7z(const uint8_t *data, size_t size, ArchiveReader *reader) {
    SevenZipReader *seven = (SevenZipReader *)calloc(1, sizeof(*seven));
    if (!seven) {
        return NES_MEDIA_OUT_OF_MEMORY;
    }

    seven->stream = (ArchiveStream){{stream_look, stream_skip, stream_read, stream_seek}, data, size, 0};
    seven->allocator.interface = (ISzAlloc){seven_allocate, seven_deallocate};
    seven->allocator.budget.limit = NES_IMAGE_SIZE_LIMIT;
    SzArEx_Init(&seven->archive);
    reader->backend = seven;
    reader->extract = seven_extract;
    reader->close = seven_close;
    /* Archive preparation runs on the application thread before activation. */
    CrcGenerateTable();
    SRes result = SzArEx_Open(&seven->archive, &seven->stream.interface,
                              &seven->allocator.interface, &seven->allocator.interface);
    if (result != SZ_OK) {
        return seven_result(seven, result);
    }

    if (seven->archive.NumFiles > NES_ARCHIVE_ENTRY_LIMIT) {
        return NES_MEDIA_TOO_LARGE;
    }

    for (UInt32 i = 0; i < seven->archive.NumFiles; i++) {
        if (SzArEx_IsDir(&seven->archive, i)) {
            continue;
        }

        size_t units = SzArEx_GetFileNameUtf16(&seven->archive, i, NULL);
        if (!units || units > NES_FILE_PATH_LIMIT) {
            return NES_MEDIA_TOO_LARGE;
        }

        UInt16 *wide = (UInt16 *)malloc(units * sizeof(*wide));
        if (!wide) {
            return NES_MEDIA_OUT_OF_MEMORY;
        }

        SzArEx_GetFileNameUtf16(&seven->archive, i, wide);
        char *name = archive_utf16_name(wide, units);
        free(wide);
        if (!name) {
            return NES_MEDIA_INVALID;
        }

        UInt64 bytes = SzArEx_GetFileSize(&seven->archive, i);
        NesMediaResult status;
        if (archive_supported_name(name) && bytes > NES_IMAGE_SIZE_LIMIT) {
            status = NES_MEDIA_TOO_LARGE;
        } else {
            status = archive_add_entry(reader, name, bytes <= SIZE_MAX ? (size_t)bytes : 0, i);
        }

        free(name);
        if (status != NES_MEDIA_OK) {
            return status;
        }
    }

    return NES_MEDIA_OK;
}
