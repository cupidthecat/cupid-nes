/*
 * archive_zip.c - ZIP image extraction with bounded allocations
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "archive_internal.h"
#include "../third_party/miniz/miniz.h"
#include "../util/file_io.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
    mz_zip_archive archive;
    ArchiveBudget budget;
    bool initialized;
} ZipReader;

static void *zip_allocate(void *opaque, size_t count, size_t size) {
    ArchiveBudget *budget = (ArchiveBudget *)opaque;
    if (size && count > SIZE_MAX / size) {
        budget->exhausted = true;
        return NULL;
    }

    return archive_allocate(budget, count * size);
}

static void *zip_reallocate(void *opaque, void *address, size_t count, size_t size) {
    ArchiveBudget *budget = (ArchiveBudget *)opaque;
    if (size && count > SIZE_MAX / size) {
        budget->exhausted = true;
        return NULL;
    }

    return archive_reallocate(budget, address, count * size);
}

static void zip_deallocate(void *opaque, void *address) {
    archive_deallocate((ArchiveBudget *)opaque, address);
}

static void zip_close(void *opaque) {
    ZipReader *zip = (ZipReader *)opaque;
    if (zip->initialized) {
        mz_zip_reader_end(&zip->archive);
    }

    free(zip);
}

static NesMediaResult zip_extract(void *opaque, unsigned index, uint8_t **data, size_t *size) {
    ZipReader *zip = (ZipReader *)opaque;
    mz_zip_archive_file_stat info;
    *data = NULL;
    *size = 0;
    if (!mz_zip_reader_file_stat(&zip->archive, index, &info)
        || !info.m_is_supported || info.m_is_directory) {
        return NES_MEDIA_INVALID;
    }

    if (info.m_uncomp_size > NES_IMAGE_SIZE_LIMIT) {
        return NES_MEDIA_TOO_LARGE;
    }

    size_t bytes = (size_t)info.m_uncomp_size;
    uint8_t *output = (uint8_t *)malloc(bytes ? bytes : 1);
    if (!output) {
        return NES_MEDIA_OUT_OF_MEMORY;
    }

    /* Extraction verifies the member's CRC as well as the compressed stream. */
    if (!mz_zip_reader_extract_to_mem(&zip->archive, index, output, bytes, 0)) {
        free(output);
        return archive_budget_result(&zip->budget);
    }

    *data = output;
    *size = bytes;
    return NES_MEDIA_OK;
}

NesMediaResult archive_open_zip(const uint8_t *data, size_t size, ArchiveReader *reader) {
    ZipReader *zip = (ZipReader *)calloc(1, sizeof(*zip));
    if (!zip) {
        return NES_MEDIA_OUT_OF_MEMORY;
    }

    zip->budget.limit = NES_IMAGE_SIZE_LIMIT;
    zip->archive.m_pAlloc = zip_allocate;
    zip->archive.m_pFree = zip_deallocate;
    zip->archive.m_pRealloc = zip_reallocate;
    zip->archive.m_pAlloc_opaque = &zip->budget;
    reader->backend = zip;
    reader->extract = zip_extract;
    reader->close = zip_close;
    if (!mz_zip_reader_init_mem(&zip->archive, data, size, 0)) {
        return archive_budget_result(&zip->budget);
    }

    zip->initialized = true;
    mz_uint count = mz_zip_reader_get_num_files(&zip->archive);
    if (count > NES_ARCHIVE_ENTRY_LIMIT) {
        return NES_MEDIA_TOO_LARGE;
    }

    for (mz_uint i = 0; i < count; i++) {
        mz_zip_archive_file_stat info;
        if (!mz_zip_reader_file_stat(&zip->archive, i, &info)) {
            return NES_MEDIA_INVALID;
        }

        if (info.m_is_directory) {
            continue;
        }

        mz_uint name_size = mz_zip_reader_get_filename(&zip->archive, i, NULL, 0);
        if (!name_size || name_size > NES_FILE_PATH_LIMIT) {
            return NES_MEDIA_TOO_LARGE;
        }

        char *raw_name = (char *)malloc(name_size);
        if (!raw_name) {
            return NES_MEDIA_OUT_OF_MEMORY;
        }

        if (mz_zip_reader_get_filename(&zip->archive, i, raw_name, name_size) != name_size
            || strlen(raw_name) + 1 != name_size) {
            free(raw_name);
            return NES_MEDIA_INVALID;
        }

        char *name = raw_name;
        if (!(info.m_bit_flag & 0x800u)) {
            name = archive_cp437_name((const uint8_t *)raw_name, name_size - 1);
            free(raw_name);
            if (!name) {
                return NES_MEDIA_OUT_OF_MEMORY;
            }
        }

        bool supported_name = archive_supported_name(name);
        bool symbolic_link = ((info.m_external_attr >> 16) & 0170000u) == 0120000u;
        NesMediaResult result = NES_MEDIA_OK;
        if (supported_name && (!info.m_is_supported || symbolic_link)) {
            result = NES_MEDIA_UNSUPPORTED;
        } else if (supported_name && info.m_uncomp_size > NES_IMAGE_SIZE_LIMIT) {
            result = NES_MEDIA_TOO_LARGE;
        } else {
            result = archive_add_entry(reader, name, (size_t)info.m_uncomp_size, i);
        }

        free(name);
        if (result != NES_MEDIA_OK) {
            return result;
        }
    }

    return NES_MEDIA_OK;
}
