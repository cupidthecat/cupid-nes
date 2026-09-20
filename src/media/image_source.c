/*
 * image_source.c - Prepared archive members and image patches
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "image_source.h"
#include "archive_internal.h"
#include "patch.h"
#include "../rom/rom.h"
#include "../rom/game_db.h"
#include "../util/file_io.h"
#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static NesMediaResult report(NesMediaResult result, char *error, size_t size) {
    if (error && size) {
        snprintf(error, size, "%s", result == NES_MEDIA_OK ? "" : nes_media_result_message(result));
    }

    return result;
}

static NesMediaResult file_result(NesFileResult result) {
    switch (result) {
        case NES_FILE_OK: return NES_MEDIA_OK;
        case NES_FILE_NOT_FOUND: return NES_MEDIA_NOT_FOUND;
        case NES_FILE_TOO_LARGE: return NES_MEDIA_TOO_LARGE;
        case NES_FILE_OUT_OF_MEMORY: return NES_MEDIA_OUT_OF_MEMORY;
        case NES_FILE_INVALID_ARGUMENT: return NES_MEDIA_INVALID;
        default: return NES_MEDIA_IO_ERROR;
    }
}

static char *copy_string(const char *source) {
    size_t size = strlen(source) + 1;
    if (size > NES_FILE_PATH_LIMIT) {
        return NULL;
    }

    char *copy = (char *)malloc(size);
    if (copy) {
        memcpy(copy, source, size);
    }

    return copy;
}

static uint64_t member_identity(const char *name) {
    uint64_t hash = UINT64_C(14695981039346656037);
    for (const unsigned char *next = (const unsigned char *)name; *next; next++) {
        hash = (hash ^ *next) * UINT64_C(1099511628211);
    }

    return hash;
}

static char *save_identity(const NesImageSource *source) {
    if (!source->archived && !source->patched) {
        return copy_string(source->path);
    }

    const char *slash = strrchr(source->path, '/');
    const char *backslash = strrchr(source->path, '\\');
    if (backslash && (!slash || backslash > slash)) {
        slash = backslash;
    }

    const char *dot = strrchr(source->path, '.');
    size_t prefix = dot && (!slash || dot > slash) ? (size_t)(dot - source->path) : strlen(source->path);
    const char *member = source->member ? source->member : "";
    uint32_t final_crc = game_db_crc32(source->data, source->size);
    char suffix[100];
    int suffix_size = snprintf(suffix, sizeof(suffix), ".%016" PRIx64 "-%08" PRIx32 "-%08" PRIx32 ".image",
                               member_identity(member), source->patched ? source->patch_crc32 : 0, final_crc);
    if (suffix_size < 0 || (size_t)suffix_size >= sizeof(suffix)
        || prefix + (size_t)suffix_size >= NES_FILE_PATH_LIMIT) {
        return NULL;
    }

    char *path = (char *)malloc(prefix + (size_t)suffix_size + 1);
    if (path) {
        memcpy(path, source->path, prefix);
        memcpy(path + prefix, suffix, (size_t)suffix_size + 1);
    }

    return path;
}

static bool archive_signature(const uint8_t *data, size_t size) {
    return (size >= 2 && !memcmp(data, "PK", 2))
        || (size >= 2 && !memcmp(data, "7z", 2));
}

NesMediaResult nes_image_list(const char *path, NesArchiveList *list,
                              char *error, size_t error_size) {
    if (!list) {
        return report(NES_MEDIA_INVALID, error, error_size);
    }

    memset(list, 0, sizeof(*list));
    uint8_t *data = NULL;
    size_t size = 0;
    NesMediaResult result = file_result(nes_file_read_all(path, NES_IMAGE_SIZE_LIMIT, &data, &size));
    if (result != NES_MEDIA_OK) {
        return report(result, error, error_size);
    }

    ArchiveReader reader;
    result = archive_open(data, size, &reader);
    if (result == NES_MEDIA_OK) {
        *list = reader.list;
        reader.list = (NesArchiveList){0};
        archive_close(&reader);
    }

    free(data);
    return report(result, error, error_size);
}

void nes_image_source_free(NesImageSource *source) {
    if (source) {
        free(source->data);
        free(source->path);
        free(source->member);
        free(source->save_path);
        memset(source, 0, sizeof(*source));
    }
}

static NesMediaResult prepare_archive(const NesImageRequest *request, NesImageSource *source) {
    ArchiveReader reader;
    NesMediaResult result = archive_open(source->data, source->size, &reader);
    if (result != NES_MEDIA_OK) {
        return result;
    }

    size_t selected = SIZE_MAX;
    if (request->member && *request->member) {
        for (size_t i = 0; i < reader.list.count; i++) {
            if (!strcmp(reader.list.entries[i].name, request->member)) {
                selected = i;
                break;
            }
        }

        if (selected == SIZE_MAX) {
            result = NES_MEDIA_MEMBER_NOT_FOUND;
        }
    } else if (reader.list.count == 1) {
        selected = 0;
    } else {
        result = NES_MEDIA_NEEDS_SELECTION;
    }

    if (result == NES_MEDIA_OK) {
        char *member = copy_string(reader.list.entries[selected].name);
        uint8_t *bytes = NULL;
        size_t size = 0;
        result = member ? reader.extract(reader.backend, reader.indices[selected], &bytes, &size)
                        : NES_MEDIA_OUT_OF_MEMORY;
        if (result == NES_MEDIA_OK) {
            free(source->data);
            source->data = bytes;
            source->size = size;
            source->member = member;
            source->archived = true;
        } else {
            free(member);
            free(bytes);
        }
    }

    archive_close(&reader);
    return result;
}

NesMediaResult nes_image_prepare(const NesImageRequest *request, NesImageSource *source,
                                 char *error, size_t error_size) {
    if (!source) {
        return report(NES_MEDIA_INVALID, error, error_size);
    }

    memset(source, 0, sizeof(*source));
    if (!request || !request->path || !*request->path) {
        return report(NES_MEDIA_INVALID, error, error_size);
    }

    NesMediaResult result = file_result(nes_file_read_all(request->path, NES_IMAGE_SIZE_LIMIT,
                                                          &source->data, &source->size));
    if (result != NES_MEDIA_OK) {
        return report(result, error, error_size);
    }

    if (archive_signature(source->data, source->size)) {
        result = prepare_archive(request, source);
    } else if (request->member && *request->member) {
        result = NES_MEDIA_MEMBER_NOT_FOUND;
    }

    if (result != NES_MEDIA_OK) {
        nes_image_source_free(source);
        return report(result, error, error_size);
    }

    source->source_crc32 = game_db_crc32(source->data, source->size);
    if (request->patch_path && *request->patch_path) {
        uint8_t *patch = NULL;
        size_t patch_size = 0;
        result = file_result(nes_file_read_all(request->patch_path, NES_IMAGE_SIZE_LIMIT, &patch, &patch_size));
        if (result != NES_MEDIA_OK) {
            nes_image_source_free(source);
            return report(result, error, error_size);
        }

        source->patch_crc32 = game_db_crc32(patch, patch_size);
        uint8_t *output = NULL;
        size_t output_size = 0;
        NesPatchResult patched = nes_patch_apply(source->data, source->size, patch, patch_size,
                                                 NES_IMAGE_SIZE_LIMIT, &output, &output_size);
        free(patch);
        if (patched != NES_PATCH_OK) {
            nes_image_source_free(source);
            if (error && error_size) {
                snprintf(error, error_size, "%s", nes_patch_result_message(patched));
            }

            return NES_MEDIA_PATCH_ERROR;
        }

        free(source->data);
        source->data = output;
        source->size = output_size;
        source->patched = true;
    }

    source->path = copy_string(request->path);
    if (source->path) {
        source->save_path = save_identity(source);
    }

    if (!source->path || !source->save_path) {
        nes_image_source_free(source);
        return report(NES_MEDIA_OUT_OF_MEMORY, error, error_size);
    }

    return report(NES_MEDIA_OK, error, error_size);
}

static bool extension_is(const char *path, const char *extension) {
    const char *dot = path ? strrchr(path, '.') : NULL;
    if (!dot) {
        return false;
    }

    while (*dot && *extension && tolower((unsigned char)*dot) == *extension) {
        dot++;
        extension++;
    }

    return !*dot && !*extension;
}

NesMediaResult nes_image_load(const NesImageSource *source, const char *fds_bios,
                              const char *studybox_bios, bool disk_write_protected,
                              char *error, size_t error_size) {
    if (!source || !source->data || !source->path || !source->save_path) {
        return report(NES_MEDIA_INVALID, error, error_size);
    }

    const char *name = source->member ? source->member : source->path;
    bool disk = (source->size >= 4 && !memcmp(source->data, "FDS\x1a", 4))
             || extension_is(name, ".fds") || extension_is(name, ".qd");
    bool studybox = source->size >= 4 && !memcmp(source->data, "STBX", 4);
    int loaded;
    if (disk || studybox) {
        const char *bios_path = disk ? fds_bios : studybox_bios;
        if (!bios_path || !*bios_path) {
            if (error && error_size) {
                snprintf(error, error_size, "%s", disk ? "Select an 8 KiB disk-system BIOS before opening this image"
                                                        : "Select a 256 KiB StudyBox BIOS before opening this image");
            }

            return NES_MEDIA_LOAD_ERROR;
        }

        uint8_t *bios = NULL;
        size_t bios_size = 0;
        NesMediaResult result = file_result(nes_file_read_all(bios_path, 256u * 1024u, &bios, &bios_size));
        if (result != NES_MEDIA_OK) {
            return report(result, error, error_size);
        }

        if (disk) {
            FdsLoadOptions options = {source->archived || source->patched ? FDS_SAVE_OVERLAY : FDS_SAVE_IN_PLACE,
                                      NULL, disk_write_protected};
            loaded = source->archived || source->patched
                ? load_fds_memory_options(source->data, source->size, bios, bios_size,
                                           source->save_path, &options)
                : load_fds_with_options(source->path, bios_path, &options);
        } else {
            loaded = load_studybox_memory(source->data, source->size, bios, bios_size);
        }

        free(bios);
    } else {
        loaded = load_rom_image(source->data, source->size, source->save_path);
    }

    return report(loaded == 0 ? NES_MEDIA_OK : NES_MEDIA_LOAD_ERROR, error, error_size);
}

const char *nes_media_result_message(NesMediaResult result) {
    switch (result) {
        case NES_MEDIA_OK: return "Image is ready";
        case NES_MEDIA_NOT_FOUND: return "Image or patch file was not found";
        case NES_MEDIA_IO_ERROR: return "Could not read the image or patch file";
        case NES_MEDIA_INVALID: return "Image or archive is truncated, corrupt, or has invalid member names";
        case NES_MEDIA_TOO_LARGE: return "Image or archive exceeds the supported size or entry limit";
        case NES_MEDIA_OUT_OF_MEMORY: return "Not enough memory to prepare the image";
        case NES_MEDIA_NEEDS_SELECTION: return "Choose an image from the archive";
        case NES_MEDIA_MEMBER_NOT_FOUND: return "The archive does not contain the requested supported image";
        case NES_MEDIA_UNSUPPORTED: return "Archive format, encryption, or compression method is not supported";
        case NES_MEDIA_PATCH_ERROR: return "Patch validation failed";
        case NES_MEDIA_LOAD_ERROR: return "The image could not be loaded; check its firmware, hardware options, and unsaved data";
        default: return "Unknown image error";
    }
}
