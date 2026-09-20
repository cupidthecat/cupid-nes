/*
 * image_source.h - Archive selection and patched image loading
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_IMAGE_SOURCE_H
#define CUPID_IMAGE_SOURCE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "../rom/rom.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { NES_IMAGE_SIZE_LIMIT = 256 * 1024 * 1024, NES_ARCHIVE_ENTRY_LIMIT = 4096 };

typedef enum {
    NES_MEDIA_OK,
    NES_MEDIA_NOT_FOUND,
    NES_MEDIA_IO_ERROR,
    NES_MEDIA_INVALID,
    NES_MEDIA_TOO_LARGE,
    NES_MEDIA_OUT_OF_MEMORY,
    NES_MEDIA_NEEDS_SELECTION,
    NES_MEDIA_MEMBER_NOT_FOUND,
    NES_MEDIA_UNSUPPORTED,
    NES_MEDIA_PATCH_ERROR,
    NES_MEDIA_LOAD_ERROR
} NesMediaResult;

typedef struct {
    char *name;
    size_t size;
} NesArchiveEntry;

typedef struct {
    NesArchiveEntry *entries;
    size_t count;
} NesArchiveList;

typedef struct {
    const char *path;
    const char *member;     /* NULL selects the sole supported archive member. */
    const char *patch_path; /* Optional IPS, UPS, or BPS file. */
} NesImageRequest;

typedef struct {
    uint8_t *data;
    size_t size;
    char *path;
    char *member;
    char *patch_path;
    char *save_path;
    uint32_t source_crc32;
    uint32_t patch_crc32;
    bool archived;
    bool patched;
} NesImageSource;

typedef struct {
    const char *fds_bios;
    const char *studybox_bios;
    bool disk_write_protected;
    FdsSaveMode disk_save_mode;
    const char *disk_overlay_path;
} NesImageLoadOptions;

/* Listing and preparation do not touch the running machine or persistent
 * saves. Every returned allocation belongs to the matching free function. */
NesMediaResult nes_image_list(const char *path, NesArchiveList *list,
                              char *error, size_t error_size);
void nes_archive_list_free(NesArchiveList *list);
NesMediaResult nes_image_prepare(const NesImageRequest *request, NesImageSource *source,
                                 char *error, size_t error_size);
void nes_image_source_free(NesImageSource *source);

/* Commit through the production cartridge/music/disk/StudyBox loaders after
 * archive and patch validation. Firmware paths may be NULL for cartridges.
 * Archived or patched disks use separate overlays instead of source writes. */
NesMediaResult nes_image_load(const NesImageSource *source, const char *fds_bios,
                              const char *studybox_bios, bool disk_write_protected,
                              char *error, size_t error_size);
NesMediaResult nes_image_load_with_options(const NesImageSource *source,
                                           const NesImageLoadOptions *options,
                                           char *error, size_t error_size);
const char *nes_media_result_message(NesMediaResult result);

#ifdef __cplusplus
}
#endif
#endif
