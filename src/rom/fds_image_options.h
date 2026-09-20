/*
 * fds_image_options.h - Prepared disk save overlays
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_FDS_IMAGE_OPTIONS_H
#define CUPID_FDS_IMAGE_OPTIONS_H

static char *fds_overlay_path(const char *image_path, const uint8_t *disk, size_t size) {
    if (!image_path || !*image_path) return NULL;
    size_t path_size = strlen(image_path);
    if (path_size > NES_FILE_PATH_LIMIT - 32) return NULL;
    const char *slash = strrchr(image_path, '/');
    const char *backslash = strrchr(image_path, '\\');
    if (backslash && (!slash || backslash > slash)) slash = backslash;
    const char *dot = strrchr(image_path, '.');
    size_t stem = dot && (!slash || dot > slash) ? (size_t)(dot - image_path) : path_size;
    char *path = (char *)malloc(stem + 32);
    if (!path) return NULL;
    memcpy(path, image_path, stem);
    snprintf(path + stem, 32, ".%08" PRIx32 ".ips", game_db_crc32(disk, size));
    return path;
}

FdsImage *fds_image_create_options(const uint8_t *disk, size_t disk_size,
                                   const uint8_t *bios, size_t bios_size,
                                   const char *disk_path, const FdsLoadOptions *options) {
    if (!options || (options->mode != FDS_SAVE_IN_PLACE && options->mode != FDS_SAVE_OVERLAY)) {
        return NULL;
    }

    if (options->mode == FDS_SAVE_IN_PLACE && options->overlay_path) return NULL;
    FdsImage *image = fds_image_create_data(disk, disk_size, bios, bios_size,
                                            options->mode == FDS_SAVE_IN_PLACE ? disk_path : NULL,
                                            options->write_protected);
    if (!image) return NULL;
    image->original_disk = (uint8_t *)malloc(disk_size);
    if (!image->original_disk) {
        fds_image_destroy(image);
        return NULL;
    }

    memcpy(image->original_disk, disk, disk_size);
    image->original_disk_size = disk_size;
    image->save_mode = options->mode;
    if (options->mode == FDS_SAVE_IN_PLACE) return image;

    image->disk_path = options->overlay_path ? fds_strdup(options->overlay_path)
                                           : fds_overlay_path(disk_path, disk, disk_size);
    if (!image->disk_path || !*image->disk_path || strlen(image->disk_path) >= NES_FILE_PATH_LIMIT) {
        fds_image_destroy(image);
        return NULL;
    }

    /* The destination must not alias the original through another spelling,
     * a hard link, or a symbolic link. Missing overlay files are expected. */
    if (disk_path) {
        bool same = false;
        NesFileResult compared = nes_file_same(disk_path, image->disk_path, &same);
        if (same || (compared != NES_FILE_OK && compared != NES_FILE_NOT_FOUND)) {
            fds_image_destroy(image);
            return NULL;
        }
    }

    uint8_t *patch = NULL;
    size_t patch_size = 0;
    NesFileResult read = nes_file_read_all(image->disk_path, 32u * 1024u * 1024u, &patch, &patch_size);
    if (read == NES_FILE_NOT_FOUND) return image;
    if (read != NES_FILE_OK) {
        fds_image_destroy(image);
        return NULL;
    }

    uint8_t *patched = NULL;
    size_t patched_size = 0;
    NesPatchResult result = patch_size >= 5 && !memcmp(patch, "PATCH", 5)
        ? nes_patch_apply(disk, disk_size, patch, patch_size, disk_size, &patched, &patched_size)
        : NES_PATCH_INVALID;
    free(patch);
    size_t prefix = image->headered ? sizeof(image->header) : 0;
    if (result != NES_PATCH_OK || patched_size != disk_size
        || (prefix && memcmp(patched, image->header, prefix))) {
        free(patched);
        fds_image_destroy(image);
        return NULL;
    }

    for (size_t side = 0; side < image->side_count; side++) {
        FdsSide *target = &image->sides[side];
        memcpy(target->raw, patched + prefix + side * image->side_capacity, image->side_capacity);
        free(target->drive);
        target->drive = NULL;
        target->drive_size = 0;
        if (!build_drive_side(target, image->side_capacity, image->qd_format)) {
            free(patched);
            fds_image_destroy(image);
            return NULL;
        }
    }

    free(patched);
    return image;
}

FdsImage *fds_image_create(const uint8_t *disk, size_t disk_size,
                           const uint8_t *bios, size_t bios_size,
                           const char *disk_path, bool write_protected) {
    FdsLoadOptions options = {FDS_SAVE_IN_PLACE, NULL, write_protected};
    return fds_image_create_options(disk, disk_size, bios, bios_size, disk_path, &options);
}

#endif
