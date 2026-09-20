/*
 * patch.h - Bounded IPS, UPS, and BPS image patches
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_MEDIA_PATCH_H
#define CUPID_MEDIA_PATCH_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NES_PATCH_OK,
    NES_PATCH_UNSUPPORTED,
    NES_PATCH_INVALID,
    NES_PATCH_CHECKSUM,
    NES_PATCH_TOO_LARGE,
    NES_PATCH_OUT_OF_MEMORY
} NesPatchResult;

/* Success transfers an allocated result to the caller. Failure clears both
 * outputs and leaves the source and patch untouched. UPS accepts either side
 * of a reversible patch when that side's size and checksum match. */
NesPatchResult nes_patch_apply(const uint8_t *source, size_t source_size,
                               const uint8_t *patch, size_t patch_size,
                               size_t limit, uint8_t **output, size_t *output_size);
NesPatchResult nes_patch_create_ips(const uint8_t *source, size_t source_size,
                                    const uint8_t *target, size_t target_size,
                                    size_t limit, uint8_t **output, size_t *output_size);
const char *nes_patch_result_message(NesPatchResult result);

#ifdef __cplusplus
}
#endif
#endif
