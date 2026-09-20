/*
 * patch.c - Validated image patch decoding
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "patch.h"
#include "../rom/game_db.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    const uint8_t *data;
    size_t size;
    size_t position;
} PatchReader;

static bool take(PatchReader *reader, size_t count, const uint8_t **bytes) {
    if (count > reader->size - reader->position) {
        return false;
    }

    *bytes = reader->data + reader->position;
    reader->position += count;
    return true;
}

static uint32_t little32(const uint8_t *data) {
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8)
           | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static size_t big24(const uint8_t *data) {
    return ((size_t)data[0] << 16) | ((size_t)data[1] << 8) | data[2];
}

static bool number(PatchReader *reader, size_t *result) {
    size_t value = 0;
    size_t weight = 1;
    for (;;) {
        const uint8_t *byte;
        if (!take(reader, 1, &byte)) {
            return false;
        }

        size_t digit = *byte & 0x7fu;
        if (digit && weight > (SIZE_MAX - value) / digit) {
            return false;
        }

        value += digit * weight;
        if (*byte & 0x80u) {
            *result = value;
            return true;
        }

        if (weight > SIZE_MAX / 128) {
            return false;
        }

        weight *= 128;
        if (value > SIZE_MAX - weight) {
            return false;
        }

        value += weight;
    }
}

static bool move_relative(size_t *position, size_t encoded, size_t limit) {
    size_t distance = encoded >> 1;
    if (encoded & 1u) {
        if (distance > *position) {
            return false;
        }

        *position -= distance;
    } else {
        if (*position > limit || distance > limit - *position) {
            return false;
        }

        *position += distance;
    }

    return *position <= limit;
}

static NesPatchResult ips_pass(const uint8_t *patch, size_t patch_size,
                                uint8_t *output, size_t source_size,
                                size_t limit, size_t *output_size) {
    PatchReader reader = {patch, patch_size, 5};
    size_t size = source_size;
    for (;;) {
        const uint8_t *record;
        if (!take(&reader, 3, &record)) {
            return NES_PATCH_INVALID;
        }

        if (!memcmp(record, "EOF", 3)) {
            size_t remainder = reader.size - reader.position;
            if (remainder != 0 && remainder != 3) {
                return NES_PATCH_INVALID;
            }

            if (remainder) {
                (void)take(&reader, 3, &record);
                size_t truncate = big24(record);
                if (size > truncate) {
                    size = truncate;
                }
            }

            *output_size = size;
            return NES_PATCH_OK;
        }

        size_t offset = big24(record);
        if (!take(&reader, 2, &record)) {
            return NES_PATCH_INVALID;
        }

        size_t count = ((size_t)record[0] << 8) | record[1];
        bool repeat = count == 0;
        if (repeat) {
            if (!take(&reader, 3, &record)) {
                return NES_PATCH_INVALID;
            }

            count = ((size_t)record[0] << 8) | record[1];
            if (!count) {
                return NES_PATCH_INVALID;
            }
        } else if (!take(&reader, count, &record)) {
            return NES_PATCH_INVALID;
        }

        if (offset > limit || count > limit - offset) {
            return NES_PATCH_TOO_LARGE;
        }

        if (size < offset + count) {
            size = offset + count;
        }

        if (output) {
            if (repeat) {
                memset(output + offset, record[2], count);
            } else {
                memcpy(output + offset, record, count);
            }
        }
    }
}

static NesPatchResult apply_ips(const uint8_t *source, size_t source_size,
                                 const uint8_t *patch, size_t patch_size,
                                 size_t limit, uint8_t **output, size_t *output_size) {
    /* The first pass also determines the pre-truncation allocation. A record
     * may write past the final size before the optional truncation is applied. */
    size_t final_size = 0;
    NesPatchResult result = ips_pass(patch, patch_size, NULL, source_size, limit, &final_size);
    if (result != NES_PATCH_OK) {
        return result;
    }

    size_t allocation = source_size;
    PatchReader reader = {patch, patch_size, 5};
    for (;;) {
        const uint8_t *bytes;
        (void)take(&reader, 3, &bytes);
        if (!memcmp(bytes, "EOF", 3)) {
            break;
        }

        size_t offset = big24(bytes);
        (void)take(&reader, 2, &bytes);
        size_t count = ((size_t)bytes[0] << 8) | bytes[1];
        if (count) {
            (void)take(&reader, count, &bytes);
        } else {
            (void)take(&reader, 3, &bytes);
            count = ((size_t)bytes[0] << 8) | bytes[1];
        }

        if (allocation < offset + count) {
            allocation = offset + count;
        }
    }

    uint8_t *data = (uint8_t *)calloc(allocation ? allocation : 1, 1);
    if (!data) {
        return NES_PATCH_OUT_OF_MEMORY;
    }

    if (source_size) {
        memcpy(data, source, source_size);
    }

    result = ips_pass(patch, patch_size, data, source_size, limit, &final_size);
    if (result != NES_PATCH_OK) {
        free(data);
        return result;
    }

    *output = data;
    *output_size = final_size;
    return NES_PATCH_OK;
}

static NesPatchResult apply_ups(const uint8_t *source, size_t source_size,
                                 const uint8_t *patch, size_t patch_size,
                                 size_t limit, uint8_t **output, size_t *output_size) {
    PatchReader reader = {patch, patch_size - 12, 4};
    size_t before, after;
    if (!number(&reader, &before) || !number(&reader, &after)) {
        return NES_PATCH_INVALID;
    }

    if (before > limit || after > limit) {
        return NES_PATCH_TOO_LARGE;
    }

    uint32_t source_crc = game_db_crc32(source, source_size);
    bool forward = before == source_size && source_crc == little32(patch + patch_size - 12);
    bool reverse = after == source_size && source_crc == little32(patch + patch_size - 8);
    if (!forward && !reverse) {
        return NES_PATCH_CHECKSUM;
    }

    size_t target_size = forward ? after : before;
    size_t span = before > after ? before : after;
    uint8_t *data = (uint8_t *)calloc(span ? span : 1, 1);
    if (!data) {
        return NES_PATCH_OUT_OF_MEMORY;
    }

    if (source_size) {
        memcpy(data, source, source_size);
    }

    size_t position = 0;
    while (reader.position < reader.size) {
        size_t distance;
        if (!number(&reader, &distance) || position > span || distance > span - position) {
            free(data);
            return NES_PATCH_INVALID;
        }

        position += distance;
        for (;;) {
            const uint8_t *byte;
            if (!take(&reader, 1, &byte) || (position >= span && *byte != 0)) {
                free(data);
                return NES_PATCH_INVALID;
            }

            if (position < span) {
                data[position] ^= *byte;
            }

            position++;
            if (!*byte) {
                break;
            }
        }
    }

    uint32_t target_crc = little32(patch + patch_size - (forward ? 8 : 12));
    if (game_db_crc32(data, target_size) != target_crc) {
        free(data);
        return NES_PATCH_CHECKSUM;
    }

    *output = data;
    *output_size = target_size;
    return NES_PATCH_OK;
}

static NesPatchResult apply_bps(const uint8_t *source, size_t source_size,
                                 const uint8_t *patch, size_t patch_size,
                                 size_t limit, uint8_t **output, size_t *output_size) {
    PatchReader reader = {patch, patch_size - 12, 4};
    size_t declared_source, target_size, metadata_size;
    if (!number(&reader, &declared_source) || !number(&reader, &target_size)
        || !number(&reader, &metadata_size) || metadata_size > reader.size - reader.position) {
        return NES_PATCH_INVALID;
    }

    if (target_size > limit) {
        return NES_PATCH_TOO_LARGE;
    }

    if (declared_source != source_size
        || game_db_crc32(source, source_size) != little32(patch + patch_size - 12)) {
        return NES_PATCH_CHECKSUM;
    }

    reader.position += metadata_size;
    uint8_t *data = (uint8_t *)malloc(target_size ? target_size : 1);
    if (!data) {
        return NES_PATCH_OUT_OF_MEMORY;
    }

    size_t position = 0, source_relative = 0, target_relative = 0;
    while (position < target_size) {
        size_t command, delta;
        const uint8_t *literal;
        if (!number(&reader, &command)) {
            goto invalid;
        }

        size_t count = (command >> 2) + 1;
        if (count > target_size - position) {
            goto invalid;
        }

        switch (command & 3u) {
            case 0:
                if (position > source_size || count > source_size - position) {
                    goto invalid;
                }

                memcpy(data + position, source + position, count);
                position += count;
                break;
            case 1:
                if (!take(&reader, count, &literal)) {
                    goto invalid;
                }

                memcpy(data + position, literal, count);
                position += count;
                break;
            case 2:
                if (!number(&reader, &delta)
                    || !move_relative(&source_relative, delta, source_size)
                    || count > source_size - source_relative) {
                    goto invalid;
                }

                memcpy(data + position, source + source_relative, count);
                source_relative += count;
                position += count;
                break;
            case 3:
                if (!number(&reader, &delta)
                    || !move_relative(&target_relative, delta, target_size)
                    || target_relative >= position) {
                    goto invalid;
                }

                /* Overlap repeats already produced bytes, one byte at a time. */
                while (count--) {
                    data[position++] = data[target_relative++];
                }

                break;
        }
    }

    if (reader.position != reader.size) {
        goto invalid;
    }

    if (game_db_crc32(data, target_size) != little32(patch + patch_size - 8)) {
        free(data);
        return NES_PATCH_CHECKSUM;
    }

    *output = data;
    *output_size = target_size;
    return NES_PATCH_OK;

invalid:
    free(data);
    return NES_PATCH_INVALID;
}

NesPatchResult nes_patch_apply(const uint8_t *source, size_t source_size,
                               const uint8_t *patch, size_t patch_size,
                               size_t limit, uint8_t **output, size_t *output_size) {
    if (output) {
        *output = NULL;
    }

    if (output_size) {
        *output_size = 0;
    }

    if (!output || !output_size || (!source && source_size) || !patch || patch_size < 4) {
        return NES_PATCH_INVALID;
    }

    if (source_size > limit) {
        return NES_PATCH_TOO_LARGE;
    }

    if (patch_size >= 5 && !memcmp(patch, "PATCH", 5)) {
        return apply_ips(source, source_size, patch, patch_size, limit, output, output_size);
    }

    bool ups = !memcmp(patch, "UPS1", 4);
    bool bps = !memcmp(patch, "BPS1", 4);
    if (!ups && !bps) {
        return NES_PATCH_UNSUPPORTED;
    }

    if (patch_size < (bps ? 19u : 18u)) {
        return NES_PATCH_INVALID;
    }

    if (game_db_crc32(patch, patch_size - 4) != little32(patch + patch_size - 4)) {
        return NES_PATCH_CHECKSUM;
    }

    return ups ? apply_ups(source, source_size, patch, patch_size, limit, output, output_size)
               : apply_bps(source, source_size, patch, patch_size, limit, output, output_size);
}

const char *nes_patch_result_message(NesPatchResult result) {
    switch (result) {
        case NES_PATCH_OK: return "Patch applied";
        case NES_PATCH_UNSUPPORTED: return "Expected an IPS, UPS, or BPS patch";
        case NES_PATCH_INVALID: return "Patch is truncated or contains invalid offsets";
        case NES_PATCH_CHECKSUM: return "Patch, source, or result checksum does not match";
        case NES_PATCH_TOO_LARGE: return "Patched image exceeds the supported size limit";
        case NES_PATCH_OUT_OF_MEMORY: return "Not enough memory to apply the patch";
        default: return "Unknown patch error";
    }
}
