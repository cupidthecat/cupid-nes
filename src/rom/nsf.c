/*
 * nsf.c - NSF and NSFe music image parsing
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#include "nsf.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint16_t read_u16(const uint8_t *p) {
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t read_i32(const uint8_t *p) { return (int32_t)read_u32(p); }

static void init_metadata(NsfMetadata *m) {
    memset(m, 0, sizeof(*m));
    for (unsigned i = 0; i < NSF_MAX_TRACKS; ++i) {
        m->track_length[i] = -1;
        m->track_fade[i] = -1;
    }
}

static void copy_fixed_string(char *dst, size_t capacity, const uint8_t *src, size_t length) {
    size_t count = length < capacity - 1 ? length : capacity - 1;
    memcpy(dst, src, count);
    dst[count] = '\0';
    for (size_t i = 0; i < count; ++i) {
        if (!dst[i]) break;
        if ((unsigned char)dst[i] < 0x20 && dst[i] != '\t') dst[i] = ' ';
    }
}

static bool validate_metadata(NsfMetadata *m) {
    if (!m->total_songs || m->load_address < 0x6000) {
        fprintf(stderr, "Invalid NSF load address or track count\n");
        return false;
    }
    if (m->starting_song >= m->total_songs) m->starting_song = 0;
    if (!m->play_speed_ntsc) m->play_speed_ntsc = 16639;
    if (!m->play_speed_pal) m->play_speed_pal = 19997;
    if (m->play_speed_ntsc == 16666 || m->play_speed_ntsc == 16667) m->play_speed_ntsc = 16639;
    if (m->play_speed_pal == 20000) m->play_speed_pal = 19997;
    return true;
}

static bool make_program(const uint8_t *payload, size_t payload_size,
                         uint16_t load_address, NsfImage *image) {
    size_t prefix = load_address & 0x0FFFu;
    if (payload_size > SIZE_MAX - prefix) return false;
    size_t bytes = prefix + payload_size;
    if (!bytes) return false;
    size_t padded = (bytes + 0x0FFFu) & ~(size_t)0x0FFFu;
    if (padded < bytes) return false;
    uint8_t *program = (uint8_t *)calloc(1, padded);
    if (!program) return false;
    memcpy(program + prefix, payload, payload_size);
    image->program = program;
    image->program_size = padded;
    image->payload_size = payload_size;
    return true;
}

static bool parse_nsf(const uint8_t *data, size_t size, NsfImage *image) {
    if (size < 0x80 || memcmp(data, "NESM\x1A", 5) != 0) return false;
    NsfMetadata *m = &image->metadata;
    init_metadata(m);
    m->total_songs = data[6];
    m->starting_song = data[7] ? (uint8_t)(data[7] - 1) : 0;
    m->load_address = read_u16(data + 8);
    m->init_address = read_u16(data + 10);
    m->play_address = read_u16(data + 12);
    copy_fixed_string(m->title, sizeof(m->title), data + 14, 32);
    copy_fixed_string(m->artist, sizeof(m->artist), data + 46, 32);
    copy_fixed_string(m->copyright, sizeof(m->copyright), data + 78, 32);
    m->play_speed_ntsc = read_u16(data + 110);
    memcpy(m->bank_setup, data + 112, sizeof(m->bank_setup));
    m->play_speed_pal = read_u16(data + 120);
    m->region_flags = data[122];
    m->sound_chips = data[123];
    if (!validate_metadata(m)) return false;
    return make_program(data + 0x80, size - 0x80, m->load_address, image);
}

static size_t copy_nsfe_strings(char output[][NSF_TRACK_NAME_BYTES], size_t output_count,
                                const uint8_t *data, size_t size) {
    size_t item = 0, start = 0;
    for (size_t i = 0; i <= size && item < output_count; ++i) {
        if (i != size && data[i]) continue;
        size_t length = i - start;
        copy_fixed_string(output[item], NSF_TRACK_NAME_BYTES, data + start, length);
        item++;
        start = i + 1;
    }
    return item;
}

static bool parse_nsfe(const uint8_t *data, size_t size, NsfImage *image) {
    if (size < 4 || memcmp(data, "NSFE", 4) != 0) return false;
    NsfMetadata *m = &image->metadata;
    init_metadata(m);
    m->nsfe = true;
    bool info_seen = false, data_seen = false, end_seen = false;
    const uint8_t *program = NULL;
    size_t program_size = 0;
    size_t offset = 4;
    while (offset < size) {
        if (size - offset < 8) { fprintf(stderr, "Truncated NSFe chunk header\n"); return false; }
        uint32_t length32 = read_u32(data + offset);
        size_t length = length32;
        const uint8_t *id = data + offset + 4;
        offset += 8;
        if (length > size - offset) { fprintf(stderr, "NSFe chunk exceeds file size\n"); return false; }
        const uint8_t *payload = data + offset;

        if (memcmp(id, "INFO", 4) == 0) {
            if (length < 10) { fprintf(stderr, "Invalid NSFe INFO chunk\n"); return false; }
            m->load_address = read_u16(payload);
            m->init_address = read_u16(payload + 2);
            m->play_address = read_u16(payload + 4);
            m->region_flags = payload[6];
            m->sound_chips = payload[7];
            m->total_songs = payload[8];
            m->starting_song = payload[9];
            m->play_speed_ntsc = 16639;
            m->play_speed_pal = 19997;
            info_seen = true;
        } else if (memcmp(id, "DATA", 4) == 0) {
            if (!info_seen || !length || data_seen) { fprintf(stderr, "Invalid NSFe DATA chunk\n"); return false; }
            program = payload;
            program_size = length;
            data_seen = true;
        } else if (memcmp(id, "NEND", 4) == 0) {
            if (length) { fprintf(stderr, "Invalid NSFe NEND chunk\n"); return false; }
            end_seen = true;
            offset += length;
            break;
        } else if (memcmp(id, "BANK", 4) == 0) {
            memset(m->bank_setup, 0, sizeof(m->bank_setup));
            memcpy(m->bank_setup, payload, length < 8 ? length : 8);
        } else if (memcmp(id, "time", 4) == 0 || memcmp(id, "fade", 4) == 0) {
            if ((length & 3u) || length / 4 > NSF_MAX_TRACKS) {
                fprintf(stderr, "Invalid NSFe timing chunk\n");
                return false;
            }
            int32_t *target = memcmp(id, "time", 4) == 0 ? m->track_length : m->track_fade;
            for (size_t i = 0; i < length / 4; ++i) target[i] = read_i32(payload + i * 4);
        } else if (memcmp(id, "tlbl", 4) == 0) {
            (void)copy_nsfe_strings(m->track_names, NSF_MAX_TRACKS, payload, length);
        } else if (memcmp(id, "auth", 4) == 0) {
            char strings[4][NSF_TRACK_NAME_BYTES] = {{0}};
            size_t count = copy_nsfe_strings(strings, 4, payload, length);
            if (count > 0) copy_fixed_string(m->title, sizeof(m->title), (uint8_t *)strings[0], strlen(strings[0]));
            if (count > 1) copy_fixed_string(m->artist, sizeof(m->artist), (uint8_t *)strings[1], strlen(strings[1]));
            if (count > 2) copy_fixed_string(m->copyright, sizeof(m->copyright), (uint8_t *)strings[2], strlen(strings[2]));
            if (count > 3) copy_fixed_string(m->ripper, sizeof(m->ripper), (uint8_t *)strings[3], strlen(strings[3]));
        } else if (memcmp(id, "plst", 4) == 0 || memcmp(id, "text", 4) == 0) {
            /* Defined by NSFe but intentionally not used by the player. */
        } else if (id[0] >= 'A' && id[0] <= 'Z') {
            fprintf(stderr, "Unsupported required NSFe chunk %.4s\n", id);
            return false;
        }
        offset += length;
    }
    if (!info_seen || !data_seen || !end_seen || !validate_metadata(m)) {
        if (!end_seen) fprintf(stderr, "NSFe image is missing NEND\n");
        return false;
    }
    return make_program(program, program_size, m->load_address, image);
}

bool nsf_parse_image(const uint8_t *data, size_t size, NsfImage *image) {
    if (!image) return false;
    memset(image, 0, sizeof(*image));
    if (!data) return false;
    bool ok = size >= 5 && memcmp(data, "NESM\x1A", 5) == 0
            ? parse_nsf(data, size, image)
            : size >= 4 && memcmp(data, "NSFE", 4) == 0
                ? parse_nsfe(data, size, image) : false;
    if (!ok) nsf_image_free(image);
    return ok;
}

void nsf_image_free(NsfImage *image) {
    if (!image) return;
    free(image->program);
    memset(image, 0, sizeof(*image));
}
