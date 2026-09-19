/*
 * nsf.h - NSF and NSFe music image parsing
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#ifndef CUPID_NSF_H
#define CUPID_NSF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
    NSF_MAX_TRACKS = 256,
    NSF_TRACK_NAME_BYTES = 128,
    NSF_SOUND_VRC6 = 0x01,
    NSF_SOUND_VRC7 = 0x02,
    NSF_SOUND_FDS = 0x04,
    NSF_SOUND_MMC5 = 0x08,
    NSF_SOUND_NAMCO163 = 0x10,
    NSF_SOUND_SUNSOFT5B = 0x20,
    NSF_SOUND_SUPPORTED = 0x3F
};

typedef struct {
    uint8_t total_songs;
    uint8_t starting_song; /* zero based internally */
    uint16_t load_address;
    uint16_t init_address;
    uint16_t play_address;
    uint16_t play_speed_ntsc;
    uint16_t play_speed_pal;
    uint8_t bank_setup[8];
    uint8_t region_flags; /* 0 NTSC, 1 PAL, 2 dual */
    uint8_t sound_chips;
    char title[256];
    char artist[256];
    char copyright[256];
    char ripper[256];
    char track_names[NSF_MAX_TRACKS][NSF_TRACK_NAME_BYTES];
    int32_t track_length[NSF_MAX_TRACKS];
    int32_t track_fade[NSF_MAX_TRACKS];
    bool nsfe;
} NsfMetadata;

typedef struct {
    NsfMetadata metadata;
    uint8_t *program;
    size_t program_size;
    size_t payload_size;
} NsfImage;

bool nsf_parse_image(const uint8_t *data, size_t size, NsfImage *image);
void nsf_image_free(NsfImage *image);

#endif
