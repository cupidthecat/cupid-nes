/*
 * fcm.c - Bounded legacy movie conversion
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "fcm.h"
#include "fm2_internal.h"
#include <stdlib.h>
#include <string.h>

static uint32_t word(const uint8_t *bytes) {
    return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 | (uint32_t)bytes[2] << 16 | (uint32_t)bytes[3] << 24;
}

static char *copy_text(const uint8_t *text, size_t length) {
    char *copy = malloc(length + 1);
    if (copy) {
        memcpy(copy, text, length);
        copy[length] = 0;
    }
    return copy;
}

NesFm2Result nes_fcm_convert(const uint8_t *data, size_t size, const NesFm2Limits *limits, NesFm2Movie *out,
                             NesFm2Diagnostic *diagnostic) {
    nes_fm2__diagnostic_clear(diagnostic);
    NesFm2Limits defaults = nes_fm2_default_limits();
    if (!limits) {
        limits = &defaults;
    }
    if (!data || !out) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "Choose a legacy recording.");
    }
    if (size > limits->max_file_bytes) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, 0, 0, "Legacy movie exceeds the file limit.");
    }
    if (size < 54) {
        return nes_fm2__fail(diagnostic, NES_FM2_TRUNCATED, size, 0, "Truncated legacy movie header.");
    }
    if (memcmp(data, "FCM\x1a", 4) || word(data + 4) != 2) {
        return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, 0, 0, "Only version-two FCM movies are supported.");
    }
    uint8_t flags = data[8];
    if (data[9] || data[10] || data[11] || (flags & 0xe0)) {
        return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, 8, 0, "Unsupported legacy movie flags.");
    }
    if (!(flags & 8) || (flags & 2)) {
        return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, 8, 0,
                             "Reset-start and save-state movies need startup memory that cannot be converted.");
    }
    if (!(flags & 16)) {
        return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, 8, 0,
                             "This movie needs a legacy synchronization hack that Cupid does not emulate.");
    }
    size_t frames = word(data + 12), stream_size = word(data + 20), stream = word(data + 28);
    size_t state = word(data + 24);
    if (frames > limits->max_frames || frames > SIZE_MAX / sizeof(NesFm2Frame)) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, 12, 0, "Legacy movie exceeds the frame limit.");
    }
    if (stream < 54 || stream > size || stream_size > size - stream ||
        (state != UINT32_MAX && (state < 54 || state > stream))) {
        return nes_fm2__fail(diagnostic, NES_FM2_TRUNCATED, 20, 0, "Invalid legacy movie offsets or length.");
    }
    size_t metadata_end = state == UINT32_MAX ? stream : state;
    const uint8_t *name_end = memchr(data + 52, 0, metadata_end - 52);
    const uint8_t *author_end = name_end ? memchr(name_end + 1, 0, (size_t)(data + metadata_end - name_end - 1)) : NULL;
    if (!author_end) {
        return nes_fm2__fail(diagnostic, NES_FM2_TRUNCATED, 52, 0, "Unterminated legacy movie metadata.");
    }
    size_t name_length = (size_t)(name_end - data - 52), author_length = (size_t)(author_end - name_end - 1);
    if (name_length + author_length > limits->max_text_bytes || (author_length && !limits->max_comments)) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, 52, 0, "Legacy metadata exceeds the text limit.");
    }
    for (const uint8_t *p = data + 52; p < author_end; ++p) {
        if (*p && (*p < 32 || *p > 126)) {
            return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, (size_t)(p - data), 0,
                                 "Legacy metadata must be printable ASCII.");
        }
    }
    NesFm2Movie movie;
    nes_fm2_movie_init(&movie);
    movie.pal = (flags & 4) != 0;
    movie.emu_version = word(data + 48);
    movie.rerecord_count = word(data + 16);
    movie.frame_count = frames;
    movie.ports[0] = movie.ports[1] = NES_FM2_PORT_GAMEPAD;
    memcpy(movie.rom_md5, data + 32, 16);
    movie.rom_filename = copy_text(data + 52, name_length);
    movie.frames = frames ? calloc(frames, sizeof(*movie.frames)) : NULL;
    if (author_length) {
        movie.comments.items = calloc(1, sizeof(*movie.comments.items));
        if (movie.comments.items) {
            movie.comments.count = 1;
            movie.comments.items[0] = copy_text(name_end + 1, author_length);
        }
    }
    NesFm2Result result = NES_FM2_OK;
    const char *error = "";
    size_t cursor = stream, event_frame = 0, filled = 0;
    uint8_t pads[4] = {0};
    if (!movie.rom_filename || (frames && !movie.frames) ||
        (author_length && (!movie.comments.items || !movie.comments.items[0]))) {
        result = NES_FM2_OUT_OF_MEMORY;
        error = "Not enough memory to convert the legacy recording.";
    }
    while (result == NES_FM2_OK && cursor < stream + stream_size) {
        unsigned event = data[cursor++], bytes = (event >> 5) & 3;
        uint32_t delta = 0;
        if (bytes > stream + stream_size - cursor) {
            result = NES_FM2_TRUNCATED;
            error = "Truncated legacy event timestamp.";
            break;
        }
        for (unsigned i = 0; i < bytes; ++i) {
            delta |= (uint32_t)data[cursor++] << (8 * i);
        }
        /* Older encoders wrote an extra high byte for a zero one/two-byte delta. */
        if ((bytes == 1 || bytes == 2) && !delta) {
            if (cursor == stream + stream_size) {
                result = NES_FM2_TRUNCATED;
                error = "Truncated legacy extended timestamp.";
                break;
            }
            delta = (uint32_t)data[cursor++] << (8 * bytes);
        }
        if (delta > frames - event_frame) {
            result = NES_FM2_CORRUPT;
            error = "Legacy event lies beyond the declared movie length.";
            break;
        }
        event_frame += delta;
        for (; filled < event_frame; ++filled) {
            memcpy(movie.frames[filled].pads, pads, sizeof(pads));
        }
        if (event & 0x80) {
            unsigned command = event & 31;
            if (command > 2) {
                result = NES_FM2_UNSUPPORTED;
                error = "Unsupported legacy system command.";
                break;
            }
            if (command && event_frame == frames) {
                result = NES_FM2_CORRUPT;
                error = "System command has no movie frame.";
                break;
            }
            if (command) {
                if (movie.frames[event_frame].commands) {
                    result = NES_FM2_UNSUPPORTED;
                    error = "Multiple system commands at one frame cannot be converted without losing their order.";
                    break;
                }
                movie.frames[event_frame].commands = command == 1 ? NES_FM2_COMMAND_RESET : NES_FM2_COMMAND_POWER;
            }
        } else {
            if (event_frame == frames) {
                result = NES_FM2_CORRUPT;
                error = "Controller event has no movie frame.";
                break;
            }
            unsigned player = (event >> 3) & 3;
            pads[player] ^= (uint8_t)(1u << (event & 7));
            movie.fourscore |= player >= 2;
        }
    }
    if (result != NES_FM2_OK) {
        nes_fm2_movie_free(&movie);
        return nes_fm2__fail(diagnostic, result, cursor, event_frame, "%s", error);
    }
    for (; filled < frames; ++filled) {
        memcpy(movie.frames[filled].pads, pads, sizeof(pads));
    }
    if (movie.fourscore) {
        movie.ports[0] = movie.ports[1] = NES_FM2_PORT_NONE;
    }
    nes_fm2_movie_free(out);
    *out = movie;
    return NES_FM2_OK;
}
