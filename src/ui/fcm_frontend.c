/*
 * fcm_frontend.c - Legacy movie conversion controls
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "fcm_frontend.h"
#include "frontend_panels.h"
#include "platform_frontend.h"
#include "output_guard.h"
#include "../replay/fcm.h"
#include "../replay/tas_startup.h"
#include "../system/timing.h"
#include "../util/file_io.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static struct {
    FrontendExecutionRuntime *execution;
    char source[4096], output[4096], status[256];
} converter;

static bool fail(char *error, size_t capacity, const char *message) {
    if (error && capacity) {
        snprintf(error, capacity, "%s", message);
    }
    return false;
}

bool fcm_frontend_convert(FrontendExecutionRuntime *execution, const char *source, const char *output, char *error,
                          size_t capacity) {
    if (error && capacity) {
        *error = 0;
    }
    if (!source || !*source || !output || !*output) {
        return fail(error, capacity, "Choose the FCM input and FM2 output paths.");
    }
    if (!frontend_output_path_allowed(output, execution, &source, 1, error, capacity)) {
        return false;
    }
    uint8_t digest[16];
    if (!nes_tas_rom_md5(digest)) {
        return fail(error, capacity, "Load the movie's game before converting so its checksum can be checked.");
    }
    NesFm2Limits limits = nes_fm2_default_limits();
    uint8_t *bytes = NULL;
    size_t size = 0;
    NesFileResult read = nes_file_read_all(source, limits.max_file_bytes, &bytes, &size);
    if (read != NES_FILE_OK) {
        return fail(error, capacity, nes_file_result_message(read));
    }
    NesFm2Movie movie;
    NesFm2Diagnostic diagnostic;
    nes_fm2_movie_init(&movie);
    NesFm2Result parsed = nes_fcm_convert(bytes, size, &limits, &movie, &diagnostic);
    free(bytes);
    if (parsed != NES_FM2_OK) {
        nes_fm2_movie_free(&movie);
        return fail(error, capacity, diagnostic.message);
    }
    bool ok = true;
    if (memcmp(digest, movie.rom_md5, sizeof(digest))) {
        ok = fail(error, capacity, "The FCM ROM checksum differs from the loaded game.");
    } else if (nes_timing()->region != (movie.pal ? NES_REGION_PAL : NES_REGION_NTSC)) {
        ok = fail(error, capacity, "Load the game with the recording's NTSC or PAL timing before converting.");
    }
    bytes = NULL;
    size_t written = 0;
    if (ok && nes_fm2_measure(&movie, NES_FM2_TEXT, &size, &diagnostic) != NES_FM2_OK) {
        ok = fail(error, capacity, diagnostic.message);
    }
    if (ok && !(bytes = malloc(size))) {
        ok = fail(error, capacity, "Not enough memory to encode the converted movie.");
    }
    if (ok && nes_fm2_export(&movie, NES_FM2_TEXT, bytes, size, &written, &diagnostic) != NES_FM2_OK) {
        ok = fail(error, capacity, diagnostic.message);
    }
    if (ok) {
        NesFileResult saved = nes_file_write_atomic(output, bytes, written);
        if (saved != NES_FILE_OK) {
            ok = fail(error, capacity, nes_file_result_message(saved));
        }
    }
    free(bytes);
    nes_fm2_movie_free(&movie);
    return ok;
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t capacity) {
    (void)context;
    FrontendPanelControl controls[] = {{FCM_CONVERTER_SOURCE, FRONTEND_PANEL_FILE_OPEN, "FCM source", converter.source,
                                        NULL, 0, FRONTEND_OPEN_LEGACY_MOVIE, true, false},
                                       {FCM_CONVERTER_OUTPUT, FRONTEND_PANEL_FILE_SAVE, "FM2 output", converter.output,
                                        NULL, 0, FRONTEND_SAVE_TAS_MOVIE, true, false},
                                       {FCM_CONVERTER_CONVERT, FRONTEND_PANEL_ACTION, "Validate game and convert", "",
                                        NULL, 0, 0, *converter.source && *converter.output, false}};
    for (size_t i = 0; i < sizeof(controls) / sizeof(*controls); ++i) {
        if (!frontend_panel_add_control(model, &controls[i])) {
            return fail(error, capacity, "Movie converter panel has too few controls.");
        }
    }
    model->status = converter.status;
    return true;
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t capacity) {
    (void)context;
    (void)selected;
    if (id == FCM_CONVERTER_SOURCE || id == FCM_CONVERTER_OUTPUT) {
        char *path = id == FCM_CONVERTER_SOURCE ? converter.source : converter.output;
        if (!value || strlen(value) >= sizeof(converter.source)) {
            return fail(error, capacity, "The selected path is too long.");
        }
        strcpy(path, value);
        return true;
    }
    if (id != FCM_CONVERTER_CONVERT) {
        return fail(error, capacity, "Unknown movie conversion action.");
    }
    bool ok = fcm_frontend_convert(converter.execution, converter.source, converter.output, error, capacity);
    snprintf(converter.status, sizeof(converter.status), "%s",
             ok ? "FM2 saved. Converted input is ready for playback checks; legacy recordings can still desynchronize."
             : error && capacity ? error
                                 : "Conversion failed.");
    return ok;
}

bool fcm_frontend_register(FrontendExecutionRuntime *execution) {
    memset(&converter, 0, sizeof(converter));
    converter.execution = execution;
    snprintf(converter.status, sizeof(converter.status),
             "Load the matching game. Supports version-two power-on movies without legacy sync hacks.");
    FrontendPanelSpec panel = {
        FCM_CONVERTER_PANEL, "Convert FCM Movie", "Tools", FRONTEND_PANEL_NEEDS_SESSION, snapshot, action, NULL};
    return frontend_panel_register(&panel);
}

void fcm_frontend_unregister(void) {
    frontend_panel_unregister(FCM_CONVERTER_PANEL);
    memset(&converter, 0, sizeof(converter));
}
