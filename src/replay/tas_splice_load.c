/*
 * tas_splice_load.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Detached TAS splice sources. SPDX-License-Identifier: GPL-3.0-or-later */
#include "tas_splice.h"
#include "tas_startup.h"
#include "../util/file_io.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static NesTasResult loaded_result(NesTasResult value, const char *message, char *error, size_t capacity) {
    if (error && capacity) {
        snprintf(error, capacity, "%s", message ? message : value == NES_TAS_OK ? "" : nes_tas_result_string(value));
    }
    return value;
}

NesTasResult nes_tas_splice_load(const char *path, NesTasProject **out, char *error, size_t capacity) {
    if (!path || !*path || !out) {
        return loaded_result(NES_TAS_INVALID_ARGUMENT, NULL, error, capacity);
    }
    FILE *file = nes_file_open(path, "rb");
    if (!file) {
        return loaded_result(NES_TAS_IO_ERROR, "The source movie or project could not be opened.", error, capacity);
    }
    uint8_t signature[NES_CTAS_MAGIC_SIZE];
    size_t count = fread(signature, 1, sizeof(signature), file);
    bool failed = ferror(file) != 0;
    if (fclose(file) != 0) {
        failed = true;
    }
    if (failed) {
        return loaded_result(NES_TAS_IO_ERROR, "The source movie or project could not be read.", error, capacity);
    }
    if (count == sizeof(signature) && !memcmp(signature, NES_CTAS_MAGIC, sizeof(signature))) {
        return loaded_result(nes_tas_project_load(path, out), NULL, error, capacity);
    }
    NesFm2Limits limits = nes_fm2_default_limits();
    uint8_t *data = NULL;
    size_t size = 0;
    NesFileResult read = nes_file_read_all(path, limits.max_file_bytes, &data, &size);
    if (read != NES_FILE_OK) {
        return loaded_result(NES_TAS_IO_ERROR, nes_file_result_message(read), error, capacity);
    }
    NesFm2Movie movie;
    NesFm2Diagnostic diagnostic;
    nes_fm2_movie_init(&movie);
    NesFm2Result parsed = nes_fm2_parse(data, size, &limits, &movie, &diagnostic);
    free(data);
    if (parsed != NES_FM2_OK) {
        nes_fm2_movie_free(&movie);
        return loaded_result(parsed == NES_FM2_OUT_OF_MEMORY ? NES_TAS_OUT_OF_MEMORY : NES_TAS_FORMAT_ERROR,
                             diagnostic.message, error, capacity);
    }
    /* Match ordinary FM2/FM3 opening: an absent cheat header inherits the
     * current configuration. An explicit header is never replaced. */
    NesTasResult created =
        nes_tas_pin_cheats(&movie) ? nes_tas_project_create_checked(&movie, out) : NES_TAS_OUT_OF_MEMORY;
    nes_fm2_movie_free(&movie);
    return loaded_result(created, NULL, error, capacity);
}
