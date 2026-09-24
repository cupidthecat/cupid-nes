/*
 * fm2.c - FCEUX FM2/FM3 movie container codec lifecycle
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "fm2_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void free_string_list(NesFm2StringList *list) {
    if (!list) {
        return;
    }

    for (size_t i = 0; i < list->count; ++i) {
        free(list->items[i]);
    }
    free(list->items);
    list->items = NULL;
    list->count = 0;
}

static void free_extensions(NesFm2Movie *movie) {
    for (size_t i = 0; i < movie->extension_count; ++i) {
        free(movie->extensions[i].key);
        free(movie->extensions[i].value);
    }
    free(movie->extensions);
    movie->extensions = NULL;
    movie->extension_count = 0;
}

NesFm2Limits nes_fm2_default_limits(void) {
    NesFm2Limits limits;

    limits.max_file_bytes = 256u * 1024u * 1024u;
    limits.max_frames = 10000000u;
    limits.max_text_bytes = 32u * 1024u * 1024u;
    limits.max_blob_bytes = 64u * 1024u * 1024u;
    limits.max_comments = 100000u;
    limits.max_subtitles = 100000u;
    limits.max_extensions = 100000u;
    limits.max_project_bytes = 256u * 1024u * 1024u;
    return limits;
}

void nes_fm2_movie_init(NesFm2Movie *movie) {
    if (!movie) {
        return;
    }

    memset(movie, 0, sizeof(*movie));
    movie->original_container = NES_FM2_TEXT;
    movie->version = 3;
    movie->project_timeline_valid = true;
}

void nes_fm2_movie_free(NesFm2Movie *movie) {
    if (!movie) {
        return;
    }

    free(movie->rom_filename);
    free_string_list(&movie->comments);
    free_string_list(&movie->subtitles);
    free(movie->savestate.data);
    free(movie->saveram.data);
    free_extensions(movie);
    free(movie->frames);
    for (size_t i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        free(movie->project_modules[i].data);
    }
    nes_fm2_movie_init(movie);
}

void nes_fm2__diagnostic_clear(NesFm2Diagnostic *diagnostic) {
    if (!diagnostic) {
        return;
    }

    diagnostic->result = NES_FM2_OK;
    diagnostic->byte_offset = 0;
    diagnostic->frame_index = 0;
    diagnostic->message[0] = '\0';
}

NesFm2Result nes_fm2__fail(NesFm2Diagnostic *diagnostic, NesFm2Result result, size_t byte_offset, size_t frame_index,
                           const char *format, ...) {
    if (diagnostic) {
        va_list args;

        diagnostic->result = result;
        diagnostic->byte_offset = byte_offset;
        diagnostic->frame_index = frame_index;
        va_start(args, format);
        vsnprintf(diagnostic->message, sizeof(diagnostic->message), format, args);
        va_end(args);
    }
    return result;
}

bool nes_fm2__checked_add(size_t a, size_t b, size_t *out) {
    if (!out || a > SIZE_MAX - b) {
        return false;
    }
    *out = a + b;
    return true;
}

bool nes_fm2__checked_mul(size_t a, size_t b, size_t *out) {
    if (!out || (a != 0 && b > SIZE_MAX / a)) {
        return false;
    }
    *out = a * b;
    return true;
}

char *nes_fm2__dup_slice(const uint8_t *data, size_t size) {
    char *copy;

    if (size == SIZE_MAX) {
        return NULL;
    }
    copy = malloc(size + 1);
    if (!copy) {
        return NULL;
    }
    if (size != 0) {
        memcpy(copy, data, size);
    }
    copy[size] = '\0';
    return copy;
}

NesFm2Result nes_fm2__copy_blob(const NesFm2Blob *source, NesFm2Blob *out, NesFm2Diagnostic *diagnostic) {
    out->data = NULL;
    out->size = 0;
    if (source->size == 0) {
        return NES_FM2_OK;
    }
    if (!source->data) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "blob has a nonzero size and no data");
    }
    out->data = malloc(source->size);
    if (!out->data) {
        return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, 0, 0, "unable to copy movie blob");
    }
    memcpy(out->data, source->data, source->size);
    out->size = source->size;
    return NES_FM2_OK;
}

NesFm2Result nes_fm2__copy_string_list(const NesFm2StringList *source, NesFm2StringList *out,
                                       NesFm2Diagnostic *diagnostic) {
    out->items = NULL;
    out->count = 0;
    if (source->count == 0) {
        return NES_FM2_OK;
    }
    if (!source->items || source->count > SIZE_MAX / sizeof(*out->items)) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "invalid string list");
    }
    out->items = calloc(source->count, sizeof(*out->items));
    if (!out->items) {
        return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, 0, 0, "unable to copy string list");
    }
    for (size_t i = 0; i < source->count; ++i) {
        if (!source->items[i]) {
            free_string_list(out);
            return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "string list contains a null item");
        }
        out->items[i] = nes_fm2__dup_slice((const uint8_t *)source->items[i], strlen(source->items[i]));
        if (!out->items[i]) {
            free_string_list(out);
            return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, 0, 0, "unable to copy string list item");
        }
        out->count = i + 1;
    }
    return NES_FM2_OK;
}

static NesFm2Result clone_extensions(const NesFm2Movie *source, NesFm2Movie *out, NesFm2Diagnostic *diagnostic) {
    if (source->extension_count == 0) {
        return NES_FM2_OK;
    }
    if (!source->extensions || source->extension_count > SIZE_MAX / sizeof(*out->extensions)) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "invalid extension list");
    }
    out->extensions = calloc(source->extension_count, sizeof(*out->extensions));
    if (!out->extensions) {
        return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, 0, 0, "unable to copy extension fields");
    }
    for (size_t i = 0; i < source->extension_count; ++i) {
        if (!source->extensions[i].key || !source->extensions[i].value) {
            return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "extension field contains a null string");
        }
        out->extensions[i].key =
            nes_fm2__dup_slice((const uint8_t *)source->extensions[i].key, strlen(source->extensions[i].key));
        out->extensions[i].value =
            nes_fm2__dup_slice((const uint8_t *)source->extensions[i].value, strlen(source->extensions[i].value));
        if (!out->extensions[i].key || !out->extensions[i].value) {
            free(out->extensions[i].key);
            free(out->extensions[i].value);
            out->extensions[i].key = NULL;
            out->extensions[i].value = NULL;
            return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, 0, 0, "unable to copy extension field");
        }
        out->extension_count = i + 1;
    }
    return NES_FM2_OK;
}

static NesFm2Result clone_frames(const NesFm2Movie *source, NesFm2Movie *out, NesFm2Diagnostic *diagnostic) {
    size_t bytes;

    if (source->frame_count == 0) {
        return NES_FM2_OK;
    }
    if (!source->frames || !nes_fm2__checked_mul(source->frame_count, sizeof(*out->frames), &bytes)) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "invalid frame array");
    }
    out->frames = malloc(bytes);
    if (!out->frames) {
        return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, 0, 0, "unable to copy movie frames");
    }
    memcpy(out->frames, source->frames, bytes);
    out->frame_count = source->frame_count;
    return NES_FM2_OK;
}

static NesFm2Result clone_project(const NesFm2Movie *source, NesFm2Movie *out, NesFm2Diagnostic *diagnostic) {
    for (size_t i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        const NesFm3ProjectModule *input = &source->project_modules[i];
        NesFm3ProjectModule *copy = &out->project_modules[i];

        if (input->size == 0) {
            continue;
        }
        if (!input->data) {
            return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0,
                                 "project module has a nonzero size and no data");
        }
        copy->data = malloc(input->size);
        if (!copy->data) {
            return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, 0, 0, "unable to copy project module");
        }
        memcpy(copy->data, input->data, input->size);
        copy->size = input->size;
    }
    return NES_FM2_OK;
}

NesFm2Result nes_fm2_movie_clone(const NesFm2Movie *source, NesFm2Movie *out, NesFm2Diagnostic *diagnostic) {
    NesFm2Movie copy;
    NesFm2Result result;

    nes_fm2__diagnostic_clear(diagnostic);
    if (!source || !out) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "source and destination movies are required");
    }
    if (source == out) {
        return NES_FM2_OK;
    }

    nes_fm2_movie_init(&copy);
    copy = *source;
    copy.rom_filename = NULL;
    copy.comments.items = NULL;
    copy.comments.count = 0;
    copy.subtitles.items = NULL;
    copy.subtitles.count = 0;
    copy.savestate.data = NULL;
    copy.savestate.size = 0;
    copy.saveram.data = NULL;
    copy.saveram.size = 0;
    copy.extensions = NULL;
    copy.extension_count = 0;
    copy.frames = NULL;
    copy.frame_count = 0;
    for (size_t i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        copy.project_modules[i].data = NULL;
        copy.project_modules[i].size = 0;
    }

    if (source->rom_filename) {
        copy.rom_filename = nes_fm2__dup_slice((const uint8_t *)source->rom_filename, strlen(source->rom_filename));
        if (!copy.rom_filename) {
            result = nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, 0, 0, "unable to copy ROM filename");
            goto fail;
        }
    }
    result = nes_fm2__copy_string_list(&source->comments, &copy.comments, diagnostic);
    if (result != NES_FM2_OK) {
        goto fail;
    }
    result = nes_fm2__copy_string_list(&source->subtitles, &copy.subtitles, diagnostic);
    if (result != NES_FM2_OK) {
        goto fail;
    }
    result = nes_fm2__copy_blob(&source->savestate, &copy.savestate, diagnostic);
    if (result != NES_FM2_OK) {
        goto fail;
    }
    result = nes_fm2__copy_blob(&source->saveram, &copy.saveram, diagnostic);
    if (result != NES_FM2_OK) {
        goto fail;
    }
    result = clone_extensions(source, &copy, diagnostic);
    if (result != NES_FM2_OK) {
        goto fail;
    }
    result = clone_frames(source, &copy, diagnostic);
    if (result != NES_FM2_OK) {
        goto fail;
    }
    result = clone_project(source, &copy, diagnostic);
    if (result != NES_FM2_OK) {
        goto fail;
    }

    nes_fm2_movie_free(out);
    *out = copy;
    return NES_FM2_OK;

fail:
    nes_fm2_movie_free(&copy);
    return result;
}

static bool limits_valid(const NesFm2Limits *limits) {
    return limits->max_file_bytes != 0 && limits->max_frames != 0 && limits->max_text_bytes != 0 &&
           limits->max_blob_bytes != 0 && limits->max_comments != 0 && limits->max_subtitles != 0 &&
           limits->max_extensions != 0 && limits->max_project_bytes != 0;
}

NesFm2Result nes_fm2_parse(const uint8_t *data, size_t size, const NesFm2Limits *limits, NesFm2Movie *out,
                           NesFm2Diagnostic *diagnostic) {
    NesFm2Limits effective;
    NesFm2Reader reader;
    NesFm2Movie parsed;
    NesFm2Result result;

    nes_fm2__diagnostic_clear(diagnostic);
    if ((!data && size != 0) || !out) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0,
                             "input bytes and destination movie are required");
    }
    effective = limits ? *limits : nes_fm2_default_limits();
    if (!limits_valid(&effective)) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "all parser limits must be nonzero");
    }
    if (size > effective.max_file_bytes) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, effective.max_file_bytes, 0, "movie exceeds max_file_bytes");
    }

    reader.data = data;
    reader.size = size;
    reader.offset = 0;
    reader.limits = effective;
    reader.text_bytes = 0;
    nes_fm2_movie_init(&parsed);
    result = nes_fm2__parse_movie(&reader, &parsed, diagnostic);
    if (result != NES_FM2_OK) {
        nes_fm2_movie_free(&parsed);
        return result;
    }

    nes_fm2_movie_free(out);
    *out = parsed;
    return NES_FM2_OK;
}

NesFm2Result nes_fm2_measure(const NesFm2Movie *movie, NesFm2Container container, size_t *size,
                             NesFm2Diagnostic *diagnostic) {
    nes_fm2__diagnostic_clear(diagnostic);
    if (!movie || !size) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "movie and output size are required");
    }
    return nes_fm2__measure_movie(movie, container, size, diagnostic);
}

NesFm2Result nes_fm2_export(const NesFm2Movie *movie, NesFm2Container container, uint8_t *destination, size_t capacity,
                            size_t *written, NesFm2Diagnostic *diagnostic) {
    size_t required = 0;
    NesFm2Result result;

    nes_fm2__diagnostic_clear(diagnostic);
    if (!movie || !written || (!destination && capacity != 0)) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0,
                             "movie, destination, and written size are invalid");
    }
    result = nes_fm2__measure_movie(movie, container, &required, diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    *written = required;
    if (required > capacity || (!destination && required != 0)) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, capacity, 0,
                             "destination capacity is smaller than the exported movie");
    }
    return nes_fm2__write_movie(movie, container, destination, capacity, written, diagnostic);
}

NesFm2StartKind nes_fm2_start_kind(const NesFm2Movie *movie) {
    if (!movie) {
        return NES_FM2_START_POWER_ON;
    }
    if (movie->savestate.size != 0) {
        return NES_FM2_START_SAVESTATE;
    }
    if (movie->saveram.size != 0) {
        return NES_FM2_START_SAVERAM;
    }
    return NES_FM2_START_POWER_ON;
}

void nes_fm2_invalidate_project_timeline(NesFm2Movie *movie) {
    if (movie && movie->project_present) {
        movie->project_timeline_valid = false;
    }
}

void nes_fm2_discard_project(NesFm2Movie *movie) {
    if (!movie) {
        return;
    }
    for (size_t i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        free(movie->project_modules[i].data);
        movie->project_modules[i].data = NULL;
        movie->project_modules[i].size = 0;
    }
    movie->project_present = false;
    movie->project_timeline_valid = true;
    movie->project_version = 0;
    movie->project_saved_modules = 0;
    if (movie->original_container == NES_FM3_PROJECT) {
        movie->original_container = NES_FM2_BINARY;
    }
}

const char *nes_fm2_result_string(NesFm2Result result) {
    switch (result) {
    case NES_FM2_OK:
        return "ok";
    case NES_FM2_INVALID_ARGUMENT:
        return "invalid argument";
    case NES_FM2_OUT_OF_MEMORY:
        return "out of memory";
    case NES_FM2_LIMIT:
        return "limit exceeded";
    case NES_FM2_TRUNCATED:
        return "truncated input";
    case NES_FM2_CORRUPT:
        return "corrupt input";
    case NES_FM2_UNSUPPORTED:
        return "unsupported input";
    default:
        return "unknown result";
    }
}
