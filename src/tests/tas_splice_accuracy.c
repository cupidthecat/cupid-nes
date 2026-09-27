/*
 * tas_splice_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* TAS splice ranges, ownership and rollback. SPDX-License-Identifier: GPL-3.0-or-later */
#include "../replay/tas_project_internal.h"
#include "../replay/tas_splice.h"
#include "../replay/tas_startup.h"
#include "../util/file_io.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition);                                            \
            ok = false;                                                                                                \
            goto cleanup;                                                                                              \
        }                                                                                                              \
    } while (0)

static NesTasProject *make_project(size_t count, uint8_t first) {
    NesFm2Movie movie;
    nes_fm2_movie_init(&movie);
    movie.version = 3;
    movie.emu_version = 20606;
    movie.ports[0] = movie.ports[1] = NES_FM2_PORT_GAMEPAD;
    movie.rom_filename = tas_strdup_limit("splice.nes", 64);
    movie.frames = calloc(count ? count : 1, sizeof(*movie.frames));
    if (!movie.rom_filename || !movie.frames) {
        nes_fm2_movie_free(&movie);
        return NULL;
    }
    movie.frame_count = count;
    for (size_t i = 0; i < count; ++i) {
        movie.frames[i].pads[0] = (uint8_t)(first + i);
        movie.frames[i].pads[1] = (uint8_t)(first ^ i);
    }
    NesTasProject *project = nes_tas_project_create(&movie);
    nes_fm2_movie_free(&movie);
    return project;
}

static bool marker_is(const NesTasProject *project, size_t index, size_t frame, const char *note) {
    NesTasMarkerView marker;
    return nes_tas_marker(project, index, &marker) && marker.frame == frame && !strcmp(marker.note, note);
}

static bool navigation_at(const NesTasProject *project, size_t index, size_t frame) {
    NesTasNavigationView bookmark;
    return nes_tas_navigation(project, index, &bookmark) && bookmark.frame == frame;
}

static bool setup_markers(NesTasProject *destination, NesTasProject *source) {
    return nes_tas_marker_set(destination, 0, "head") == NES_TAS_OK &&
           nes_tas_marker_set(destination, 2, "cut") == NES_TAS_OK &&
           nes_tas_marker_set(destination, 5, "tail") == NES_TAS_OK &&
           nes_tas_marker_set(source, 1, "import") == NES_TAS_OK &&
           nes_tas_marker_set(source, 3, "outside") == NES_TAS_OK;
}

static bool modes_and_history(NesTasSpliceMode mode) {
    bool ok = true;
    NesTasProject *destination = make_project(6, 0x10), *source = make_project(4, 0x80);
    TasBookmark branch = {0};
    CHECK(destination && source && setup_markers(destination, source));
    CHECK(nes_tas_navigation_add(destination, 0, "start", "", NULL) == NES_TAS_OK);
    CHECK(nes_tas_navigation_add(destination, 2, "middle", "", NULL) == NES_TAS_OK);
    CHECK(nes_tas_navigation_add(destination, 6, "end", "", NULL) == NES_TAS_OK);
    CHECK(nes_tas_selection_set(destination, 1, true) == NES_TAS_OK);
    CHECK(nes_tas_selection_set(destination, 4, true) == NES_TAS_OK);
    CHECK(nes_tas_copy_selection(destination) == NES_TAS_OK);
    CHECK(nes_tas_bookmark_set(destination, 0, 4, "original") == NES_TAS_OK);
    CHECK(tas_bookmark_clone(&destination->state.bookmarks[0], &branch) == NES_TAS_OK);
    for (size_t i = 0; i < 6; ++i) {
        CHECK(nes_tas_set_lag(destination, i, NES_TAS_LAG_YES) == NES_TAS_OK);
    }
    source->state.timeline.movie.frames[1].commands = NES_FM2_COMMAND_RESET;
    source->state.timeline.movie.frames[2].commands = NES_FM2_COMMAND_POWER;
    NesFm2Frame before[6], imported[2];
    memcpy(before, nes_tas_project_movie(destination)->frames, sizeof(before));
    memcpy(imported, nes_tas_project_frame(source, 1), sizeof(imported));
    uint64_t revision = nes_tas_project_revision(destination), source_revision = nes_tas_project_revision(source);
    size_t undo_count = destination->undo_count;
    NesFm2Frame *clipboard = destination->clipboard.frames;
    NesTasSpliceOptions options = {mode, 1, 2, 2, 3, true, NES_FM2_COMMAND_RESET | NES_FM2_COMMAND_POWER};
    NesTasSplicePreview preview;
    static const size_t lengths[] = {8, 8, 5, 2}, firsts[] = {6, 2, 2, 0}, removed[] = {0, 0, 3, 6};
    static const size_t markers[] = {4, 4, 3, 1};
    static const size_t navigation[][3] = {{0, 2, 8}, {0, 4, 8}, {0, 4, 5}, {2, 2, 2}};
    CHECK(nes_tas_splice_preview(destination, source, &options, &preview, NULL, 0) == NES_TAS_OK);
    CHECK(preview.source_frames == 4 && preview.destination_frames == 6 && preview.inserted == 2);
    CHECK(preview.resulting_frames == lengths[mode] && preview.first == firsts[mode] &&
          preview.removed == removed[mode] && preview.resulting_markers == markers[mode] &&
          preview.imported_markers == 1);
    CHECK(nes_tas_project_revision(destination) == revision && destination->undo_count == undo_count);
    CHECK(nes_tas_splice_apply(destination, source, &options, NULL, 0) == NES_TAS_OK);
    CHECK(nes_tas_project_frame_count(destination) == lengths[mode]);
    for (size_t i = 0; i < lengths[mode]; ++i) {
        const NesFm2Frame *expected = i < firsts[mode]       ? &before[i]
                                      : i < firsts[mode] + 2 ? &imported[i - firsts[mode]]
                                                             : &before[i - 2 + removed[mode]];
        CHECK(tas_frame_equal(nes_tas_project_frame(destination, i), expected));
        CHECK(nes_tas_lag(destination, i) == (i < firsts[mode] ? NES_TAS_LAG_YES : NES_TAS_LAG_UNKNOWN));
    }
    CHECK(nes_tas_selection_contains(destination, firsts[mode]) &&
          nes_tas_selection_contains(destination, firsts[mode] + 1));
    CHECK(nes_tas_project_change_since(destination, revision).first_changed_frame == firsts[mode]);
    CHECK(destination->undo_count == undo_count + 1 && !nes_tas_edit_active(destination));
    CHECK(nes_tas_marker_count(destination) == markers[mode]);
    CHECK(marker_is(destination,
                    mode == NES_TAS_SPLICE_APPEND    ? 3
                    : mode == NES_TAS_SPLICE_EXTRACT ? 0
                                                     : 1,
                    firsts[mode], "import"));
    if (mode != NES_TAS_SPLICE_EXTRACT) {
        CHECK(marker_is(destination, 0, 0, "head"));
        CHECK(marker_is(destination, mode == NES_TAS_SPLICE_INSERT ? 3 : 2,
                        mode == NES_TAS_SPLICE_APPEND   ? 5
                        : mode == NES_TAS_SPLICE_INSERT ? 7
                                                        : 4,
                        "tail"));
    }
    for (size_t i = 0; i < 3; ++i) {
        CHECK(navigation_at(destination, i, navigation[mode][i]));
    }
    CHECK(destination->clipboard.frames == clipboard && destination->clipboard.count == 2);
    CHECK(tas_bookmark_equal(&destination->state.bookmarks[0], &branch));
    CHECK(nes_tas_project_revision(source) == source_revision && nes_tas_project_frame_count(source) == 4);
    CHECK(marker_is(source, 0, 1, "import") && marker_is(source, 1, 3, "outside"));
    CHECK(nes_tas_project_undo(destination) == NES_TAS_OK);
    CHECK(nes_tas_project_frame_count(destination) == 6 && destination->undo_count == undo_count);
    for (size_t i = 0; i < 6; ++i) {
        CHECK(tas_frame_equal(nes_tas_project_frame(destination, i), &before[i]));
        CHECK(nes_tas_lag(destination, i) == NES_TAS_LAG_YES);
        CHECK(nes_tas_selection_contains(destination, i) == (i == 1 || i == 4));
    }
    CHECK(marker_is(destination, 1, 2, "cut") && navigation_at(destination, 2, 6));
    CHECK(nes_tas_project_redo(destination) == NES_TAS_OK);
    CHECK(nes_tas_project_frame_count(destination) == lengths[mode] && destination->undo_count == undo_count + 1);
cleanup:
    tas_bookmark_free(&branch);
    nes_tas_project_destroy(destination);
    nes_tas_project_destroy(source);
    return ok;
}

static bool self_splice_and_equal_input(void) {
    bool ok = true;
    NesTasProject *project = make_project(6, 0x10), *source = NULL, *empty = make_project(0, 0);
    CHECK(project && empty);
    CHECK(nes_tas_marker_set(project, 2, "source") == NES_TAS_OK);
    NesTasSpliceOptions options = {NES_TAS_SPLICE_INSERT, 1, 3, 2, 0, true, 3};
    CHECK(nes_tas_splice_apply(project, project, &options, NULL, 0) == NES_TAS_OK);
    static const uint8_t expected[] = {0x10, 0x11, 0x11, 0x12, 0x13, 0x12, 0x13, 0x14, 0x15};
    CHECK(nes_tas_project_frame_count(project) == sizeof(expected));
    for (size_t i = 0; i < sizeof(expected); ++i) {
        CHECK(nes_tas_project_frame(project, i)->pads[0] == expected[i]);
    }
    CHECK(marker_is(project, 0, 3, "source") && marker_is(project, 1, 5, "source"));
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK);
    CHECK(nes_tas_selection_set_range(project, 1, 3, true) == NES_TAS_OK);
    for (size_t i = 0; i < 6; ++i) {
        CHECK(nes_tas_set_lag(project, i, NES_TAS_LAG_NO) == NES_TAS_OK);
    }
    options = (NesTasSpliceOptions){NES_TAS_SPLICE_REPLACE, 1, 3, 1, 3, true, 3};
    uint64_t revision = nes_tas_project_revision(project);
    size_t history = project->undo_count;
    CHECK(nes_tas_splice_apply(project, project, &options, NULL, 0) == NES_TAS_OK);
    CHECK(nes_tas_project_revision(project) == revision && project->undo_count == history);
    source = nes_tas_project_create(nes_tas_project_movie(project));
    CHECK(source && nes_tas_marker_set(source, 2, "new note") == NES_TAS_OK);
    CHECK(nes_tas_splice_apply(project, source, &options, NULL, 0) == NES_TAS_OK);
    CHECK(nes_tas_project_change_since(project, revision).first_changed_frame == SIZE_MAX);
    CHECK(marker_is(project, 0, 2, "new note") && nes_tas_lag(project, 5) == NES_TAS_LAG_NO);
    source->state.timeline.movie.frames[3].pads[0] ^= 1;
    revision = nes_tas_project_revision(project);
    CHECK(nes_tas_splice_apply(project, source, &options, NULL, 0) == NES_TAS_OK);
    CHECK(nes_tas_project_change_since(project, revision).first_changed_frame == 3);
    CHECK(nes_tas_lag(project, 2) == NES_TAS_LAG_NO && nes_tas_lag(project, 3) == NES_TAS_LAG_UNKNOWN &&
          nes_tas_lag(project, 5) == NES_TAS_LAG_UNKNOWN);
    options.mode = NES_TAS_SPLICE_EXTRACT;
    options.copy_markers = false;
    CHECK(nes_tas_splice_apply(project, source, &options, NULL, 0) == NES_TAS_OK);
    CHECK(nes_tas_project_frame_count(project) == 3 && nes_tas_marker_count(project) == 0);
    options.mode = NES_TAS_SPLICE_APPEND;
    CHECK(nes_tas_splice_apply(empty, source, &options, NULL, 0) == NES_TAS_OK);
    CHECK(nes_tas_project_frame_count(empty) == 3 && nes_tas_project_frame(empty, 0)->pads[0] == 0x11);
cleanup:
    nes_tas_project_destroy(project);
    nes_tas_project_destroy(source);
    nes_tas_project_destroy(empty);
    return ok;
}

static bool rejected(NesTasProject *destination, NesTasProject *source, const NesTasSpliceOptions *options,
                     NesTasResult expected) {
    NesTasSplicePreview preview, sentinel;
    memset(&preview, 0x5A, sizeof(preview));
    memcpy(&sentinel, &preview, sizeof(sentinel));
    char error[256] = {0};
    uint64_t revision = nes_tas_project_revision(destination), source_revision = nes_tas_project_revision(source);
    size_t history = destination->undo_count;
    return nes_tas_splice_preview(destination, source, options, &preview, error, sizeof(error)) == expected && *error &&
           !memcmp(&preview, &sentinel, sizeof(preview)) &&
           nes_tas_splice_apply(destination, source, options, error, sizeof(error)) == expected && *error &&
           nes_tas_project_revision(destination) == revision && nes_tas_project_revision(source) == source_revision &&
           destination->undo_count == history;
}

static bool profiles_and_ranges(void) {
    bool ok = true;
    NesTasProject *a = make_project(6, 0), *b = make_project(4, 0);
    CHECK(a && b);
    NesTasSpliceOptions options = {NES_TAS_SPLICE_REPLACE, 1, 2, 2, 2, true, 3};
    NesTasSplicePreview preview;
    NesFm2Movie *left = &a->state.timeline.movie, *right = &b->state.timeline.movie;
    right->rom_md5[0] = 1;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    right->rom_md5[0] = 0;
    right->pal = true;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    right->pal = false;
    right->fourscore = true;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    right->fourscore = false;
    right->microphone = true;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    right->microphone = false;
    right->ports[1] = NES_FM2_PORT_ZAPPER;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    right->ports[1] = NES_FM2_PORT_GAMEPAD;
    right->fds = true;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    right->fds = false;
    right->new_ppu = true;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    right->new_ppu = false;
    right->ram_init_option = 3;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    right->ram_init_option = 0;
    right->ram_init_seed = 1;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    right->ram_init_seed = 0;
    left->saveram.data = calloc(4, 1);
    right->saveram.data = calloc(4, 1);
    CHECK(left->saveram.data && right->saveram.data);
    right->saveram.size = 4;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    left->saveram.size = 4;
    CHECK(nes_tas_splice_preview(a, b, &options, &preview, NULL, 0) == NES_TAS_OK);
    right->saveram.data[0] = 1;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    right->saveram.data[0] = 0;
    right->savestate.data = calloc(1, 1);
    CHECK(right->savestate.data);
    right->savestate.size = 1;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    right->savestate.size = 0;
    /* Commands outside the selected range still belong to the imported profile. */
    right->frames[0].commands = 0x80;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    right->frames[0].commands = NES_FM2_COMMAND_FDS_INSERT;
    options.allowed_commands = 0x7F;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    right->frames[0].commands = NES_FM2_COMMAND_VS_SERVICE;
    options.allowed_commands = 3;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    options.allowed_commands |= NES_FM2_COMMAND_VS_SERVICE;
    CHECK(nes_tas_splice_preview(a, b, &options, &preview, NULL, 0) == NES_TAS_OK);
    right->frames[0].commands = 0;
    left->frames[5].commands = 0x80;
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    left->frames[5].commands = 0;
    options.allowed_commands = 0xFF;
    CHECK(rejected(a, b, &options, NES_TAS_INVALID_ARGUMENT));
    options.allowed_commands = 3;
    options.source_count = 0;
    CHECK(rejected(a, b, &options, NES_TAS_RANGE_ERROR));
    options.source_count = SIZE_MAX;
    CHECK(rejected(a, b, &options, NES_TAS_RANGE_ERROR));
    options.source_count = 2;
    options.source_first = SIZE_MAX;
    CHECK(rejected(a, b, &options, NES_TAS_RANGE_ERROR));
    options.source_first = 1;
    options.destination_first = 7;
    CHECK(rejected(a, b, &options, NES_TAS_RANGE_ERROR));
    options.destination_first = 2;
    options.destination_count = SIZE_MAX;
    CHECK(rejected(a, b, &options, NES_TAS_RANGE_ERROR));
    options.destination_count = 2;
    options.mode = (NesTasSpliceMode)99;
    CHECK(rejected(a, b, &options, NES_TAS_INVALID_ARGUMENT));
    options.mode = NES_TAS_SPLICE_REPLACE;
    CHECK(nes_tas_edit_begin(a) == NES_TAS_OK);
    CHECK(rejected(a, b, &options, NES_TAS_INVALID_ARGUMENT) && nes_tas_edit_active(a));
    nes_tas_edit_cancel(a);
    CHECK(nes_tas_edit_begin(b) == NES_TAS_OK);
    CHECK(rejected(a, b, &options, NES_TAS_INVALID_ARGUMENT) && nes_tas_edit_active(b));
    nes_tas_edit_cancel(b);
    /* Display metadata may differ while the startup and hardware stay equal. */
    memset(right->guid, 7, sizeof(right->guid));
    right->emu_version++;
    right->rerecord_count++;
    CHECK(nes_tas_splice_preview(a, b, &options, &preview, NULL, 0) == NES_TAS_OK);
cleanup:
    nes_tas_project_destroy(a);
    nes_tas_project_destroy(b);
    return ok;
}

static bool add_field(NesFm2Movie *movie, const char *key, const char *value) {
    char *key_copy = tas_strdup_limit(key, 128), *value_copy = tas_strdup_limit(value, 128);
    if (!key_copy || !value_copy) {
        free(key_copy);
        free(value_copy);
        return false;
    }
    NesFm2HeaderField *fields = realloc(movie->extensions, (movie->extension_count + 1) * sizeof(*fields));
    if (!fields) {
        free(key_copy);
        free(value_copy);
        return false;
    }
    movie->extensions = fields;
    movie->extensions[movie->extension_count++] = (NesFm2HeaderField){key_copy, value_copy};
    return true;
}

static bool extensions_and_devices(void) {
    bool ok = true;
    NesTasProject *a = make_project(2, 0), *b = make_project(2, 0x80);
    CHECK(a && b);
    NesFm2Movie *left = &a->state.timeline.movie, *right = &b->state.timeline.movie;
    CHECK(add_field(left, "cupidCheatsHash", "12345678") && add_field(left, "profile", "a"));
    CHECK(add_field(right, "profile", "a") && add_field(right, "cupidCheatsHash", "12345678"));
    NesTasSpliceOptions options = {NES_TAS_SPLICE_APPEND, 0, 2, 0, 0, true, 0x7F};
    NesTasSplicePreview preview;
    CHECK(nes_tas_splice_preview(a, b, &options, &preview, NULL, 0) == NES_TAS_OK);
    right->extensions[1].value[0] = '9';
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    right->extensions[1].value[0] = '1';
    CHECK(add_field(left, "profile", "a"));
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    CHECK(add_field(right, "cupidCheatsHash", "12345678"));
    CHECK(rejected(a, b, &options, NES_TAS_UNSUPPORTED));
    free(right->extensions[2].key);
    free(right->extensions[2].value);
    right->extensions[2].key = tas_strdup_limit("profile", 128);
    right->extensions[2].value = tas_strdup_limit("a", 128);
    CHECK(right->extensions[2].key && right->extensions[2].value);
    CHECK(nes_tas_splice_preview(a, b, &options, &preview, NULL, 0) == NES_TAS_OK);
    left->ports[0] = right->ports[0] = NES_FM2_PORT_ZAPPER;
    right->frames[0].zappers[0] = (NesFm2Zapper){128, 239, 1, 9, UINT64_C(0x123456789)};
    NesFm2Frame expected = right->frames[0];
    CHECK(nes_tas_splice_apply(a, b, &options, NULL, 0) == NES_TAS_OK);
    CHECK(tas_frame_equal(nes_tas_project_frame(a, 2), &expected));
    CHECK(nes_tas_project_undo(a) == NES_TAS_OK);
    left = &a->state.timeline.movie;
    left->ports[0] = right->ports[0] = NES_FM2_PORT_GAMEPAD;
    left->fourscore = right->fourscore = true;
    memset(&right->frames[0].zappers[0], 0, sizeof(right->frames[0].zappers[0]));
    right->frames[0].pads[2] = 0x42;
    right->frames[0].pads[3] = 0x81;
    CHECK(nes_tas_splice_apply(a, b, &options, NULL, 0) == NES_TAS_OK);
    CHECK(nes_tas_project_frame(a, 2)->pads[2] == 0x42 && nes_tas_project_frame(a, 2)->pads[3] == 0x81);
cleanup:
    nes_tas_project_destroy(a);
    nes_tas_project_destroy(b);
    return ok;
}

static bool saved_bytes(const NesTasProject *project, const char *path, uint8_t **data, size_t *size) {
    return nes_tas_project_save(project, path) == NES_TAS_OK &&
           nes_file_read_all(path, 64u * 1024u * 1024u, data, size) == NES_FILE_OK;
}

static bool allocation_failures(void) {
    bool ok = true;
    NesTasProject *a = make_project(6, 0), *b = make_project(4, 0x80);
    uint8_t *before = NULL, *after = NULL, *source_before = NULL, *source_after = NULL;
    size_t before_size = 0, after_size = 0, source_size = 0, source_after_size = 0;
    const char *path = "build/tas-splice-rollback.ctas";
    CHECK(a && b && setup_markers(a, b));
    CHECK(nes_tas_navigation_add(a, 4, "navigation", "note", NULL) == NES_TAS_OK);
    CHECK(nes_tas_bookmark_set(a, 0, 4, "branch") == NES_TAS_OK);
    CHECK(nes_tas_selection_set(a, 4, true) == NES_TAS_OK && nes_tas_copy_selection(a) == NES_TAS_OK);
    CHECK(nes_tas_marker_set(a, 4, "redo item") == NES_TAS_OK && nes_tas_project_undo(a) == NES_TAS_OK);
    CHECK(saved_bytes(a, path, &before, &before_size) && saved_bytes(b, path, &source_before, &source_size));
    uint64_t revision = nes_tas_project_revision(a), source_revision = nes_tas_project_revision(b);
    NesFm2Frame *clipboard = a->clipboard.frames;
    NesTasSpliceOptions options = {NES_TAS_SPLICE_REPLACE, 1, 2, 2, 3, true, 3};
    size_t rejected_allocations = 0;
    bool applied = false;
    for (size_t allowed = 0; allowed < 32; ++allowed) {
        tas_project_test_fail_alloc_after(allowed);
        NesTasResult result = nes_tas_splice_apply(a, b, &options, NULL, 0);
        tas_project_test_reset_alloc_fail();
        if (result == NES_TAS_OK) {
            applied = true;
            break;
        }
        ++rejected_allocations;
        CHECK(result == NES_TAS_OUT_OF_MEMORY && !nes_tas_edit_active(a));
        CHECK(nes_tas_project_revision(a) == revision && nes_tas_project_revision(b) == source_revision);
        CHECK(nes_tas_project_can_redo(a) && a->clipboard.frames == clipboard);
        CHECK(saved_bytes(a, path, &after, &after_size));
        CHECK(after_size == before_size && !memcmp(before, after, before_size));
        free(after);
        after = NULL;
        CHECK(saved_bytes(b, path, &source_after, &source_after_size));
        CHECK(source_after_size == source_size && !memcmp(source_before, source_after, source_size));
        free(source_after);
        source_after = NULL;
    }
    /* Frames, lag, selection, markers, three notes and the history's navigation
     * snapshot must all fail without mutating either project. */
    CHECK(applied && rejected_allocations >= 8 && !nes_tas_project_can_redo(a));
    CHECK(nes_tas_project_frame_count(a) == 5 && nes_tas_project_undo(a) == NES_TAS_OK);
    CHECK(nes_tas_project_frame_count(a) == 6 && marker_is(a, 1, 2, "cut"));
cleanup:
    tas_project_test_reset_alloc_fail();
    free(before);
    free(after);
    free(source_before);
    free(source_after);
    nes_tas_project_destroy(a);
    nes_tas_project_destroy(b);
    (void)nes_file_remove(path);
    return ok;
}

static bool detached_files(void) {
    bool ok = true;
    NesTasProject *project = make_project(6, 0x10), *loaded = NULL;
    uint8_t *data = NULL;
    size_t size = 0, written = 0;
    const char *path = "build/tas-splice-source.movie";
    char error[256];
    NesFm2Diagnostic diagnostic;
    CHECK(project && nes_tas_marker_set(project, 2, "saved marker") == NES_TAS_OK);
    CHECK(nes_tas_navigation_add(project, 5, "saved bookmark", "note", NULL) == NES_TAS_OK);
    CHECK(nes_tas_project_save(project, path) == NES_TAS_OK);
    CHECK(nes_tas_splice_load(path, &loaded, error, sizeof(error)) == NES_TAS_OK && !*error);
    CHECK(loaded != project && marker_is(loaded, 0, 2, "saved marker") && navigation_at(loaded, 0, 5));
    CHECK(nes_tas_project_frame(loaded, 0) != nes_tas_project_frame(project, 0));
    NesTasProject *retained = loaded;
    CHECK(nes_tas_splice_load("build/tas-splice-does-not-exist", &loaded, error, sizeof(error)) == NES_TAS_IO_ERROR);
    CHECK(loaded == retained && *error);
    CHECK(nes_file_write_atomic(path, (const uint8_t *)"invalid", 7) == NES_FILE_OK);
    CHECK(nes_tas_splice_load(path, &loaded, error, sizeof(error)) == NES_TAS_FORMAT_ERROR && loaded == retained);
    CHECK(nes_file_write_atomic(path, (const uint8_t *)NES_CTAS_MAGIC, NES_CTAS_MAGIC_SIZE) == NES_FILE_OK);
    CHECK(nes_tas_splice_load(path, &loaded, error, sizeof(error)) == NES_TAS_FORMAT_ERROR && loaded == retained);
    nes_tas_project_destroy(loaded);
    loaded = NULL;
    for (unsigned format = NES_FM2_TEXT; format <= NES_FM3_PROJECT; ++format) {
        bool fm3 = format == NES_FM3_PROJECT;
        CHECK(fm3 ? nes_tas_project_measure_fm3(project, &size, &diagnostic) == NES_TAS_OK
                  : nes_fm2_measure(nes_tas_project_movie(project), (NesFm2Container)format, &size, &diagnostic) ==
                        NES_FM2_OK);
        data = malloc(size);
        CHECK(data);
        CHECK(fm3 ? nes_tas_project_export_fm3(project, data, size, &written, &diagnostic) == NES_TAS_OK
                  : nes_fm2_export(nes_tas_project_movie(project), (NesFm2Container)format, data, size, &written,
                                   &diagnostic) == NES_FM2_OK);
        CHECK(written == size && nes_file_write_atomic(path, data, written) == NES_FILE_OK);
        CHECK(nes_tas_splice_load(path, &loaded, error, sizeof(error)) == NES_TAS_OK);
        CHECK(nes_tas_project_frame_count(loaded) == 6 && nes_tas_navigation_count(loaded) == 0);
        CHECK(!fm3 || marker_is(loaded, 0, 2, "saved marker"));
        CHECK(nes_tas_project_movie(loaded)->extension_count == 1);
        CHECK(!strcmp(nes_tas_project_movie(loaded)->extensions[0].key, "cupidCheatsHash"));
        free(data);
        data = NULL;
        nes_tas_project_destroy(loaded);
        loaded = NULL;
    }
    CHECK(add_field(&project->state.timeline.movie, "cupidCheatsHash", "76543210"));
    CHECK(nes_tas_project_measure_fm3(project, &size, &diagnostic) == NES_TAS_OK);
    data = malloc(size);
    CHECK(data && nes_tas_project_export_fm3(project, data, size, &written, &diagnostic) == NES_TAS_OK);
    CHECK(nes_file_write_atomic(path, data, written) == NES_FILE_OK);
    CHECK(nes_tas_splice_load(path, &loaded, error, sizeof(error)) == NES_TAS_OK);
    CHECK(!strcmp(nes_tas_project_movie(loaded)->extensions[0].value, "76543210"));
cleanup:
    free(data);
    nes_tas_project_destroy(project);
    nes_tas_project_destroy(loaded);
    (void)nes_file_remove(path);
    return ok;
}

int test_tas_splice_accuracy(void) {
    int failures = 0;
    for (unsigned mode = NES_TAS_SPLICE_APPEND; mode <= NES_TAS_SPLICE_EXTRACT; ++mode) {
        failures += !modes_and_history((NesTasSpliceMode)mode);
    }
    failures += !self_splice_and_equal_input();
    failures += !profiles_and_ranges();
    failures += !extensions_and_devices();
    failures += !allocation_failures();
    failures += !detached_files();
    printf("TAS splicer: range, profile, ownership, undo, rollback and detached loading checks, %d failures\n",
           failures);
    return failures;
}
