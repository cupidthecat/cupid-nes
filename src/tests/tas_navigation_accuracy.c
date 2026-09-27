/*
 * tas_navigation_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* TAS navigation ownership, persistence and failure paths. SPDX-License-Identifier: GPL-3.0-or-later */
#include "../replay/tas_project_internal.h"
#include "../third_party/miniz/miniz.h"
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

static NesTasProject *make_project(size_t count) {
    NesFm2Movie movie;
    nes_fm2_movie_init(&movie);
    movie.version = 3;
    movie.emu_version = 20606;
    movie.ports[0] = movie.ports[1] = NES_FM2_PORT_GAMEPAD;
    movie.rom_filename = (char *)malloc(sizeof("navigation.nes"));
    movie.frames = (NesFm2Frame *)calloc(count ? count : 1, sizeof(*movie.frames));
    if (!movie.rom_filename || !movie.frames) {
        nes_fm2_movie_free(&movie);
        return NULL;
    }
    memcpy(movie.rom_filename, "navigation.nes", sizeof("navigation.nes"));
    movie.frame_count = count;
    for (size_t i = 0; i < count; ++i) {
        movie.frames[i].pads[0] = (uint8_t)i;
    }
    NesTasProject *project = nes_tas_project_create(&movie);
    nes_fm2_movie_free(&movie);
    return project;
}

static bool bookmark_is(const NesTasProject *project, size_t index, size_t frame, const char *name, const char *note) {
    NesTasNavigationView entry;
    return nes_tas_navigation(project, index, &entry) && entry.frame == frame && !strcmp(entry.name, name) &&
           !strcmp(entry.note, note);
}

static bool model_and_history(void) {
    bool ok = true;
    NesTasProject *project = make_project(12);
    CHECK(project);
    CHECK(nes_tas_marker_set(project, 3, "marker") == NES_TAS_OK);
    CHECK(nes_tas_bookmark_set(project, 0, 8, "branch") == NES_TAS_OK);
    CHECK(nes_tas_set_lag(project, 6, NES_TAS_LAG_YES) == NES_TAS_OK);
    uint64_t revision = nes_tas_project_revision(project);
    uint64_t rerecords = nes_tas_rerecord_count(project);
    int branch = nes_tas_current_branch(project);
    bool branch_changed = project->state.current_branch_changed;
    size_t index = SIZE_MAX;
    CHECK(nes_tas_navigation_add(project, 8, "late", "route A", &index) == NES_TAS_OK && index == 0);
    CHECK(nes_tas_navigation_add(project, 0, "start", NULL, &index) == NES_TAS_OK && index == 0);
    CHECK(nes_tas_navigation_add(project, 12, "end", "", &index) == NES_TAS_OK && index == 2);
    CHECK(nes_tas_navigation_add(project, 8, "same", "route B", &index) == NES_TAS_OK && index == 2);
    CHECK(bookmark_is(project, 0, 0, "start", "") && bookmark_is(project, 1, 8, "late", "route A"));
    CHECK(bookmark_is(project, 2, 8, "same", "route B") && bookmark_is(project, 3, 12, "end", ""));
    CHECK(nes_tas_project_change_since(project, revision).first_changed_frame == SIZE_MAX);
    CHECK(nes_tas_lag(project, 6) == NES_TAS_LAG_YES && nes_tas_marker_count(project) == 1);
    CHECK(nes_tas_current_branch(project) == branch && project->state.current_branch_changed == branch_changed);
    CHECK(nes_tas_rerecord_count(project) == rerecords);
    CHECK(nes_tas_navigation_neighbor(project, 8, false, &index) && index == 0);
    CHECK(nes_tas_navigation_neighbor(project, 8, true, &index) && index == 3);
    index = 99;
    CHECK(!nes_tas_navigation_neighbor(project, 12, true, &index) && index == 99);
    CHECK(!nes_tas_navigation_neighbor(project, 0, false, &index) && index == 99);
    CHECK(nes_tas_navigation_rename(project, 1, "renamed", "new note") == NES_TAS_OK);
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK && bookmark_is(project, 1, 8, "late", "route A"));
    CHECK(nes_tas_project_redo(project) == NES_TAS_OK && bookmark_is(project, 1, 8, "renamed", "new note"));
    CHECK(nes_tas_navigation_remove(project, 2) == NES_TAS_OK && nes_tas_navigation_count(project) == 3);
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK && bookmark_is(project, 2, 8, "same", "route B"));
    CHECK(nes_tas_edit_begin(project) == NES_TAS_OK);
    CHECK(nes_tas_navigation_add(project, 4, "temporary", NULL, NULL) == NES_TAS_OK);
    CHECK(nes_tas_navigation_remove(project, 0) == NES_TAS_OK);
    nes_tas_edit_cancel(project);
    CHECK(nes_tas_navigation_count(project) == 4 && bookmark_is(project, 0, 0, "start", ""));
    CHECK(nes_tas_insert_frames(project, 8, 2) == NES_TAS_OK);
    CHECK(bookmark_is(project, 1, 10, "renamed", "new note") && bookmark_is(project, 3, 14, "end", ""));
    CHECK(nes_tas_delete_frames(project, 9, 3) == NES_TAS_OK);
    CHECK(bookmark_is(project, 1, 9, "renamed", "new note") && bookmark_is(project, 2, 9, "same", "route B"));
    CHECK(bookmark_is(project, 3, 11, "end", ""));
    CHECK(nes_tas_truncate(project, 5) == NES_TAS_OK);
    CHECK(bookmark_is(project, 3, 5, "end", ""));
    CHECK(nes_tas_bookmark_set(project, 1, 5, "short branch") == NES_TAS_OK);
    CHECK(nes_tas_insert_frames(project, 5, 10) == NES_TAS_OK && bookmark_is(project, 3, 15, "end", ""));
    CHECK(nes_tas_bookmark_deploy(project, 1) == NES_TAS_OK);
    CHECK(nes_tas_project_frame_count(project) == 5 && bookmark_is(project, 3, 5, "end", ""));
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK);
    CHECK(nes_tas_project_frame_count(project) == 15 && bookmark_is(project, 3, 15, "end", ""));
    CHECK(nes_tas_truncate(project, 0) == NES_TAS_OK);
    CHECK(bookmark_is(project, 0, 0, "start", "") && bookmark_is(project, 3, 0, "end", ""));
    CHECK(nes_tas_navigation_add(project, 0, "empty endpoint", NULL, NULL) == NES_TAS_OK);
cleanup:
    nes_tas_project_destroy(project);
    return ok;
}

static bool rejected_edits(void) {
    bool ok = true;
    NesTasProject *project = make_project(4);
    CHECK(project);
    char long_name[NES_TAS_BOOKMARK_NAME_MAX + 2], long_note[NES_TAS_NAVIGATION_NOTE_MAX + 2];
    memset(long_name, 'n', sizeof(long_name) - 1);
    long_name[sizeof(long_name) - 1] = '\0';
    memset(long_note, 't', sizeof(long_note) - 1);
    long_note[sizeof(long_note) - 1] = '\0';
    size_t index = 123;
    CHECK(nes_tas_navigation_add(project, 5, "bad", NULL, &index) == NES_TAS_RANGE_ERROR && index == 123);
    CHECK(nes_tas_navigation_add(project, 1, "", NULL, &index) == NES_TAS_INVALID_ARGUMENT);
    CHECK(nes_tas_navigation_add(project, 1, NULL, NULL, &index) == NES_TAS_INVALID_ARGUMENT);
    CHECK(nes_tas_navigation_add(project, 1, long_name, NULL, &index) == NES_TAS_INVALID_ARGUMENT);
    CHECK(nes_tas_navigation_add(project, 1, "name", long_note, &index) == NES_TAS_INVALID_ARGUMENT);
    CHECK(nes_tas_project_revision(project) == 0 && nes_tas_navigation_count(project) == 0);
    for (unsigned i = 0; i < 8; ++i) {
        CHECK(nes_tas_navigation_add(project, i % 5, "name", "note", NULL) == NES_TAS_OK);
    }
    NesTasNavigationView view = {321, "unchanged", "unchanged"};
    CHECK(!nes_tas_navigation(project, 8, &view) && view.frame == 321 && !strcmp(view.name, "unchanged"));
    CHECK(nes_tas_navigation_remove(project, SIZE_MAX) == NES_TAS_RANGE_ERROR);
    CHECK(nes_tas_navigation_rename(project, SIZE_MAX, "name", NULL) == NES_TAS_RANGE_ERROR);
    uint64_t revision = nes_tas_project_revision(project);
    CHECK(nes_tas_navigation(project, 0, &view));
    CHECK(nes_tas_navigation_rename(project, 0, view.name, view.note) == NES_TAS_OK);
    CHECK(nes_tas_project_revision(project) == revision);
    /* Reject cloning the snapshot, then reject growing the live bookmark array.
     * Both failures must preserve retained history and every current entry. */
    for (size_t allowed = 0; allowed < 2; ++allowed) {
        size_t undo_count = project->undo_count, bytes = project->history_bytes;
        tas_project_test_fail_alloc_after(allowed);
        CHECK(nes_tas_navigation_add(project, 3, "rejected", NULL, &index) == NES_TAS_OUT_OF_MEMORY);
        tas_project_test_reset_alloc_fail();
        CHECK(nes_tas_navigation_count(project) == 8 && !nes_tas_edit_active(project));
        CHECK(project->undo_count == undo_count && project->history_bytes == bytes);
        for (size_t i = 0; i < 8; ++i) {
            CHECK(nes_tas_navigation(project, i, &view) && !strcmp(view.name, "name") && !strcmp(view.note, "note"));
        }
    }
    CHECK(nes_tas_edit_begin(project) == NES_TAS_OK);
    while (nes_tas_navigation_count(project) < NES_TAS_NAVIGATION_LIMIT) {
        CHECK(nes_tas_navigation_add(project, 4, "limit", NULL, NULL) == NES_TAS_OK);
    }
    CHECK(nes_tas_navigation_add(project, 4, "overflow", NULL, NULL) == NES_TAS_RANGE_ERROR);
    nes_tas_edit_cancel(project);
    CHECK(nes_tas_navigation_count(project) == 8);
cleanup:
    tas_project_test_reset_alloc_fail();
    nes_tas_project_destroy(project);
    return ok;
}

static void write_le(uint8_t *data, uint64_t value, unsigned bytes) {
    for (unsigned i = 0; i < bytes; ++i) {
        data[i] = (uint8_t)(value >> (8 * i));
    }
}

static bool write_checked(const char *path, uint8_t *data, size_t size) {
    write_le(data + size - 4, (uint32_t)mz_crc32(MZ_CRC32_INIT, data, size - 4), 4);
    return nes_file_write_atomic(path, data, size) == NES_FILE_OK;
}

static bool persistence_and_legacy(void) {
    bool ok = true;
    NesTasProject *project = make_project(6), *loaded = NULL, *legacy = NULL;
    uint8_t *data = NULL;
    size_t size = 0;
    const char *path = "build/tas-navigation-round-trip.ctas";
    CHECK(project);
    CHECK(nes_tas_navigation_add(project, 0, "start", "intro", NULL) == NES_TAS_OK);
    CHECK(nes_tas_navigation_add(project, 6, "end", "last boundary", NULL) == NES_TAS_OK);
    CHECK(nes_tas_navigation_rename(project, 1, "finish", "last boundary") == NES_TAS_OK);
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK);
    CHECK(nes_tas_project_save(project, path) == NES_TAS_OK);
    CHECK(nes_tas_project_load(path, &loaded) == NES_TAS_OK);
    CHECK(bookmark_is(loaded, 0, 0, "start", "intro") && bookmark_is(loaded, 1, 6, "end", "last boundary"));
    CHECK(nes_tas_project_redo(loaded) == NES_TAS_OK && bookmark_is(loaded, 1, 6, "finish", "last boundary"));
    CHECK(nes_tas_project_history_seek(loaded, 0) == NES_TAS_OK && nes_tas_navigation_count(loaded) == 0);
    CHECK(nes_tas_project_history_seek(loaded, 3) == NES_TAS_OK && nes_tas_navigation_count(loaded) == 2);
    NesTasHistoryView history;
    CHECK(nes_tas_project_history_entry(loaded, 3, &history) && strstr(history.description, "navigation bookmarks"));
    nes_tas_project_destroy(loaded);
    loaded = NULL;
    legacy = make_project(6);
    CHECK(legacy);
    CHECK(nes_tas_marker_set(legacy, 2, "legacy marker") == NES_TAS_OK);
    CHECK(nes_tas_insert_frames(legacy, 1, 2) == NES_TAS_OK);
    CHECK(nes_tas_project_undo(legacy) == NES_TAS_OK);
    CHECK(nes_tas_project_save(legacy, path) == NES_TAS_OK);
    CHECK(nes_file_read_all(path, 1024u * 1024u, &data, &size) == NES_FILE_OK);
    /* Version 2 has the identical preceding state layout, without any of the
     * appended navigation arrays for current/undo/redo states. */
    size_t tail = 8 * (1 + legacy->undo_count + legacy->redo_count);
    CHECK(size > tail + 16);
    size -= tail;
    write_le(data + NES_CTAS_MAGIC_SIZE, 2, 4);
    CHECK(write_checked(path, data, size));
    CHECK(nes_tas_project_load(path, &loaded) == NES_TAS_OK);
    CHECK(nes_tas_navigation_count(loaded) == 0 && nes_tas_marker_count(loaded) == 1);
    CHECK(nes_tas_project_redo(loaded) == NES_TAS_OK && nes_tas_project_frame_count(loaded) == 8);
    CHECK(nes_tas_project_history_seek(loaded, 0) == NES_TAS_OK && nes_tas_marker_count(loaded) == 0);
cleanup:
    free(data);
    nes_tas_project_destroy(legacy);
    nes_tas_project_destroy(loaded);
    nes_tas_project_destroy(project);
    (void)nes_file_remove(path);
    return ok;
}

static bool malformed_navigation(void) {
    bool ok = true;
    NesTasProject *project = make_project(6), *existing = make_project(2);
    uint8_t *data = NULL, *copy = NULL;
    size_t size = 0;
    const char *path = "build/tas-navigation-malformed.ctas";
    CHECK(project && existing);
    nes_tas_project_set_history_limit(project, 0, 0);
    CHECK(nes_tas_navigation_add(project, 1, "A", NULL, NULL) == NES_TAS_OK);
    CHECK(nes_tas_navigation_add(project, 5, "B", NULL, NULL) == NES_TAS_OK);
    CHECK(nes_tas_project_save(project, path) == NES_TAS_OK);
    CHECK(nes_file_read_all(path, 1024u * 1024u, &data, &size) == NES_FILE_OK);
    CHECK(size > 46);
    size_t nav = size - 4 - 42; /* count + two frame/name/note entries */
    copy = (uint8_t *)malloc(size + 1);
    CHECK(copy);
    for (unsigned bad = 0; bad < 8; ++bad) {
        memcpy(copy, data, size);
        size_t length = size;
        switch (bad) {
        case 0:
            write_le(copy + nav, NES_TAS_NAVIGATION_LIMIT + 1, 8);
            break;
        case 1:
            write_le(copy + nav + 8, 7, 8);
            break;
        case 2:
            write_le(copy + nav + 25, 0, 8);
            break;
        case 3:
            write_le(copy + nav + 16, 0, 4);
            break;
        case 4:
            write_le(copy + nav + 16, NES_TAS_BOOKMARK_NAME_MAX + 1, 4);
            break;
        case 5:
            copy[nav + 20] = '\0';
            break;
        case 6:
            --length;
            break;
        case 7:
            copy[size - 4] = 0;
            ++length;
            break;
        }
        CHECK(write_checked(path, copy, length));
        NesTasProject *out = existing;
        CHECK(nes_tas_project_load(path, &out) == NES_TAS_FORMAT_ERROR && out == existing);
        CHECK(nes_tas_project_frame_count(existing) == 2 && nes_tas_navigation_count(existing) == 0);
    }
cleanup:
    free(copy);
    free(data);
    nes_tas_project_destroy(existing);
    nes_tas_project_destroy(project);
    (void)nes_file_remove(path);
    return ok;
}

int test_tas_navigation_accuracy(void) {
    int failures = 0;
    failures += !model_and_history();
    failures += !rejected_edits();
    failures += !persistence_and_legacy();
    failures += !malformed_navigation();
    printf("TAS navigation: model, history, limits, persistence and malformed input: %d failures\n", failures);
    return failures;
}
