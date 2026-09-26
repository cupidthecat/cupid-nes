/*
 * tas_history_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* TAS history observations, atomic navigation and persistence. SPDX-License-Identifier: GPL-3.0-or-later */
#include "../replay/tas_project_internal.h"
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
    movie.rom_filename = (char *)malloc(sizeof("history-fixture.nes"));
    movie.frames = (NesFm2Frame *)calloc(count, sizeof(*movie.frames));
    if (!movie.rom_filename || !movie.frames) {
        nes_fm2_movie_free(&movie);
        return NULL;
    }
    memcpy(movie.rom_filename, "history-fixture.nes", sizeof("history-fixture.nes"));
    movie.frame_count = count;
    for (size_t i = 0; i < count; ++i) {
        movie.frames[i].pads[0] = (uint8_t)i;
    }
    NesTasProject *project = nes_tas_project_create(&movie);
    nes_fm2_movie_free(&movie);
    return project;
}

static bool descriptions_and_navigation(void) {
    bool ok = true;
    NesTasProject *project = make_project(16);
    NesTasProject *loaded = NULL;
    const char *path = "build/tas-history-round-trip.ctas";
    NesTasHistoryInfo info;
    NesTasHistoryView view, saved;
    CHECK(project);
    nes_tas_project_history_info(project, &info);
    CHECK(info.position == 0 && info.operations == 0 && !info.edit_active);
    CHECK(nes_tas_project_history_entry(project, 0, &view) && view.current && view.applied);
    saved = view;
    CHECK(!nes_tas_project_history_entry(project, 1, &view));
    CHECK(!memcmp(&view, &saved, sizeof(view)));
    CHECK(nes_tas_edit_begin(project) == NES_TAS_OK);
    NesFm2Frame frame = *nes_tas_project_frame(project, 2);
    frame.pads[0] ^= 0x80;
    CHECK(nes_tas_set_frame(project, 2, &frame) == NES_TAS_OK);
    frame = *nes_tas_project_frame(project, 7);
    frame.pads[1] = 0x20;
    CHECK(nes_tas_set_frame(project, 7, &frame) == NES_TAS_OK);
    CHECK(nes_tas_marker_set(project, 5, "checkpoint") == NES_TAS_OK);
    nes_tas_project_history_info(project, &info);
    CHECK(info.operations == 0 && info.edit_active);
    CHECK(nes_tas_project_history_seek(project, 0) == NES_TAS_INVALID_ARGUMENT);
    CHECK(nes_tas_edit_end(project) == NES_TAS_OK);
    CHECK(nes_tas_project_history_entry(project, 1, &view));
    CHECK(strstr(view.description, "Edit input") && strstr(view.description, "Edit markers"));
    CHECK(view.first_frame == 2 && view.last_frame == 7 && view.current);
    CHECK(nes_tas_edit_begin(project) == NES_TAS_OK);
    CHECK(nes_tas_annotate_lag(project, 8, NES_TAS_LAG_YES) == NES_TAS_OK);
    CHECK(nes_tas_annotate_lag(project, 10, NES_TAS_LAG_YES) == NES_TAS_OK);
    CHECK(nes_tas_edit_end(project) == NES_TAS_OK);
    CHECK(nes_tas_project_history_entry(project, 2, &view));
    CHECK(!strcmp(view.description, "Lag annotation") && view.first_frame == 8 && view.last_frame == 10);
    CHECK(nes_tas_bookmark_set(project, 2, 9, "route") == NES_TAS_OK);
    CHECK(nes_tas_project_history_entry(project, 3, &view));
    CHECK(!strcmp(view.description, "Store branch 3") && view.first_frame == 9);
    frame = *nes_tas_project_frame(project, 4);
    frame.pads[0] ^= 0x40;
    CHECK(nes_tas_set_frame(project, 4, &frame) == NES_TAS_OK);
    CHECK(nes_tas_bookmark_deploy(project, 2) == NES_TAS_OK);
    CHECK(nes_tas_project_history_entry(project, 5, &view));
    CHECK(strstr(view.description, "Deploy branch 3") && view.first_frame == 4);
    uint64_t revision = nes_tas_project_revision(project);
    CHECK(nes_tas_project_history_seek(project, 1) == NES_TAS_OK);
    CHECK(nes_tas_project_change_since(project, revision).first_changed_frame == 4);
    CHECK(nes_tas_project_revision(project) == revision + 1);
    CHECK(nes_tas_marker_count(project) == 1 && nes_tas_current_branch(project) == -1);
    CHECK(nes_tas_lag(project, 8) == NES_TAS_LAG_UNKNOWN);
    nes_tas_project_history_info(project, &info);
    CHECK(info.position == 1 && info.operations == 5);
    CHECK(nes_tas_project_history_entry(project, 5, &view) && !view.current && !view.applied);
    saved = view;
    revision = nes_tas_project_revision(project);
    CHECK(nes_tas_project_history_seek(project, SIZE_MAX) == NES_TAS_RANGE_ERROR);
    CHECK(nes_tas_project_revision(project) == revision && project->undo_count == 1 && project->redo_count == 4);
    CHECK(nes_tas_project_save(project, path) == NES_TAS_OK);
    CHECK(nes_tas_project_load(path, &loaded) == NES_TAS_OK && loaded);
    nes_tas_project_history_info(loaded, &info);
    CHECK(info.position == 1 && info.operations == 5);
    CHECK(nes_tas_project_history_entry(loaded, 5, &view));
    CHECK(!strcmp(saved.description, view.description) && saved.first_frame == view.first_frame);
    CHECK(nes_tas_project_history_seek(loaded, 5) == NES_TAS_OK);
    CHECK(nes_tas_current_branch(loaded) == 2 && nes_tas_project_frame(loaded, 4)->pads[0] == 4);
    CHECK(nes_tas_project_history_seek(loaded, 0) == NES_TAS_OK);
    CHECK(nes_tas_marker_count(loaded) == 0 && nes_tas_project_frame(loaded, 2)->pads[0] == 2);
    CHECK(nes_tas_project_history_seek(loaded, 5) == NES_TAS_OK);
    CHECK(nes_tas_project_frame(loaded, 2)->pads[0] == (2 ^ 0x80));
cleanup:
    nes_tas_project_destroy(loaded);
    nes_tas_project_destroy(project);
    (void)nes_file_remove(path);
    return ok;
}

static bool budgets_transactions_and_failures(void) {
    bool ok = true;
    NesTasProject *project = make_project(128);
    NesTasHistoryInfo info;
    NesTasHistoryView view;
    CHECK(project);
    CHECK(nes_tas_delete_frames(project, 2, 126) == NES_TAS_OK);
    CHECK(nes_tas_insert_frames(project, 2, 126) == NES_TAS_OK);
    CHECK(project->undo_count == 2);
    size_t budget = project->history_bytes;
    nes_tas_project_set_history_limit(project, 32, budget);
    TasHistoryEntry *first_node = project->undo;
    TasHistoryEntry *second_node = project->undo->next;
    /* A single intermediate undo exceeds this budget. A multi-entry jump must
     * reach its target before eviction and must reuse both retained nodes. */
    CHECK(nes_tas_project_history_seek(project, 0) == NES_TAS_OK);
    CHECK(nes_tas_project_frame(project, 127)->pads[0] == 127);
    CHECK(project->redo == second_node && project->redo->next == first_node);
    CHECK(project->redo_count == 2 && project->history_bytes <= budget);
    CHECK(nes_tas_project_history_seek(project, 2) == NES_TAS_OK);
    CHECK(nes_tas_project_frame(project, 127)->pads[0] == 0 && project->undo == first_node);
    uint64_t revision = nes_tas_project_revision(project);
    tas_project_test_fail_alloc_after(0);
    CHECK(nes_tas_insert_frames(project, 3, 1) == NES_TAS_OUT_OF_MEMORY);
    tas_project_test_reset_alloc_fail();
    CHECK(project->undo_count == 2 && !project->redo_count && nes_tas_project_revision(project) == revision);
    CHECK(nes_tas_edit_begin(project) == NES_TAS_OK);
    CHECK(nes_tas_marker_set(project, 3, "cancelled") == NES_TAS_OK);
    CHECK(nes_tas_project_history_seek(project, 0) == NES_TAS_INVALID_ARGUMENT);
    nes_tas_edit_cancel(project);
    CHECK(nes_tas_marker_count(project) == 0 && project->undo_count == 2);
    nes_tas_project_set_history_limit(project, 32, NES_TAS_DEFAULT_HISTORY_BYTES);
    CHECK(nes_tas_edit_begin(project) == NES_TAS_OK);
    CHECK(nes_tas_increment_rerecord(project) == NES_TAS_OK);
    NesFm2Frame frame = *nes_tas_project_frame(project, 10);
    frame.pads[0] = 0x40;
    CHECK(nes_tas_record_frame(project, 10, &frame, false) == NES_TAS_OK);
    CHECK(nes_tas_record_frame(project, 11, &frame, false) == NES_TAS_OK);
    CHECK(nes_tas_edit_end(project) == NES_TAS_OK);
    CHECK(nes_tas_project_history_entry(project, 3, &view));
    CHECK(!strcmp(view.description, "Record take") && view.first_frame == 10 && view.last_frame == 11);
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK);
    CHECK(nes_tas_project_redo(project) == NES_TAS_OK);
    nes_tas_project_set_history_limit(project, 1, NES_TAS_DEFAULT_HISTORY_BYTES);
    nes_tas_project_history_info(project, &info);
    CHECK(info.operations == 1 && info.position == 1);
    CHECK(nes_tas_project_history_entry(project, 1, &view) && !strcmp(view.description, "Record take"));
    CHECK(nes_tas_project_history_seek(project, 0) == NES_TAS_OK);
    CHECK(nes_tas_project_frame(project, 10)->pads[0] == 0);
    nes_tas_project_set_history_limit(project, 0, 0);
    nes_tas_project_history_info(project, &info);
    CHECK(info.operations == 0 && info.position == 0 && info.bytes == 0);
    CHECK(!nes_tas_project_can_undo(project) && !nes_tas_project_can_redo(project));
    CHECK(nes_tas_project_history_seek(project, 0) == NES_TAS_OK);
cleanup:
    tas_project_test_reset_alloc_fail();
    nes_tas_project_destroy(project);
    return ok;
}

static bool oversized_edit_history_boundary(void) {
    bool ok = true;
    NesTasProject *project = make_project(2);
    NesTasHistoryInfo info;
    CHECK(project);
    CHECK(nes_tas_insert_frames(project, 2, 126) == NES_TAS_OK);
    CHECK(project->undo_count == 1);
    size_t budget = project->history_bytes;
    nes_tas_project_set_history_limit(project, 32, budget);
    CHECK(project->undo_count == 1);
    /* The next edit's preceding state is larger than the entire history
     * budget. An older snapshot cannot represent an undo of this edit. */
    CHECK(nes_tas_marker_set(project, 7, "retained in the current project") == NES_TAS_OK);
    nes_tas_project_history_info(project, &info);
    CHECK(info.position == 0 && info.operations == 0 && info.bytes == 0);
    CHECK(!nes_tas_project_can_undo(project) && !nes_tas_project_can_redo(project));
    CHECK(nes_tas_project_frame_count(project) == 128 && nes_tas_marker_count(project) == 1);
    nes_tas_project_set_history_limit(project, 32, NES_TAS_DEFAULT_HISTORY_BYTES);
    CHECK(nes_tas_marker_set(project, 8, "new retained edit") == NES_TAS_OK);
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK);
    CHECK(nes_tas_project_frame_count(project) == 128 && nes_tas_marker_count(project) == 1);
cleanup:
    nes_tas_project_destroy(project);
    return ok;
}

int test_tas_history_accuracy(void) {
    int failures = 0;
    if (!descriptions_and_navigation()) {
        ++failures;
    }
    if (!budgets_transactions_and_failures()) {
        ++failures;
    }
    if (!oversized_edit_history_boundary()) {
        ++failures;
    }
    printf("TAS history: 3 groups, %d failures\n", failures);
    return failures;
}
