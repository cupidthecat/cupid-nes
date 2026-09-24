/* TAS project model, CTAS persistence and FM3 metadata regressions. SPDX-License-Identifier: GPL-3.0-or-later */
#include "../replay/tas_project.h"
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

static char *tas_test_strdup(const char *text) {
    size_t size = strlen(text) + 1;
    char *copy = (char *)malloc(size);
    if (copy) {
        memcpy(copy, text, size);
    }
    return copy;
}

static bool make_movie(NesFm2Movie *movie, size_t frames) {
    nes_fm2_movie_init(movie);
    movie->version = 3;
    movie->emu_version = 20606;
    movie->rerecord_count = 3;
    movie->rom_filename = tas_test_strdup("tas-project-fixture.nes");
    movie->ports[0] = NES_FM2_PORT_GAMEPAD;
    movie->ports[1] = NES_FM2_PORT_GAMEPAD;
    movie->frames = frames ? (NesFm2Frame *)calloc(frames, sizeof(*movie->frames)) : NULL;
    movie->frame_count = frames;
    if (!movie->rom_filename || (frames && !movie->frames)) {
        nes_fm2_movie_free(movie);
        return false;
    }
    for (size_t i = 0; i < frames; ++i) {
        movie->frames[i].pads[0] = (uint8_t)(i * 17u + 3u);
        movie->frames[i].pads[1] = (uint8_t)(i * 29u + 5u);
    }
    return true;
}

static bool export_project(const NesTasProject *project, uint8_t **data, size_t *size) {
    NesFm2Diagnostic diagnostic;
    size_t measured = 0;
    size_t written = 0;
    *data = NULL;
    *size = 0;
    if (nes_tas_project_measure_fm3(project, &measured, &diagnostic) != NES_TAS_OK) {
        return false;
    }
    uint8_t *output = (uint8_t *)malloc(measured ? measured : 1);
    if (!output) {
        return false;
    }
    if (nes_tas_project_export_fm3(project, output, measured, &written, &diagnostic) != NES_TAS_OK ||
        written != measured) {
        free(output);
        return false;
    }
    *data = output;
    *size = written;
    return true;
}

static bool parse_movie(const uint8_t *data, size_t size, NesFm2Movie *movie) {
    NesFm2Limits limits = nes_fm2_default_limits();
    NesFm2Diagnostic diagnostic;
    return nes_fm2_parse(data, size, &limits, movie, &diagnostic) == NES_FM2_OK;
}

static bool module_is(const NesFm2Movie *movie, unsigned kind, const char *id, size_t size) {
    if (movie->project_modules[kind].size != size) {
        return false;
    }
    if (size == 0) {
        return true;
    }
    return movie->project_modules[kind].data != NULL && memcmp(movie->project_modules[kind].data, id, size) == 0;
}

static bool test_input_history_branches_and_rollback(void) {
    bool ok = true;
    NesFm2Movie movie;
    NesTasProject *project = NULL;
    NesTasMarkerView marker;
    NesTasBookmarkView bookmark;
    NesFm2Frame before;
    size_t before_frames = 0;
    size_t before_markers = 0;
    size_t before_selection = 0;
    uint64_t before_revision = 0;

    CHECK(make_movie(&movie, 12));
    project = nes_tas_project_create(&movie);
    CHECK(project != NULL);
    CHECK(nes_tas_set_intro_note(project, "Start") == NES_TAS_OK);
    CHECK(nes_tas_marker_set(project, 3, "Door") == NES_TAS_OK);
    CHECK(nes_tas_selection_set(project, 2, true) == NES_TAS_OK);
    CHECK(nes_tas_selection_set(project, 4, true) == NES_TAS_OK);
    CHECK(nes_tas_bookmark_set(project, 0, 3, "Base") == NES_TAS_OK);
    CHECK(nes_tas_bookmark(project, 0, &bookmark) && bookmark.occupied && bookmark.key_frame == 3 &&
          strcmp(bookmark.name, "Base") == 0);

    NesFm2Frame edited = *nes_tas_project_frame(project, 5);
    edited.pads[0] ^= 0x80;
    CHECK(nes_tas_edit_begin(project) == NES_TAS_OK);
    CHECK(nes_tas_set_frame(project, 5, &edited) == NES_TAS_OK);
    CHECK(nes_tas_selection_set(project, 6, true) == NES_TAS_OK);
    CHECK(nes_tas_marker_set(project, 7, "After") == NES_TAS_OK);
    CHECK(nes_tas_edit_end(project) == NES_TAS_OK);
    CHECK(nes_tas_project_can_undo(project));
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK);
    CHECK(nes_tas_project_frame(project, 5)->pads[0] != edited.pads[0]);
    CHECK(!nes_tas_selection_contains(project, 6));
    CHECK(nes_tas_marker_count(project) == 1);
    CHECK(nes_tas_project_redo(project) == NES_TAS_OK);
    CHECK(nes_tas_project_frame(project, 5)->pads[0] == edited.pads[0]);
    CHECK(nes_tas_selection_contains(project, 6));
    CHECK(nes_tas_marker_count(project) == 2);

    CHECK(nes_tas_bookmark_set(project, 1, 5, "Branch") == NES_TAS_OK);
    uint64_t branch_rerecord = nes_tas_rerecord_count(project);
    CHECK(nes_tas_increment_rerecord(project) == NES_TAS_OK);
    CHECK(nes_tas_increment_rerecord(project) == NES_TAS_OK);
    uint64_t rerecord_floor = nes_tas_rerecord_count(project);
    CHECK(rerecord_floor == branch_rerecord + 2);
    edited = *nes_tas_project_frame(project, 1);
    edited.pads[1] ^= 0x40;
    CHECK(nes_tas_set_frame(project, 1, &edited) == NES_TAS_OK);
    CHECK(nes_tas_bookmark_deploy(project, 1) == NES_TAS_OK);
    CHECK(nes_tas_current_branch(project) == 1);
    CHECK(nes_tas_rerecord_count(project) >= rerecord_floor);

    before = *nes_tas_project_frame(project, 2);
    before_frames = nes_tas_project_frame_count(project);
    before_markers = nes_tas_marker_count(project);
    before_selection = nes_tas_selection_count(project);
    before_revision = nes_tas_project_revision(project);
    tas_project_test_fail_alloc_after(0);
    CHECK(nes_tas_insert_frames(project, 2, 3) == NES_TAS_OUT_OF_MEMORY);
    tas_project_test_reset_alloc_fail();
    CHECK(nes_tas_project_frame_count(project) == before_frames);
    CHECK(tas_frame_equal(nes_tas_project_frame(project, 2), &before));
    CHECK(nes_tas_marker_count(project) == before_markers);
    CHECK(nes_tas_selection_count(project) == before_selection);
    CHECK(nes_tas_project_revision(project) == before_revision);
    CHECK(nes_tas_marker(project, 0, &marker));

cleanup:
    tas_project_test_reset_alloc_fail();
    nes_tas_project_destroy(project);
    nes_fm2_movie_free(&movie);
    return ok;
}

static bool test_ctas_round_trip(void) {
    static const char path[] = "build/tas-project-accuracy.ctas";
    bool ok = true;
    NesFm2Movie movie;
    NesTasProject *project = NULL;
    NesTasProject *loaded = NULL;
    NesTasBookmarkView bookmark;
    NesTasMarkerView marker;

    CHECK(make_movie(&movie, 20));
    project = nes_tas_project_create(&movie);
    CHECK(project != NULL);
    nes_tas_project_set_history_limit(project, 64, 8u * 1024u * 1024u);
    CHECK(nes_tas_marker_set(project, 4, "CTAS marker") == NES_TAS_OK);
    CHECK(nes_tas_selection_set_range(project, 3, 6, true) == NES_TAS_OK);
    CHECK(nes_tas_copy_selection(project) == NES_TAS_OK);
    CHECK(nes_tas_bookmark_set(project, 2, 6, "Saved branch") == NES_TAS_OK);

    NesFm2Frame first_edit = *nes_tas_project_frame(project, 8);
    first_edit.pads[0] ^= 0x01;
    CHECK(nes_tas_set_frame(project, 8, &first_edit) == NES_TAS_OK);
    NesFm2Frame second_edit = *nes_tas_project_frame(project, 9);
    second_edit.pads[0] ^= 0x02;
    CHECK(nes_tas_set_frame(project, 9, &second_edit) == NES_TAS_OK);
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK);
    CHECK(nes_tas_project_can_undo(project) && nes_tas_project_can_redo(project));

    CHECK(nes_tas_project_save(project, path) == NES_TAS_OK);
    CHECK(nes_tas_project_load(path, &loaded) == NES_TAS_OK && loaded != NULL);
    CHECK(nes_tas_project_frame_count(loaded) == nes_tas_project_frame_count(project));
    CHECK(nes_tas_project_frame(loaded, 8)->pads[0] == nes_tas_project_frame(project, 8)->pads[0]);
    CHECK(nes_tas_marker_count(loaded) == 1);
    CHECK(nes_tas_marker(loaded, 0, &marker) && marker.frame == 4 && strcmp(marker.note, "CTAS marker") == 0);
    CHECK(nes_tas_selection_count(loaded) == 4);
    CHECK(nes_tas_clipboard_available(loaded));
    CHECK(nes_tas_bookmark(loaded, 2, &bookmark) && bookmark.occupied && bookmark.key_frame == 6 &&
          strcmp(bookmark.name, "Saved branch") == 0);
    CHECK(nes_tas_project_can_undo(loaded) && nes_tas_project_can_redo(loaded));
    CHECK(nes_tas_project_redo(loaded) == NES_TAS_OK);
    CHECK(nes_tas_project_frame(loaded, 9)->pads[0] == second_edit.pads[0]);
    CHECK(nes_tas_project_undo(loaded) == NES_TAS_OK);
    CHECK(nes_tas_project_frame(loaded, 9)->pads[0] != second_edit.pads[0]);

cleanup:
    nes_tas_project_destroy(loaded);
    nes_tas_project_destroy(project);
    nes_fm2_movie_free(&movie);
    (void)nes_file_remove(path);
    return ok;
}

static bool test_ctas_malformed_strings_and_counts(void) {
    static const char path[] = "build/tas-project-malformed.ctas";
    bool ok = true;
    NesFm2Movie movie;
    NesTasProject *project = NULL;
    NesTasProject *sentinel = (NesTasProject *)(uintptr_t)1;
    uint8_t *data = NULL;
    size_t size = 0;

    CHECK(make_movie(&movie, 4));
    project = nes_tas_project_create(&movie);
    CHECK(project != NULL);
    CHECK(nes_tas_set_intro_note(project, "NULMAGIC") == NES_TAS_OK);
    CHECK(nes_tas_project_save(project, path) == NES_TAS_OK);
    CHECK(nes_file_read_all(path, 16u * 1024u * 1024u, &data, &size) == NES_FILE_OK);
    CHECK(size > 24);

    size_t note_offset = SIZE_MAX;
    static const uint8_t note[] = "NULMAGIC";
    for (size_t i = 0; i + sizeof(note) - 1 <= size - 4; ++i) {
        if (memcmp(data + i, note, sizeof(note) - 1) == 0) {
            note_offset = i;
            break;
        }
    }
    CHECK(note_offset != SIZE_MAX);
    data[note_offset + 3] = 0;
    uint32_t crc = (uint32_t)mz_crc32(MZ_CRC32_INIT, data, size - 4);
    data[size - 4] = (uint8_t)crc;
    data[size - 3] = (uint8_t)(crc >> 8);
    data[size - 2] = (uint8_t)(crc >> 16);
    data[size - 1] = (uint8_t)(crc >> 24);
    CHECK(nes_file_write_atomic(path, data, size) == NES_FILE_OK);
    CHECK(nes_tas_project_load(path, &sentinel) == NES_TAS_FORMAT_ERROR);
    CHECK(sentinel == (NesTasProject *)(uintptr_t)1);

    free(data);
    data = NULL;
    nes_tas_project_destroy(project);
    project = nes_tas_project_create(&movie);
    CHECK(project != NULL);
    CHECK(nes_tas_project_save(project, path) == NES_TAS_OK);
    CHECK(nes_file_read_all(path, 16u * 1024u * 1024u, &data, &size) == NES_FILE_OK);
    CHECK(size >= 20);
    size_t undo_count = size - 4 - 16;
    uint64_t claimed = 1000000;
    for (unsigned i = 0; i < 8; ++i) {
        data[undo_count + i] = (uint8_t)(claimed >> (i * 8));
    }
    crc = (uint32_t)mz_crc32(MZ_CRC32_INIT, data, size - 4);
    data[size - 4] = (uint8_t)crc;
    data[size - 3] = (uint8_t)(crc >> 8);
    data[size - 2] = (uint8_t)(crc >> 16);
    data[size - 1] = (uint8_t)(crc >> 24);
    CHECK(nes_file_write_atomic(path, data, size) == NES_FILE_OK);
    sentinel = (NesTasProject *)(uintptr_t)1;
    CHECK(nes_tas_project_load(path, &sentinel) == NES_TAS_FORMAT_ERROR);
    CHECK(sentinel == (NesTasProject *)(uintptr_t)1);

cleanup:
    free(data);
    nes_tas_project_destroy(project);
    nes_fm2_movie_free(&movie);
    (void)nes_file_remove(path);
    return ok;
}

static bool test_fm3_metadata_and_stale_modules(void) {
    static const char bookmarks_id[] = "BOOKMARKS";
    static const char greenzone_skip[] = "GREENZONX";
    static const char history_skip[] = "HISTORX";
    static const char piano_skip[] = "PIANO_ROLX";
    bool ok = true;
    NesFm2Movie movie;
    NesFm2Movie source;
    NesFm2Movie edited;
    NesTasProject *seed = NULL;
    NesTasProject *project = NULL;
    NesTasProject *malformed = NULL;
    NesTasProject *round_trip = NULL;
    uint8_t *encoded = NULL;
    size_t encoded_size = 0;
    uint8_t *output = NULL;
    size_t output_size = 0;
    NesTasMarkerView marker;
    NesTasBookmarkView branch;

    nes_fm2_movie_init(&source);
    nes_fm2_movie_init(&edited);
    CHECK(make_movie(&movie, 16));
    seed = nes_tas_project_create(&movie);
    CHECK(seed != NULL);
    CHECK(nes_tas_set_intro_note(seed, "Project start") == NES_TAS_OK);
    CHECK(nes_tas_marker_set(seed, 5, "Marker five") == NES_TAS_OK);
    CHECK(nes_tas_selection_set(seed, 7, true) == NES_TAS_OK);
    CHECK(nes_tas_set_lag(seed, 2, NES_TAS_LAG_YES) == NES_TAS_OK);
    CHECK(nes_tas_set_lag(seed, 5, NES_TAS_LAG_NO) == NES_TAS_OK);
    CHECK(nes_tas_bookmark_set(seed, 0, 4, "First branch") == NES_TAS_OK);
    CHECK(export_project(seed, &encoded, &encoded_size));

    CHECK(parse_movie(encoded, encoded_size, &source));
    CHECK(source.project_present && source.project_version == 3);
    project = nes_tas_project_create(&source);
    CHECK(project != NULL);
    CHECK(strcmp(nes_tas_intro_note(project), "Project start") == 0);
    CHECK(nes_tas_marker_count(project) == 1);
    CHECK(nes_tas_marker(project, 0, &marker) && marker.frame == 5 && strcmp(marker.note, "Marker five") == 0);
    CHECK(nes_tas_selection_count(project) == 1 && nes_tas_selection_contains(project, 7));
    CHECK(nes_tas_lag(project, 2) == NES_TAS_LAG_YES && nes_tas_lag(project, 5) == NES_TAS_LAG_NO);
    CHECK(nes_tas_bookmark(project, 0, &branch) && branch.occupied && branch.key_frame == 4 &&
          !strcmp(branch.name, "First branch"));
    CHECK(export_project(project, &output, &output_size));
    CHECK(output_size == encoded_size && !memcmp(output, encoded, encoded_size));
    CHECK(parse_movie(output, output_size, &edited));
    CHECK((edited.project_saved_modules & (1u << NES_FM3_MODULE_BOOKMARKS)) != 0);
    CHECK(edited.project_modules[NES_FM3_MODULE_BOOKMARKS].size > sizeof(bookmarks_id));
    CHECK(!memcmp(edited.project_modules[NES_FM3_MODULE_BOOKMARKS].data, bookmarks_id, sizeof(bookmarks_id)));
    nes_fm2_movie_free(&edited);
    free(output);
    output = NULL;
    nes_fm2_movie_init(&edited);

    NesFm2Frame input = *nes_tas_project_frame(project, 10);
    input.pads[0] ^= 0x04;
    CHECK(nes_tas_set_frame(project, 10, &input) == NES_TAS_OK);
    CHECK(export_project(project, &output, &output_size));
    CHECK(parse_movie(output, output_size, &edited));
    CHECK(edited.project_saved_modules ==
          ((1u << NES_FM3_MODULE_MARKERS) | (1u << NES_FM3_MODULE_BOOKMARKS) | (1u << NES_FM3_MODULE_SELECTION)));
    CHECK(edited.project_modules[NES_FM3_MODULE_BOOKMARKS].size > sizeof(bookmarks_id));
    CHECK(!memcmp(edited.project_modules[NES_FM3_MODULE_BOOKMARKS].data, bookmarks_id, sizeof(bookmarks_id)));
    CHECK(edited.project_modules[NES_FM3_MODULE_GREENZONE].size >= sizeof(greenzone_skip) + 8);
    CHECK(!memcmp(edited.project_modules[NES_FM3_MODULE_GREENZONE].data, greenzone_skip, sizeof(greenzone_skip)));
    CHECK(module_is(&edited, NES_FM3_MODULE_HISTORY, history_skip, sizeof(history_skip)));
    CHECK(module_is(&edited, NES_FM3_MODULE_PIANO_ROLL, piano_skip, sizeof(piano_skip)));
    round_trip = nes_tas_project_create(&edited);
    CHECK(round_trip != NULL);
    CHECK(nes_tas_marker_count(round_trip) == 1 && nes_tas_selection_contains(round_trip, 7));
    CHECK(nes_tas_lag(round_trip, 2) == NES_TAS_LAG_YES && nes_tas_lag(round_trip, 5) == NES_TAS_LAG_NO);
    CHECK(nes_tas_bookmark(round_trip, 0, &branch) && branch.occupied && !strcmp(branch.name, "First branch"));
    CHECK(nes_tas_bookmark_deploy(round_trip, 0) == NES_TAS_OK);
    CHECK(nes_tas_project_frame(round_trip, 10)->pads[0] == movie.frames[10].pads[0]);
    nes_tas_project_destroy(round_trip);
    round_trip = NULL;
    nes_fm2_movie_free(&edited);
    free(output);
    output = NULL;

    const unsigned validated_modules[] = {NES_FM3_MODULE_MARKERS, NES_FM3_MODULE_BOOKMARKS, NES_FM3_MODULE_GREENZONE,
                                          NES_FM3_MODULE_SELECTION};
    for (size_t i = 0; i < sizeof(validated_modules) / sizeof(validated_modules[0]); ++i) {
        NesFm3ProjectModule *module = &source.project_modules[validated_modules[i]];
        CHECK(module->size && module->data);
        module->data[0] ^= 0x55;
        malformed = nes_tas_project_create(&source);
        CHECK(malformed == NULL);
        module->data[0] ^= 0x55;
    }
    size_t greenzone_size = source.project_modules[NES_FM3_MODULE_GREENZONE].size;
    source.project_modules[NES_FM3_MODULE_GREENZONE].size = sizeof(greenzone_skip);
    malformed = nes_tas_project_create(&source);
    CHECK(malformed == NULL);
    source.project_modules[NES_FM3_MODULE_GREENZONE].size = greenzone_size;

    /* A small compressed branch must not request hundreds of megabytes of
     * decoded input through a forged frame count. Reject it before allocation. */
    NesFm3ProjectModule *bookmarks = &source.project_modules[NES_FM3_MODULE_BOOKMARKS];
    CHECK(bookmarks->size >= 40 && bookmarks->data[10] == 1);
    size_t input_count_offset = 36u + bookmarks->data[35];
    CHECK(input_count_offset <= bookmarks->size - 4);
    uint8_t saved_count[4];
    memcpy(saved_count, bookmarks->data + input_count_offset, sizeof(saved_count));
    for (unsigned i = 0; i < 4; ++i) {
        bookmarks->data[input_count_offset + i] = (uint8_t)(10000000u >> (i * 8));
    }
    NesTasProject *sentinel = (NesTasProject *)(uintptr_t)1;
    CHECK(nes_tas_project_create_checked(&source, &sentinel) == NES_TAS_RANGE_ERROR);
    CHECK(sentinel == (NesTasProject *)(uintptr_t)1);
    memcpy(bookmarks->data + input_count_offset, saved_count, sizeof(saved_count));

    NesFm2Diagnostic diagnostic;
    size_t measured = 0;
    CHECK(nes_tas_project_measure_fm3(project, &measured, &diagnostic) == NES_TAS_OK && measured > 0);
    output = (uint8_t *)malloc(measured);
    CHECK(output != NULL);
    size_t written = 0;
    CHECK(nes_tas_project_export_fm3(project, output, measured - 1, &written, &diagnostic) == NES_TAS_RANGE_ERROR);

cleanup:
    free(output);
    free(encoded);
    nes_tas_project_destroy(malformed);
    nes_tas_project_destroy(round_trip);
    nes_tas_project_destroy(project);
    nes_tas_project_destroy(seed);
    nes_fm2_movie_free(&edited);
    nes_fm2_movie_free(&source);
    nes_fm2_movie_free(&movie);
    return ok;
}

static bool test_lag_transactions_and_branch_relations(void) {
    bool ok = true;
    NesFm2Movie movie;
    NesFm2Movie parsed;
    NesTasProject *project = NULL;
    NesTasProject *loaded = NULL;
    uint8_t *encoded = NULL;
    size_t encoded_size = 0;
    NesTasBookmarkView branch;
    nes_fm2_movie_init(&parsed);
    CHECK(make_movie(&movie, 24));
    project = nes_tas_project_create(&movie);
    CHECK(project != NULL);

    uint64_t revision = nes_tas_project_revision(project);
    CHECK(nes_tas_edit_begin(project) == NES_TAS_OK);
    CHECK(nes_tas_set_lag(project, 4, NES_TAS_LAG_YES) == NES_TAS_OK);
    CHECK(project->fm3_dirty_modules & TAS_FM3_MODULE_BIT(NES_FM3_MODULE_GREENZONE));
    nes_tas_edit_cancel(project);
    CHECK(nes_tas_lag(project, 4) == NES_TAS_LAG_UNKNOWN);
    CHECK(project->fm3_dirty_modules == 0);
    CHECK(nes_tas_project_change_since(project, revision).first_changed_frame == SIZE_MAX);

    CHECK(nes_tas_edit_begin(project) == NES_TAS_OK);
    CHECK(nes_tas_set_lag(project, 4, NES_TAS_LAG_NO) == NES_TAS_OK);
    CHECK(nes_tas_edit_end(project) == NES_TAS_OK);
    CHECK(nes_tas_project_can_undo(project));
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK);
    CHECK(nes_tas_lag(project, 4) == NES_TAS_LAG_UNKNOWN);
    CHECK(nes_tas_project_redo(project) == NES_TAS_OK);
    CHECK(nes_tas_lag(project, 4) == NES_TAS_LAG_NO);
    CHECK(nes_tas_edit_begin(project) == NES_TAS_OK);
    nes_tas_invalidate_lag(project, 4);
    nes_tas_edit_cancel(project);
    CHECK(nes_tas_lag(project, 4) == NES_TAS_LAG_NO);

    CHECK(nes_tas_bookmark_set(project, 0, 5, "Root") == NES_TAS_OK);
    CHECK(nes_tas_current_branch(project) == 0);
    CHECK(nes_tas_bookmark_set(project, 1, 9, "Child") == NES_TAS_OK);
    CHECK(nes_tas_current_branch(project) == 1);
    CHECK(nes_tas_bookmark(project, 1, &branch) && branch.parent_slot == 0);
    CHECK(nes_tas_bookmark_set(project, 0, 12, "Replacement") == NES_TAS_OK);
    CHECK(nes_tas_current_branch(project) == 0);
    CHECK(nes_tas_bookmark(project, 0, &branch) && branch.parent_slot == -1);
    CHECK(nes_tas_bookmark(project, 1, &branch) && branch.parent_slot == -1);
    CHECK(export_project(project, &encoded, &encoded_size));
    CHECK(parse_movie(encoded, encoded_size, &parsed));
    loaded = nes_tas_project_create(&parsed);
    CHECK(loaded != NULL);
    CHECK(nes_tas_bookmark_clear(loaded, 0) == NES_TAS_OK);
    CHECK(nes_tas_current_branch(loaded) == -1);
    CHECK(nes_tas_bookmark(loaded, 1, &branch) && branch.occupied && branch.parent_slot == -1);
    CHECK(nes_tas_bookmark_deploy(loaded, 1) == NES_TAS_OK);
    CHECK(nes_tas_bookmark_set(loaded, 1, 9, "Same slot") == NES_TAS_OK);
    CHECK(nes_tas_bookmark(loaded, 1, &branch) && branch.parent_slot == -1);
    NesFm2Frame changed = *nes_tas_project_frame(loaded, 0);
    changed.pads[0] ^= 0x80;
    CHECK(nes_tas_set_frame(loaded, 0, &changed) == NES_TAS_OK);
    CHECK(nes_tas_bookmark_set(loaded, 2, 15, "Changed prefix") == NES_TAS_OK);
    CHECK(nes_tas_bookmark(loaded, 2, &branch) && branch.parent_slot == -1);
    free(encoded);
    encoded = NULL;
    CHECK(export_project(loaded, &encoded, &encoded_size));

cleanup:
    free(encoded);
    nes_tas_project_destroy(loaded);
    nes_tas_project_destroy(project);
    nes_fm2_movie_free(&parsed);
    nes_fm2_movie_free(&movie);
    return ok;
}

int test_tas_project_accuracy(void) {
    int failures = 0;
    if (!test_input_history_branches_and_rollback()) {
        failures++;
    }
    if (!test_ctas_round_trip()) {
        failures++;
    }
    if (!test_ctas_malformed_strings_and_counts()) {
        failures++;
    }
    if (!test_fm3_metadata_and_stale_modules()) {
        failures++;
    }
    if (!test_lag_transactions_and_branch_relations()) {
        failures++;
    }
    return failures;
}
