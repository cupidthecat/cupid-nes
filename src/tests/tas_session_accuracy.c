/*
 * tas_session_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* TAS execution, editing and live-session restoration. SPDX-License-Identifier: GPL-3.0-or-later */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../replay/tas_session.h"
#include "../replay/tas_project.h"
#include "../replay/tas_project_internal.h"
#include "../replay/tas_session_internal.h"
#include "../replay/tas_startup.h"
#include "../replay/input_event.h"
#include "../ui/frontend_execution.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"
#include "../ui/tas_frontend.h"
#include "../ui/state_runtime.h"
#include "../ui/machine_actions.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../state/state.h"
#include "../system/hardware.h"
#include "../system/execution_policy.h"
#include "../system/vs_system.h"
#include "../debugger/debugger.h"
#include "../util/file_io.h"
#include "../util/md5.h"
#include "../../include/globals.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        if (!(x)) {                                                                                                    \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #x);                                                    \
            ++failures;                                                                                                \
            goto cleanup;                                                                                              \
        }                                                                                                              \
    } while (0)

static bool same_state(const NesStateBlob *expected) {
    NesStateBlob actual = {0};
    bool equal = nes_state_capture(&actual) == NES_STATE_OK && expected->size == actual.size &&
                 !memcmp(expected->data, actual.data, actual.size);
    nes_state_blob_free(&actual);
    return equal;
}

static bool fixture(BoardImage *image) {
    if (!board_image_create(image, 0, 0x8000, 0x2000, true)) {
        return false;
    }
    uint8_t *prg = image->data + 16;
    static const uint8_t program[] = {0x78, 0xA9, 0x01, 0x8D, 0x16, 0x40, 0xA9, 0x00, 0x8D, 0x16, 0x40,
                                      0xAD, 0x16, 0x40, 0x85, 0x10, 0xE6, 0x11, 0xAD, 0x17, 0x40, 0x85,
                                      0x12, 0xA5, 0x11, 0x8D, 0x00, 0x60, 0x4C, 0x01, 0x80};
    memcpy(prg, program, sizeof(program));
    for (unsigned i = 0x7FFA; i <= 0x7FFE; i += 2) {
        prg[i] = 0;
        prg[i + 1] = 0x80;
    }
    apu_power_on(&apu);
    if (board_image_load(image)) {
        return false;
    }
    cpu_use_default_startup_alignment();
    debugger_init();
    debugger_resume();
    if (!frontend_machine_power_cycle()) {
        return false;
    }
    ppu_begin_frame_render(framebuffer);
    return true;
}

static bool write_movie(const char *path, unsigned count) {
    NesFm2Movie movie;
    nes_fm2_movie_init(&movie);
    movie.version = 3;
    movie.ports[0] = movie.ports[1] = NES_FM2_PORT_GAMEPAD;
    movie.frames = calloc(count ? count : 1, sizeof(*movie.frames));
    movie.frame_count = count;
    if (!movie.frames || !nes_tas_rom_md5(movie.rom_md5)) {
        nes_fm2_movie_free(&movie);
        return false;
    }
    for (unsigned i = 0; i < count; ++i) {
        movie.frames[i].pads[0] = (uint8_t)(i * 17u);
        movie.frames[i].pads[1] = (uint8_t)(i * 31u);
    }
    if (count > 60) {
        movie.frames[60].commands = NES_FM2_COMMAND_RESET;
    }
    size_t size = 0, written = 0;
    NesFm2Diagnostic diagnostic;
    bool ok = nes_fm2_measure(&movie, NES_FM2_TEXT, &size, &diagnostic) == NES_FM2_OK;
    uint8_t *bytes = ok ? malloc(size) : NULL;
    ok = bytes && nes_fm2_export(&movie, NES_FM2_TEXT, bytes, size, &written, &diagnostic) == NES_FM2_OK &&
         written == size && nes_file_write_atomic(path, bytes, size) == NES_FILE_OK;
    free(bytes);
    nes_fm2_movie_free(&movie);
    return ok;
}

static bool step(NesTasSession *session) {
    if (nes_tas_session_frame_boundary(session) != NES_MOVIE_OK) {
        return false;
    }
    vs_start_frame();
    unsigned limit = 100000;
    while (!ppu.frame_complete && limit--) {
        if (vs_cpu_step() <= 0 || cpu.halted) {
            return false;
        }
    }
    return ppu.frame_complete && nes_tas_session_frame_complete(session, true) == NES_MOVIE_OK;
}

static bool seek(NesTasSession *session, size_t target) {
    if (nes_tas_session_seek(session, target) != NES_MOVIE_OK) {
        return false;
    }
    for (size_t limit = 0; nes_tas_session_seeking(session) && limit < 10000; ++limit) {
        if (!step(session)) {
            return false;
        }
    }
    NesTasProgress progress;
    nes_tas_session_progress(session, &progress);
    return progress.frame == target && !progress.seeking;
}

static int playback_seek_and_restore(void) {
    int failures = 0;
    const char *path = "build/tas-session-seek.fm2";
    BoardImage image = {0};
    NesTasSession *session = NULL;
    NesStateBlob before = {0}, at40 = {0}, at100 = {0};
    CHECK(fixture(&image) && write_movie(path, 120));
    CHECK(nes_state_capture(&before) == NES_STATE_OK);
    session = nes_tas_session_create();
    CHECK(session && nes_tas_session_set_cache(session, 8u * 1024u * 1024u, 10) == NES_MOVIE_OK);
    CHECK(nes_tas_session_open(session, path) == NES_MOVIE_OK);
    CHECK(!nes_execution_allows_persistence() && !nes_tas_session_project(session));
    for (unsigned i = 0; i < 100; ++i) {
        CHECK(!joypad_set_player(0, BTN_A, true));
        CHECK(step(session));
        if (i == 39) {
            CHECK(nes_state_capture(&at40) == NES_STATE_OK);
        }
    }
    CHECK(nes_state_capture(&at100) == NES_STATE_OK);
    CHECK(seek(session, 40) && same_state(&at40));
    CHECK(seek(session, 100) && same_state(&at100));
    CHECK(seek(session, 0));
    for (unsigned i = 0; i < 100; ++i) {
        CHECK(step(session));
    }
    CHECK(same_state(&at100));
    NesTasProgress progress;
    nes_tas_session_progress(session, &progress);
    CHECK(progress.frame == 100 && !progress.dirty && progress.checkpoint_count > 1);
    CHECK(nes_tas_session_seek(session, 121) == NES_MOVIE_INVALID_ARGUMENT);
    CHECK(same_state(&at100));
    CHECK(nes_tas_session_stop(session) == NES_MOVIE_OK && same_state(&before));
    CHECK(nes_execution_policy() == NES_EXECUTION_LIVE);
cleanup:
    nes_tas_session_destroy(session);
    nes_state_blob_free(&before);
    nes_state_blob_free(&at40);
    nes_state_blob_free(&at100);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)unload_rom();
    board_image_free(&image);
    (void)nes_file_remove(path);
    return failures;
}

static int cache_views_and_invalidation(void) {
    int failures = 0;
    const char *path = "build/tas-cache.fm2", *state_path = "build/tas-cache.state";
    BoardImage image = {0};
    NesTasSession *session = NULL;
    NesStateBlob live = {0}, original = {0}, bound = {0}, partial = {0};
    NesTasCacheInfo cache;
    NesTasCacheEntry entry;
    const size_t budget = 64u * 1024u * 1024u;
    CHECK(fixture(&image) && write_movie(path, 120));
    CHECK(nes_state_capture(&live) == NES_STATE_OK);
    session = nes_tas_session_create();
    CHECK(session && nes_tas_session_set_cache(session, budget, 10) == NES_MOVIE_OK);
    CHECK(!nes_tas_session_cache_before(session, 0, &entry));
    CHECK(nes_tas_session_clear_cache(session, 0) == NES_MOVIE_CONFLICT);
    CHECK(nes_tas_session_open(session, path) == NES_MOVIE_OK);
    CHECK(seek(session, 40));
    CHECK(nes_tas_session_capture_state(session, &bound) == NES_STATE_OK);
    CHECK(nes_file_write_atomic(state_path, bound.data, bound.size) == NES_FILE_OK);
    CHECK(seek(session, 95));
    CHECK(nes_state_capture(&original) == NES_STATE_OK);
    nes_tas_session_cache_info(session, &cache);
    CHECK(cache.checkpoint_count == 9 && cache.initial_bytes > 0 && cache.byte_limit == budget && cache.interval == 10);
    CHECK(nes_tas_session_cache_entry(session, 0, &entry) && entry.frame == 10 && entry.bytes > 0 && !entry.initial);
    size_t state_bytes = entry.bytes;
    CHECK(!nes_tas_session_cache_entry(session, 9, &entry));
    CHECK(!nes_tas_session_cache_entry(session, SIZE_MAX, &entry));
    CHECK(!nes_tas_session_cache_before(session, 121, &entry));
    CHECK(!nes_tas_session_cache_before(session, 1, NULL));
    for (size_t frame = 0; frame <= 120; ++frame) {
        CHECK(nes_tas_session_cache_before(session, frame, &entry));
        CHECK(entry.frame == (frame < 90 ? frame / 10 * 10 : 90));
        CHECK(entry.initial == (frame < 10));
    }
    CHECK(same_state(&original));
    CHECK(nes_tas_session_clear_cache(session, 121) == NES_MOVIE_INVALID_ARGUMENT);
    CHECK(nes_tas_session_clear_cache(session, 60) == NES_MOVIE_OK);
    nes_tas_session_cache_info(session, &cache);
    CHECK(cache.checkpoint_count == 6);
    CHECK(nes_tas_session_cache_before(session, 95, &entry) && entry.frame == 60);
    CHECK(same_state(&original));
    CHECK(seek(session, 0) && seek(session, 95) && same_state(&original));
    CHECK(nes_tas_session_set_cache(session, state_bytes * 2, 10) == NES_MOVIE_OK);
    nes_tas_session_cache_info(session, &cache);
    CHECK(cache.checkpoint_count <= 2 && cache.checkpoint_bytes <= state_bytes * 2);
    CHECK(same_state(&original));
    CHECK(nes_tas_session_set_cache(session, budget, 0) == NES_MOVIE_INVALID_ARGUMENT);
    CHECK(nes_tas_session_set_cache(session, budget, 3601) == NES_MOVIE_INVALID_ARGUMENT);
    CHECK(nes_tas_session_set_cache(session, (size_t)1024 * 1024 * 1024 + 1, 10) == NES_MOVIE_INVALID_ARGUMENT);
    CHECK(nes_tas_session_set_cache(session, budget, 10) == NES_MOVIE_OK);
    CHECK(seek(session, 0) && seek(session, 95) && same_state(&original));
    nes_tas_session_cache_info(session, &cache);
    CHECK(cache.checkpoint_count == 2);
    /* Seeking reuses the later states retained by the smaller budget. Clear
       them so the edit below starts with checkpoints at every ten frames. */
    CHECK(nes_tas_session_clear_cache(session, 0) == NES_MOVIE_OK);
    CHECK(seek(session, 0) && seek(session, 95) && same_state(&original));
    nes_tas_session_cache_info(session, &cache);
    CHECK(cache.checkpoint_count == 9);

    CHECK(nes_tas_session_set_read_only(session, false) == NES_MOVIE_OK);
    NesTasProject *project = nes_tas_session_project(session);
    CHECK(project != NULL);
    NesFm2Frame input = *nes_tas_project_frame(project, 20);
    input.pads[0] ^= 1;
    CHECK(nes_tas_set_frame(project, 20, &input) == NES_TAS_OK);
    /* Reading the view must hide stale states without reconciling execution. */
    nes_tas_session_cache_info(session, &cache);
    CHECK(cache.checkpoint_count == 2 && session->checkpoint_count == 9);
    CHECK(nes_tas_session_cache_before(session, 95, &entry) && entry.frame == 20);
    NesTasProgress progress;
    nes_tas_session_progress(session, &progress);
    CHECK(progress.frame == 95 && progress.checkpoint_count == 3 && same_state(&original));
    CHECK(nes_tas_session_reconcile(session) == NES_MOVIE_OK);
    CHECK(seek(session, 95));
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK);
    CHECK(nes_tas_session_cache_before(session, 95, &entry) && entry.frame == 20);
    CHECK(seek(session, 95) && same_state(&original));

    NesTasLagState previous_lag = nes_tas_lag(project, 30);
    uint64_t revision = nes_tas_project_revision(project);
    CHECK(nes_tas_annotate_lag(project, 30, previous_lag == NES_TAS_LAG_YES ? NES_TAS_LAG_NO : NES_TAS_LAG_YES) == NES_TAS_OK);
    CHECK(nes_tas_project_revision(project) > revision);
    CHECK(nes_tas_session_cache_before(session, 95, &entry) && entry.frame == 30);
    CHECK(same_state(&original));
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK && nes_tas_lag(project, 30) == previous_lag);
    CHECK(seek(session, 95) && same_state(&original));

    CHECK(nes_tas_session_load_state(session, state_path) == NES_STATE_OK);
    CHECK(nes_tas_session_cache_before(session, 95, &entry) && entry.frame == 30);
    CHECK(seek(session, 95) && same_state(&original));
    CHECK(nes_tas_session_clear_cache(session, 0) == NES_MOVIE_OK);
    nes_tas_session_cache_info(session, &cache);
    CHECK(!cache.checkpoint_count && !cache.checkpoint_bytes && cache.initial_bytes > 0);
    CHECK(same_state(&original));
    CHECK(nes_tas_session_set_cache(session, 0, 10) == NES_MOVIE_OK);
    CHECK(seek(session, 0) && seek(session, 95) && same_state(&original));
    nes_tas_session_cache_info(session, &cache);
    CHECK(!cache.checkpoint_count && !cache.byte_limit);
    CHECK(nes_tas_session_frame_boundary(session) == NES_MOVIE_OK);
    CHECK(nes_state_capture(&partial) == NES_STATE_OK);
    CHECK(nes_tas_session_clear_cache(session, 0) == NES_MOVIE_OK && same_state(&partial));
    CHECK(step(session));
    CHECK(nes_tas_session_discard(session) == NES_MOVIE_OK && same_state(&live));
cleanup:
    nes_tas_session_destroy(session);
    nes_state_blob_free(&live);
    nes_state_blob_free(&original);
    nes_state_blob_free(&bound);
    nes_state_blob_free(&partial);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)unload_rom();
    board_image_free(&image);
    (void)nes_file_remove(path);
    (void)nes_file_remove(state_path);
    return failures;
}

static int edits_recording_and_branches(void) {
    int failures = 0;
    const char *path = "build/tas-session-edit.fm2", *saved = "build/tas-session-edit.ctas";
    BoardImage image = {0};
    NesTasSession *session = NULL;
    NesStateBlob expected = {0}, changed = {0};
    CHECK(fixture(&image) && write_movie(path, 100));
    session = nes_tas_session_create();
    CHECK(session && nes_tas_session_open(session, path) == NES_MOVIE_OK);
    CHECK(seek(session, 80));
    CHECK(nes_state_capture(&expected) == NES_STATE_OK);
    CHECK(nes_tas_session_set_read_only(session, false) == NES_MOVIE_OK);
    CHECK(nes_tas_session_bookmark_set(session, 0, "original") == NES_MOVIE_OK);
    NesTasProject *project = nes_tas_session_project(session);
    /* The fixture stores its latest A input. Frame 79 originally presses A, so
     * clearing it produces an observable state change at the comparison point. */
    CHECK(project && nes_tas_paint_button(project, 20, 79, 0, 1, false) == NES_TAS_OK);
    CHECK(nes_tas_session_reconcile(session) == NES_MOVIE_OK);
    while (nes_tas_session_seeking(session)) {
        CHECK(step(session));
    }
    CHECK(!same_state(&expected));
    CHECK(nes_state_capture(&changed) == NES_STATE_OK);
    CHECK(seek(session, 0) && seek(session, 80) && same_state(&changed));
    CHECK(nes_tas_session_bookmark_load(session, 0) == NES_MOVIE_OK);
    while (nes_tas_session_seeking(session)) {
        CHECK(step(session));
    }
    CHECK(same_state(&expected));
    project = nes_tas_session_project(session);
    CHECK(project && nes_tas_project_undo(project) == NES_TAS_OK);
    CHECK(seek(session, 80) && same_state(&changed));
    CHECK(nes_tas_project_redo(project) == NES_TAS_OK);
    CHECK(seek(session, 80) && same_state(&expected));
    CHECK(seek(session, 10));
    CHECK(nes_tas_session_set_recording(session, true, NES_TAS_RECORD_INSERT, 1) == NES_MOVIE_OK);
    CHECK(!joypad_set_player(0, BTN_A, true));
    CHECK(nes_tas_session_set_hold(session, 0, 0x80) == NES_MOVIE_OK);
    CHECK(nes_tas_session_set_autofire(session, 0, 2, 1, 1) == NES_MOVIE_OK);
    CHECK(step(session) && step(session) && step(session));
    CHECK(nes_tas_session_set_recording(session, false, NES_TAS_RECORD_INSERT, 1) == NES_MOVIE_OK);
    project = nes_tas_session_project(session);
    CHECK(project && nes_tas_project_frame_count(project) == 103);
    CHECK(nes_tas_project_frame(project, 10)->pads[0] == 0x83);
    CHECK(nes_tas_project_frame(project, 11)->pads[0] == 0x81);
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK && nes_tas_project_frame_count(project) == 100);
    CHECK(nes_tas_project_redo(project) == NES_TAS_OK && nes_tas_project_frame_count(project) == 103);
    CHECK(nes_tas_session_save(session, saved) == NES_MOVIE_OK);
    CHECK(nes_tas_session_stop(session) == NES_MOVIE_OK);
    CHECK(nes_tas_session_open(session, saved) == NES_MOVIE_OK);
    CHECK(nes_tas_project_frame_count(nes_tas_session_project_const(session)) == 103);
    CHECK(nes_tas_session_stop(session) == NES_MOVIE_OK);
cleanup:
    nes_tas_session_destroy(session);
    nes_state_blob_free(&expected);
    nes_state_blob_free(&changed);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)unload_rom();
    board_image_free(&image);
    (void)nes_file_remove(path);
    (void)nes_file_remove(saved);
    return failures;
}

static int frontend_end_and_partial_recording(void) {
    int failures = 0;
    BoardImage image = {0};
    FrontendExecutionRuntime runtime = {0};
    bool initialized = false;
    const char *path = "build/tas-session-ui.fm2", *record = "build/tas-session-record.fm2";
    char error[256] = {0};
    CHECK(fixture(&image) && write_movie(path, 3));
    frontend_execution_init(&runtime, NULL, 0, NULL, NULL, NULL, NULL);
    initialized = true;
    CHECK(frontend_execution_register_commands(&runtime));
    CHECK(tas_frontend_open(&runtime, path, false, error, sizeof(error)));
    for (unsigned i = 0; i < 3; ++i) {
        CHECK(execution_control_request_frame(&runtime.execution));
        CHECK(frontend_execution_run_frame(&runtime));
    }
    CHECK(execution_control_request_frame(&runtime.execution));
    CHECK(!frontend_execution_run_frame(&runtime));
    CHECK(nes_movie_mode(runtime.movie) == NES_MOVIE_PLAYBACK && frontend_execution_paused(&runtime));
    CHECK(frontend_command_invoke(FRONTEND_COMMAND_SPEED_DOUBLE, error, sizeof(error)));
    CHECK(tas_frontend_seek(&runtime, 1, error, sizeof(error)));
    while (nes_tas_session_seeking(nes_movie_tas(runtime.movie))) {
        CHECK(frontend_execution_run_frame(&runtime));
    }
    CHECK(frontend_execution_movie_stop(&runtime, error, sizeof(error)));
    NesTasSession *session = nes_movie_tas(runtime.movie);
    CHECK(nes_tas_session_new(session, record, true) == NES_MOVIE_OK);
    CHECK(frontend_command_invoke(FRONTEND_COMMAND_SOFT_RESET, error, sizeof(error)));
    CHECK(step(session));
    CHECK(nes_tas_project_frame(nes_tas_session_project_const(session), 0)->commands == NES_FM2_COMMAND_RESET);
    CHECK(nes_tas_session_frame_boundary(session) == NES_MOVIE_OK);
    vs_start_frame();
    for (unsigned i = 0; i < 10; ++i) {
        CHECK(vs_cpu_step() > 0);
    }
    CHECK(nes_tas_session_frame_complete(session, false) == NES_MOVIE_OK);
    CHECK(nes_tas_project_frame_count(nes_tas_session_project_const(session)) == 1);
    CHECK(nes_tas_session_stop(session) == NES_MOVIE_OK);
    CHECK(nes_tas_session_open(session, record) == NES_MOVIE_OK);
    CHECK(nes_tas_project_frame_count(nes_tas_session_project_const(session)) == 1);
    CHECK(nes_tas_session_stop(session) == NES_MOVIE_OK);
    size_t frame = 73;
    CHECK(!tas_frontend_parse_frame("-1", &frame) && frame == 73);
    CHECK(!tas_frontend_parse_frame("7x", &frame) && frame == 73);
cleanup:
    if (initialized) {
        frontend_execution_shutdown(&runtime);
    }
    frontend_commands_reset();
    frontend_panels_reset();
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)unload_rom();
    board_image_free(&image);
    (void)nes_file_remove(path);
    (void)nes_file_remove(record);
    return failures;
}

static int rejected_file_preserves_machine(void) {
    int failures = 0;
    BoardImage image = {0};
    NesTasSession *session = NULL;
    NesStateBlob before = {0};
    const char *path = "build/tas-session-reject.fm2";
    CHECK(fixture(&image));
    session = nes_tas_session_create();
    CHECK(session && nes_state_capture(&before) == NES_STATE_OK);
    const char bad[] = "version 3\nromChecksum 00000000000000000000000000000000\n|0|........|........||\n";
    CHECK(nes_file_write_atomic(path, bad, sizeof(bad) - 1) == NES_FILE_OK);
    CHECK(nes_tas_session_open(session, path) != NES_MOVIE_OK && same_state(&before));
    CHECK(nes_execution_policy() == NES_EXECUTION_LIVE && !nes_tas_session_active(session));
    const char truncated[] = "version 3\nbinary 1\nlength 100\n|";
    CHECK(nes_file_write_atomic(path, truncated, sizeof(truncated) - 1) == NES_FILE_OK);
    CHECK(nes_tas_session_open(session, path) != NES_MOVIE_OK && same_state(&before));
cleanup:
    nes_tas_session_destroy(session);
    nes_state_blob_free(&before);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)unload_rom();
    board_image_free(&image);
    (void)nes_file_remove(path);
    return failures;
}

static int md5_vectors(void) {
    const char *input[] = {"", "abc", "message digest"};
    const char *expected[] = {"d41d8cd98f00b204e9800998ecf8427e", "900150983cd24fb0d6963f7d28e17f72",
                              "f96b697d7cb7938d525a2f31aaf161d0"};
    for (unsigned i = 0; i < 3; ++i) {
        NesMd5 hash;
        uint8_t digest[16];
        char hex[33];
        nes_md5_init(&hash);
        for (size_t j = 0; j < strlen(input[i]); ++j) {
            if (!nes_md5_update(&hash, input[i] + j, 1)) {
                return 1;
            }
        }
        if (!nes_md5_final(&hash, digest)) {
            return 1;
        }
        for (unsigned j = 0; j < 16; ++j) {
            snprintf(hex + j * 2, 3, "%02x", digest[j]);
        }
        if (strcmp(hex, expected[i])) {
            return 1;
        }
    }
    return 0;
}

static int startup_checkpoint_and_bound_state(void) {
    int failures = 0;
    BoardImage image = {0};
    NesTasSession *session = NULL;
    NesStateBlob live = {0}, first = {0}, second = {0}, bound = {0};
    const char *movie_path = "build/tas-startup-checkpoint.fm2";
    const char *state_path = "build/tas-startup-checkpoint.state";
    CHECK(fixture(&image) && write_movie(movie_path, 6));
    CHECK(nes_state_capture(&live) == NES_STATE_OK);
    CHECK(!ppu.tas_postrender_boundary);
    session = nes_tas_session_create();
    CHECK(session && nes_tas_session_open(session, movie_path) == NES_MOVIE_OK);
    CHECK(ppu.tas_postrender_boundary && ppu.tas_startup_frames == 2);
    CHECK(step(session) && ppu.tas_startup_frames == 2 && ppu.scanline == 240);
    CHECK(nes_state_capture(&first) == NES_STATE_OK);
    CHECK(nes_tas_session_capture_state(session, &bound) == NES_STATE_OK);
    CHECK(nes_file_write_atomic(state_path, bound.data, bound.size) == NES_FILE_OK);
    CHECK(step(session) && ppu.tas_startup_frames == 1 && ppu.scanline == 240);
    CHECK(nes_state_capture(&second) == NES_STATE_OK);
    CHECK(step(session) && ppu.tas_startup_frames == 0 && ppu.scanline == 240);
    CHECK(seek(session, 1) && same_state(&first));
    CHECK(step(session) && same_state(&second));
    CHECK(nes_tas_session_load_state(session, state_path) == NES_STATE_OK && same_state(&first));
    CHECK(step(session) && same_state(&second));
    CHECK(nes_tas_session_stop(session) == NES_MOVIE_OK && same_state(&live));
    CHECK(!ppu.tas_postrender_boundary);
cleanup:
    nes_tas_session_destroy(session);
    nes_state_blob_free(&live);
    nes_state_blob_free(&first);
    nes_state_blob_free(&second);
    nes_state_blob_free(&bound);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)unload_rom();
    board_image_free(&image);
    (void)nes_file_remove(movie_path);
    (void)nes_file_remove(state_path);
    return failures;
}

static int movie_profile_restores_live_options(void) {
    int failures = 0;
    BoardImage image = {0};
    NesTasSession *session = NULL;
    NesStateBlob baseline = {0}, selected = {0};
    const char *path = "build/tas-hardware-profile.fm2";
    CHECK(fixture(&image) && write_movie(path, 4));
    CHECK(nes_state_capture(&baseline) == NES_STATE_OK);
    CHECK(nes_set_console_model(NES_CONSOLE_HVC101));
    CHECK(apu_set_cpu_revision(APU_CPU_REVISION_LATE_2A03));
    cpu_set_test_mode(true);
    apu_set_disable_noise_mode(true);
    apu_set_swap_duty_cycles(true);
    ppu_set_oamdata_read_disabled(true);
    ppu_set_palette_readback_disabled(true);
    ppu_set_reset_suppression(true);
    ppu_set_startup_write_restriction(true);
    CHECK(nes_state_capture(&selected) == NES_STATE_OK);
    session = nes_tas_session_create();
    CHECK(session && nes_tas_session_open(session, path) == NES_MOVIE_OK);
    CHECK(nes_console_model() == NES_CONSOLE_NES001);
    CHECK(apu_get_cpu_revision() == APU_CPU_REVISION_EARLY_2A03);
    CHECK(!ppu_oamdata_read_disabled() && !ppu_palette_readback_disabled());
    CHECK(!ppu_reset_suppression_enabled() && !ppu_startup_write_restriction_enabled());
    CHECK(step(session));
    CHECK(nes_tas_session_stop(session) == NES_MOVIE_OK && same_state(&selected));
cleanup:
    nes_tas_session_destroy(session);
    if (baseline.data) {
        (void)nes_state_restore(baseline.data, baseline.size);
    }
    nes_state_blob_free(&baseline);
    nes_state_blob_free(&selected);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)unload_rom();
    board_image_free(&image);
    (void)nes_file_remove(path);
    return failures;
}

static int recorded_commands_and_cancelled_queue(void) {
    int failures = 0;
    BoardImage image = {0};
    NesTasSession *session = NULL;
    NesStateBlob after_reset = {0};
    const char *path = "build/tas-command-queue.fm2";
    CHECK(fixture(&image) && write_movie(path, 4));
    session = nes_tas_session_create();
    CHECK(session && nes_tas_session_open(session, path) == NES_MOVIE_OK);
    CHECK(nes_tas_session_queue_commands(session, NES_FM2_COMMAND_RESET) == NES_MOVIE_CONFLICT);
    CHECK(nes_tas_session_set_read_only(session, false) == NES_MOVIE_OK);
    CHECK(nes_tas_session_set_recording(session, true, NES_TAS_RECORD_OVERWRITE, 1) == NES_MOVIE_OK);
    CHECK(nes_tas_session_queue_commands(session, NES_FM2_COMMAND_POWER) == NES_MOVIE_OK);
    CHECK(nes_tas_session_set_recording(session, false, NES_TAS_RECORD_OVERWRITE, 1) == NES_MOVIE_OK);
    CHECK(nes_tas_session_set_recording(session, true, NES_TAS_RECORD_OVERWRITE, 1) == NES_MOVIE_OK);
    CHECK(step(session));
    const NesTasProject *project = nes_tas_session_project_const(session);
    CHECK(nes_tas_project_frame(project, 0)->commands == 0);
    CHECK(nes_tas_session_queue_commands(session, NES_FM2_COMMAND_FDS_INSERT) == NES_MOVIE_INVALID_ARGUMENT);
    CHECK(nes_tas_session_queue_commands(session, NES_FM2_COMMAND_RESET) == NES_MOVIE_OK);
    CHECK(step(session));
    CHECK(nes_tas_project_frame(project, 1)->commands == NES_FM2_COMMAND_RESET);
    CHECK(nes_state_capture(&after_reset) == NES_STATE_OK);
    CHECK(nes_tas_session_set_recording(session, false, NES_TAS_RECORD_OVERWRITE, 1) == NES_MOVIE_OK);
    CHECK(seek(session, 1) && step(session) && same_state(&after_reset));
    CHECK(nes_tas_session_discard(session) == NES_MOVIE_OK);
cleanup:
    nes_tas_session_destroy(session);
    nes_state_blob_free(&after_reset);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)unload_rom();
    board_image_free(&image);
    (void)nes_file_remove(path);
    return failures;
}

static int zapper_recorded_trigger_pulse(void) {
    int failures = 0;
    BoardImage image = {0};
    NesTasSession *session = NULL;
    const char *path = "build/tas-zapper-pulse.ctas";
    CHECK(fixture(&image));
    CHECK(joypad_set_port_device(1, NES_PORT_ZAPPER));
    session = nes_tas_session_create();
    CHECK(session && nes_tas_session_new(session, path, false) == NES_MOVIE_OK);
    CHECK(nes_tas_session_set_recording(session, true, NES_TAS_RECORD_OVERWRITE, 2) == NES_MOVIE_OK);
    CHECK(!joypad_set_zapper(1, 100, 80, true));
    const NesTasProject *project = nes_tas_session_project_const(session);
    for (unsigned i = 0; i < 6; ++i) {
        if (i == 2) {
            CHECK(!joypad_set_zapper(1, 110, 85, false));
        }
        CHECK(step(session));
        const NesFm2Zapper *zapper = &nes_tas_project_frame(project, i)->zappers[1];
        CHECK(zapper->bogo == (i < 5 ? 5 - i : 0));
        CHECK(zapper->button == 1);
        CHECK((joypad_read_port(&pad2, 1) & 0x10u) == (i < 5 ? 0x10u : 0));
    }
    CHECK(!joypad_set_zapper(1, -1, -1, true));
    CHECK(step(session));
    CHECK(nes_tas_project_frame(project, 6)->zappers[1].button == 2);
    CHECK(nes_tas_project_frame(project, 6)->zappers[1].bogo == 5);
    CHECK((joypad_read_port(&pad2, 1) & 0x18u) == 0x18u);
    CHECK(nes_tas_session_set_recording(session, false, NES_TAS_RECORD_OVERWRITE, 2) == NES_MOVIE_OK);
    CHECK(seek(session, 0));
    for (unsigned i = 0; i < 6; ++i) {
        CHECK(step(session));
        CHECK((joypad_read_port(&pad2, 1) & 0x10u) == (i < 5 ? 0x10u : 0));
    }
    CHECK(step(session));
    CHECK((joypad_read_port(&pad2, 1) & 0x18u) == 0x18u);
    CHECK(nes_tas_session_discard(session) == NES_MOVIE_OK);
cleanup:
    nes_tas_session_destroy(session);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)joypad_set_port_device(1, NES_PORT_GAMEPAD);
    (void)unload_rom();
    board_image_free(&image);
    (void)nes_file_remove(path);
    return failures;
}

static int bound_states_and_dirty_close(void) {
    int failures = 0;
    BoardImage image = {0};
    FrontendExecutionRuntime runtime = {0};
    StateRuntime states = {0};
    FrontendSettings settings;
    bool initialized = false, state_initialized = false;
    const char *path = "build/tas-state-movie.fm2", *state_path = "build/tas-bound.state";
    const char *export_path = "build/tas-state-export.fm2";
    NesStateBlob at30 = {0};
    char error[256] = {0};
    CHECK(fixture(&image) && write_movie(path, 70));
    frontend_execution_init(&runtime, NULL, 0, NULL, NULL, NULL, NULL);
    initialized = true;
    CHECK(frontend_execution_register_commands(&runtime));
    frontend_settings_defaults(&settings);
    snprintf(settings.state_file_path, sizeof(settings.state_file_path), "%s", state_path);
    state_runtime_init(&states, &settings, "build", &runtime);
    state_initialized = true;
    CHECK(state_runtime_register_ui(&states));
    CHECK(tas_frontend_open(&runtime, path, false, error, sizeof(error)));
    NesTasSession *session = nes_movie_tas(runtime.movie);
    CHECK(seek(session, 30) && nes_state_capture(&at30) == NES_STATE_OK);
    CHECK(frontend_command_invoke(STATE_COMMAND_SAVE_FILE, error, sizeof(error)));
    CHECK(seek(session, 60));
    CHECK(frontend_command_invoke(STATE_COMMAND_LOAD_FILE, error, sizeof(error)) && same_state(&at30));
    NesTasProgress progress;
    nes_tas_session_progress(session, &progress);
    CHECK(progress.frame == 30 && !progress.dirty);
    snprintf(settings.state_file_path, sizeof(settings.state_file_path), "%s", path);
    CHECK(!frontend_command_invoke(STATE_COMMAND_SAVE_FILE, error, sizeof(error)));
    snprintf(settings.state_file_path, sizeof(settings.state_file_path), "%s", state_path);
    CHECK(nes_tas_session_set_read_only(session, false) == NES_MOVIE_OK);
    NesTasProject *project = nes_tas_session_project(session);
    CHECK(project && nes_tas_paint_button(project, 40, 45, 0, 1, true) == NES_TAS_OK);
    CHECK(frontend_command_invoke(STATE_COMMAND_LOAD_FILE, error, sizeof(error)) && same_state(&at30));
    project = nes_tas_session_project(session);
    CHECK(project && nes_tas_paint_button(project, 10, 12, 0, 1, true) == NES_TAS_OK);
    CHECK(!frontend_command_invoke(STATE_COMMAND_LOAD_FILE, error, sizeof(error)) && same_state(&at30));
    CHECK(nes_tas_session_stop(session) == NES_MOVIE_CONFLICT && nes_tas_session_active(session));
    CHECK(nes_tas_session_save(session, export_path) == NES_MOVIE_OK);
    nes_tas_session_progress(session, &progress);
    CHECK(progress.dirty && !strcmp(progress.path, path));
    CHECK(nes_tas_session_discard(session) == NES_MOVIE_OK);
cleanup:
    if (state_initialized) {
        state_runtime_shutdown(&states);
    }
    if (initialized) {
        frontend_execution_shutdown(&runtime);
    }
    frontend_commands_reset();
    frontend_panels_reset();
    nes_state_blob_free(&at30);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)unload_rom();
    board_image_free(&image);
    (void)nes_file_remove(path);
    (void)nes_file_remove(state_path);
    (void)nes_file_remove(export_path);
    return failures;
}

static int recording_retry_after_allocation_failure(void) {
    int failures = 0;
    const char *path = "build/tas-recording-retry.fm2";
    BoardImage image = {0};
    NesTasSession *session = NULL;
    NesTasProject *project = NULL;
    NesTasProgress progress;

    CHECK(fixture(&image));
    session = nes_tas_session_create();
    CHECK(session != NULL);
    CHECK(nes_tas_session_new(session, path, true) == NES_MOVIE_OK);
    CHECK(step(session));
    nes_tas_session_progress(session, &progress);
    CHECK(progress.frame == 1 && progress.total_frames == 1 && progress.recording && !progress.frame_in_progress);
    CHECK(session->take_open && session->take_changed);
    CHECK(nes_tas_edit_active(session->project));
    CHECK(!nes_tas_project_can_undo(session->project));

    /* Fail storage after the second frame has executed. The first frame and
     * the pending completion must remain available without advancing twice. */
    tas_project_test_fail_alloc_after(0);
    CHECK(nes_tas_session_frame_boundary(session) == NES_MOVIE_OK);
    vs_start_frame();
    unsigned limit = 100000;
    while (!ppu.frame_complete && limit--) {
        CHECK(vs_cpu_step() > 0 && !cpu.halted);
    }
    CHECK(ppu.frame_complete);
    CHECK(nes_tas_session_frame_complete(session, true) == NES_MOVIE_OUT_OF_MEMORY);
    nes_tas_session_progress(session, &progress);
    CHECK(progress.frame == 1 && progress.total_frames == 1 && progress.frame_in_progress && progress.recording);
    CHECK(session->completed_pending && session->pending_record);
    CHECK(session->take_open && session->take_changed && nes_tas_edit_active(session->project));
    CHECK(!nes_tas_project_can_undo(session->project));

    CHECK(tas_finish_take(session) == NES_MOVIE_OUT_OF_MEMORY);
    CHECK(nes_tas_session_save(session, path) == NES_MOVIE_OUT_OF_MEMORY);
    CHECK(nes_tas_session_stop(session) == NES_MOVIE_OUT_OF_MEMORY);
    CHECK(nes_tas_session_active(session));
    CHECK(nes_tas_session_set_recording(session, false, NES_TAS_RECORD_OVERWRITE, 1) == NES_MOVIE_CONFLICT);
    nes_tas_session_progress(session, &progress);
    CHECK(progress.frame == 1 && progress.total_frames == 1 && progress.frame_in_progress && progress.recording);
    CHECK(session->take_open && session->take_changed && nes_tas_edit_active(session->project));
    CHECK(!nes_tas_project_can_undo(session->project));

    tas_project_test_reset_alloc_fail();
    CHECK(nes_tas_session_frame_complete(session, true) == NES_MOVIE_OK);
    nes_tas_session_progress(session, &progress);
    CHECK(progress.frame == 2 && progress.total_frames == 2 && !progress.frame_in_progress && progress.recording);
    CHECK(!session->completed_pending && !session->pending_record);
    CHECK(nes_tas_session_frame_complete(session, true) == NES_MOVIE_OK);
    nes_tas_session_progress(session, &progress);
    CHECK(progress.frame == 2 && progress.total_frames == 2);
    CHECK(nes_tas_session_set_recording(session, false, NES_TAS_RECORD_OVERWRITE, 1) == NES_MOVIE_OK);
    project = nes_tas_session_project(session);
    CHECK(project != NULL && nes_tas_project_frame_count(project) == 2);
    CHECK(nes_tas_project_can_undo(project));
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK);
    CHECK(nes_tas_project_frame_count(project) == 0);
    CHECK(!nes_tas_project_can_undo(project));

cleanup:
    tas_project_test_reset_alloc_fail();
    nes_tas_session_destroy(session);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)unload_rom();
    board_image_free(&image);
    (void)nes_file_remove(path);
    return failures;
}

int test_tas_session_accuracy(void) {
    int failures = md5_vectors() + playback_seek_and_restore() + cache_views_and_invalidation() + edits_recording_and_branches() +
                   frontend_end_and_partial_recording() + rejected_file_preserves_machine() +
                   startup_checkpoint_and_bound_state() + movie_profile_restores_live_options() +
                   recorded_commands_and_cancelled_queue() + zapper_recorded_trigger_pulse() +
                   bound_states_and_dirty_close() + recording_retry_after_allocation_failure();
    printf("TAS session: 12 groups, %d failures\n", failures);
    return failures;
}
