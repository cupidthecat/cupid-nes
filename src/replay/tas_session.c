/* TAS session lifecycle and project files. SPDX-License-Identifier: GPL-3.0-or-later */
#include "tas_session_internal.h"
#include "tas_startup.h"
#include "input_event.h"
#include "rewind.h"
#include "../joypad/joypad.h"
#include "../rom/rom.h"
#include "../system/execution_policy.h"
#include "../system/timing.h"
#include "../util/file_io.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../util/md5.h"

NesMovieResult tas_result(NesTasSession *session, NesMovieResult result, const char *message) {
    if (session) {
        session->last_result = result;
        if (message != session->error) {
            snprintf(session->error, sizeof(session->error), "%s",
                     message                                                  ? message
                     : result == NES_MOVIE_OK || result == NES_MOVIE_COMPLETE ? ""
                                                                              : nes_movie_result_string(result));
        }
    }
    return result;
}

NesMovieResult tas_model_result(NesTasSession *session, NesTasResult result) {
    if (result == NES_TAS_OK) {
        return tas_result(session, NES_MOVIE_OK, NULL);
    }
    NesMovieResult translated = result == NES_TAS_OUT_OF_MEMORY  ? NES_MOVIE_OUT_OF_MEMORY
                                : result == NES_TAS_IO_ERROR     ? NES_MOVIE_IO_ERROR
                                : result == NES_TAS_FORMAT_ERROR ? NES_MOVIE_FORMAT_ERROR
                                                                 : NES_MOVIE_INVALID_ARGUMENT;
    return tas_result(session, translated, nes_tas_result_string(result));
}

bool tas_extension(const char *path, const char *extension) {
    if (!path || !extension) {
        return false;
    }
    size_t length = strlen(path), suffix = strlen(extension);
    if (length < suffix) {
        return false;
    }
    for (size_t i = 0; i < suffix; ++i) {
        if (tolower((unsigned char)path[length - suffix + i]) != (unsigned char)extension[i]) {
            return false;
        }
    }
    return true;
}

char *tas_copy_path(const char *path) {
    if (!path || !*path) {
        return NULL;
    }
    size_t size = strlen(path);
    if (size >= NES_FILE_PATH_LIMIT) {
        return NULL;
    }
    char *copy = malloc(size + 1);
    if (copy) {
        memcpy(copy, path, size + 1);
    }
    return copy;
}

NesTasSession *nes_tas_session_create(void) {
    NesTasSession *session = calloc(1, sizeof(*session));
    if (session) {
        session->cache_limit = TAS_DEFAULT_CACHE_BYTES;
        session->cache_interval = 30;
        session->read_only = true;
        session->record_players = 1;
        for (unsigned player = 0; player < 4; ++player) {
            session->fire_on[player] = session->fire_off[player] = 1;
        }
    }
    return session;
}

static void clear_project(NesTasSession *session) {
    tas_clear_checkpoints(session);
    nes_state_blob_free(&session->initial);
    nes_state_restore_free(session->resume);
    session->resume = NULL;
    nes_tas_project_destroy(session->project);
    session->project = NULL;
    free(session->path);
    session->path = NULL;
    session->active = session->recording = session->seeking = false;
    session->frame_in_progress = session->take_open = session->auto_save = false;
    session->frame = session->lag_count = session->seek_target = session->selected_side = 0;
    session->pending_commands = 0;
    session->completed_pending = session->pending_record = session->take_changed = false;
    memset(session->live_pads, 0, sizeof(session->live_pads));
    memset(session->live_zappers, 0, sizeof(session->live_zappers));
    memset(session->last_zapper_pressed, 0, sizeof(session->last_zapper_pressed));
    memset(session->pending_zapper_pressed, 0, sizeof(session->pending_zapper_pressed));
}

void nes_tas_session_destroy(NesTasSession *session) {
    if (!session) {
        return;
    }
    if (session->active) {
        nes_input_event_clear_observer();
        if (session->resume) {
            (void)nes_state_apply_prepared(session->resume);
        }
        (void)nes_execution_set_policy(session->previous_policy);
    }
    clear_project(session);
    free(session);
}

static NesMovieResult preflight(NesTasSession *session, const char *path) {
    if (!session || !path || !*path || strlen(path) >= NES_FILE_PATH_LIMIT) {
        return tas_result(session, NES_MOVIE_INVALID_ARGUMENT, NULL);
    }
    if (session->active || nes_execution_policy() != NES_EXECUTION_LIVE) {
        return tas_result(session, NES_MOVIE_CONFLICT, NULL);
    }
    if (rom_metadata_source() == ROM_METADATA_NONE) {
        return tas_result(session, NES_MOVIE_NO_IMAGE, NULL);
    }
    if (!nes_replay_host_state_supported()) {
        return tas_result(session, NES_MOVIE_UNSUPPORTED_HOST_STATE, NULL);
    }
    return NES_MOVIE_OK;
}

static NesMovieResult activate(NesTasSession *session, NesTasProject *project, const char *path) {
    const NesFm2Movie *movie = nes_tas_project_movie(project);
    NesMovieResult valid = nes_tas_startup_validate(movie, session->error, sizeof(session->error));
    if (valid != NES_MOVIE_OK) {
        return tas_result(session, valid, session->error);
    }
    char *copy = tas_copy_path(path);
    if (!copy) {
        return tas_result(session, NES_MOVIE_OUT_OF_MEMORY, NULL);
    }
    NesStateBlob live = {0}, initial = {0};
    NesStateRestore *resume = NULL;
    NesStateResult state = nes_state_capture(&live);
    if (state == NES_STATE_OK) {
        state = nes_state_prepare_restore(live.data, live.size, &resume);
    }
    nes_state_blob_free(&live);
    if (state != NES_STATE_OK) {
        free(copy);
        return tas_result(session,
                          state == NES_STATE_ERROR_OUT_OF_MEMORY ? NES_MOVIE_OUT_OF_MEMORY : NES_MOVIE_STATE_ERROR,
                          nes_state_result_string(state));
    }
    uint32_t previous = nes_execution_policy();
    bool configured = nes_tas_startup_configure(movie);
    bool isolated = nes_execution_set_policy(NES_EXECUTION_MOVIE_PLAYBACK);
    bool powered = configured && isolated && nes_tas_startup_power(movie);
    if (powered) {
        state = nes_state_capture(&initial);
    }
    if (!powered || state != NES_STATE_OK) {
        NesStateResult restored = nes_state_apply_prepared(resume);
        (void)nes_execution_set_policy(previous);
        nes_state_restore_free(resume);
        nes_state_blob_free(&initial);
        free(copy);
        return tas_result(session, NES_MOVIE_STATE_ERROR,
                          restored != NES_STATE_OK ? "The previous game state could not be restored."
                                                   : "Movie startup could not be prepared.");
    }
    session->project = project;
    session->path = copy;
    session->initial = initial;
    session->resume = resume;
    session->previous_policy = previous;
    session->active = true;
    session->read_only = true;
    session->recording = false;
    session->saved_revision = session->seen_revision = nes_tas_project_revision(project);
    session->frame = session->lag_count = session->seek_target = session->selected_side = 0;
    nes_input_event_set_observer(tas_input_observer, session);
    return tas_result(session, NES_MOVIE_OK, NULL);
}

NesMovieResult nes_tas_session_open(NesTasSession *session, const char *path) {
    NesMovieResult ready = preflight(session, path);
    if (ready != NES_MOVIE_OK) {
        return ready;
    }
    FILE *file = nes_file_open(path, "rb");
    if (!file) {
        return tas_result(session, NES_MOVIE_IO_ERROR, "The movie or project file could not be opened.");
    }
    uint8_t signature[NES_CTAS_MAGIC_SIZE];
    size_t signature_size = fread(signature, 1, sizeof(signature), file);
    bool read_error = ferror(file) != 0;
    if (fclose(file) != 0) {
        read_error = true;
    }
    if (read_error) {
        return tas_result(session, NES_MOVIE_IO_ERROR, "The movie or project file could not be read.");
    }
    NesTasProject *project = NULL;
    if (signature_size == NES_CTAS_MAGIC_SIZE && !memcmp(signature, NES_CTAS_MAGIC, NES_CTAS_MAGIC_SIZE)) {
        NesTasResult loaded = nes_tas_project_load(path, &project);
        if (loaded != NES_TAS_OK) {
            return tas_model_result(session, loaded);
        }
    } else {
        uint8_t *data = NULL;
        size_t size = 0;
        NesFm2Limits limits = nes_fm2_default_limits();
        NesFileResult read = nes_file_read_all(path, limits.max_file_bytes, &data, &size);
        if (read != NES_FILE_OK) {
            return tas_result(session, NES_MOVIE_IO_ERROR, nes_file_result_message(read));
        }
        NesFm2Movie movie;
        NesFm2Diagnostic diagnostic;
        nes_fm2_movie_init(&movie);
        NesFm2Result parsed = nes_fm2_parse(data, size, &limits, &movie, &diagnostic);
        free(data);
        if (parsed != NES_FM2_OK) {
            nes_fm2_movie_free(&movie);
            return tas_result(session,
                              parsed == NES_FM2_OUT_OF_MEMORY ? NES_MOVIE_OUT_OF_MEMORY : NES_MOVIE_FORMAT_ERROR,
                              diagnostic.message);
        }
        if (!nes_tas_pin_cheats(&movie)) {
            nes_fm2_movie_free(&movie);
            return tas_result(session, NES_MOVIE_OUT_OF_MEMORY, NULL);
        }
        NesTasResult created = nes_tas_project_create_checked(&movie, &project);
        nes_fm2_movie_free(&movie);
        if (created != NES_TAS_OK) {
            return tas_model_result(session, created);
        }
    }
    NesMovieResult result = activate(session, project, path);
    if (result != NES_MOVIE_OK) {
        nes_tas_project_destroy(project);
    }
    return result;
}

NesMovieResult nes_tas_session_new(NesTasSession *session, const char *path, bool record) {
    NesMovieResult ready = preflight(session, path);
    if (ready != NES_MOVIE_OK) {
        return ready;
    }
    if (!tas_extension(path, ".fm2") && !tas_extension(path, ".fm3") && !tas_extension(path, ".ctas")) {
        return tas_result(session, NES_MOVIE_INVALID_ARGUMENT, "Choose an .fm2, .fm3, or .ctas filename.");
    }
    NesFm2Movie movie;
    nes_fm2_movie_init(&movie);
    movie.version = 3;
    movie.emu_version = 0;
    movie.ports[0] = movie.ports[1] = NES_FM2_PORT_GAMEPAD;
    movie.fourscore = joypad_adapter() == NES_ADAPTER_FOUR_SCORE;
    movie.pal = nes_timing()->region == NES_REGION_PAL;
    movie.fds = rom_is_fds();
    movie.rom_filename = tas_copy_path("Loaded game");
    if (!movie.rom_filename || !nes_tas_rom_md5(movie.rom_md5)) {
        nes_fm2_movie_free(&movie);
        return tas_result(session, NES_MOVIE_NO_IMAGE, NULL);
    }
    static uint64_t guid_sequence;
    struct timespec now = {0};
    (void)timespec_get(&now, TIME_UTC);
    NesMd5 identity;
    nes_md5_init(&identity);
    (void)nes_md5_update(&identity, movie.rom_md5, sizeof(movie.rom_md5));
    (void)nes_md5_update(&identity, &now.tv_sec, sizeof(now.tv_sec));
    (void)nes_md5_update(&identity, &now.tv_nsec, sizeof(now.tv_nsec));
    ++guid_sequence;
    (void)nes_md5_update(&identity, &guid_sequence, sizeof(guid_sequence));
    (void)nes_md5_final(&identity, movie.guid);
    if (nes_timing()->region == NES_REGION_DENDY || joypad_adapter() >= NES_ADAPTER_FAMICOM_TWO) {
        nes_fm2_movie_free(&movie);
        return tas_result(session, NES_MOVIE_INCOMPATIBLE,
                          "FM2 recording requires NTSC or PAL timing with gamepads, Four Score, or Zappers.");
    }
    for (unsigned port = 0; port < 2; ++port) {
        NesPortDevice device = joypad_port_device(port);
        if (device == NES_PORT_NONE) {
            movie.ports[port] = NES_FM2_PORT_NONE;
        } else if (device == NES_PORT_ZAPPER) {
            movie.ports[port] = NES_FM2_PORT_ZAPPER;
        } else if (device != NES_PORT_GAMEPAD) {
            nes_fm2_movie_free(&movie);
            return tas_result(session, NES_MOVIE_INCOMPATIBLE, "FM2 recording supports gamepads and Zappers.");
        }
    }
    if (joypad_expansion_device() != NES_EXPANSION_NONE) {
        nes_fm2_movie_free(&movie);
        return tas_result(session, NES_MOVIE_INCOMPATIBLE, "Disconnect the expansion controller before recording FM2.");
    }
    if (!nes_tas_pin_cheats(&movie)) {
        nes_fm2_movie_free(&movie);
        return tas_result(session, NES_MOVIE_OUT_OF_MEMORY, NULL);
    }
    NesTasProject *project = nes_tas_project_create(&movie);
    nes_fm2_movie_free(&movie);
    if (!project) {
        return tas_result(session, NES_MOVIE_OUT_OF_MEMORY, NULL);
    }
    NesMovieResult result = activate(session, project, path);
    if (result != NES_MOVIE_OK) {
        nes_tas_project_destroy(project);
        return result;
    }
    session->read_only = false;
    session->auto_save = record;
    if (!record) {
        session->saved_revision = 0;
    }
    if (record) {
        result = nes_tas_session_set_recording(session, true, NES_TAS_RECORD_OVERWRITE, 1);
        if (result != NES_MOVIE_OK) {
            session->auto_save = false;
            (void)nes_tas_session_stop(session);
        }
    }
    return result;
}

NesMovieResult nes_tas_session_save(NesTasSession *session, const char *path) {
    if (!session || !session->active || !path || !*path) {
        return tas_result(session, NES_MOVIE_INVALID_ARGUMENT, NULL);
    }
    char *copy = tas_copy_path(path);
    if (!copy) {
        return tas_result(session, NES_MOVIE_OUT_OF_MEMORY, NULL);
    }
    NesMovieResult finished = tas_finish_take(session);
    if (finished != NES_MOVIE_OK) {
        free(copy);
        return finished;
    }
    NesMovieResult result = NES_MOVIE_OK;
    if (tas_extension(path, ".ctas")) {
        result = tas_model_result(session, nes_tas_project_save(session->project, path));
    } else if (tas_extension(path, ".fm2") || tas_extension(path, ".fm3")) {
        bool fm3 = tas_extension(path, ".fm3");
        size_t size = 0, written = 0;
        NesFm2Diagnostic diagnostic = {0};
        const NesFm2Movie *movie = nes_tas_project_movie(session->project);
        bool measured = fm3 ? nes_tas_project_measure_fm3(session->project, &size, &diagnostic) == NES_TAS_OK
                            : nes_fm2_measure(movie, NES_FM2_TEXT, &size, &diagnostic) == NES_FM2_OK;
        uint8_t *data = measured ? malloc(size ? size : 1) : NULL;
        if (!measured) {
            result = tas_result(session, NES_MOVIE_FORMAT_ERROR, diagnostic.message);
        } else if (!data) {
            result = tas_result(session, NES_MOVIE_OUT_OF_MEMORY, NULL);
        } else {
            bool exported =
                fm3 ? nes_tas_project_export_fm3(session->project, data, size, &written, &diagnostic) == NES_TAS_OK
                    : nes_fm2_export(movie, NES_FM2_TEXT, data, size, &written, &diagnostic) == NES_FM2_OK;
            if (!exported || written != size) {
                result = tas_result(session, NES_MOVIE_FORMAT_ERROR, diagnostic.message);
            } else {
                NesFileResult write = nes_file_write_atomic(path, data, size);
                if (write != NES_FILE_OK) {
                    result = tas_result(session, NES_MOVIE_IO_ERROR, nes_file_result_message(write));
                }
            }
        }
        free(data);
    } else {
        result = tas_result(session, NES_MOVIE_INVALID_ARGUMENT, "Choose an .fm2, .fm3, or .ctas filename.");
    }
    if (result == NES_MOVIE_OK && (tas_extension(path, ".ctas") || session->auto_save)) {
        free(session->path);
        session->path = copy;
        session->saved_revision = nes_tas_project_revision(session->project);
    } else {
        free(copy);
    }
    return result;
}

NesMovieResult nes_tas_session_set_path(NesTasSession *session, const char *path) {
    if (!session || !session->active) {
        return tas_result(session, NES_MOVIE_CONFLICT, NULL);
    }
    char *copy = tas_copy_path(path);
    if (!copy) {
        return tas_result(session, NES_MOVIE_INVALID_ARGUMENT, NULL);
    }
    free(session->path);
    session->path = copy;
    return tas_result(session, NES_MOVIE_OK, NULL);
}

NesMovieResult nes_tas_session_stop(NesTasSession *session) {
    if (!session || !session->active) {
        return tas_result(session, NES_MOVIE_CONFLICT, NULL);
    }
    if (!session->auto_save && nes_tas_project_revision(session->project) != session->saved_revision) {
        return tas_result(
            session, NES_MOVIE_CONFLICT,
            "The TAS project has unsaved changes. Save a .ctas project or choose Discard changes to close it.");
    }
    if (session->auto_save) {
        NesMovieResult saved = nes_tas_session_save(session, session->path);
        if (saved != NES_MOVIE_OK) {
            return saved;
        }
    }
    nes_input_event_clear_observer();
    NesStateResult restored = nes_state_apply_prepared(session->resume);
    (void)nes_execution_set_policy(session->previous_policy);
    clear_project(session);
    return tas_result(session, restored == NES_STATE_OK ? NES_MOVIE_OK : NES_MOVIE_STATE_ERROR, NULL);
}

NesMovieResult nes_tas_session_discard(NesTasSession *session) {
    if (!session || !session->active) {
        return tas_result(session, NES_MOVIE_CONFLICT, NULL);
    }
    nes_input_event_clear_observer();
    NesStateResult restored = nes_state_apply_prepared(session->resume);
    (void)nes_execution_set_policy(session->previous_policy);
    clear_project(session);
    return tas_result(session, restored == NES_STATE_OK ? NES_MOVIE_OK : NES_MOVIE_STATE_ERROR, NULL);
}

bool nes_tas_session_active(const NesTasSession *session) {
    return session && session->active;
}

NesMovieMode nes_tas_session_mode(const NesTasSession *session) {
    return !nes_tas_session_active(session) ? NES_MOVIE_IDLE
           : session->auto_save             ? NES_MOVIE_RECORDING
                                            : NES_MOVIE_PLAYBACK;
}

bool nes_tas_session_seeking(const NesTasSession *session) {
    return session && session->active && session->seeking;
}

NesTasProject *nes_tas_session_project(NesTasSession *session) {
    return session && session->active && !session->read_only && !session->recording && !session->frame_in_progress
               ? session->project
               : NULL;
}

const NesTasProject *nes_tas_session_project_const(const NesTasSession *session) {
    return session && session->active ? session->project : NULL;
}

const char *nes_tas_session_error(const NesTasSession *session) {
    return session ? session->error : "Invalid movie session";
}

void nes_tas_session_progress(const NesTasSession *session, NesTasProgress *out) {
    if (!out) {
        return;
    }
    *out = (NesTasProgress){0};
    if (!session) {
        return;
    }
    out->active = session->active;
    out->read_only = session->read_only;
    out->recording = session->recording;
    out->seeking = session->seeking;
    out->auto_save = session->auto_save;
    out->frame_in_progress = session->frame_in_progress;
    out->dirty = session->active && nes_tas_project_revision(session->project) != session->saved_revision;
    out->frame = session->frame;
    out->total_frames = nes_tas_project_frame_count(session->project);
    out->seek_target = session->seek_target;
    out->lag_count = session->lag_count;
    out->checkpoint_count = session->checkpoint_count + (session->initial.data != NULL);
    out->checkpoint_bytes = session->checkpoint_bytes + session->initial.size;
    out->rerecord_count = nes_tas_rerecord_count(session->project);
    out->record_players = session->record_players;
    out->record_mode = session->record_mode;
    out->path = session->path;
    out->error = session->error;
}
