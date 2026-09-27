/*
 * movie_runner.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Production movie playback acceptance runner. SPDX-License-Identifier: GPL-3.0-or-later */
#include "../replay/movie.h"
#include "../replay/tas_session.h"
#include "../replay/tas_project.h"
#include "../ui/machine_actions.h"
#include "../rom/rom.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../system/execution_policy.h"
#include "../system/vs_system.h"
#include "../state/state.h"
#include "../debugger/debugger.h"
#include "../util/file_io.h"
#include "../util/md5.h"
#include "../../include/globals.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t hash_bytes(uint64_t hash, const void *bytes, size_t size) {
    const uint8_t *data = bytes;
    for (size_t i = 0; i < size; ++i) {
        hash ^= data[i];
        hash *= UINT64_C(1099511628211);
    }
    return hash;
}

static bool save_frame(const char *path) {
    if (!path) {
        return true;
    }
    const char header[] = "P6\n256 240\n255\n";
    size_t size = sizeof(header) - 1 + 256u * 240u * 3u;
    uint8_t *bytes = malloc(size);
    if (!bytes) {
        return false;
    }
    memcpy(bytes, header, sizeof(header) - 1);
    for (size_t i = 0; i < 256u * 240u; ++i) {
        bytes[sizeof(header) - 1 + i * 3] = (uint8_t)(framebuffer[i] >> 16);
        bytes[sizeof(header) + i * 3] = (uint8_t)(framebuffer[i] >> 8);
        bytes[sizeof(header) + 1 + i * 3] = (uint8_t)framebuffer[i];
    }
    bool saved = nes_file_write_atomic(path, bytes, size) == NES_FILE_OK;
    free(bytes);
    return saved;
}

static char *trace_path(const char *prefix, const char *suffix) {
    size_t prefix_size = strlen(prefix), suffix_size = strlen(suffix);
    if (!prefix_size || prefix_size > NES_FILE_PATH_LIMIT - suffix_size - 1) {
        return NULL;
    }
    char *path = malloc(prefix_size + suffix_size + 1);
    if (path) {
        memcpy(path, prefix, prefix_size);
        memcpy(path + prefix_size, suffix, suffix_size + 1);
    }
    return path;
}

static bool trace_begin(const char *prefix, const char *suffix, uint64_t limit, NesFileTransaction **transaction) {
    char *path = trace_path(prefix, suffix);
    bool ok = path && nes_file_transaction_begin(path, limit, transaction) == NES_FILE_OK;
    free(path);
    return ok;
}

static bool trace_screen(const char *prefix) {
    char *path = trace_path(prefix, ".ppm");
    bool ok = path && save_frame(path);
    free(path);
    path = trace_path(prefix, ".pixels");
    uint8_t indices[256 * 240];
    for (size_t i = 0; i < sizeof(indices); ++i) {
        indices[i] = (uint8_t)(ppu.pixel_indices[i] & 0x3F);
    }
    ok = ok && path && nes_file_write_atomic(path, indices, sizeof(indices)) == NES_FILE_OK;
    free(path);
    return ok;
}

static bool output_distinct(const char *path, const char *rom, const char *movie) {
    if (!path) {
        return true;
    }
    bool same = false;
    NesFileResult result = nes_file_same(path, rom, &same);
    if (same || (result != NES_FILE_OK && result != NES_FILE_NOT_FOUND)) {
        return false;
    }
    result = nes_file_same(path, movie, &same);
    return !same && (result == NES_FILE_OK || result == NES_FILE_NOT_FOUND);
}

static bool trace_outputs_distinct(const char *prefix, const char *rom, const char *movie) {
    if (!prefix) {
        return true;
    }
    static const char *const suffixes[] = {".ram", ".csv", ".ppm", ".pixels"};
    for (size_t i = 0; i < sizeof(suffixes) / sizeof(suffixes[0]); ++i) {
        char *path = trace_path(prefix, suffixes[i]);
        bool allowed = path && output_distinct(path, rom, movie);
        free(path);
        if (!allowed) {
            return false;
        }
    }
    return true;
}

static int run_movie(const char *rom_path, const char *movie_path, unsigned limit, const char *output,
                     const char *trace_prefix) {
    int result = 1;
    NesMovieSession *movie = NULL;
    NesStateBlob before = {0}, after = {0};
    NesFileTransaction *ram_trace = NULL, *frame_trace = NULL;
    if (!output_distinct(output, rom_path, movie_path) || !trace_outputs_distinct(trace_prefix, rom_path, movie_path)) {
        fprintf(stderr, "Movie outputs must be separate from the input files.\n");
        return 1;
    }
    if (load_rom(rom_path) != 0) {
        return 1;
    }
    debugger_init();
    cpu_use_default_startup_alignment();
    if (!frontend_machine_power_cycle()) {
        goto cleanup;
    }
    ppu_begin_frame_render(framebuffer);
    movie = nes_movie_create();
    if (!movie || nes_state_capture(&before) != NES_STATE_OK) {
        goto cleanup;
    }
    if (nes_movie_play_start(movie, movie_path) != NES_MOVIE_OK) {
        fprintf(stderr, "Movie open failed: %s\n", nes_movie_error(movie));
        goto cleanup;
    }
    NesMovieProgress progress;
    nes_movie_progress(movie, &progress);
    uint64_t target = progress.total_frames;
    if (limit && target > limit) {
        target = limit;
    }
    if (trace_prefix) {
        static const char header[] = "frame,lag_count,lagged,pc\n";
        if (!nes_tas_session_active(nes_movie_tas(movie)) || target > (UINT64_MAX - 128) / 2048 ||
            !trace_begin(trace_prefix, ".ram", target * 2048, &ram_trace) ||
            !trace_begin(trace_prefix, ".csv", (target + 1) * 128, &frame_trace) ||
            nes_file_transaction_write(frame_trace, header, sizeof(header) - 1) != NES_FILE_OK) {
            goto cleanup;
        }
    }
    uint64_t video_hash = UINT64_C(14695981039346656037);
    for (uint64_t frame = 0; frame < target; ++frame) {
        if (nes_movie_frame_boundary(movie) != NES_MOVIE_OK) {
            goto cleanup;
        }
        vs_start_frame();
        unsigned instructions = 0;
        while (!ppu.frame_complete) {
            if (vs_cpu_step() <= 0 || cpu.halted || ++instructions > 200000) {
                fprintf(stderr, "Movie failed at frame %" PRIu64 ", PC=$%04X\n", frame, cpu.pc);
                goto cleanup;
            }
        }
        if (nes_movie_frame_complete(movie, true) != NES_MOVIE_OK) {
            goto cleanup;
        }
        video_hash = hash_bytes(video_hash, ppu.pixel_indices, sizeof(ppu.pixel_indices));
        if (trace_prefix) {
            NesTasProgress state;
            NesTasSession *session = nes_movie_tas(movie);
            nes_tas_session_progress(session, &state);
            uint8_t ram[0x800];
            for (unsigned i = 0; i < sizeof(ram); ++i) {
                ram[i] = cpu_peek_internal_ram((uint16_t)i);
            }
            char row[128];
            int length = snprintf(row, sizeof(row), "%" PRIu64 ",%zu,%u,%u\n", frame + 1, state.lag_count,
                                  nes_tas_lag(nes_tas_session_project_const(session), (size_t)frame) == NES_TAS_LAG_YES,
                                  (unsigned)cpu.pc);
            if (length < 0 || (size_t)length >= sizeof(row) ||
                nes_file_transaction_write(ram_trace, ram, sizeof(ram)) != NES_FILE_OK ||
                nes_file_transaction_write(frame_trace, row, (size_t)length) != NES_FILE_OK) {
                goto cleanup;
            }
        }
    }
    if (!save_frame(output)) {
        goto cleanup;
    }
    if (trace_prefix && !trace_screen(trace_prefix)) {
        goto cleanup;
    }
    NesTasProgress tas;
    nes_tas_session_progress(nes_movie_tas(movie), &tas);
    uint8_t ram_bytes[0x800];
    for (unsigned i = 0; i < sizeof(ram_bytes); ++i) {
        ram_bytes[i] = cpu_peek_internal_ram((uint16_t)i);
    }
    uint64_t ram_hash = hash_bytes(UINT64_C(14695981039346656037), ram_bytes, sizeof(ram_bytes));
    printf("Movie: %" PRIu64 "/%" PRIu64 " frames, lag %zu, PC=$%04X, video=%016" PRIx64 ", RAM=%016" PRIx64 "\n",
           target, progress.total_frames, tas.lag_count, cpu.pc, video_hash, ram_hash);
    if (target == progress.total_frames && nes_movie_frame_boundary(movie) != NES_MOVIE_COMPLETE) {
        goto cleanup;
    }
    if (nes_movie_stop(movie) != NES_MOVIE_OK || nes_state_capture(&after) != NES_STATE_OK ||
        before.size != after.size || memcmp(before.data, after.data, before.size)) {
        fprintf(stderr, "Movie did not restore the previous machine exactly.\n");
        goto cleanup;
    }
    printf("Movie playback and live-state restoration: PASS\n");
    if (trace_prefix && (nes_file_transaction_commit(&ram_trace) != NES_FILE_OK ||
                         nes_file_transaction_commit(&frame_trace) != NES_FILE_OK)) {
        goto cleanup;
    }
    result = 0;
cleanup:
    nes_file_transaction_abort(&ram_trace);
    nes_file_transaction_abort(&frame_trace);
    nes_movie_destroy(movie);
    nes_state_blob_free(&before);
    nes_state_blob_free(&after);
    if (!unload_rom()) {
        result = 1;
    }
    return result;
}

int run_movie_rom(const char *rom_path, const char *movie_path, unsigned limit, const char *output) {
    return run_movie(rom_path, movie_path, limit, output, NULL);
}

int run_movie_trace(const char *rom_path, const char *movie_path, unsigned limit, const char *prefix) {
    return prefix && *prefix ? run_movie(rom_path, movie_path, limit, NULL, prefix) : 2;
}
