/* Save states bound to movie input and startup. SPDX-License-Identifier: GPL-3.0-or-later */
#include "tas_session_internal.h"
#include "../state/state_io.h"
#include "../cheats/cheats.h"
#include "../rom/fds.h"
#include "../util/file_io.h"
#include "../util/md5.h"
#include "../video/video_trace.h"
#include <stdlib.h>
#include <string.h>

static const uint8_t magic[8] = {'C', 'U', 'P', 'T', 'S', 'T', 'A', 'T'};

enum { TAS_STATE_VERSION = 1, TAS_STATE_OVERHEAD = 128 };

static void digest(const void *bytes, size_t size, uint8_t out[16]) {
    NesMd5 hash;
    nes_md5_init(&hash);
    (void)nes_md5_update(&hash, bytes, size);
    (void)nes_md5_final(&hash, out);
}

static void input_digest(const NesTasProject *project, size_t frames, uint8_t out[16]) {
    NesMd5 hash;
    nes_md5_init(&hash);
    for (size_t i = 0; i < frames; ++i) {
        const NesFm2Frame *frame = nes_tas_project_frame(project, i);
        (void)nes_md5_update(&hash, &frame->commands, 1);
        (void)nes_md5_update(&hash, frame->pads, sizeof(frame->pads));
        for (unsigned port = 0; port < 2; ++port) {
            const NesFm2Zapper *zapper = &frame->zappers[port];
            uint8_t bytes[12] = {zapper->x, zapper->y, zapper->button, zapper->bogo};
            for (unsigned j = 0; j < 8; ++j) {
                bytes[j + 4] = (uint8_t)(zapper->zaphit >> (j * 8));
            }
            (void)nes_md5_update(&hash, bytes, sizeof(bytes));
        }
    }
    (void)nes_md5_final(&hash, out);
}

NesStateResult nes_tas_session_capture_state(NesTasSession *session, NesStateBlob *out) {
    if (!session || !out || !session->active) {
        return NES_STATE_ERROR_ARGUMENT;
    }
    if (session->frame_in_progress || session->seeking) {
        return NES_STATE_ERROR_UNSUPPORTED;
    }
    NesTasChange changes = nes_tas_project_change_since(session->project, session->seen_revision);
    if (changes.first_changed_frame < session->frame) {
        return NES_STATE_ERROR_UNSUPPORTED;
    }
    if (changes.first_changed_frame != SIZE_MAX) {
        tas_invalidate_after(session, changes.first_changed_frame);
    }
    session->seen_revision = changes.revision;
    NesStateBlob machine = {0};
    NesStateResult result = nes_state_capture(&machine);
    if (result != NES_STATE_OK) {
        return result;
    }
    uint8_t initial[16], input[16], checksum[16];
    digest(session->initial.data, session->initial.size, initial);
    input_digest(session->project, session->frame, input);
    const NesFm2Movie *movie = nes_tas_project_movie(session->project);
    NesStateWriter writer;
    nes_state_writer_init(&writer, (size_t)NES_STATE_MAX_SIZE + TAS_STATE_OVERHEAD);
    bool written =
        nes_state_write_bytes(&writer, magic, sizeof(magic)) && nes_state_write_u32(&writer, TAS_STATE_VERSION) &&
        nes_state_write_bytes(&writer, movie->guid, sizeof(movie->guid)) &&
        nes_state_write_bytes(&writer, initial, sizeof(initial)) &&
        nes_state_write_bytes(&writer, input, sizeof(input)) &&
        nes_state_write_u32(&writer, cheats_compatibility_hash()) && nes_state_write_u64(&writer, session->frame) &&
        nes_state_write_u64(&writer, session->lag_count) && nes_state_write_u64(&writer, session->selected_side) &&
        nes_state_write_u32(&writer, (uint32_t)machine.size) &&
        nes_state_write_bytes(&writer, machine.data, machine.size);
    nes_state_blob_free(&machine);
    if (written) {
        digest(writer.data, writer.size, checksum);
        written = nes_state_write_bytes(&writer, checksum, sizeof(checksum));
    }
    if (!written) {
        nes_state_writer_destroy(&writer);
        return NES_STATE_ERROR_OUT_OF_MEMORY;
    }
    size_t size = 0;
    uint8_t *data = nes_state_writer_release(&writer, &size);
    *out = (NesStateBlob){data, size};
    return NES_STATE_OK;
}

NesStateResult nes_tas_session_load_state(NesTasSession *session, const char *path) {
    if (!session || !session->active || !path || !*path) {
        return NES_STATE_ERROR_ARGUMENT;
    }
    uint8_t *data = NULL;
    size_t size = 0;
    NesFileResult file = nes_file_read_all(path, (size_t)NES_STATE_MAX_SIZE + TAS_STATE_OVERHEAD, &data, &size);
    if (file != NES_FILE_OK) {
        return file == NES_FILE_OUT_OF_MEMORY ? NES_STATE_ERROR_OUT_OF_MEMORY : NES_STATE_ERROR_IO;
    }
    NesStateResult result = NES_STATE_ERROR_FORMAT;
    uint8_t actual[16], saved_initial[16], saved_input[16], guid[16], header[8];
    if (size < sizeof(magic) + 16 || memcmp(data, magic, sizeof(magic))) {
        goto done;
    }
    digest(data, size - 16, actual);
    if (memcmp(actual, data + size - 16, 16)) {
        result = NES_STATE_ERROR_CORRUPT;
        goto done;
    }
    NesStateReader reader;
    nes_state_reader_init(&reader, data, size - 16);
    uint32_t version, cheat_hash, state_size;
    uint64_t frame, lag_count, selected_side;
    if (!nes_state_read_bytes(&reader, header, sizeof(header)) || memcmp(header, magic, sizeof(magic)) ||
        !nes_state_read_u32(&reader, &version)) {
        goto done;
    }
    if (version != TAS_STATE_VERSION) {
        result = NES_STATE_ERROR_VERSION;
        goto done;
    }
    if (!nes_state_read_bytes(&reader, guid, sizeof(guid)) ||
        !nes_state_read_bytes(&reader, saved_initial, sizeof(saved_initial)) ||
        !nes_state_read_bytes(&reader, saved_input, sizeof(saved_input)) || !nes_state_read_u32(&reader, &cheat_hash) ||
        !nes_state_read_u64(&reader, &frame) || !nes_state_read_u64(&reader, &lag_count) ||
        !nes_state_read_u64(&reader, &selected_side) || !nes_state_read_u32(&reader, &state_size) ||
        frame > nes_tas_project_frame_count(session->project) || lag_count > frame || selected_side > SIZE_MAX ||
        (fds_active() && selected_side >= fds_side_count()) || state_size != nes_state_reader_remaining(&reader)) {
        goto done;
    }
    const NesFm2Movie *movie = nes_tas_project_movie(session->project);
    result = NES_STATE_ERROR_INCOMPATIBLE;
    if (memcmp(guid, movie->guid, sizeof(guid)) || cheat_hash != cheats_compatibility_hash()) {
        goto done;
    }
    digest(session->initial.data, session->initial.size, actual);
    if (memcmp(actual, saved_initial, sizeof(actual))) {
        goto done;
    }
    input_digest(session->project, (size_t)frame, actual);
    if (memcmp(actual, saved_input, sizeof(actual))) {
        goto done;
    }
    NesStateRestore *restore = NULL;
    result = nes_state_prepare_restore(reader.data + reader.offset, state_size, &restore);
    if (result != NES_STATE_OK) {
        goto done;
    }
    if (tas_finish_take(session) != NES_MOVIE_OK) {
        nes_state_restore_free(restore);
        result = NES_STATE_ERROR_OUT_OF_MEMORY;
        goto done;
    }
    if (!session->read_only && nes_tas_increment_rerecord(session->project) != NES_TAS_OK) {
        nes_state_restore_free(restore);
        result = NES_STATE_ERROR_OUT_OF_MEMORY;
        goto done;
    }
    result = nes_state_apply_prepared(restore);
    nes_state_restore_free(restore);
    if (result != NES_STATE_OK) {
        goto done;
    }
    NesTasChange changes = nes_tas_project_change_since(session->project, session->seen_revision);
    if (changes.first_changed_frame != SIZE_MAX) {
        tas_invalidate_after(session, changes.first_changed_frame);
    }
    session->frame = (size_t)frame;
    session->lag_count = (size_t)lag_count;
    session->selected_side = (size_t)selected_side;
    session->frame_in_progress = session->completed_pending = session->pending_record = session->seeking = false;
    session->pending_commands = 0;
    session->seen_revision = nes_tas_project_revision(session->project);
    nes_video_trace_reset_side(0);
    nes_video_trace_reset_side(1);
done:
    free(data);
    return result;
}
