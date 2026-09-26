/*
 * recovery_store.c - Atomic automatic-session storage
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "recovery_store.h"
#include "../util/file_io.h"
#include "../util/sha1.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { RECOVERY_LIMIT = 128 * 1024 * 1024, REQUEST_LIMIT = 8192 };

static bool fail(char *error, size_t size, const char *message) {
    if (error && size) {
        snprintf(error, size, "%s", message);
    }
    return false;
}

static bool key_valid(const char *key) {
    if (!key || strlen(key) != 40) {
        return false;
    }
    return strspn(key, "0123456789abcdef") == 40;
}

bool recovery_game_key(const FrontendSession *session, char key[41]) {
    if (!session || !session->active || !key || !key_valid(session->current_result.sha1)) {
        return false;
    }
    char identity[FRONTEND_IMAGE_MEMBER_MAX + 42];
    int size =
        snprintf(identity, sizeof(identity), "%s:%s", session->current_result.sha1, session->current.archive_member);
    if (size < 0 || (size_t)size >= sizeof(identity)) {
        return false;
    }
    nes_sha1(identity, (size_t)size, key);
    return true;
}

bool recovery_path(const char *directory, const char *key, const char *suffix, char *path, size_t capacity) {
    if (!directory || !*directory || !suffix || !path || !capacity || (key && !key_valid(key))) {
        return false;
    }
    int size = snprintf(path, capacity, "%s/%s%s", directory, key ? key : "", suffix);
    return size >= 0 && (size_t)size < capacity;
}

static void put32(uint8_t *p, uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) {
        p[i] = (uint8_t)(value >> (i * 8));
    }
}

static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

bool recovery_save_session(const char *directory, const FrontendSession *session, const NesStateBlob *state,
                           char *error, size_t error_size) {
    char key[41], path[4096], last[4096];
    if (!state || !state->data || !state->size || state->size > RECOVERY_LIMIT - REQUEST_LIMIT ||
        !recovery_game_key(session, key) || !recovery_path(directory, key, ".session", path, sizeof(path)) ||
        !recovery_path(directory, NULL, "last-session", last, sizeof(last))) {
        return fail(error, error_size, "Invalid session identity or state size");
    }
    const FrontendImageRequest *r = &session->current;
    const char *fields[] = {r->path,          r->archive_member,     r->patch_path,      r->save_identity,
                            r->fds_bios_path, r->studybox_bios_path, r->fds_overlay_path};
    size_t metadata = 8 + 4 + 4 + 2 + 40;
    for (size_t i = 0; i < 7; ++i) {
        metadata += 4 + strlen(fields[i]);
    }
    if (metadata > REQUEST_LIMIT) {
        return fail(error, error_size, "Session metadata is too large");
    }
    size_t size = metadata + state->size;
    uint8_t *bytes = malloc(size);
    if (!bytes) {
        return fail(error, error_size, "Out of memory saving session");
    }
    memcpy(bytes, "CUPSESS1", 8);
    put32(bytes + 8, (uint32_t)metadata);
    put32(bytes + 12, (uint32_t)state->size);
    bytes[16] = r->fds_write_protected ? 1 : 0;
    bytes[17] = (uint8_t)r->fds_save_mode;
    size_t offset = 58;
    for (size_t i = 0; i < 7; ++i) {
        size_t length = strlen(fields[i]);
        put32(bytes + offset, (uint32_t)length);
        memcpy(bytes + offset + 4, fields[i], length);
        offset += length + 4;
    }
    memcpy(bytes + metadata, state->data, state->size);
    /* The digest covers metadata and state, excluding the digest field. */
    memset(bytes + 18, 0, 40);
    char digest[41];
    nes_sha1(bytes, size, digest);
    memcpy(bytes + 18, digest, 40);
    NesFileResult result = nes_file_write_atomic(path, bytes, size);
    free(bytes);
    if (result == NES_FILE_OK) {
        result = nes_file_write_atomic(last, key, 40);
    }
    if (result != NES_FILE_OK) {
        return fail(error, error_size, nes_file_result_message(result));
    }
    if (error && error_size) {
        error[0] = 0;
    }
    return true;
}

bool recovery_read_session(const char *directory, FrontendImageRequest *request, NesStateBlob *state, char *error,
                           size_t error_size) {
    if (!request || !state) {
        return false;
    }
    char path[4096], key[41];
    uint8_t *bytes = NULL;
    size_t size = 0;
    if (!recovery_path(directory, NULL, "last-session", path, sizeof(path))) {
        return false;
    }
    NesFileResult result = nes_file_read_all(path, 40, &bytes, &size);
    if (result != NES_FILE_OK) {
        return fail(error, error_size, "No readable last session is available");
    }
    if (size != 40) {
        free(bytes);
        return fail(error, error_size, "Invalid last-session index");
    }
    memcpy(key, bytes, 40);
    key[40] = 0;
    free(bytes);
    if (!recovery_path(directory, key, ".session", path, sizeof(path))) {
        return fail(error, error_size, "Invalid last-session identity");
    }
    result = nes_file_read_all(path, RECOVERY_LIMIT, &bytes, &size);
    if (result != NES_FILE_OK) {
        return fail(error, error_size, nes_file_result_message(result));
    }
    bool valid = size >= 58 && !memcmp(bytes, "CUPSESS1", 8);
    uint32_t metadata = valid ? get32(bytes + 8) : 0;
    uint32_t payload = valid ? get32(bytes + 12) : 0;
    valid = valid && metadata >= 58 && metadata <= REQUEST_LIMIT && payload && (uint64_t)metadata + payload == size &&
            bytes[16] <= 1 && (bytes[17] == FDS_SAVE_OVERLAY || bytes[17] == FDS_SAVE_IN_PLACE);
    if (valid) {
        char expected[40], digest[41];
        memcpy(expected, bytes + 18, 40);
        memset(bytes + 18, 0, 40);
        nes_sha1(bytes, size, digest);
        valid = !memcmp(expected, digest, 40);
    }
    FrontendImageRequest parsed = {0};
    char *fields[] = {parsed.path,          parsed.archive_member,     parsed.patch_path,      parsed.save_identity,
                      parsed.fds_bios_path, parsed.studybox_bios_path, parsed.fds_overlay_path};
    const size_t capacities[] = {sizeof(parsed.path),
                                 sizeof(parsed.archive_member),
                                 sizeof(parsed.patch_path),
                                 sizeof(parsed.save_identity),
                                 sizeof(parsed.fds_bios_path),
                                 sizeof(parsed.studybox_bios_path),
                                 sizeof(parsed.fds_overlay_path)};
    size_t offset = 58;
    for (size_t i = 0; valid && i < 7; ++i) {
        if (offset + 4 > metadata) {
            valid = false;
            break;
        }
        uint32_t length = get32(bytes + offset);
        offset += 4;
        if (length >= capacities[i] || length > metadata - offset || memchr(bytes + offset, 0, length)) {
            valid = false;
            break;
        }
        memcpy(fields[i], bytes + offset, length);
        offset += length;
    }
    valid = valid && offset == metadata && parsed.path[0];
    if (!valid) {
        free(bytes);
        return fail(error, error_size, "Session is corrupt or has an incompatible format");
    }
    parsed.fds_write_protected = bytes[16] != 0;
    parsed.fds_save_mode = (FdsSaveMode)bytes[17];
    memmove(bytes, bytes + metadata, payload);
    *state = (NesStateBlob){bytes, payload};
    *request = parsed;
    if (error && error_size) {
        error[0] = 0;
    }
    return true;
}
