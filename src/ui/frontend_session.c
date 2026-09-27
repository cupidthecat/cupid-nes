/*
 * frontend_session.c - Image opening, reload, and recent-session state
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "frontend_session.h"
#include "../util/file_io.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { RECENT_FILE_LIMIT = 256 * 1024, RECENT_SAVE_LIMIT = 128 * 1024 };

static void set_error(char *error, size_t error_size, const char *message) {
    if (error && error_size) snprintf(error, error_size, "%s", message ? message : "");
}

static bool copy_text(char *dst, size_t capacity, const char *src) {
    if (!dst || !capacity || !src || strlen(src) >= capacity) return false;
    memcpy(dst, src, strlen(src) + 1);
    return true;
}

bool frontend_image_request_init(FrontendImageRequest *request, const char *path) {
    if (!request || !path || !*path) return false;
    memset(request, 0, sizeof(*request));
    request->fds_save_mode = FDS_SAVE_OVERLAY;
    return copy_text(request->path, sizeof(request->path), path);
}

bool frontend_image_request_set_member(FrontendImageRequest *request, const char *member) {
    if (!request) return false;
    if (!member || !*member) {
        request->archive_member[0] = '\0';
        return true;
    }
    return copy_text(request->archive_member, sizeof(request->archive_member), member);
}

bool frontend_image_request_set_patch(FrontendImageRequest *request, const char *patch_path) {
    if (!request) return false;
    if (!patch_path || !*patch_path) {
        request->patch_path[0] = '\0';
        return true;
    }
    return copy_text(request->patch_path, sizeof(request->patch_path), patch_path);
}

bool frontend_image_request_set_save_identity(FrontendImageRequest *request,
                                              const char *save_identity) {
    if (!request) return false;
    if (!save_identity || !*save_identity) {
        request->save_identity[0] = '\0';
        return true;
    }
    return copy_text(request->save_identity, sizeof(request->save_identity), save_identity);
}

bool frontend_image_request_set_fds_bios(FrontendImageRequest *request, const char *path) {
    if (!request) return false;
    if (!path || !*path) {
        request->fds_bios_path[0] = '\0';
        return true;
    }
    return copy_text(request->fds_bios_path, sizeof(request->fds_bios_path), path);
}

bool frontend_image_request_set_studybox_bios(FrontendImageRequest *request, const char *path) {
    if (!request) return false;
    if (!path || !*path) {
        request->studybox_bios_path[0] = '\0';
        return true;
    }
    return copy_text(request->studybox_bios_path, sizeof(request->studybox_bios_path), path);
}

bool frontend_image_request_set_fds_overlay(FrontendImageRequest *request, const char *path) {
    if (!request) return false;
    if (!path || !*path) {
        request->fds_overlay_path[0] = '\0';
        return true;
    }
    return copy_text(request->fds_overlay_path, sizeof(request->fds_overlay_path), path);
}

void frontend_session_init(FrontendSession *session,
                           FrontendImageOpenHandler open, void *userdata) {
    if (!session) return;
    memset(session, 0, sizeof(*session));
    session->open = open;
    session->userdata = userdata;
}

static bool same_request(const FrontendImageRequest *a, const FrontendImageRequest *b) {
    return strcmp(a->path, b->path) == 0
        && strcmp(a->archive_member, b->archive_member) == 0
        && strcmp(a->patch_path, b->patch_path) == 0
        && strcmp(a->fds_bios_path, b->fds_bios_path) == 0
        && strcmp(a->studybox_bios_path, b->studybox_bios_path) == 0
        && a->fds_write_protected == b->fds_write_protected
        && a->fds_save_mode == b->fds_save_mode
        && strcmp(a->fds_overlay_path, b->fds_overlay_path) == 0;
}

static void remember_request(FrontendSession *session, const FrontendImageRequest *request) {
    size_t found = session->recent_count;
    for (size_t i = 0; i < session->recent_count; ++i) {
        if (same_request(&session->recent[i], request)) {
            found = i;
            break;
        }
    }
    if (found < session->recent_count) {
        FrontendImageRequest saved = session->recent[found];
        memmove(&session->recent[1], &session->recent[0], found * sizeof(session->recent[0]));
        session->recent[0] = saved;
        return;
    }
    size_t move_count = session->recent_count < FRONTEND_RECENT_MAX
        ? session->recent_count : FRONTEND_RECENT_MAX - 1;
    memmove(&session->recent[1], &session->recent[0], move_count * sizeof(session->recent[0]));
    session->recent[0] = *request;
    if (session->recent_count < FRONTEND_RECENT_MAX) ++session->recent_count;
}

bool frontend_session_open(FrontendSession *session,
                           const FrontendImageRequest *request,
                           char *error, size_t error_size) {
    if (!session || !session->open || !request || !request->path[0]) {
        set_error(error, error_size, "No image path was selected");
        return false;
    }
    FrontendImageResult result;
    memset(&result, 0, sizeof(result));
    if (!session->open(session->userdata, request, &result, error, error_size)) return false;

    if (!frontend_session_record_success(session, request, &result)) {
        set_error(error, error_size, "Loaded image metadata is too long");
        return false;
    }
    if (error && error_size) error[0] = '\0';
    return true;
}

bool frontend_session_record_success(FrontendSession *session,
                                     const FrontendImageRequest *request,
                                     const FrontendImageResult *result) {
    if (!session || !request || !request->path[0]) return false;
    FrontendImageRequest current = result && result->request_valid ? result->opened_request : *request;
    FrontendImageResult loaded = {0};
    if (result) loaded = *result;
    if (loaded.save_identity[0]
        && !copy_text(current.save_identity, sizeof(current.save_identity),
                      loaded.save_identity)) return false;
    if (loaded.archive_member[0]
        && !copy_text(current.archive_member, sizeof(current.archive_member),
                      loaded.archive_member)) return false;
    session->current = current;
    session->current_result = loaded;
    session->active = true;
    remember_request(session, &session->current);
    return true;
}

bool frontend_session_reload(FrontendSession *session, char *error, size_t error_size) {
    if (!session || !session->active) {
        set_error(error, error_size, "No image is loaded");
        return false;
    }
    return frontend_session_open(session, &session->current, error, error_size);
}

bool frontend_session_open_recent(FrontendSession *session, size_t index,
                                  char *error, size_t error_size) {
    if (!session || index >= session->recent_count) {
        set_error(error, error_size, "Recent image entry is unavailable");
        return false;
    }
    FrontendImageRequest request = session->recent[index];
    return frontend_session_open(session, &request, error, error_size);
}

size_t frontend_session_recent_count(const FrontendSession *session) {
    return session ? session->recent_count : 0;
}

const FrontendImageRequest *frontend_session_recent(const FrontendSession *session,
                                                    size_t index) {
    return session && index < session->recent_count ? &session->recent[index] : NULL;
}

void frontend_session_trim_recent(FrontendSession *session, size_t limit) {
    if (!session) return;
    if (limit > FRONTEND_RECENT_MAX) limit = FRONTEND_RECENT_MAX;
    if (session->recent_count > limit) session->recent_count = limit;
}

static bool encode_field(const char *source, char *dst, size_t capacity, size_t *used) {
    static const char hex[] = "0123456789ABCDEF";
    for (const unsigned char *p = (const unsigned char *)source; *p; ++p) {
        bool escape = *p == '%' || *p == '|' || *p == '\n' || *p == '\r';
        size_t needed = escape ? 3 : 1;
        if (*used + needed >= capacity) return false;
        if (escape) {
            dst[(*used)++] = '%';
            dst[(*used)++] = hex[*p >> 4];
            dst[(*used)++] = hex[*p & 15];
        } else {
            dst[(*used)++] = (char)*p;
        }
    }
    return true;
}

static int hex_digit(char value) {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    return -1;
}

static bool decode_field(const char *source, size_t length, char *dst, size_t capacity) {
    size_t used = 0;
    for (size_t i = 0; i < length; ++i) {
        unsigned char value = (unsigned char)source[i];
        if (value == '%') {
            if (i + 2 >= length) return false;
            int hi = hex_digit(source[++i]);
            int lo = hex_digit(source[++i]);
            if (hi < 0 || lo < 0) return false;
            value = (unsigned char)((hi << 4) | lo);
            if (!value) return false;
        }
        if (used + 1 >= capacity) return false;
        dst[used++] = (char)value;
    }
    dst[used] = '\0';
    return true;
}

static bool append_recent(char *buffer, size_t capacity, size_t *used,
                          const FrontendImageRequest *request) {
    const char *fields[] = {request->path, request->archive_member, request->patch_path,
                            request->save_identity, request->fds_bios_path,
                            request->studybox_bios_path};
    const char prefix[] = "recent=";
    if (*used + sizeof(prefix) >= capacity) return false;
    memcpy(buffer + *used, prefix, sizeof(prefix) - 1);
    *used += sizeof(prefix) - 1;
    for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i) {
        if (!encode_field(fields[i], buffer, capacity, used)) return false;
        if (*used + 1 >= capacity) return false;
        buffer[(*used)++] = '|';
    }
    const char *mode = request->fds_save_mode == FDS_SAVE_IN_PLACE ? "in-place" : "overlay";
    if (!encode_field(mode, buffer, capacity, used) || *used + 1 >= capacity) return false;
    buffer[(*used)++] = '|';
    if (!encode_field(request->fds_overlay_path, buffer, capacity, used)
        || *used + 1 >= capacity) return false;
    buffer[(*used)++] = '|';
    if (*used + 2 >= capacity) return false;
    buffer[(*used)++] = request->fds_write_protected ? '1' : '0';
    buffer[(*used)++] = '\n';
    return true;
}

bool frontend_session_save_recent(const FrontendSession *session, const char *path,
                                  char *error, size_t error_size) {
    if (!session || !path) return false;
    char *buffer = (char *)malloc(RECENT_SAVE_LIMIT);
    if (!buffer) {
        set_error(error, error_size, "Out of memory while saving recent images");
        return false;
    }
    size_t used = 0;
    const char header[] = "version=2\n";
    memcpy(buffer, header, sizeof(header) - 1);
    used = sizeof(header) - 1;
    bool ok = true;
    for (size_t i = 0; ok && i < session->recent_count; ++i)
        ok = append_recent(buffer, RECENT_SAVE_LIMIT, &used, &session->recent[i]);
    NesFileResult result = ok ? nes_file_write_atomic(path, buffer, used) : NES_FILE_TOO_LARGE;
    free(buffer);
    if (result != NES_FILE_OK) {
        char message[160];
        snprintf(message, sizeof(message), "Cannot save recent images: %s",
                 nes_file_result_message(result));
        set_error(error, error_size, message);
        return false;
    }
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool parse_recent_line(const char *line, unsigned version, FrontendImageRequest *request) {
    static const size_t capacities[] = {
        FRONTEND_IMAGE_PATH_MAX, FRONTEND_IMAGE_MEMBER_MAX, FRONTEND_IMAGE_PATH_MAX,
        FRONTEND_IMAGE_PATH_MAX, FRONTEND_IMAGE_PATH_MAX, FRONTEND_IMAGE_PATH_MAX
    };
    char *destinations[] = {request->path, request->archive_member, request->patch_path,
                            request->save_identity, request->fds_bios_path,
                            request->studybox_bios_path};
    const char *cursor = line;
    for (size_t i = 0; i < 6; ++i) {
        const char *separator = strchr(cursor, '|');
        if (!separator || !decode_field(cursor, (size_t)(separator - cursor),
                                        destinations[i], capacities[i])) return false;
        cursor = separator + 1;
    }
    request->fds_save_mode = FDS_SAVE_OVERLAY;
    if (version >= 2) {
        const char *separator = strchr(cursor, '|');
        if (!separator) return false;
        size_t mode_length = (size_t)(separator - cursor);
        if (mode_length == strlen("overlay") && !memcmp(cursor, "overlay", mode_length))
            request->fds_save_mode = FDS_SAVE_OVERLAY;
        else if (mode_length == strlen("in-place") && !memcmp(cursor, "in-place", mode_length))
            request->fds_save_mode = FDS_SAVE_IN_PLACE;
        else return false;
        cursor = separator + 1;
        separator = strchr(cursor, '|');
        if (!separator || !decode_field(cursor, (size_t)(separator - cursor),
                                        request->fds_overlay_path,
                                        sizeof(request->fds_overlay_path))) return false;
        cursor = separator + 1;
    }
    if ((cursor[0] != '0' && cursor[0] != '1') || cursor[1] != '\0') return false;
    request->fds_write_protected = cursor[0] == '1';
    return request->path[0] != '\0';
}

bool frontend_session_load_recent(FrontendSession *session, const char *path,
                                  char *error, size_t error_size) {
    if (!session || !path) return false;
    uint8_t *data = NULL;
    size_t size = 0;
    NesFileResult result = nes_file_read_all(path, RECENT_FILE_LIMIT, &data, &size);
    if (result == NES_FILE_NOT_FOUND) {
        session->recent_count = 0;
        if (error && error_size) error[0] = '\0';
        return true;
    }
    if (result != NES_FILE_OK) {
        char message[160];
        snprintf(message, sizeof(message), "Cannot read recent images: %s",
                 nes_file_result_message(result));
        set_error(error, error_size, message);
        return false;
    }
    if (memchr(data, '\0', size)) {
        free(data);
        set_error(error, error_size, "Recent image list contains an embedded NUL byte");
        return false;
    }
    char *text = (char *)malloc(size + 1);
    if (!text) {
        free(data);
        set_error(error, error_size, "Out of memory while reading recent images");
        return false;
    }
    memcpy(text, data, size);
    text[size] = '\0';
    free(data);
    FrontendImageRequest parsed[FRONTEND_RECENT_MAX];
    size_t parsed_count = 0;
    char *cursor = text;
    unsigned version = 0;
    unsigned line_number = 0;
    while (cursor && *cursor) {
        char *next = strchr(cursor, '\n');
        if (next) *next++ = '\0';
        size_t length = strlen(cursor);
        if (length && cursor[length - 1] == '\r') cursor[length - 1] = '\0';
        ++line_number;
        if (!version) {
            if (strcmp(cursor, "version=1") == 0) version = 1;
            else if (strcmp(cursor, "version=2") == 0) version = 2;
            else break;
        } else if (strncmp(cursor, "recent=", 7) == 0 && parsed_count < FRONTEND_RECENT_MAX) {
            FrontendImageRequest request;
            memset(&request, 0, sizeof(request));
            if (!parse_recent_line(cursor + 7, version, &request)) break;
            parsed[parsed_count++] = request;
        } else if (*cursor) {
            break;
        }
        cursor = next;
    }
    bool valid = version != 0 && (!cursor || !*cursor);
    free(text);
    if (!valid) {
        char message[160];
        snprintf(message, sizeof(message), "Recent image list is malformed near line %u", line_number);
        set_error(error, error_size, message);
        return false;
    }
    memcpy(session->recent, parsed, parsed_count * sizeof(parsed[0]));
    session->recent_count = parsed_count;
    if (error && error_size) error[0] = '\0';
    return true;
}
