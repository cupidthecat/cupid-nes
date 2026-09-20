/*
 * frontend_session.h - Image opening, reload, and recent-session state
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_SESSION_H
#define FRONTEND_SESSION_H

#include <stdbool.h>
#include <stddef.h>
#include "../rom/rom.h"

enum {
    FRONTEND_IMAGE_PATH_MAX = 1024,
    FRONTEND_IMAGE_MEMBER_MAX = 512,
    FRONTEND_IMAGE_TITLE_MAX = 160,
    FRONTEND_RECENT_MAX = 12
};

typedef struct {
    char path[FRONTEND_IMAGE_PATH_MAX];
    char archive_member[FRONTEND_IMAGE_MEMBER_MAX];
    char patch_path[FRONTEND_IMAGE_PATH_MAX];
    char save_identity[FRONTEND_IMAGE_PATH_MAX];
    char fds_bios_path[FRONTEND_IMAGE_PATH_MAX];
    char studybox_bios_path[FRONTEND_IMAGE_PATH_MAX];
    bool fds_write_protected;
    FdsSaveMode fds_save_mode;
    char fds_overlay_path[FRONTEND_IMAGE_PATH_MAX];
} FrontendImageRequest;

typedef struct {
    char title[FRONTEND_IMAGE_TITLE_MAX];
    char save_identity[FRONTEND_IMAGE_PATH_MAX];
    char archive_member[FRONTEND_IMAGE_MEMBER_MAX];
} FrontendImageResult;

typedef bool (*FrontendImageOpenHandler)(void *userdata,
                                        const FrontendImageRequest *request,
                                        FrontendImageResult *result,
                                        char *error, size_t error_size);

typedef struct {
    FrontendImageOpenHandler open;
    void *userdata;
    bool active;
    FrontendImageRequest current;
    FrontendImageResult current_result;
    FrontendImageRequest recent[FRONTEND_RECENT_MAX];
    size_t recent_count;
} FrontendSession;

bool frontend_image_request_init(FrontendImageRequest *request, const char *path);
bool frontend_image_request_set_member(FrontendImageRequest *request, const char *member);
bool frontend_image_request_set_patch(FrontendImageRequest *request, const char *patch_path);
bool frontend_image_request_set_save_identity(FrontendImageRequest *request,
                                              const char *save_identity);
bool frontend_image_request_set_fds_bios(FrontendImageRequest *request, const char *path);
bool frontend_image_request_set_studybox_bios(FrontendImageRequest *request, const char *path);
bool frontend_image_request_set_fds_overlay(FrontendImageRequest *request, const char *path);
void frontend_session_init(FrontendSession *session,
                           FrontendImageOpenHandler open, void *userdata);
bool frontend_session_open(FrontendSession *session,
                           const FrontendImageRequest *request,
                           char *error, size_t error_size);
bool frontend_session_record_success(FrontendSession *session,
                                     const FrontendImageRequest *request,
                                     const FrontendImageResult *result);
bool frontend_session_reload(FrontendSession *session, char *error, size_t error_size);
bool frontend_session_open_recent(FrontendSession *session, size_t index,
                                  char *error, size_t error_size);
size_t frontend_session_recent_count(const FrontendSession *session);
const FrontendImageRequest *frontend_session_recent(const FrontendSession *session,
                                                    size_t index);
void frontend_session_trim_recent(FrontendSession *session, size_t limit);
bool frontend_session_load_recent(FrontendSession *session, const char *path,
                                  char *error, size_t error_size);
bool frontend_session_save_recent(const FrontendSession *session, const char *path,
                                  char *error, size_t error_size);

#endif
