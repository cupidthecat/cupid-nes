/*
 * hd_pack_frontend.h - HD pack panel registration
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_NES_HD_PACK_FRONTEND_H
#define CUPID_NES_HD_PACK_FRONTEND_H

#include "../hd/hd_runtime.h"

#include <stdbool.h>
#include <stddef.h>

typedef struct NesHdFrontend NesHdFrontend;
struct FrontendExecutionRuntime;
void nes_hd_frontend_bind_execution(NesHdFrontend *frontend, struct FrontendExecutionRuntime *execution);

enum {
    HD_PACK_PANEL = 0x1800,
    HD_CONTROL_STATUS = 0x1801,
    HD_CONTROL_ACTIVE = 0x1802,
    HD_CONTROL_ENABLED = 0x1803,
    HD_CONTROL_PACK = 0x1804,
    HD_CONTROL_RESCAN = 0x1805,
    HD_CONTROL_INSTALL_SOURCE = 0x1806,
    HD_CONTROL_INSTALL_NAME = 0x1807,
    HD_CONTROL_INSTALL = 0x1808,
    HD_CONTROL_EXPORT_PATH = 0x1809,
    HD_CONTROL_EXPORT = 0x180A,
    HD_CONTROL_CAPTURE_ARMED = 0x180B,
    HD_CONTROL_CAPTURE_PATH = 0x180C,
    HD_CONTROL_CAPTURE = 0x180D
};

NesHdFrontend *nes_hd_frontend_create(NesHdRuntime *runtime,
                                      char *error, size_t error_size);
/* Main supplies the presentation source paired with the completed frame. The
 * frontend copies the structure; trace storage remains owned by video_trace. */
void nes_hd_frontend_set_capture_source(NesHdFrontend *frontend,
                                        const NesHdFrameSource *source);
bool nes_hd_frontend_restore_preferences(NesHdFrontend *frontend, const char *sha1, char *error, size_t error_size);
void nes_hd_frontend_destroy(NesHdFrontend *frontend);

#endif
