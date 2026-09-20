/*
 * hd_pack_frontend.c - HD pack panel registration
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "hd_pack_frontend.h"
#include "frontend_panels.h"
#include "frontend_execution.h"
#include "output_guard.h"
#include "app_paths.h"
#include "../util/file_io.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct NesHdFrontend {
    NesHdRuntime *runtime;
    FrontendExecutionRuntime *execution;
    char names[NES_HD_MAX_CANDIDATES][NES_HD_PACK_NAME_MAX];
    const char *items[NES_HD_MAX_CANDIDATES];
    char status[NES_HD_STATUS_MAX];
    char active[NES_HD_PACK_NAME_MAX];
    char install_source[32768];
    char install_name[NES_HD_PACK_NAME_MAX];
    char export_path[32768];
    char capture_path[32768];
    char preference_path[4096];
    NesHdFrameSource capture_source;
    bool capture_source_valid;
    bool capture_armed;
};

static void set_error(char *error, size_t error_size, const char *message) {
    if (error && error_size) snprintf(error, error_size, "%s", message ? message : "");
}

static void format_path(char *output, size_t output_size,
                        const char *prefix, const char *path) {
    if (!output || !output_size) return;
    if (!prefix) prefix = "";
    if (!path) path = "";
    size_t prefix_size = strlen(prefix);
    size_t path_size = strlen(path);
    if (prefix_size + path_size < output_size) {
        snprintf(output, output_size, "%s%s", prefix, path);
        return;
    }
    if (prefix_size + 4 >= output_size) {
        snprintf(output, output_size, "%s", prefix);
        return;
    }
    size_t tail_size = output_size - prefix_size - 4;
    const char *tail = path + path_size - tail_size;
    while (*tail && (((unsigned char)*tail & 0xC0u) == 0x80u)) ++tail;
    snprintf(output, output_size, "%s...%s", prefix, tail);
}

static bool complete_capture_source(const NesHdFrameSource *source) {
    if (!source || source->screens < 1 || source->screens > 2) return false;
    for (unsigned side = 0; side < source->screens; ++side)
        if (!source->trace[side] || !source->trace[side]->complete || !source->trace[side]->pixels)
            return false;
    return source->screens != 2
        || source->trace[0]->frame_number == source->trace[1]->frame_number;
}

static bool snapshot(void *userdata, FrontendPanelModel *model,
                     char *error, size_t error_size) {
    NesHdFrontend *frontend = userdata;
    NesHdRuntimeInfo info;
    if (!frontend || !model || !nes_hd_runtime_info(frontend->runtime, &info)) {
        set_error(error, error_size, "HD runtime is unavailable");
        return false;
    }
    snprintf(frontend->status, sizeof(frontend->status), "%s", info.status ? info.status : "");
    format_path(frontend->active, sizeof(frontend->active), "",
                info.active_path ? info.active_path : "None");

    size_t count = nes_hd_runtime_candidate_count(frontend->runtime);
    if (count > NES_HD_MAX_CANDIDATES) count = NES_HD_MAX_CANDIDATES;
    int selected = -1;
    for (size_t i = 0; i < count; ++i) {
        NesHdPackCandidateInfo candidate;
        if (!nes_hd_runtime_candidate_at(frontend->runtime, i, &candidate)) continue;
        snprintf(frontend->names[i], sizeof(frontend->names[i]), "%s%s",
                 candidate.name ? candidate.name : "HD pack",
                 candidate.compatible ? "" : " (different game)");
        frontend->items[i] = frontend->names[i];
        if (candidate.active) selected = (int)i;
    }

    FrontendPanelControl controls[] = {
        {HD_CONTROL_STATUS, FRONTEND_PANEL_TEXT, "Status", frontend->status,
         NULL, 0, -1, true, true},
        {HD_CONTROL_ACTIVE, FRONTEND_PANEL_TEXT, "Active pack", frontend->active,
         NULL, 0, -1, true, true},
        {HD_CONTROL_ENABLED, FRONTEND_PANEL_CHECKBOX, "Enable HD pack", NULL,
         NULL, 0, info.enabled ? 1 : 0, info.pack_loaded, false},
        {HD_CONTROL_PACK, FRONTEND_PANEL_CHOICE, "Pack", NULL,
         frontend->items, count, selected, count != 0, false},
        {HD_CONTROL_RESCAN, FRONTEND_PANEL_ACTION, "Rescan packs", NULL,
         NULL, 0, -1, info.game_attached, false},
        {HD_CONTROL_INSTALL_SOURCE, FRONTEND_PANEL_TEXT, "Install pack path", frontend->install_source,
         NULL, 0, -1, info.game_attached, false},
        {HD_CONTROL_INSTALL_NAME, FRONTEND_PANEL_TEXT, "Installed ZIP name", frontend->install_name,
         NULL, 0, -1, info.game_attached, false},
        {HD_CONTROL_INSTALL, FRONTEND_PANEL_ACTION, "Validate and install pack", NULL,
         NULL, 0, -1, info.game_attached && frontend->install_source[0] != '\0', false},
        {HD_CONTROL_EXPORT_PATH, FRONTEND_PANEL_TEXT, "Export current pack to ZIP", frontend->export_path,
         NULL, 0, -1, info.pack_loaded, false},
        {HD_CONTROL_EXPORT, FRONTEND_PANEL_ACTION, "Export current pack", NULL,
         NULL, 0, -1, info.pack_loaded && frontend->export_path[0] != '\0', false},
        {HD_CONTROL_CAPTURE_ARMED, FRONTEND_PANEL_CHECKBOX, "Arm HD pack capture", NULL,
         NULL, 0, frontend->capture_armed ? 1 : 0, info.game_attached, false},
        {HD_CONTROL_CAPTURE_PATH, FRONTEND_PANEL_TEXT, "Captured pack ZIP path", frontend->capture_path,
         NULL, 0, -1, info.game_attached, false},
        {HD_CONTROL_CAPTURE, FRONTEND_PANEL_ACTION, "Capture completed frame as pack", NULL,
         NULL, 0, -1, info.game_attached && frontend->capture_armed
             && frontend->capture_source_valid && frontend->capture_path[0] != '\0', false}
    };
    for (size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); ++i)
        if (!frontend_panel_add_control(model, &controls[i])) {
            set_error(error, error_size, "HD pack panel has insufficient control capacity");
            return false;
        }
    model->status = frontend->status;
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool remember(NesHdFrontend *frontend, bool changed, char *error, size_t size) {
    if(!changed || !frontend->preference_path[0])return changed;
    NesHdRuntimeInfo info;
    if(!nes_hd_runtime_info(frontend->runtime,&info))return false;
    const char *path=info.active_path?info.active_path:"";
    size_t length=strlen(path);
    if(length>32767)return false;
    uint8_t *data=malloc(length+9);
    if(!data)return false;
    memcpy(data,"CHDP1",5);data[5]=info.enabled?1:0;data[6]=(uint8_t)length;data[7]=(uint8_t)(length>>8);
    memcpy(data+8,path,length+1);
    NesFileResult result=nes_file_write_atomic(frontend->preference_path,data,length+9);
    free(data);
    if(result!=NES_FILE_OK){set_error(error,size,"HD pack is active, but its preference could not be saved");return false;}
    return true;
}

bool nes_hd_frontend_restore_preferences(NesHdFrontend *frontend,const char *sha1,char *error,size_t size) {
    if(!frontend)return false;
    frontend->preference_path[0]='\0';
    if(!sha1 || strlen(sha1)!=40)return true;
    for(unsigned i=0;i<40;++i)if(!strchr("0123456789abcdefABCDEF",sha1[i]))return false;
    char name[80];snprintf(name,sizeof(name),"hd-%s.pref",sha1);
    if(!frontend_paths_join(frontend->preference_path,sizeof(frontend->preference_path),name))return false;
    uint8_t *data=NULL;size_t count=0;
    NesFileResult result=nes_file_read_all(frontend->preference_path,32776,&data,&count);
    if(result==NES_FILE_NOT_FOUND)return true;
    bool valid=result==NES_FILE_OK && count>=9 && !memcmp(data,"CHDP1",5) && data[5]<=1;
    size_t length=valid?(size_t)data[6]+((size_t)data[7]<<8):0;
    valid=valid && count==length+9 && data[count-1]==0 && !memchr(data+8,0,length);
    if(!valid){free(data);set_error(error,size,"The saved HD pack selection is invalid");return false;}
    bool loaded=!length || nes_hd_runtime_load(frontend->runtime,(char *)data+8,error,size);
    bool enabled=loaded && (!length || nes_hd_runtime_enable(frontend->runtime,data[5]!=0,error,size));
    free(data);return enabled;
}

static bool action(void *userdata, unsigned control_id, const char *value, int selected,
                   char *error, size_t error_size) {
    NesHdFrontend *frontend = userdata;
    if (!frontend) {
        set_error(error, error_size, "HD runtime is unavailable");
        return false;
    }
    switch (control_id) {
        case HD_CONTROL_INSTALL_SOURCE:
            if (!value || strlen(value) >= sizeof(frontend->install_source)) {
                set_error(error, error_size, "HD pack install path is too long");
                return false;
            }
            memcpy(frontend->install_source, value, strlen(value) + 1);
            return true;
        case HD_CONTROL_INSTALL_NAME:
            if (!value || strlen(value) >= sizeof(frontend->install_name)) {
                set_error(error, error_size, "HD pack install name is too long");
                return false;
            }
            memcpy(frontend->install_name, value, strlen(value) + 1);
            return true;
        case HD_CONTROL_EXPORT_PATH:
            if (!value || strlen(value) >= sizeof(frontend->export_path)) {
                set_error(error, error_size, "HD pack export path is too long");
                return false;
            }
            memcpy(frontend->export_path, value, strlen(value) + 1);
            return true;
        case HD_CONTROL_CAPTURE_PATH:
            if (!value || strlen(value) >= sizeof(frontend->capture_path)) {
                set_error(error, error_size, "HD pack capture path is too long");
                return false;
            }
            memcpy(frontend->capture_path, value, strlen(value) + 1);
            return true;
        case HD_CONTROL_CAPTURE_ARMED: {
            bool armed = selected != 0;
            if (armed == frontend->capture_armed) return true;
            if (!nes_video_trace_use(NES_VIDEO_TRACE_EXPORT, armed)) {
                set_error(error, error_size, armed
                    ? "Video trace buffers could not be allocated for HD pack capture"
                    : "HD pack capture trace could not be released");
                return false;
            }
            frontend->capture_armed = armed;
            if (!armed) frontend->capture_source_valid = false;
            return true;
        }
        case HD_CONTROL_ENABLED:
            return remember(frontend,nes_hd_runtime_enable(frontend->runtime, selected != 0, error, error_size),error,error_size);
        case HD_CONTROL_PACK:
            if (selected < 0) {
                set_error(error, error_size, "No HD pack was selected");
                return false;
            }
            return remember(frontend,nes_hd_runtime_switch(frontend->runtime, (size_t)selected, error, error_size),error,error_size);
        case HD_CONTROL_RESCAN:
            return nes_hd_runtime_discover(frontend->runtime, error, error_size);
        case HD_CONTROL_INSTALL: {
            char installed[32768];
            bool ok = nes_hd_runtime_install(frontend->runtime, frontend->install_source,
                                             frontend->install_name[0] ? frontend->install_name : NULL,
                                             installed, sizeof(installed), error, error_size);
            if (ok) {
                format_path(frontend->status, sizeof(frontend->status), "Installed ", installed);
                (void)nes_hd_runtime_discover(frontend->runtime, NULL, 0);
            }
            return remember(frontend,ok,error,error_size);
        }
        case HD_CONTROL_EXPORT:
            if (!frontend_output_path_allowed(frontend->export_path, frontend->execution,
                    frontend->execution ? frontend->execution->protected_paths : NULL,
                    frontend->execution ? frontend->execution->protected_path_count : 0,
                    error, error_size)) return false;
            return nes_hd_runtime_export(frontend->runtime, frontend->export_path,
                                         error, error_size);
        case HD_CONTROL_CAPTURE: {
            if (!frontend_output_path_allowed(frontend->capture_path, frontend->execution,
                    frontend->execution ? frontend->execution->protected_paths : NULL,
                    frontend->execution ? frontend->execution->protected_path_count : 0,
                    error, error_size)) return false;
            if (!frontend->capture_armed || !frontend->capture_source_valid) {
                set_error(error, error_size, "Arm capture and complete a video frame before creating an HD pack");
                return false;
            }
            bool ok = nes_hd_runtime_capture(frontend->runtime, &frontend->capture_source,
                                             frontend->capture_path, error, error_size);
            if (ok) format_path(frontend->status, sizeof(frontend->status),
                                "Captured ", frontend->capture_path);
            return ok;
        }
        default:
            set_error(error, error_size, "Unknown HD pack panel control");
            return false;
    }
}

NesHdFrontend *nes_hd_frontend_create(NesHdRuntime *runtime,
                                      char *error, size_t error_size) {
    if (error && error_size) error[0] = '\0';
    if (!runtime) {
        set_error(error, error_size, "HD runtime is unavailable");
        return NULL;
    }
    NesHdFrontend *frontend = calloc(1, sizeof(*frontend));
    if (!frontend) {
        set_error(error, error_size, "Out of memory while creating the HD pack panel");
        return NULL;
    }
    frontend->runtime = runtime;
    FrontendPanelSpec panel = {
        .id = HD_PACK_PANEL, .title = "HD Packs", .category = "Video",
        .flags = FRONTEND_PANEL_NEEDS_SESSION,
        .snapshot = snapshot, .action = action, .userdata = frontend
    };
    if (!frontend_panel_register(&panel)) {
        free(frontend);
        set_error(error, error_size, "HD pack panel could not be registered");
        return NULL;
    }
    return frontend;
}

void nes_hd_frontend_set_capture_source(NesHdFrontend *frontend,
                                        const NesHdFrameSource *source) {
    if (!frontend) return;
    frontend->capture_source_valid = complete_capture_source(source);
    if (frontend->capture_source_valid) frontend->capture_source = *source;
}

void nes_hd_frontend_destroy(NesHdFrontend *frontend) {
    if (!frontend) return;
    if (frontend->capture_armed) (void)nes_video_trace_use(NES_VIDEO_TRACE_EXPORT, false);
    (void)frontend_panel_unregister(HD_PACK_PANEL);
    free(frontend);
}

void nes_hd_frontend_bind_execution(NesHdFrontend *frontend, FrontendExecutionRuntime *execution) {
    if (frontend) frontend->execution = execution;
}
