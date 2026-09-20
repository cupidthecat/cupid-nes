/*
 * session_actions.c - Live image replacement and reload actions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "session_actions.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "idle_frontend.h"
#include "machine_actions.h"
#include "../cpu/cpu.h"
#include "../joypad/joypad.h"
#include "../rom/fds.h"
#include "../rom/rom.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>

static void set_error(char *error, size_t error_size, const char *message) {
    if (error && error_size) snprintf(error, error_size, "%s", message ? message : "");
}

static bool copy_optional(char *dst, size_t capacity, const char *source) {
    if (!source || !*source) return true;
    if (strlen(source) >= capacity) return false;
    strcpy(dst, source);
    return true;
}

void frontend_session_actions_init(FrontendSessionActions *actions,
                                   FrontendSession *session,
                                   FrontendSettings *settings,
                                   const char *recent_path) {
    if (!actions) return;
    actions->session = session;
    actions->settings = settings;
    actions->recent_path = recent_path;
    actions->execution = NULL;
    actions->image_changed = NULL;
    actions->image_changed_context = NULL;
}

void frontend_session_actions_set_execution(FrontendSessionActions *actions,
                                            FrontendExecutionRuntime *execution) {
    if (actions) actions->execution = execution;
}

void frontend_session_actions_set_image_changed(FrontendSessionActions *actions,
                                                void (*callback)(void *context),
                                                void *context) {
    if (!actions) return;
    actions->image_changed = callback;
    actions->image_changed_context = context;
}

void frontend_image_request_apply_settings(FrontendImageRequest *request,
                                           const FrontendSettings *settings) {
    if (!request || !settings) return;
    request->fds_save_mode = settings->disk_save_mode;
    request->fds_write_protected = settings->fds_write_protected;
    if (!request->fds_overlay_path[0])
        (void)copy_optional(request->fds_overlay_path, sizeof(request->fds_overlay_path),
                            settings->disk_overlay_path);
    if (!request->fds_bios_path[0])
        (void)copy_optional(request->fds_bios_path, sizeof(request->fds_bios_path),
                            settings->fds_bios_path);
    if (!request->studybox_bios_path[0])
        (void)copy_optional(request->studybox_bios_path,
                            sizeof(request->studybox_bios_path), settings->studybox_bios_path);
}

static void remember_firmware(FrontendSessionActions *actions) {
    if (!actions || !actions->session || !actions->settings || !actions->session->active) return;
    const FrontendImageRequest *current = &actions->session->current;
    (void)copy_optional(actions->settings->fds_bios_path,
                        sizeof(actions->settings->fds_bios_path), current->fds_bios_path);
    (void)copy_optional(actions->settings->studybox_bios_path,
                        sizeof(actions->settings->studybox_bios_path), current->studybox_bios_path);
}

static bool persist_recent(FrontendSessionActions *actions, char *error, size_t error_size) {
    if (!actions->recent_path || !*actions->recent_path) return true;
    return frontend_session_save_recent(actions->session, actions->recent_path, error, error_size);
}

static bool activate_request(FrontendSessionActions *actions,
                             const FrontendImageRequest *request,
                             bool preserve_disk, char *error, size_t error_size) {
    if (!actions || !actions->session || !request) return false;
    if (actions->execution && actions->execution->before_machine_change
        && !actions->execution->before_machine_change(
            actions->execution->machine_change_context, error, error_size)) return false;
    if (actions->execution) frontend_execution_begin_machine_change(actions->execution);
    CpuStartupAlignment previous_alignment = cpu_get_startup_alignment();
    bool was_fds = preserve_disk && rom_is_fds();
    bool disk_inserted = was_fds && fds_disk_inserted();
    size_t disk_side = disk_inserted ? fds_current_side() : 0;
    if (!preserve_disk) cpu_use_default_startup_alignment();

    bool opened = preserve_disk
        ? frontend_session_reload(actions->session, error, error_size)
        : frontend_session_open(actions->session, request, error, error_size);
    if (!opened) {
        if (!preserve_disk)
            (void)cpu_set_startup_alignment(previous_alignment.cpu_offset,
                                            previous_alignment.ppu_phase);
        if (actions->execution) frontend_execution_end_machine_change(actions->execution);
        return false;
    }
    if (actions->execution) frontend_execution_clear_timeline(actions->execution);
    if (was_fds && rom_is_fds()) {
        if (disk_inserted) (void)fds_insert_disk(disk_side);
        else fds_eject_disk();
    }
    if (actions->settings) {
        FdsAutomationOptions automation = {
            .insert_automatically = actions->settings->fds_auto_insert,
            .fast_forward_loading = actions->settings->fds_loading_fast_forward
        };
        fds_set_automation_options(automation);
    }

    const char *storage_identity = actions->session->current_result.save_identity[0]
        ? actions->session->current_result.save_identity : actions->session->current.path;
    bool storage_ready = joypad_persistent_configure(storage_identity);
    bool powered = frontend_machine_power_cycle();
    remember_firmware(actions);
    frontend_command_set_session_active(true);
    frontend_panel_set_session_active(true);
    if (actions->execution) {
        actions->execution->rom_path = actions->session->current.path;
        actions->execution->fds_bios_path = actions->session->current.fds_bios_path[0]
            ? actions->session->current.fds_bios_path : NULL;
        actions->execution->studybox_bios_path = actions->session->current.studybox_bios_path[0]
            ? actions->session->current.studybox_bios_path : NULL;
        if (actions->execution->fds_side && rom_is_fds() && fds_side_count())
            *actions->execution->fds_side = fds_current_side();
    }
    if (actions->image_changed)
        actions->image_changed(actions->image_changed_context);
    char recent_error[160] = {0};
    bool recent_saved = persist_recent(actions, recent_error, sizeof(recent_error));
    if (!powered) {
        set_error(error, error_size, "The image opened but its startup alignment is invalid");
        if (actions->execution) frontend_execution_end_machine_change(actions->execution);
        return false;
    }
    if (!storage_ready) {
        set_error(error, error_size, "The image opened but peripheral storage could not be loaded");
        if (actions->execution) frontend_execution_end_machine_change(actions->execution);
        return false;
    }
    if (!recent_saved) {
        set_error(error, error_size, recent_error);
        if (actions->execution) frontend_execution_end_machine_change(actions->execution);
        return false;
    }
    if (actions->execution) frontend_execution_end_machine_change(actions->execution);
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool extension_is(const char *path, const char *extension) {
    const char *dot = path ? strrchr(path, '.') : NULL;
    if (!dot) return false;
    while (*dot && *extension
           && tolower((unsigned char)*dot) == tolower((unsigned char)*extension)) {
        ++dot;
        ++extension;
    }
    return !*dot && !*extension;
}

bool frontend_session_action_open(void *userdata, char *error, size_t error_size) {
    FrontendSessionActions *actions = (FrontendSessionActions *)userdata;
    if (!actions || !actions->session) return false;
    FrontendImageRequest request;
    if (!frontend_idle_choose_request(actions->session, &request, error, error_size)) return false;
    if (!request.save_identity[0]) frontend_image_request_apply_settings(&request, actions->settings);
    return activate_request(actions, &request, false, error, error_size);
}

bool frontend_session_action_reload(void *userdata, char *error, size_t error_size) {
    FrontendSessionActions *actions = (FrontendSessionActions *)userdata;
    if (!actions || !actions->session || !actions->session->active) {
        set_error(error, error_size, "No image is loaded");
        return false;
    }
    return activate_request(actions, &actions->session->current, true, error, error_size);
}

bool frontend_session_action_open_path(FrontendSessionActions *actions, const char *path,
                                       char *error, size_t error_size) {
    if (!actions || !path || !*path) return false;
    FrontendImageRequest request;
    bool patch = extension_is(path, ".ips") || extension_is(path, ".ups")
              || extension_is(path, ".bps");
    if (patch) {
        if (!actions->session || !actions->session->active) {
            set_error(error, error_size, "Open an image before dropping a patch");
            return false;
        }
        request = actions->session->current;
        if (!frontend_image_request_set_patch(&request, path)) {
            set_error(error, error_size, "The patch path is too long");
            return false;
        }
    } else {
        if (!frontend_image_request_init(&request, path)) {
            set_error(error, error_size, "The selected path is too long");
            return false;
        }
        frontend_image_request_apply_settings(&request, actions->settings);
    }
    return activate_request(actions, &request, false, error, error_size);
}
