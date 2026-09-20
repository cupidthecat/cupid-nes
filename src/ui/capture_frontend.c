/*
 * capture_frontend.c - Screenshot and recording commands and controls
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "capture_frontend.h"
#include "output_guard.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    CAPTURE_PATH_PNG = 1,
    CAPTURE_PATH_WAV,
    CAPTURE_PATH_AVI,
    CAPTURE_DISPLAYED,
    CAPTURE_SAMPLE_RATE,
    CAPTURE_BYTE_LIMIT,
    CAPTURE_BROWSE_PNG,
    CAPTURE_BROWSE_WAV,
    CAPTURE_BROWSE_AVI
};

typedef struct {
    unsigned commands;
    bool panel;
    char status[512];
    char limit[32];
} CaptureUi;

static bool fail(char *error, size_t capacity, const char *message) {
    if (error && capacity) snprintf(error, capacity, "%s", message);
    return false;
}

bool nes_capture_path_allowed(const char *path, const char *const *protected_paths, size_t count,
                               char *error, size_t error_size) {
    return frontend_output_path_excludes(path, protected_paths, count, error, error_size);
}

bool nes_capture_frontend_set_path(NesCaptureFrontend *frontend, FrontendSaveFileType type,
                                    const char *path, char *error, size_t error_size) {
    if (!frontend || (unsigned)type > FRONTEND_SAVE_AVI || !path)
        return fail(error, error_size, "The capture path is invalid");
    if (frontend->session.info.recording && type == (frontend->session.info.video ? FRONTEND_SAVE_AVI : FRONTEND_SAVE_WAV))
        return fail(error, error_size, "Stop recording before changing its output path");
    if (!*path) {
        frontend->paths[type][0] = '\0';
        if (error && error_size) error[0] = '\0';
        return true;
    }
    char selected[CAPTURE_PATH_CAPACITY];
    if (!frontend_parse_dialog_output(path, strlen(path), selected, sizeof(selected), error, error_size)) return false;
    if (!frontend->hooks.validate_path(frontend->hooks.context, selected, error, error_size)) return false;
    memcpy(frontend->paths[type], selected, strlen(selected) + 1u);
    return true;
}

static bool choose_path(NesCaptureFrontend *frontend, FrontendSaveFileType type,
                          bool always, char *error, size_t error_size) {
    if (!always && frontend->paths[type][0]) return true;
    char selected[CAPTURE_PATH_CAPACITY] = {0};
    if (!frontend_save_file_dialog(type, selected, sizeof(selected), error, error_size)) return false;
    return nes_capture_frontend_set_path(frontend, type, selected, error, error_size);
}

static bool validate_path(NesCaptureFrontend *frontend, FrontendSaveFileType type,
                            char *error, size_t error_size) {
    if (!frontend->hooks.has_image(frontend->hooks.context))
        return fail(error, error_size, "Open an image before capturing output");
    if (!choose_path(frontend, type, false, error, error_size)) return false;
    const char *path = frontend->paths[type];
    if (frontend->session.info.recording) {
        const char *active = frontend->paths[frontend->session.info.video ? FRONTEND_SAVE_AVI : FRONTEND_SAVE_WAV];
        if (!nes_capture_path_allowed(path, &active, 1, error, error_size)) return false;
    }
    return frontend->hooks.validate_path(frontend->hooks.context, path, error, error_size);
}

static bool command_screenshot(void *context, char *error, size_t error_size) {
    NesCaptureFrontend *frontend = context;
    if (!validate_path(frontend, FRONTEND_SAVE_PNG, error, error_size)) return false;
    NesCaptureFrame frame;
    if (!frontend->hooks.get_frame(frontend->hooks.context, frontend->options.displayed_output,
                                    &frame, error, error_size)) return false;
    NesFileResult result = nes_capture_png(frontend->paths[FRONTEND_SAVE_PNG], &frame);
    if (result != NES_FILE_OK) return fail(error, error_size, nes_file_result_message(result));
    return true;
}

static bool command_record(NesCaptureFrontend *frontend, bool video, char *error, size_t error_size) {
    if (frontend->session.info.recording)
        return fail(error, error_size, "Stop the current recording before starting another");
    FrontendSaveFileType type = video ? FRONTEND_SAVE_AVI : FRONTEND_SAVE_WAV;
    if (!validate_path(frontend, type, error, error_size)) return false;
    NesCaptureFrame frame = {0};
    if (video && !frontend->hooks.get_frame(frontend->hooks.context, frontend->options.displayed_output,
                                            &frame, error, error_size)) return false;
    NesFileResult result = nes_capture_session_start(&frontend->session, frontend->paths[type], video,
                                                      &frame, &frontend->options);
    nes_capture_frontend_refresh(frontend);
    if (result != NES_FILE_OK)
        return fail(error, error_size, frontend->session.error[0]
            ? frontend->session.error : nes_file_result_message(result));
    return true;
}

static bool command_audio(void *context, char *error, size_t error_size) {
    return command_record(context, false, error, error_size);
}

static bool command_video(void *context, char *error, size_t error_size) {
    return command_record(context, true, error, error_size);
}

static bool command_stop(void *context, char *error, size_t error_size) {
    NesCaptureFrontend *frontend = context;
    NesFileResult result = nes_capture_session_stop(&frontend->session);
    nes_capture_frontend_refresh(frontend);
    return result == NES_FILE_OK || fail(error, error_size, frontend->session.error);
}

static const struct {
    const char *label;
    const char *shortcut;
    FrontendCommandHandler handler;
} capture_commands[] = {
    {"Save Screenshot", "F12", command_screenshot},
    {"Record Audio", "Ctrl+F12", command_audio},
    {"Record Video", "Shift+F12", command_video},
    {"Stop Recording", "Ctrl+Shift+F12", command_stop}
};

static bool add_control(FrontendPanelModel *model, unsigned id, FrontendPanelControlType type,
                         const char *label, const char *value, bool enabled) {
    FrontendPanelControl control = {.id = id, .type = type, .label = label, .value = value, .enabled = enabled};
    if (type == FRONTEND_PANEL_FILE_SAVE) control.selected = (int)(id - CAPTURE_PATH_PNG);
    return frontend_panel_add_control(model, &control);
}

static bool capture_snapshot(void *context, FrontendPanelModel *model,
                               char *error, size_t error_size) {
    NesCaptureFrontend *frontend = context;
    CaptureUi *ui = frontend->ui;
    if (!ui) return false;
    const char *const labels[] = {"Screenshot path", "Audio recording path", "Video recording path"};
    bool recording = frontend->session.info.recording;
    bool ok = true;
    for (unsigned i = 0; i < 3; ++i) {
        bool enabled = !recording || i == FRONTEND_SAVE_PNG;
        ok = ok && add_control(model, CAPTURE_PATH_PNG + i, FRONTEND_PANEL_FILE_SAVE,
                                labels[i], frontend->paths[i], enabled);
        ok = ok && add_control(model, CAPTURE_BROWSE_PNG + i, FRONTEND_PANEL_ACTION,
                                i == 0 ? "Choose screenshot file" : i == 1 ? "Choose audio file" : "Choose video file",
                                "", enabled);
    }
    FrontendPanelControl displayed = {
        .id = CAPTURE_DISPLAYED, .type = FRONTEND_PANEL_CHECKBOX,
        .label = "Capture displayed output (filters included; UI overlays excluded)",
        .selected = frontend->options.displayed_output ? 1 : 0, .enabled = !recording
    };
    ok = ok && frontend_panel_add_control(model, &displayed);
    static const char *const rates[] = {"44100 Hz", "48000 Hz", "96000 Hz"};
    FrontendPanelControl rate = {
        .id = CAPTURE_SAMPLE_RATE, .type = FRONTEND_PANEL_CHOICE, .label = "Recording sample rate",
        .items = rates, .item_count = 3, .enabled = !recording,
        .selected = frontend->options.sample_rate == 48000 ? 1 : frontend->options.sample_rate == 96000 ? 2 : 0
    };
    ok = ok && frontend_panel_add_control(model, &rate);
    snprintf(ui->limit, sizeof(ui->limit), "%" PRIu64, frontend->options.byte_limit);
    ok = ok && add_control(model, CAPTURE_BYTE_LIMIT, FRONTEND_PANEL_TEXT,
                            "Maximum recording bytes (up to 4294967295)", ui->limit, !recording);
    for (unsigned i = 0; i < 4; ++i) {
        FrontendCommandInfo command;
        if (!frontend_command_get(CAPTURE_COMMAND_SCREENSHOT + i, &command)) return false;
        ok = ok && add_control(model, command.id, FRONTEND_PANEL_ACTION, command.label, "", command.enabled);
    }
    const NesCaptureInfo *info = &frontend->session.info;
    if (frontend->session.error[0]) {
        snprintf(ui->status, sizeof(ui->status), "%s", frontend->session.error);
    } else {
        snprintf(ui->status, sizeof(ui->status), "%s | %s | %" PRIu64 " frames | %" PRIu64
                 " stereo audio frames | %" PRIu64 " bytes",
                 recording ? "Recording" : "Stopped", info->video ? "AVI" : "WAV",
                 info->completed_frames, info->audio_frames, info->bytes);
    }
    model->status = ui->status;
    if (!ok) return fail(error, error_size, "The capture panel needs 13 controls");
    return true;
}

static bool capture_action(void *context, unsigned id, const char *value, int selected,
                             char *error, size_t error_size) {
    NesCaptureFrontend *frontend = context;
    if (id >= CAPTURE_COMMAND_SCREENSHOT && id <= CAPTURE_COMMAND_STOP)
        return frontend_command_invoke(id, error, error_size);
    if (id >= CAPTURE_PATH_PNG && id <= CAPTURE_PATH_AVI)
        return nes_capture_frontend_set_path(frontend, (FrontendSaveFileType)(id - CAPTURE_PATH_PNG),
                                              value, error, error_size);
    if (id >= CAPTURE_BROWSE_PNG && id <= CAPTURE_BROWSE_AVI)
        return choose_path(frontend, (FrontendSaveFileType)(id - CAPTURE_BROWSE_PNG), true, error, error_size);
    if (frontend->session.info.recording)
        return fail(error, error_size, "Stop recording before changing capture settings");
    if (id == CAPTURE_DISPLAYED) {
        frontend->options.displayed_output = !frontend->options.displayed_output;
        return true;
    }
    if (id == CAPTURE_SAMPLE_RATE && selected >= 0 && selected < 3) {
        const unsigned rates[] = {44100, 48000, 96000};
        frontend->options.sample_rate = rates[selected];
        return true;
    }
    if (id == CAPTURE_BYTE_LIMIT && value && value[0] >= '0' && value[0] <= '9') {
        char *end = NULL;
        errno = 0;
        unsigned long long limit = strtoull(value, &end, 10);
        if (!errno && end && !*end && limit >= 1024 && limit <= UINT32_MAX) {
            frontend->options.byte_limit = (uint64_t)limit;
            return true;
        }
    }
    return fail(error, error_size, "Enter a valid capture setting");
}

bool nes_capture_frontend_init(NesCaptureFrontend *frontend, const NesCaptureFrontendHooks *hooks) {
    if (!frontend || !hooks || !hooks->has_image || !hooks->get_frame || !hooks->validate_path) return false;
    memset(frontend, 0, sizeof(*frontend));
    frontend->hooks = *hooks;
    nes_capture_options_defaults(&frontend->options);
    CaptureUi *ui = calloc(1, sizeof(*ui));
    if (!ui) return false;
    frontend->ui = ui;
    for (unsigned i = 0; i < 4; ++i) {
        FrontendCommandSpec command = {
            .id = CAPTURE_COMMAND_SCREENSHOT + i, .label = capture_commands[i].label,
            .shortcut = capture_commands[i].shortcut, .menu = "Tools",
            .flags = i == 3 ? 0u : FRONTEND_COMMAND_NEEDS_SESSION,
            .handler = capture_commands[i].handler, .userdata = frontend
        };
        if (!frontend_command_register(&command)) {
            (void)nes_capture_frontend_shutdown(frontend);
            return false;
        }
        ++ui->commands;
    }
    FrontendPanelSpec panel = {
        .id = CAPTURE_PANEL, .title = "Capture", .category = "Tools",
        .snapshot = capture_snapshot, .action = capture_action, .userdata = frontend
    };
    if (!frontend_panel_register(&panel)) {
        (void)nes_capture_frontend_shutdown(frontend);
        return false;
    }
    ui->panel = true;
    nes_capture_frontend_refresh(frontend);
    return true;
}

NesFileResult nes_capture_frontend_shutdown(NesCaptureFrontend *frontend) {
    if (!frontend) return NES_FILE_INVALID_ARGUMENT;
    NesFileResult result = nes_capture_session_stop(&frontend->session);
    CaptureUi *ui = frontend->ui;
    if (ui) {
        for (unsigned i = 0; i < ui->commands; ++i)
            (void)frontend_command_unregister(CAPTURE_COMMAND_SCREENSHOT + i);
        if (ui->panel) (void)frontend_panel_unregister(CAPTURE_PANEL);
        free(ui);
    }
    frontend->ui = NULL;
    return result;
}

void nes_capture_frontend_refresh(NesCaptureFrontend *frontend) {
    if (!frontend || !frontend->ui) return;
    bool loaded = frontend->hooks.has_image(frontend->hooks.context);
    bool recording = frontend->session.info.recording;
    (void)frontend_command_set_enabled(CAPTURE_COMMAND_SCREENSHOT, loaded);
    (void)frontend_command_set_enabled(CAPTURE_COMMAND_AUDIO, loaded && !recording);
    (void)frontend_command_set_enabled(CAPTURE_COMMAND_VIDEO, loaded && !recording);
    (void)frontend_command_set_enabled(CAPTURE_COMMAND_STOP, recording);
}

void nes_capture_frontend_begin_frame(NesCaptureFrontend *frontend) {
    if (!frontend) return;
    (void)nes_capture_session_begin_frame(&frontend->session);
    nes_capture_frontend_refresh(frontend);
}

void nes_capture_frontend_end_frame(NesCaptureFrontend *frontend, bool completed) {
    if (!frontend || !completed || !frontend->session.info.recording) return;
    NesCaptureFrame frame = {0};
    char error[256] = {0};
    if (frontend->session.info.video
        && !frontend->hooks.get_frame(frontend->hooks.context, frontend->options.displayed_output,
                                       &frame, error, sizeof(error))) {
        (void)nes_capture_session_stop(&frontend->session);
        snprintf(frontend->session.error, sizeof(frontend->session.error), "%s",
                 error[0] ? error : "The output frame could not be captured");
    } else {
        (void)nes_capture_session_end_frame(&frontend->session, &frame);
    }
    nes_capture_frontend_refresh(frontend);
}

bool nes_capture_frontend_handle_shortcut(NesCaptureFrontend *frontend,
                                          const SDL_KeyboardEvent *event,
                                          char *error, size_t error_size) {
    if (!frontend || !event || event->keysym.scancode != SDL_SCANCODE_F12
        || (event->keysym.mod & (KMOD_ALT | KMOD_GUI))) return false;
    if (event->type == SDL_KEYDOWN && !event->repeat) {
        bool control = (event->keysym.mod & KMOD_CTRL) != 0;
        bool shift = (event->keysym.mod & KMOD_SHIFT) != 0;
        unsigned command = control && shift ? CAPTURE_COMMAND_STOP : control ? CAPTURE_COMMAND_AUDIO
            : shift ? CAPTURE_COMMAND_VIDEO : CAPTURE_COMMAND_SCREENSHOT;
        nes_capture_frontend_refresh(frontend);
        (void)frontend_command_invoke(command, error, error_size);
    }
    return true;
}
