/*
 * device_panels.c - Desktop controls for disk, cassette, barcode and VS devices
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "device_frontend.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "../joypad/family_basic.h"
#include "../joypad/joypad.h"
#include "../rom/fds.h"
#include "../rom/mapper.h"
#include "../system/execution_policy.h"
#include "../system/vs_system.h"
#include <stdio.h>

void frontend_devices_refresh(FrontendDeviceRuntime *runtime) {
    if (!runtime) return;
    bool disk = fds_active();
    bool tape = joypad_expansion_device() == NES_EXPANSION_FAMILY_BASIC;
    bool barcode = cart_barcode_supported() || joypad_expansion_device() == NES_EXPANSION_BARCODE_BATTLER;
    bool replaying = (nes_execution_policy() & NES_EXECUTION_MOVIE_PLAYBACK) != 0;
    (void)frontend_panel_set_enabled(DEVICE_PANEL_DISK, disk);
    (void)frontend_panel_set_enabled(DEVICE_PANEL_TAPE, tape || runtime->tape_capture_pending);
    (void)frontend_panel_set_enabled(DEVICE_PANEL_BARCODE, barcode);
    (void)frontend_panel_set_enabled(DEVICE_PANEL_VS, vs_enabled());
    (void)frontend_command_set_enabled(DEVICE_COMMAND_DISK_TOGGLE, disk && !replaying);
    (void)frontend_command_set_enabled(DEVICE_COMMAND_DISK_NEXT, disk && fds_side_count() > 1 && !replaying);
    (void)frontend_command_set_enabled(DEVICE_COMMAND_TAPE_PLAY, tape && !replaying);
    (void)frontend_command_set_enabled(DEVICE_COMMAND_TAPE_RECORD, tape && !replaying);
    (void)frontend_command_set_enabled(DEVICE_COMMAND_TAPE_STOP, (tape || runtime->tape_capture_pending) && !replaying);
    (void)frontend_command_set_enabled(DEVICE_COMMAND_BARCODE_SCAN, barcode && !replaying);
}

static bool button(FrontendPanelModel *model, unsigned id, const char *label, bool enabled) {
    FrontendPanelControl control = {.id = id, .type = FRONTEND_PANEL_ACTION, .label = label, .enabled = enabled};
    return frontend_panel_add_control(model, &control);
}

static bool field(FrontendPanelModel *model, unsigned id, const char *label,
                    const char *value, bool enabled, bool read_only) {
    FrontendPanelControl control = {
        .id = id, .type = FRONTEND_PANEL_TEXT, .label = label,
        .value = value, .enabled = enabled, .read_only = read_only
    };
    return frontend_panel_add_control(model, &control);
}

static bool checkbox(FrontendPanelModel *model, unsigned id, const char *label, bool checked, bool enabled) {
    FrontendPanelControl control = {
        .id = id, .type = FRONTEND_PANEL_CHECKBOX, .label = label,
        .selected = checked, .enabled = enabled
    };
    return frontend_panel_add_control(model, &control);
}

static bool snapshot_disk(void *context, FrontendPanelModel *model, char *error, size_t error_size) {
    (void)error; (void)error_size;
    FrontendDeviceRuntime *runtime = context;
    bool live = nes_execution_policy() == NES_EXECUTION_LIVE;
    bool input = !(nes_execution_policy() & NES_EXECUTION_MOVIE_PLAYBACK);
    size_t side = fds_disk_inserted() ? fds_current_side() : runtime->disk_side ? *runtime->disk_side : 0;
    snprintf(runtime->disk_side_text, sizeof(runtime->disk_side_text), "%zu", side + 1);
    snprintf(runtime->disk_status, sizeof(runtime->disk_status),
             "%zu sides | %s | %s%s%s", fds_side_count(),
             fds_disk_inserted() ? "inserted" : "ejected",
             fds_save_mode() == FDS_SAVE_OVERLAY ? "overlay saving" : "in-place saving",
             fds_disk_dirty() ? " | unsaved disk changes" : "",
             fds_automatic_insert_ambiguous() ? " | several sides match; select one manually" : "");
    FdsAutomationOptions automatic = fds_automation_options();
    model->status = runtime->status;
    return field(model, DEVICE_DISK_SIDE, "Disk side (1 is side A of the first disk)", runtime->disk_side_text, input, false)
        && button(model, DEVICE_DISK_TOGGLE, fds_disk_inserted() ? "Eject disk" : "Insert selected disk", input)
        && checkbox(model, DEVICE_DISK_WRITE_PROTECT, "Write protected", fds_write_protected(), live)
        && checkbox(model, DEVICE_DISK_AUTO_INSERT, "Insert requested sides automatically", automatic.insert_automatically, live)
        && checkbox(model, DEVICE_DISK_FAST_FORWARD, "Fast-forward during disk loading", automatic.fast_forward_loading, live)
        && field(model, DEVICE_DISK_STATUS, "Disk status", runtime->disk_status, true, true);
}

static bool snapshot_tape(void *context, FrontendPanelModel *model, char *error, size_t error_size) {
    (void)error; (void)error_size;
    FrontendDeviceRuntime *runtime = context;
    bool live = nes_execution_policy() == NES_EXECUTION_LIVE;
    bool input = !(nes_execution_policy() & NES_EXECUTION_MOVIE_PLAYBACK);
    bool tape = joypad_expansion_device() == NES_EXPANSION_FAMILY_BASIC;
    FamilyBasicTapeMode mode = family_basic_tape_mode();
    const char *mode_text = mode == FB_TAPE_RECORDING ? "Recording" : mode == FB_TAPE_PLAYING ? "Playing" : "Stopped";
    snprintf(runtime->tape_status, sizeof(runtime->tape_status), "%s%s%s", mode_text,
             runtime->tape_capture_pending ? " | recording has not been saved" : "",
             !live ? " | isolated replay session" : "");
    model->status = runtime->status[0] ? runtime->status : "F10 starts the selected tape mode; F11 stops and saves a recording.";
    return field(model, DEVICE_TAPE_INPUT, "Input tape", runtime->tape_input, live && !runtime->tape_capture_pending, false)
        && button(model, DEVICE_TAPE_BROWSE_INPUT, "Choose input tape...", live && !runtime->tape_capture_pending)
        && button(model, DEVICE_TAPE_LOAD, "Load selected tape", live && tape)
        && field(model, DEVICE_TAPE_OUTPUT, "Recording destination", runtime->tape_output, live, false)
        && button(model, DEVICE_TAPE_BROWSE_OUTPUT, "Choose recording destination...", live)
        && button(model, DEVICE_TAPE_PLAY, "Play from beginning", tape && input)
        && button(model, DEVICE_TAPE_RECORD, "Record", tape && input && mode != FB_TAPE_RECORDING)
        && button(model, DEVICE_TAPE_STOP, runtime->tape_capture_pending ? "Stop and save recording" : "Stop", input)
        && field(model, DEVICE_TAPE_STATUS, "Tape status", runtime->tape_status, true, true);
}

static bool snapshot_barcode(void *context, FrontendPanelModel *model, char *error, size_t error_size) {
    (void)error; (void)error_size;
    FrontendDeviceRuntime *runtime = context;
    bool input = !(nes_execution_policy() & NES_EXECUTION_MOVIE_PLAYBACK);
    model->status = runtime->status[0] ? runtime->status : "Enter 8 or 13 decimal digits, then scan.";
    return field(model, DEVICE_BARCODE_TEXT, "Barcode", runtime->barcode, input, false)
        && button(model, DEVICE_BARCODE_SCAN, "Scan barcode", input);
}

static bool snapshot_vs(void *context, FrontendPanelModel *model, char *error, size_t error_size) {
    (void)error; (void)error_size;
    FrontendDeviceRuntime *runtime = context;
    bool input = !(nes_execution_policy() & NES_EXECUTION_MOVIE_PLAYBACK);
    model->status = runtime->status[0] ? runtime->status : "Coin inputs use the hardware pulse duration. Service lasts one emulated frame.";
    if (!button(model, DEVICE_VS_COIN_1, "Insert coin 1", input)
        || !button(model, DEVICE_VS_COIN_2, "Insert coin 2", input)
        || !button(model, DEVICE_VS_SERVICE_1, "Press service button", input)) return false;
    return !vs_dual_system()
        || (button(model, DEVICE_VS_COIN_3, "Insert coin 1 on second system", input)
            && button(model, DEVICE_VS_COIN_4, "Insert coin 2 on second system", input)
            && button(model, DEVICE_VS_SERVICE_2, "Press service on second system", input));
}

static bool panel_action(void *context, unsigned id, const char *value, int selected,
                          char *error, size_t error_size) {
    bool result = frontend_devices_action(context, id, value, selected, error, error_size);
    frontend_devices_refresh(context);
    return result;
}

static bool command_disk_toggle(void *context, char *error, size_t error_size) {
    return panel_action(context, DEVICE_DISK_TOGGLE, NULL, 0, error, error_size);
}

static bool command_disk_next(void *context, char *error, size_t error_size) {
    FrontendDeviceRuntime *runtime = context;
    size_t count = fds_side_count();
    if (!count) {
        if (error && error_size) snprintf(error, error_size, "No disk-system image is loaded");
        return false;
    }
    size_t side = fds_disk_inserted() ? fds_current_side() : runtime->disk_side ? *runtime->disk_side : 0;
    char number[32];
    snprintf(number, sizeof(number), "%zu", (side + 1) % count + 1);
    return panel_action(context, DEVICE_DISK_SIDE, number, 0, error, error_size);
}

static bool command_tape_play(void *context, char *error, size_t error_size) {
    return panel_action(context, DEVICE_TAPE_PLAY, NULL, 0, error, error_size);
}

static bool command_tape_record(void *context, char *error, size_t error_size) {
    return panel_action(context, DEVICE_TAPE_RECORD, NULL, 0, error, error_size);
}

static bool command_tape_stop(void *context, char *error, size_t error_size) {
    return panel_action(context, DEVICE_TAPE_STOP, NULL, 0, error, error_size);
}

static bool command_barcode_scan(void *context, char *error, size_t error_size) {
    return panel_action(context, DEVICE_BARCODE_SCAN, NULL, 0, error, error_size);
}

bool frontend_devices_register(FrontendDeviceRuntime *runtime) {
    if (!runtime) return false;
    const FrontendPanelSpec panels[] = {
        {DEVICE_PANEL_DISK, "Disk System", "Media", FRONTEND_PANEL_NEEDS_SESSION, snapshot_disk, panel_action, runtime},
        {DEVICE_PANEL_TAPE, "Family BASIC Tape", "Media", FRONTEND_PANEL_NEEDS_SESSION, snapshot_tape, panel_action, runtime},
        {DEVICE_PANEL_BARCODE, "Barcode Reader", "Media", FRONTEND_PANEL_NEEDS_SESSION, snapshot_barcode, panel_action, runtime},
        {DEVICE_PANEL_VS, "Arcade Inputs", "Media", FRONTEND_PANEL_NEEDS_SESSION, snapshot_vs, panel_action, runtime}
    };
    const FrontendCommandSpec commands[] = {
        {DEVICE_COMMAND_DISK_TOGGLE, "Insert / Eject Disk", "Media", "F8", FRONTEND_COMMAND_NEEDS_SESSION, command_disk_toggle, runtime},
        {DEVICE_COMMAND_DISK_NEXT, "Next Disk Side", "Media", "F9", FRONTEND_COMMAND_NEEDS_SESSION, command_disk_next, runtime},
        {DEVICE_COMMAND_TAPE_PLAY, "Play Tape", "Media", NULL, FRONTEND_COMMAND_NEEDS_SESSION, command_tape_play, runtime},
        {DEVICE_COMMAND_TAPE_RECORD, "Record Tape", "Media", NULL, FRONTEND_COMMAND_NEEDS_SESSION, command_tape_record, runtime},
        {DEVICE_COMMAND_TAPE_STOP, "Stop / Save Tape", "Media", "F11", FRONTEND_COMMAND_NEEDS_SESSION, command_tape_stop, runtime},
        {DEVICE_COMMAND_BARCODE_SCAN, "Scan Barcode", "Media", "F8", FRONTEND_COMMAND_NEEDS_SESSION, command_barcode_scan, runtime}
    };
    size_t registered_panels = 0, registered_commands = 0;
    for (; registered_panels < sizeof(panels) / sizeof(panels[0]); ++registered_panels)
        if (!frontend_panel_register(&panels[registered_panels])) goto fail;
    for (; registered_commands < sizeof(commands) / sizeof(commands[0]); ++registered_commands)
        if (!frontend_command_register(&commands[registered_commands])) goto fail;
    frontend_devices_refresh(runtime);
    return true;
fail:
    while (registered_panels) (void)frontend_panel_unregister(panels[--registered_panels].id);
    while (registered_commands) (void)frontend_command_unregister(commands[--registered_commands].id);
    return false;
}

void frontend_devices_unregister(void) {
    for (unsigned id = DEVICE_PANEL_DISK; id <= DEVICE_PANEL_VS; ++id)
        (void)frontend_panel_unregister(id);
    for (unsigned id = DEVICE_COMMAND_DISK_TOGGLE; id <= DEVICE_COMMAND_BARCODE_SCAN; ++id)
        (void)frontend_command_unregister(id);
}
