/*
 * device_frontend.c - Device actions and cassette recording ownership
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "device_frontend.h"
#include "output_guard.h"
#include "platform_frontend.h"
#include "../cpu/cpu.h"
#include "../joypad/family_basic.h"
#include "../joypad/joypad.h"
#include "../rom/fds.h"
#include "../rom/mapper.h"
#include "../system/execution_policy.h"
#include "../system/vs_system.h"
#include "../util/file_io.h"
#include "../../include/globals.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool message(FrontendDeviceRuntime *runtime, bool success, const char *text,
                     char *error, size_t error_size) {
    if (runtime) snprintf(runtime->status, sizeof(runtime->status), "%s", text);
    if (error && error_size) snprintf(error, error_size, "%s", success ? "" : text);
    return success;
}

static bool family_keyboard(void) {
    return joypad_expansion_device() == NES_EXPANSION_FAMILY_BASIC;
}

static bool live_configuration(FrontendDeviceRuntime *runtime, char *error, size_t error_size) {
    return nes_execution_policy() == NES_EXECUTION_LIVE
        || message(runtime, false, "Stop the replay session before changing device configuration", error, error_size);
}

static bool path_copy(const char *path, char *copy, size_t capacity,
                       char *error, size_t error_size) {
    if (!path || !*path) { copy[0] = '\0'; return true; }
    return frontend_parse_dialog_output(path, strlen(path), copy, capacity, error, error_size);
}

static bool tape_output_allowed(FrontendDeviceRuntime *runtime, const char *path,
                                 char *error, size_t error_size) {
    if (!runtime) return false;
    if (!frontend_output_path_allowed(path, runtime->execution, runtime->protected_paths,
                                       runtime->protected_path_count, error, error_size)) return false;
    if (runtime->tape_input[0]) {
        const char *source = runtime->tape_input;
        if (!frontend_output_path_excludes(path, &source, 1, error, error_size)) return false;
    }
    return true;
}

void frontend_devices_init(FrontendDeviceRuntime *runtime, FrontendExecutionRuntime *execution,
                            FrontendSettings *settings, size_t *disk_side) {
    if (!runtime) return;
    memset(runtime, 0, sizeof(*runtime));
    runtime->execution = execution;
    runtime->settings = settings;
    runtime->disk_side = disk_side;
}

void frontend_devices_set_protected_paths(FrontendDeviceRuntime *runtime,
                                          const char *const *paths, size_t count) {
    if (!runtime || (count && !paths)) return;
    runtime->protected_paths = paths;
    runtime->protected_path_count = count;
}

bool frontend_devices_set_tape_paths(FrontendDeviceRuntime *runtime,
                                     const char *input, const char *output,
                                     char *error, size_t error_size) {
    if (!runtime || !live_configuration(runtime, error, error_size)) return false;
    char next_input[FRONTEND_DEVICE_PATH_CAPACITY], next_output[FRONTEND_DEVICE_PATH_CAPACITY];
    if (!path_copy(input, next_input, sizeof(next_input), error, error_size)
        || !path_copy(output, next_output, sizeof(next_output), error, error_size)) return false;
    if (runtime->tape_capture_pending && strcmp(next_input, runtime->tape_input))
        return message(runtime, false, "Save the pending tape recording before changing its input", error, error_size);
    if (next_output[0]) {
        if (!tape_output_allowed(runtime, next_output, error, error_size)) return false;
        if (next_input[0]) {
            const char *source = next_input;
            if (!frontend_output_path_excludes(next_output, &source, 1, error, error_size)) return false;
        }
    }
    memcpy(runtime->tape_input, next_input, strlen(next_input) + 1);
    memcpy(runtime->tape_output, next_output, strlen(next_output) + 1);
    runtime->tape_record_selected = next_output[0] != '\0';
    return message(runtime, true, "Tape paths updated", error, error_size);
}

bool frontend_devices_finish(FrontendDeviceRuntime *runtime, char *error, size_t error_size) {
    if (!runtime || !runtime->tape_capture_pending) return true;
    if (!nes_execution_allows_persistence())
        return message(runtime, false, "Stop the replay session before saving the pending tape", error, error_size);
    family_basic_tape_stop();
    if (family_basic_tape_mode() != FB_TAPE_STOPPED)
        return message(runtime, false, "The active session rejected the tape stop", error, error_size);
    if (!tape_output_allowed(runtime, runtime->tape_output, error, error_size)) return false;
    bool shortened = family_basic_tape_failed();
    if (!family_basic_tape_save_file(runtime->tape_output))
        return message(runtime, false, "Could not save tape; the captured signal remains available for retry", error, error_size);
    runtime->tape_capture_pending = false;
    return message(runtime, true, shortened
        ? "The tape reached its memory limit. The completed signal was saved."
        : "Tape recording saved", error, error_size);
}

bool frontend_devices_tape_load(FrontendDeviceRuntime *runtime, const char *path,
                                char *error, size_t error_size) {
    if (!runtime || !family_keyboard())
        return message(runtime, false, "Select the Family BASIC keyboard before loading a tape", error, error_size);
    if (!live_configuration(runtime, error, error_size)) return false;
    char selected[FRONTEND_DEVICE_PATH_CAPACITY];
    if (!path_copy(path, selected, sizeof(selected), error, error_size) || !selected[0])
        return message(runtime, false, "Choose a tape input file first", error, error_size);
    uint8_t *data = NULL;
    size_t size = 0;
    NesFileResult read = nes_file_read_all(selected, 256u * 1024u * 1024u, &data, &size);
    if (read != NES_FILE_OK) return message(runtime, false, nes_file_result_message(read), error, error_size);
    if (!frontend_devices_finish(runtime, error, error_size)) { free(data); return false; }
    bool loaded = family_basic_tape_load(data, size);
    free(data);
    if (!loaded) return message(runtime, false, "Could not prepare the selected tape", error, error_size);
    memcpy(runtime->tape_input, selected, strlen(selected) + 1);
    runtime->tape_record_selected = false;
    if (runtime->execution) frontend_execution_clear_timeline(runtime->execution);
    return message(runtime, true, "Tape loaded. Press Play to start it.", error, error_size);
}

bool frontend_devices_tape_play(FrontendDeviceRuntime *runtime, char *error, size_t error_size) {
    if (!runtime || !family_keyboard())
        return message(runtime, false, "The Family BASIC keyboard is not selected", error, error_size);
    if (!frontend_devices_finish(runtime, error, error_size)) return false;
    if (!family_basic_tape_play(cpu_total_cycles))
        return message(runtime, false, "Load a nonempty tape, or stop the session that owns its inputs", error, error_size);
    runtime->tape_record_selected = false;
    return message(runtime, true, "Tape playback started", error, error_size);
}

bool frontend_devices_tape_record(FrontendDeviceRuntime *runtime, char *error, size_t error_size) {
    if (!runtime || !family_keyboard())
        return message(runtime, false, "The Family BASIC keyboard is not selected", error, error_size);
    if (family_basic_tape_mode() == FB_TAPE_RECORDING) return true;
    bool live = nes_execution_policy() == NES_EXECUTION_LIVE;
    if (live && (!tape_output_allowed(runtime, runtime->tape_output, error, error_size)
                 || !frontend_devices_finish(runtime, error, error_size))) return false;
    family_basic_tape_record(cpu_total_cycles);
    if (family_basic_tape_mode() != FB_TAPE_RECORDING)
        return message(runtime, false, "The active session rejected tape recording", error, error_size);
    runtime->tape_capture_pending = live;
    runtime->tape_record_selected = true;
    return message(runtime, true, live ? "Tape recording started" : "Tape recording is isolated with the replay session",
                   error, error_size);
}

bool frontend_devices_tape_stop(FrontendDeviceRuntime *runtime, char *error, size_t error_size) {
    if (!runtime) return false;
    if (runtime->tape_capture_pending) return frontend_devices_finish(runtime, error, error_size);
    family_basic_tape_stop();
    return message(runtime, family_basic_tape_mode() == FB_TAPE_STOPPED,
                   family_basic_tape_mode() == FB_TAPE_STOPPED ? "Tape stopped"
                       : "The active session rejected the tape stop", error, error_size);
}

bool frontend_devices_handle_tape_key(FrontendDeviceRuntime *runtime,
                                      const SDL_KeyboardEvent *event,
                                      char *error, size_t error_size) {
    if (!runtime || !event || !family_keyboard()) return false;
    if (event->keysym.scancode != SDL_SCANCODE_F10 && event->keysym.scancode != SDL_SCANCODE_F11) return false;
    if (event->type == SDL_KEYDOWN && !event->repeat) {
        if (event->keysym.scancode == SDL_SCANCODE_F11)
            (void)frontend_devices_tape_stop(runtime, error, error_size);
        else if (runtime->tape_record_selected)
            (void)frontend_devices_tape_record(runtime, error, error_size);
        else (void)frontend_devices_tape_play(runtime, error, error_size);
    }
    return true;
}

void frontend_devices_frame_complete(FrontendDeviceRuntime *runtime) {
    if (!runtime) return;
    for (unsigned side = 0; side < 2; ++side)
        if ((runtime->service_pulses & (1u << side)) && vs_set_service(side, false))
            runtime->service_pulses &= ~(1u << side);
    frontend_devices_refresh(runtime);
}

void frontend_devices_session_changed(FrontendDeviceRuntime *runtime) {
    if (!runtime) return;
    runtime->service_pulses = 0;
    if (runtime->disk_side && fds_disk_inserted()) *runtime->disk_side = fds_current_side();
    frontend_devices_refresh(runtime);
}

bool frontend_devices_set_barcode(FrontendDeviceRuntime *runtime, const char *digits,
                                  char *error, size_t error_size) {
    if (!runtime || !digits || strlen(digits) >= sizeof(runtime->barcode))
        return message(runtime, false, "A barcode can contain at most 13 digits", error, error_size);
    for (const char *p = digits; *p; ++p)
        if (*p < '0' || *p > '9') return message(runtime, false, "A barcode contains decimal digits only", error, error_size);
    memcpy(runtime->barcode, digits, strlen(digits) + 1);
    return message(runtime, true, "Barcode updated", error, error_size);
}

static bool disk_action(FrontendDeviceRuntime *runtime, unsigned control,
                         const char *value, int selected, char *error, size_t error_size) {
    if (!fds_active()) return message(runtime, false, "No disk-system image is loaded", error, error_size);
    size_t current = runtime->disk_side ? *runtime->disk_side : 0;
    if (fds_disk_inserted()) current = fds_current_side();
    if (control == DEVICE_DISK_SIDE) {
        if (!value || !*value) return message(runtime, false, "Enter a disk side number", error, error_size);
        char *end = NULL;
        errno = 0;
        unsigned long side = strtoul(value, &end, 10);
        if (errno || *end || !side || side > fds_side_count())
            return message(runtime, false, "The disk side number is outside this image", error, error_size);
        if (!fds_insert_disk(side - 1u))
            return message(runtime, false, "The active session rejected the disk insertion", error, error_size);
        if (runtime->disk_side) *runtime->disk_side = side - 1u;
        return message(runtime, true, "Disk side inserted", error, error_size);
    }
    if (control == DEVICE_DISK_TOGGLE) {
        if (fds_disk_inserted()) {
            if (runtime->disk_side) *runtime->disk_side = current;
            fds_eject_disk();
            return message(runtime, !fds_disk_inserted(), !fds_disk_inserted()
                ? "Disk ejected" : "The active session rejected the disk ejection", error, error_size);
        }
        bool inserted = current < fds_side_count() && fds_insert_disk(current);
        return message(runtime, inserted, inserted ? "Disk inserted"
            : "The active session rejected the disk insertion", error, error_size);
    }
    if (!live_configuration(runtime, error, error_size)) return false;
    if (selected != 0 && selected != 1) return message(runtime, false, "Select On or Off", error, error_size);
    if (control == DEVICE_DISK_WRITE_PROTECT) {
        fds_set_write_protected(selected != 0);
        if (runtime->settings) runtime->settings->fds_write_protected = selected != 0;
    } else {
        FdsAutomationOptions options = fds_automation_options();
        if (control == DEVICE_DISK_AUTO_INSERT) options.insert_automatically = selected != 0;
        else if (control == DEVICE_DISK_FAST_FORWARD) options.fast_forward_loading = selected != 0;
        else return message(runtime, false, "Unknown disk control", error, error_size);
        fds_set_automation_options(options);
        if (runtime->settings) {
            runtime->settings->fds_auto_insert = options.insert_automatically;
            runtime->settings->fds_loading_fast_forward = options.fast_forward_loading;
        }
    }
    return message(runtime, true, "Disk option updated", error, error_size);
}

bool frontend_devices_action(FrontendDeviceRuntime *runtime, unsigned control,
                              const char *value, int selected, char *error, size_t error_size) {
    if (!runtime) return false;
    if (control >= DEVICE_DISK_SIDE && control <= DEVICE_DISK_FAST_FORWARD)
        return disk_action(runtime, control, value, selected, error, error_size);
    switch (control) {
        case DEVICE_TAPE_INPUT:
            return frontend_devices_set_tape_paths(runtime, value, runtime->tape_output, error, error_size);
        case DEVICE_TAPE_OUTPUT:
            return frontend_devices_set_tape_paths(runtime, runtime->tape_input, value, error, error_size);
        case DEVICE_TAPE_BROWSE_INPUT:
        case DEVICE_TAPE_BROWSE_OUTPUT: {
            if (!live_configuration(runtime, error, error_size)) return false;
            char path[FRONTEND_DEVICE_PATH_CAPACITY] = {0};
            bool output = control == DEVICE_TAPE_BROWSE_OUTPUT;
            bool chosen = output ? frontend_save_file_dialog(FRONTEND_SAVE_TAPE, path, sizeof(path), error, error_size)
                : frontend_open_file_dialog(FRONTEND_OPEN_TAPE, path, sizeof(path), error, error_size);
            if (!chosen) return false;
            return frontend_devices_set_tape_paths(runtime, output ? runtime->tape_input : path,
                                                   output ? path : runtime->tape_output, error, error_size);
        }
        case DEVICE_TAPE_LOAD: return frontend_devices_tape_load(runtime, runtime->tape_input, error, error_size);
        case DEVICE_TAPE_PLAY: return frontend_devices_tape_play(runtime, error, error_size);
        case DEVICE_TAPE_RECORD: return frontend_devices_tape_record(runtime, error, error_size);
        case DEVICE_TAPE_STOP: return frontend_devices_tape_stop(runtime, error, error_size);
        case DEVICE_BARCODE_TEXT: return frontend_devices_set_barcode(runtime, value, error, error_size);
        case DEVICE_BARCODE_SCAN: {
            size_t length = strlen(runtime->barcode);
            if (length != 8 && length != 13)
                return message(runtime, false, "Enter an 8-digit or 13-digit barcode", error, error_size);
            bool supported = cart_barcode_supported();
            bool scanned = supported ? cart_set_barcode(runtime->barcode)
                : joypad_expansion_device() == NES_EXPANSION_BARCODE_BATTLER
                    && joypad_scan_barcode_battler(runtime->barcode);
            return message(runtime, scanned, scanned ? "Barcode scanned"
                : "The loaded device or replay session does not accept this barcode", error, error_size);
        }
        case DEVICE_VS_COIN_1: case DEVICE_VS_COIN_2:
        case DEVICE_VS_COIN_3: case DEVICE_VS_COIN_4: {
            unsigned slot = control - DEVICE_VS_COIN_1;
            bool accepted = vs_set_coin(slot, true);
            if (accepted) (void)vs_set_coin(slot, false);
            return message(runtime, accepted, accepted ? "Coin inserted"
                : "The loaded system or replay session rejected this coin input", error, error_size);
        }
        case DEVICE_VS_SERVICE_1: case DEVICE_VS_SERVICE_2: {
            unsigned side = control - DEVICE_VS_SERVICE_1;
            bool accepted = vs_set_service(side, true);
            if (accepted) runtime->service_pulses |= 1u << side;
            return message(runtime, accepted, accepted ? "Service button pressed for one frame"
                : "The loaded system or replay session rejected the service input", error, error_size);
        }
        default: return message(runtime, false, "Unknown device control", error, error_size);
    }
}
