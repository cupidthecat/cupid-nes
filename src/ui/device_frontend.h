/*
 * device_frontend.h - Disk, cassette, barcode and arcade device controls
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_DEVICE_FRONTEND_H
#define CUPID_DEVICE_FRONTEND_H

#include "frontend_execution.h"

enum {
    FRONTEND_DEVICE_PATH_CAPACITY = 1024,
    DEVICE_PANEL_DISK = 0x1A00,
    DEVICE_PANEL_TAPE,
    DEVICE_PANEL_BARCODE,
    DEVICE_PANEL_VS,
    DEVICE_COMMAND_DISK_TOGGLE = 0x1A10,
    DEVICE_COMMAND_DISK_NEXT,
    DEVICE_COMMAND_TAPE_PLAY,
    DEVICE_COMMAND_TAPE_RECORD,
    DEVICE_COMMAND_TAPE_STOP,
    DEVICE_COMMAND_BARCODE_SCAN
};

typedef enum {
    DEVICE_DISK_SIDE = 1,
    DEVICE_DISK_TOGGLE,
    DEVICE_DISK_WRITE_PROTECT,
    DEVICE_DISK_AUTO_INSERT,
    DEVICE_DISK_FAST_FORWARD,
    DEVICE_DISK_STATUS,
    DEVICE_TAPE_INPUT = 20,
    DEVICE_TAPE_OUTPUT,
    DEVICE_TAPE_BROWSE_INPUT,
    DEVICE_TAPE_BROWSE_OUTPUT,
    DEVICE_TAPE_LOAD,
    DEVICE_TAPE_PLAY,
    DEVICE_TAPE_RECORD,
    DEVICE_TAPE_STOP,
    DEVICE_TAPE_STATUS,
    DEVICE_BARCODE_TEXT = 40,
    DEVICE_BARCODE_SCAN,
    DEVICE_VS_COIN_1 = 50,
    DEVICE_VS_COIN_2,
    DEVICE_VS_COIN_3,
    DEVICE_VS_COIN_4,
    DEVICE_VS_SERVICE_1,
    DEVICE_VS_SERVICE_2
} FrontendDeviceControl;

typedef struct {
    FrontendExecutionRuntime *execution;
    FrontendSettings *settings;
    size_t *disk_side;
    const char *const *protected_paths;
    size_t protected_path_count;
    char tape_input[FRONTEND_DEVICE_PATH_CAPACITY];
    char tape_output[FRONTEND_DEVICE_PATH_CAPACITY];
    char barcode[14];
    char status[256];
    char disk_status[256];
    char tape_status[256];
    char disk_side_text[32];
    bool tape_capture_pending;
    bool tape_record_selected;
    unsigned service_pulses;
} FrontendDeviceRuntime;

void frontend_devices_init(FrontendDeviceRuntime *runtime, FrontendExecutionRuntime *execution,
                            FrontendSettings *settings, size_t *disk_side);
bool frontend_devices_register(FrontendDeviceRuntime *runtime);
void frontend_devices_unregister(void);
void frontend_devices_refresh(FrontendDeviceRuntime *runtime);
/* Include input media, firmware and unrelated outputs. The tape recording
 * destination is owned by this runtime and must not reserve itself here. */
void frontend_devices_set_protected_paths(FrontendDeviceRuntime *runtime,
                                          const char *const *paths, size_t count);
/* Changes path preferences only. Loading a tape is an explicit, validated action. */
bool frontend_devices_set_tape_paths(FrontendDeviceRuntime *runtime,
                                     const char *input, const char *output,
                                     char *error, size_t error_size);
bool frontend_devices_tape_load(FrontendDeviceRuntime *runtime, const char *path,
                                char *error, size_t error_size);
bool frontend_devices_tape_play(FrontendDeviceRuntime *runtime, char *error, size_t error_size);
bool frontend_devices_tape_record(FrontendDeviceRuntime *runtime, char *error, size_t error_size);
bool frontend_devices_tape_stop(FrontendDeviceRuntime *runtime, char *error, size_t error_size);
/* Called before a machine replacement, state restore or normal quit. Failed
 * recording writes keep the captured signal and path available for retry. */
bool frontend_devices_finish(FrontendDeviceRuntime *runtime, char *error, size_t error_size);
bool frontend_devices_handle_tape_key(FrontendDeviceRuntime *runtime,
                                      const SDL_KeyboardEvent *event,
                                      char *error, size_t error_size);
/* Call after the authoritative frame has completed, outside movie/netplay's
 * in-progress frame scope, to release service-button pulses. */
void frontend_devices_frame_complete(FrontendDeviceRuntime *runtime);
/* Refresh host controls after image/state changes without submitting inputs. */
void frontend_devices_session_changed(FrontendDeviceRuntime *runtime);
bool frontend_devices_set_barcode(FrontendDeviceRuntime *runtime, const char *digits,
                                  char *error, size_t error_size);
bool frontend_devices_action(FrontendDeviceRuntime *runtime, unsigned control,
                              const char *value, int selected, char *error, size_t error_size);

#endif
