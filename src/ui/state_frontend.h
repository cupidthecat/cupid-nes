/*
 * state_frontend.h - Save-state commands and frontend panel
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_STATE_FRONTEND_H
#define FRONTEND_STATE_FRONTEND_H

#include <stdbool.h>
#include <stddef.h>
#include "settings.h"
#include "../replay/movie.h"

enum {
    STATE_COMMAND_SAVE_SLOT = 0x1310,
    STATE_COMMAND_LOAD_SLOT,
    STATE_COMMAND_SAVE_FILE,
    STATE_COMMAND_LOAD_FILE,
    STATE_PANEL = 0x1310
};

typedef bool (*FrontendBeforeStateLoad)(void *context, char *error, size_t error_size);
typedef void (*FrontendAfterStateLoad)(void *context, bool loaded);
typedef bool (*FrontendBeforeStateSave)(void *context, const char *path,
                                       char *error, size_t error_size);
/* End the snapshot guard before writing the captured bytes to disk. */
typedef void (*FrontendAfterStateCapture)(void *context, bool captured);

typedef struct {
    FrontendSettings *settings;
    NesMovieSession *movie;
    const char *slot_directory;
    FrontendBeforeStateLoad before_load;
    FrontendAfterStateLoad after_load;
    FrontendBeforeStateSave before_save;
    FrontendAfterStateCapture after_capture;
    void *context;
    char status[160];
    char slot_text[24];
} FrontendStateRuntime;

void frontend_state_init(FrontendStateRuntime *runtime, FrontendSettings *settings,
                         const char *slot_directory);
void frontend_state_set_hooks(FrontendStateRuntime *runtime,
                              FrontendBeforeStateLoad before_load,
                              FrontendAfterStateLoad after_load,
                              void *context);
void frontend_state_set_save_hooks(FrontendStateRuntime *runtime,
                                   FrontendBeforeStateSave before_save,
                                   FrontendAfterStateCapture after_capture);
bool frontend_state_register_ui(FrontendStateRuntime *runtime);
void frontend_state_unregister_ui(void);

#endif
