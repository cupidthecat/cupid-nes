/*
 * idle_frontend.c - Startup Open and recent-image chooser
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "idle_frontend.h"
#include "platform_frontend.h"
#include <SDL2/SDL.h>
#include <stdio.h>
#include <string.h>

enum { IDLE_OPEN_BUTTON = 1000, IDLE_QUIT_BUTTON = 1001 };

static const char *display_name(const FrontendImageRequest *request) {
    if (request->archive_member[0]) return request->archive_member;
    const char *slash = strrchr(request->path, '/');
    const char *backslash = strrchr(request->path, '\\');
    const char *name = slash && backslash ? (slash > backslash ? slash : backslash)
                     : slash ? slash : backslash;
    return name ? name + 1 : request->path;
}

bool frontend_idle_choose_request(const FrontendSession *session,
                                  FrontendImageRequest *request,
                                  char *error, size_t error_size) {
    if (!request) return false;
    if (error && error_size) error[0] = '\0';
    bool initialized_video = (SDL_WasInit(SDL_INIT_VIDEO) & SDL_INIT_VIDEO) != 0;
    if (!initialized_video && SDL_InitSubSystem(SDL_INIT_VIDEO) != 0) {
        if (error && error_size)
            snprintf(error, error_size, "Could not start the desktop interface: %s", SDL_GetError());
        return false;
    }

    SDL_MessageBoxButtonData buttons[FRONTEND_RECENT_MAX + 2];
    char labels[FRONTEND_RECENT_MAX][192];
    size_t recent_count = frontend_session_recent_count(session);
    size_t button_count = 0;
    buttons[button_count++] = (SDL_MessageBoxButtonData){
        SDL_MESSAGEBOX_BUTTON_RETURNKEY_DEFAULT, IDLE_OPEN_BUTTON, "Open Game..."
    };
    for (size_t i = 0; i < recent_count; ++i) {
        const FrontendImageRequest *recent = frontend_session_recent(session, i);
        snprintf(labels[i], sizeof(labels[i]), "Recent: %.180s", display_name(recent));
        buttons[button_count++] = (SDL_MessageBoxButtonData){0, (int)i, labels[i]};
    }
    buttons[button_count++] = (SDL_MessageBoxButtonData){
        SDL_MESSAGEBOX_BUTTON_ESCAPEKEY_DEFAULT, IDLE_QUIT_BUTTON, "Quit"
    };
    SDL_MessageBoxData box = {
        .flags = SDL_MESSAGEBOX_INFORMATION,
        .window = NULL,
        .title = "Cupid NES",
        .message = "Open a game or choose a recent image.",
        .numbuttons = (int)button_count,
        .buttons = buttons,
        .colorScheme = NULL
    };
    int button = IDLE_QUIT_BUTTON;
    bool ok = SDL_ShowMessageBox(&box, &button) == 0;
    if (!initialized_video) SDL_QuitSubSystem(SDL_INIT_VIDEO);
    if (!ok) {
        if (error && error_size)
            snprintf(error, error_size, "Could not show the Open dialog: %s", SDL_GetError());
        return false;
    }
    if (button >= 0 && (size_t)button < recent_count) {
        *request = *frontend_session_recent(session, (size_t)button);
        return true;
    }
    if (button != IDLE_OPEN_BUTTON) return false;

    char path[FRONTEND_IMAGE_PATH_MAX];
    if (!frontend_open_image_dialog(path, sizeof(path), error, error_size)) return false;
    if (!frontend_image_request_init(request, path)) {
        if (error && error_size) snprintf(error, error_size, "The selected path is too long");
        return false;
    }
    return true;
}
