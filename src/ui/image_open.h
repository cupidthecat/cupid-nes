/*
 * image_open.h - Desktop image-source selection and activation
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_IMAGE_OPEN_H
#define FRONTEND_IMAGE_OPEN_H

#include "frontend_session.h"
#include "game_config.h"

typedef struct {
    GameConfigFrontend *configuration;
    FrontendSettings *effective;
    const FrontendSettings *rollback_settings;
    bool machine_changed;
    char cheat_path[1024];
    bool cheat_override;
} FrontendImageEnvironment;

bool frontend_image_open(void *userdata, const FrontendImageRequest *request, FrontendImageResult *result, char *error,
                         size_t error_size);

#endif
