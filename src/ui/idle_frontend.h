/*
 * idle_frontend.h - Startup Open and recent-image chooser
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef IDLE_FRONTEND_H
#define IDLE_FRONTEND_H

#include "frontend_session.h"

bool frontend_idle_choose_request(const FrontendSession *session,
                                  FrontendImageRequest *request,
                                  char *error, size_t error_size);

#endif
