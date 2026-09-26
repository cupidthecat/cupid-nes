/*
 * fcm_frontend.h - Legacy movie conversion controls
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_FCM_FRONTEND_H
#define CUPID_FCM_FRONTEND_H
#include "frontend_execution.h"

enum { FCM_CONVERTER_PANEL = 0x2800, FCM_CONVERTER_SOURCE = 1, FCM_CONVERTER_OUTPUT, FCM_CONVERTER_CONVERT };

bool fcm_frontend_register(FrontendExecutionRuntime *execution);
void fcm_frontend_unregister(void);
bool fcm_frontend_convert(FrontendExecutionRuntime *execution, const char *source, const char *output, char *error,
                          size_t capacity);
#endif
