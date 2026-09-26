/*
 * presentation_tools.h - Shader, HD draft, and native audio panels
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_PRESENTATION_TOOLS_H
#define CUPID_PRESENTATION_TOOLS_H
#include "video_runtime.h"
#include "audio_runtime.h"
typedef struct FrontendPresentationTools FrontendPresentationTools;
/* changed persists existing settings plus the backend/shader runtime choices. */
FrontendPresentationTools *frontend_presentation_tools_create(FrontendVideoRuntime *video, FrontendAudioRuntime *audio,
                                                              void (*changed)(void *), void *context);
void frontend_presentation_tools_destroy(FrontendPresentationTools *tools);
bool frontend_presentation_tools_event(FrontendPresentationTools *tools, const SDL_Event *event);
void frontend_presentation_tools_tick(FrontendPresentationTools *tools);
/* Restore after video/audio initialization or an effective game-config change.
 * Failure reports a recoverable host-output error; ordinary video remains usable. */
bool frontend_presentation_restore_settings(FrontendVideoRuntime *video, FrontendAudioRuntime *audio,
                                            const FrontendSettings *settings, char *error, size_t size);
bool frontend_presentation_capture_settings(const FrontendVideoRuntime *video, const FrontendAudioRuntime *audio,
                                            FrontendSettings *settings);
#endif
