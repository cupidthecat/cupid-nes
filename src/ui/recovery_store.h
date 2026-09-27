/*
 * recovery_store.h - Atomic automatic-session storage
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_RECOVERY_STORE_H
#define CUPID_RECOVERY_STORE_H
#include "frontend_session.h"
#include "../state/state.h"
#include <stdint.h>
/* Content identity includes the selected archive member and patched bytes. */
bool recovery_game_key(const FrontendSession *session, char key[41]);
bool recovery_path(const char *directory, const char *key, const char *suffix, char *path, size_t capacity);
bool recovery_save_session(const char *directory, const FrontendSession *session, const NesStateBlob *state,
                           char *error, size_t error_size);
bool recovery_read_session(const char *directory, FrontendImageRequest *request, NesStateBlob *state, char *error,
                           size_t error_size);
#endif
