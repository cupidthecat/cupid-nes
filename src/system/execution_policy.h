/*
 * execution_policy.h - Host policies for deterministic execution modes
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_EXECUTION_POLICY_H
#define CUPID_EXECUTION_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    NES_EXECUTION_LIVE = 0,
    NES_EXECUTION_MOVIE_RECORDING = 1u << 0,
    NES_EXECUTION_MOVIE_PLAYBACK = 1u << 1,
    NES_EXECUTION_NETPLAY = 1u << 2,
    NES_EXECUTION_SPECULATIVE = 1u << 3,
    NES_EXECUTION_REWIND = 1u << 4
};

/* The application thread owns these host policies. They are not part of a
 * machine snapshot: restoring emulated state must not turn host writes on.
 * Nested operations save the previous mask and restore it when they finish. */
uint32_t nes_execution_policy(void);
bool nes_execution_set_policy(uint32_t policy);
bool nes_execution_allows_persistence(void);
bool nes_execution_allows_automatic_media(void);
bool nes_execution_allows_host_configuration(void);

#ifdef __cplusplus
}
#endif
#endif
