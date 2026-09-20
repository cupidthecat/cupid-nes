/*
 * execution_policy.c - Persistence and automatic-media permissions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "execution_policy.h"

static uint32_t active_policy;

uint32_t nes_execution_policy(void) {
    return active_policy;
}

bool nes_execution_set_policy(uint32_t policy) {
    const uint32_t known = NES_EXECUTION_MOVIE_RECORDING | NES_EXECUTION_MOVIE_PLAYBACK
                         | NES_EXECUTION_NETPLAY | NES_EXECUTION_SPECULATIVE | NES_EXECUTION_REWIND;
    if (policy & ~known) return false;
    active_policy = policy;
    return true;
}

bool nes_execution_allows_persistence(void) {
    const uint32_t isolated = NES_EXECUTION_MOVIE_RECORDING | NES_EXECUTION_MOVIE_PLAYBACK | NES_EXECUTION_NETPLAY
                            | NES_EXECUTION_SPECULATIVE | NES_EXECUTION_REWIND;
    return (active_policy & isolated) == 0;
}

bool nes_execution_allows_automatic_media(void) {
    return active_policy == NES_EXECUTION_LIVE;
}

bool nes_execution_allows_host_configuration(void) {
    return active_policy == NES_EXECUTION_LIVE;
}
