/*
 * fcm.h - Legacy movie conversion
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_REPLAY_FCM_H
#define CUPID_REPLAY_FCM_H
#include "fm2.h"

/* Convert supported version-two, power-on recordings. On error out is intact.
 * Conversion preserves recorded events; it does not establish game sync. */
NesFm2Result nes_fcm_convert(const uint8_t *data, size_t size, const NesFm2Limits *limits, NesFm2Movie *out,
                             NesFm2Diagnostic *diagnostic);
#endif
