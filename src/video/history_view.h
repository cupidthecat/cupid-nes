/*
 * history_view.h - Non-destructive inspection of authoritative rewind snapshots
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_HISTORY_VIEW_H
#define CUPID_HISTORY_VIEW_H
#include "../replay/rewind.h"
#ifdef __cplusplus
extern "C" {
#endif
/* All calls run on the emulation thread with host audio locked. Index zero is
 * the oldest retained boundary. Preview always restores the live machine. */
NesReplayResult nes_history_preview(const NesRewindHistory *history, size_t index, uint32_t *pixels, size_t capacity,
                                    unsigned *width, NesStateResult *state);
NesReplayResult nes_history_resume(NesRewindHistory *history, size_t index, NesStateResult *state);
NesStateResult nes_history_save(const NesRewindHistory *history, size_t index, const char *path);
#ifdef __cplusplus
}
#endif
#endif
