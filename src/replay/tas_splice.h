/*
 * tas_splice.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Transactional TAS range splicing. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_TAS_SPLICE_H
#define CUPID_TAS_SPLICE_H

#include "tas_project.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NES_TAS_SPLICE_APPEND = 0,
    NES_TAS_SPLICE_INSERT,
    NES_TAS_SPLICE_REPLACE,
    NES_TAS_SPLICE_EXTRACT
} NesTasSpliceMode;

typedef struct {
    NesTasSpliceMode mode;
    size_t source_first;
    size_t source_count;
    size_t destination_first;
    size_t destination_count;
    bool copy_markers;
    /* Commands supported by the destination hardware; bit 7 is always invalid. */
    uint8_t allowed_commands;
} NesTasSpliceOptions;

typedef struct {
    size_t source_frames;
    size_t destination_frames;
    size_t first;
    size_t removed;
    size_t inserted;
    size_t resulting_frames;
    size_t resulting_markers;
    size_t imported_markers;
} NesTasSplicePreview;

/* Ranges are zero-based [first, first + count). Append uses the destination
 * end. Insert removes nothing. Replace removes destination_count frames.
 * Extract replaces the entire destination input timeline with the source range;
 * it retains the destination startup profile, not a state at source_first.
 * Surviving destination markers stay bound to their input. Markers inside the
 * removed range are deleted; copy_markers imports source markers in the range.
 * Destination navigation bookmarks follow ordinary delete/insert boundaries.
 * Branch snapshots and the clipboard remain owned by the destination.
 * Source and destination may be the same project. Active grouped edits are
 * rejected. Failure preserves both projects and the previous preview output. */
NesTasResult nes_tas_splice_preview(const NesTasProject *destination, const NesTasProject *source,
                                    const NesTasSpliceOptions *options, NesTasSplicePreview *out, char *error,
                                    size_t capacity);
NesTasResult nes_tas_splice_apply(NesTasProject *destination, const NesTasProject *source,
                                  const NesTasSpliceOptions *options, char *error, size_t capacity);

/* Load an independent source without opening a session or changing the machine.
 * As with normal movie opening, FM2/FM3 files without a cheat header inherit
 * the current cheat configuration. Existing headers and CTAS profiles stay intact.
 * The caller owns the new project on success; failure retains *out. */
NesTasResult nes_tas_splice_load(const char *path, NesTasProject **out, char *error, size_t capacity);

#ifdef __cplusplus
}
#endif
#endif
