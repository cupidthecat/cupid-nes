/*
 * tas_project.h - Editable tool-assisted movie project model
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_TAS_PROJECT_H
#define CUPID_TAS_PROJECT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "fm2.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    NES_TAS_BOOKMARK_COUNT = 10,
    NES_TAS_MARKER_NOTE_MAX = 99,
    NES_TAS_BOOKMARK_NAME_MAX = 63,
    NES_CTAS_MAGIC_SIZE = 8,
    NES_CTAS_VERSION = 1
};

#define NES_CTAS_MAGIC "CUPIDTAS"
#define NES_TAS_DEFAULT_HISTORY_BYTES ((size_t)64u * 1024u * 1024u)

typedef enum {
    NES_TAS_OK = 0,
    NES_TAS_INVALID_ARGUMENT,
    NES_TAS_OUT_OF_MEMORY,
    NES_TAS_RANGE_ERROR,
    NES_TAS_EMPTY_SELECTION,
    NES_TAS_EMPTY_CLIPBOARD,
    NES_TAS_EMPTY_BOOKMARK,
    NES_TAS_FORMAT_ERROR,
    NES_TAS_IO_ERROR,
    NES_TAS_UNSUPPORTED
} NesTasResult;

typedef enum {
    NES_TAS_LAG_UNKNOWN = 0,
    NES_TAS_LAG_NO,
    NES_TAS_LAG_YES
} NesTasLagState;

typedef enum {
    NES_TAS_PASTE_OVERWRITE = 0,
    NES_TAS_PASTE_INSERT
} NesTasPasteMode;

typedef struct {
    uint64_t revision;
    size_t first_changed_frame;
} NesTasChange;

typedef struct {
    size_t frame;
    const char *note;
} NesTasMarkerView;

typedef struct {
    bool occupied;
    size_t key_frame;
    const char *name;
    int parent_slot;
} NesTasBookmarkView;

typedef struct NesTasProject NesTasProject;

/* Creation takes an immutable source and deep-copies all movie-owned data. */
NesTasProject *nes_tas_project_create(const NesFm2Movie *movie);
/* Reports invalid metadata separately from allocation failure. Failure retains
 * the caller's previous output pointer. */
NesTasResult nes_tas_project_create_checked(const NesFm2Movie *movie, NesTasProject **out);
void nes_tas_project_destroy(NesTasProject *project);

const NesFm2Movie *nes_tas_project_movie(const NesTasProject *project);
size_t nes_tas_project_frame_count(const NesTasProject *project);
const NesFm2Frame *nes_tas_project_frame(const NesTasProject *project, size_t frame);

uint64_t nes_tas_project_revision(const NesTasProject *project);
/* Returns the minimum input frame changed after revision. SIZE_MAX means no
 * emulation-affecting input changed; a revision older than the retained journal
 * conservatively returns frame 0. */
NesTasChange nes_tas_project_change_since(const NesTasProject *project, uint64_t revision);

/* Group edits that belong to one user gesture or recording take. begin captures
 * one rollback/undo state. end commits all successful edits as one history item;
 * cancel restores movie input, markers, selection, lag, bookmarks and rerecord
 * count exactly to their values at begin. Nested edits are rejected. */
bool nes_tas_edit_active(const NesTasProject *project);
NesTasResult nes_tas_edit_begin(NesTasProject *project);
NesTasResult nes_tas_edit_end(NesTasProject *project);
void nes_tas_edit_cancel(NesTasProject *project);

void nes_tas_project_set_history_limit(NesTasProject *project, size_t max_entries,
                                       size_t max_bytes);
bool nes_tas_project_can_undo(const NesTasProject *project);
bool nes_tas_project_can_redo(const NesTasProject *project);
NesTasResult nes_tas_project_undo(NesTasProject *project);
NesTasResult nes_tas_project_redo(NesTasProject *project);

void nes_tas_selection_clear(NesTasProject *project);
NesTasResult nes_tas_selection_set(NesTasProject *project, size_t frame, bool selected);
NesTasResult nes_tas_selection_set_range(NesTasProject *project, size_t first, size_t last,
                                         bool selected);
bool nes_tas_selection_contains(const NesTasProject *project, size_t frame);
size_t nes_tas_selection_count(const NesTasProject *project);
bool nes_tas_selection_bounds(const NesTasProject *project, size_t *first, size_t *last);

NesTasResult nes_tas_copy_selection(NesTasProject *project);
NesTasResult nes_tas_cut_selection(NesTasProject *project);
NesTasResult nes_tas_paste(NesTasProject *project, size_t first, NesTasPasteMode mode);
bool nes_tas_clipboard_available(const NesTasProject *project);

NesTasResult nes_tas_insert_frames(NesTasProject *project, size_t first, size_t count);
NesTasResult nes_tas_delete_frames(NesTasProject *project, size_t first, size_t count);
NesTasResult nes_tas_clone_frames(NesTasProject *project, size_t first, size_t count);
NesTasResult nes_tas_delete_selection(NesTasProject *project);
NesTasResult nes_tas_clone_selection(NesTasProject *project);
NesTasResult nes_tas_truncate(NesTasProject *project, size_t frame_count);

NesTasResult nes_tas_set_frame(NesTasProject *project, size_t frame,
                               const NesFm2Frame *input);
/* Record one complete frame atomically. frame may equal the current frame count
 * to append. With insert=true, frame may be anywhere from 0 through frame_count
 * and the old frame at that index shifts forward. Allocation failure leaves the
 * input timeline untouched. This can be called repeatedly inside one grouped
 * edit so a recording take produces one undo item. */
NesTasResult nes_tas_record_frame(NesTasProject *project, size_t frame,
                                  const NesFm2Frame *input, bool insert);
NesTasResult nes_tas_paint_button(NesTasProject *project, size_t first, size_t last,
                                  unsigned controller, uint8_t button_mask, bool pressed);
NesTasResult nes_tas_apply_pattern(NesTasProject *project, size_t first, size_t last,
                                   unsigned controller, uint8_t button_mask,
                                   const uint8_t *pattern, size_t pattern_length,
                                   bool skip_known_lag);

NesTasLagState nes_tas_lag(const NesTasProject *project, size_t frame);
NesTasResult nes_tas_set_lag(NesTasProject *project, size_t frame, NesTasLagState state);
void nes_tas_invalidate_lag(NesTasProject *project, size_t first);

size_t nes_tas_marker_count(const NesTasProject *project);
bool nes_tas_marker(const NesTasProject *project, size_t index, NesTasMarkerView *out);
NesTasResult nes_tas_marker_set(NesTasProject *project, size_t frame, const char *note);
NesTasResult nes_tas_marker_remove(NesTasProject *project, size_t frame);
NesTasResult nes_tas_marker_move(NesTasProject *project, size_t from_frame, size_t to_frame);
const char *nes_tas_intro_note(const NesTasProject *project);
NesTasResult nes_tas_set_intro_note(NesTasProject *project, const char *note);

bool nes_tas_bookmark(const NesTasProject *project, unsigned slot, NesTasBookmarkView *out);
NesTasResult nes_tas_bookmark_set(NesTasProject *project, unsigned slot, size_t key_frame,
                                  const char *name);
NesTasResult nes_tas_bookmark_clear(NesTasProject *project, unsigned slot);
NesTasResult nes_tas_bookmark_deploy(NesTasProject *project, unsigned slot);
int nes_tas_current_branch(const NesTasProject *project);

uint64_t nes_tas_rerecord_count(const NesTasProject *project);
NesTasResult nes_tas_increment_rerecord(NesTasProject *project);

NesTasResult nes_tas_project_save(const NesTasProject *project, const char *path);
NesTasResult nes_tas_project_load(const char *path, NesTasProject **out);

/* CTAS files start with exactly eight ASCII bytes "CUPIDTAS", followed by a
 * little-endian uint32 version (currently 1). CTAS retains editable project
 * history. FM3 export rebuilds safe editable modules and preserves untouched
 * source modules byte-for-byte; stale machine-state modules are emitted using
 * their standard skip records instead of foreign executable state. */
NesTasResult nes_tas_project_measure_fm3(const NesTasProject *project, size_t *size,
                                         NesFm2Diagnostic *diagnostic);
NesTasResult nes_tas_project_export_fm3(const NesTasProject *project, uint8_t *destination,
                                        size_t capacity, size_t *written,
                                        NesFm2Diagnostic *diagnostic);

const char *nes_tas_result_string(NesTasResult result);

#ifdef __cplusplus
}
#endif
#endif
