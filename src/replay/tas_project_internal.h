/* Internal helpers shared by the TAS project implementation. */
#ifndef CUPID_TAS_PROJECT_INTERNAL_H
#define CUPID_TAS_PROJECT_INTERNAL_H

#include "tas_project.h"

enum {
    TAS_CHANGE_JOURNAL_CAPACITY = 256,
    TAS_DEFAULT_HISTORY_ENTRIES = 256
};

#define TAS_FM3_MODULE_BIT(kind) (1u << (unsigned)(kind))
#define TAS_FM3_ALL_MODULES ((1u << NES_FM3_PROJECT_MODULE_COUNT) - 1u)

typedef struct {
    size_t frame;
    char *note;
} TasMarker;

typedef struct {
    NesFm2Movie movie;
    uint8_t *lag;
    TasMarker *markers;
    size_t marker_count;
    size_t marker_capacity;
    char *intro_note;
} TasTimeline;

typedef struct {
    bool occupied;
    size_t key_frame;
    char *name;
    int parent_slot;
    TasTimeline timeline;
} TasBookmark;

typedef struct {
    TasTimeline timeline;
    TasBookmark bookmarks[NES_TAS_BOOKMARK_COUNT];
    int current_branch;
    uint8_t *selection;
} TasProjectState;

typedef struct {
    NesFm2Frame *frames;
    uint8_t *present;
    size_t span;
    size_t count;
} TasClipboard;

typedef struct TasHistoryEntry {
    TasProjectState state;
    size_t first_changed;
    size_t bytes;
    struct TasHistoryEntry *next;
} TasHistoryEntry;

typedef struct {
    uint64_t revision;
    size_t first_changed;
} TasJournalEntry;

struct NesTasProject {
    TasProjectState state;
    TasClipboard clipboard;

    TasHistoryEntry *undo;
    TasHistoryEntry *redo;
    size_t undo_count;
    size_t redo_count;
    size_t history_bytes;
    size_t max_history_entries;
    size_t max_history_bytes;

    uint64_t revision;
    TasJournalEntry journal[TAS_CHANGE_JOURNAL_CAPACITY];
    size_t journal_head;
    size_t journal_count;

    bool edit_active;
    bool edit_changed;
    size_t edit_first_changed;
    TasHistoryEntry *edit_history_entry;
    uint32_t edit_old_dirty_modules;

    bool source_project_present;
    uint32_t source_project_version;
    uint32_t source_saved_modules;
    NesFm3ProjectModule source_modules[NES_FM3_PROJECT_MODULE_COUNT];
    uint32_t fm3_dirty_modules;
};

char *tas_strdup_limit(const char *text, size_t max_length);
bool tas_frame_equal(const NesFm2Frame *left, const NesFm2Frame *right);

void tas_timeline_init(TasTimeline *timeline);
void tas_timeline_free(TasTimeline *timeline);
NesTasResult tas_timeline_from_movie(TasTimeline *timeline, const NesFm2Movie *movie);
NesTasResult tas_timeline_clone(const TasTimeline *source, TasTimeline *out);

void tas_bookmark_init(TasBookmark *bookmark);
void tas_bookmark_free(TasBookmark *bookmark);
NesTasResult tas_bookmark_clone(const TasBookmark *source, TasBookmark *out);

void tas_state_init(TasProjectState *state);
void tas_state_free(TasProjectState *state);
NesTasResult tas_state_clone(const TasProjectState *source, TasProjectState *out);
size_t tas_state_estimated_bytes(const TasProjectState *state);

void tas_clipboard_free(TasClipboard *clipboard);
NesTasResult tas_clipboard_clone(const TasClipboard *source, TasClipboard *out);

void tas_project_mark_change(NesTasProject *project, size_t first_changed,
                             uint32_t dirty_modules);
void tas_project_mark_dirty(NesTasProject *project, uint32_t dirty_modules);
NesTasResult tas_project_auto_begin(NesTasProject *project, bool *owned);
NesTasResult tas_project_auto_finish(NesTasProject *project, bool owned,
                                     NesTasResult result);

NesTasResult tas_project_resize_insert(NesTasProject *project, size_t first, size_t count);
NesTasResult tas_project_resize_delete(NesTasProject *project, size_t first, size_t count);

/* Allocation fault injection used only by the focused model regression binary. */
void tas_project_test_fail_alloc_after(size_t successful_allocations);
void tas_project_test_reset_alloc_fail(void);

NesTasResult tas_import_fm3_metadata(NesTasProject *project,
                                     const NesFm2Movie *source);
NesTasResult tas_project_write_fm3(const NesTasProject *project, uint8_t *destination,
                                   size_t capacity, size_t *written,
                                   NesFm2Diagnostic *diagnostic, bool measure_only,
                                   size_t *measured);

#endif
