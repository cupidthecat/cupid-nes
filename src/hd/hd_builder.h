/*
 * hd_builder.h - Editable format-109 pack drafts
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_HD_BUILDER_H
#define CUPID_HD_BUILDER_H
#include "hd_runtime.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct NesHdBuilder NesHdBuilder;
NesHdBuilder *nes_hd_builder_create(void);
void nes_hd_builder_destroy(NesHdBuilder *builder);
bool nes_hd_builder_open(NesHdBuilder *builder, const char *path, char *error, size_t size);
bool nes_hd_builder_new(NesHdBuilder *builder, unsigned scale, char *error, size_t size);
size_t nes_hd_builder_line_count(const NesHdBuilder *builder);
const char *nes_hd_builder_line(const NesHdBuilder *builder, size_t index);
/* index == count appends. NULL deletes an existing line. Draft edits do not
 * replace the last validated preview until validation succeeds. */
bool nes_hd_builder_set_line(NesHdBuilder *builder, size_t index, const char *line, char *error, size_t size);
bool nes_hd_builder_import_asset(NesHdBuilder *builder, const char *member, const char *path, char *error, size_t size);
bool nes_hd_builder_validate(NesHdBuilder *builder, char *error, size_t size);
bool nes_hd_builder_export(NesHdBuilder *builder, const char *path, char *error, size_t size);
bool nes_hd_builder_set_game(NesHdBuilder *builder, const NesHdGameInfo *game, char *error, size_t size);
/* Release game and rendered-frame storage while keeping the editable draft. */
void nes_hd_builder_clear_game(NesHdBuilder *builder);
bool nes_hd_builder_set_protected_paths(NesHdBuilder *builder, const char *const *paths, size_t count);
bool nes_hd_builder_preview(NesHdBuilder *builder, const NesHdFrameSource *source, NesHdMemoryReader memory,
                            void *context, NesHdFrame *frame, char *error, size_t size);
#ifdef __cplusplus
}
#endif
#endif
