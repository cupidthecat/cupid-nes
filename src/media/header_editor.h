/*
 * header_editor.h - Cartridge header editing
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_HEADER_EDITOR_H
#define CUPID_HEADER_EDITOR_H
#include "../rom/rom.h"
#include <stdint.h>
struct FrontendExecutionRuntime;
typedef struct HeaderEditor HeaderEditor;

typedef enum {
    HEADER_FORMAT,
    HEADER_PRG,
    HEADER_CHR,
    HEADER_MAPPER,
    HEADER_SUBMAPPER,
    HEADER_MIRROR,
    HEADER_BATTERY,
    HEADER_TRAINER,
    HEADER_PRG_RAM,
    HEADER_PRG_NVRAM,
    HEADER_CHR_RAM,
    HEADER_CHR_NVRAM,
    HEADER_CONSOLE,
    HEADER_TIMING,
    HEADER_VS_PPU,
    HEADER_VS_HARDWARE,
    HEADER_EXTENDED_CONSOLE,
    HEADER_MISC_ROMS,
    HEADER_INPUT,
    HEADER_LEGACY_RAM,
    HEADER_FIELD_COUNT
} HeaderEditorField;

typedef struct {
    uint64_t value[HEADER_FIELD_COUNT];
} HeaderEditorMetadata;

extern const char *const header_editor_labels[HEADER_FIELD_COUNT];
HeaderEditor *header_editor_create(void);
void header_editor_destroy(HeaderEditor *editor);
bool header_editor_open(HeaderEditor *editor, const char *path, char *error, size_t size);
bool header_editor_metadata(const HeaderEditor *editor, HeaderEditorMetadata *metadata);
/* Invalid drafts are retained by the caller; validation never changes the machine. */
bool header_editor_encode(const HeaderEditorMetadata *metadata, iNESHeader *header, char *error, size_t size);
bool header_editor_validate(const HeaderEditor *editor, const HeaderEditorMetadata *metadata, iNESHeader *header,
                            char *error, size_t size);
bool header_editor_save_copy(const HeaderEditor *editor, const HeaderEditorMetadata *metadata, const char *path,
                             const struct FrontendExecutionRuntime *execution, char *error, size_t size);
#endif
