/*
 * debug_catalog.h - Image symbols, source mapping and static references
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_DEBUG_CATALOG_H
#define CUPID_DEBUG_CATALOG_H
#include "debug_analysis.h"

enum { DEBUG_SYMBOL_CAP = 8192, DEBUG_SOURCE_LIMIT = 4 * 1024 * 1024 };

typedef struct {
    uint64_t key;
    uint16_t address;
    uint32_t size, line;
    bool function;
    char name[64], file[256], comment[192];
} DebugSymbol;

typedef struct {
    uint64_t key;
    uint16_t address, target;
    uint8_t opcode, length;
    bool constant, indirect;
    char instruction[64];
} DebugReference;

void debug_catalog_clear(void);
void debug_catalog_image_changed(void);
size_t debug_symbol_count(void);
bool debug_symbol_at(size_t index, DebugSymbol *out);
bool debug_symbol_set(const DebugSymbol *symbol);
bool debug_symbol_remove(uint64_t key);
bool debug_symbol_find(uint64_t key, DebugSymbol *out);
bool debug_symbol_resolve(const char *name, uint16_t *address, uint64_t *key);
bool debug_symbol_import(const char *path, char *error, size_t size);
bool debug_symbol_export(const char *path);
/* Source paths in symbols are relative to the user-selected root. */
bool debug_source_line(const char *root, const DebugSymbol *symbol, char *out, size_t capacity, char *error,
                       size_t error_size);
uint32_t debug_source_breakpoint(const DebugSymbol *symbol);
/* Static linear decode; current mapping for physical_start == DEBUG_KEY_NONE,
 * otherwise a physical PRG slice mapped by the caller at first. Results do not
 * imply execution; indexed/indirect operands are base/pointer references. */
size_t debug_references(uint16_t first, uint16_t last, uint64_t physical_start, uint16_t target, bool constants,
                        DebugReference *out, size_t capacity);
#endif
