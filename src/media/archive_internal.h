/*
 * archive_internal.h - In-memory archive reader services
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_ARCHIVE_INTERNAL_H
#define CUPID_ARCHIVE_INTERNAL_H

#include "image_source.h"

typedef struct {
    size_t used;
    size_t limit;
    bool exhausted;
    bool allocation_failed;
} ArchiveBudget;

typedef struct {
    NesArchiveList list;
    unsigned *indices;
    void *backend;
    NesMediaResult (*extract)(void *backend, unsigned index, uint8_t **data, size_t *size);
    void (*close)(void *backend);
} ArchiveReader;

void *archive_allocate(ArchiveBudget *budget, size_t size);
void *archive_reallocate(ArchiveBudget *budget, void *address, size_t size);
void archive_deallocate(ArchiveBudget *budget, void *address);
NesMediaResult archive_budget_result(const ArchiveBudget *budget);
bool archive_supported_name(const char *name);
bool archive_valid_utf8(const char *name);
char *archive_utf16_name(const uint16_t *name, size_t units);
char *archive_cp437_name(const uint8_t *name, size_t size);
NesMediaResult archive_add_entry(ArchiveReader *reader, const char *name, size_t size, unsigned index);
NesMediaResult archive_open_zip(const uint8_t *data, size_t size, ArchiveReader *reader);
NesMediaResult archive_open_7z(const uint8_t *data, size_t size, ArchiveReader *reader);
NesMediaResult archive_open(const uint8_t *data, size_t size, ArchiveReader *reader);
void archive_close(ArchiveReader *reader);

#endif
