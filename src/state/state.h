/*
 * state.h - Versioned emulator save-state interface
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#ifndef CUPID_NES_STATE_H
#define CUPID_NES_STATE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { NES_STATE_SLOT_COUNT = 10 };

typedef enum {
    NES_STATE_OK = 0,
    NES_STATE_ERROR_ARGUMENT,
    NES_STATE_ERROR_NO_IMAGE,
    NES_STATE_ERROR_OUT_OF_MEMORY,
    NES_STATE_ERROR_IO,
    NES_STATE_ERROR_FORMAT,
    NES_STATE_ERROR_VERSION,
    NES_STATE_ERROR_INCOMPATIBLE,
    NES_STATE_ERROR_CORRUPT,
    NES_STATE_ERROR_UNSUPPORTED
} NesStateResult;

typedef struct {
    uint8_t *data;
    size_t size;
} NesStateBlob;

/* Capture owns a newly allocated blob on success. */
NesStateResult nes_state_capture(NesStateBlob *out);

/* Restore validates the complete image before changing the active machine. */
NesStateResult nes_state_restore(const void *data, size_t size);

typedef struct NesStateRestore NesStateRestore;

/* Prepare validates and owns a copy of all state data and allocates every
 * restore resource. Apply performs no allocation. Keep the same loaded image
 * and device configuration until the token is applied and released. These
 * operations belong to the emulation thread with host audio coordinated. */
NesStateResult nes_state_prepare_restore(const void *data, size_t size, NesStateRestore **out);
NesStateResult nes_state_apply_prepared(NesStateRestore *restore);
void nes_state_restore_free(NesStateRestore *restore);

void nes_state_blob_free(NesStateBlob *blob);

/* File saves replace an existing destination only after the complete write succeeds. */
NesStateResult nes_state_save_file(const char *path);
NesStateResult nes_state_load_file(const char *path);

/* Slots are numbered 0 through NES_STATE_SLOT_COUNT - 1 within directory. */
NesStateResult nes_state_slot_path(const char *directory, unsigned slot,
                                   char *path, size_t capacity);
NesStateResult nes_state_save_slot(const char *directory, unsigned slot);
NesStateResult nes_state_load_slot(const char *directory, unsigned slot);

const char *nes_state_result_string(NesStateResult result);

#ifdef __cplusplus
}
#endif
#endif
