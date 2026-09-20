/* blip_buf 1.1.0
 *
 * Copyright (C) 2003-2009 Shay Green.
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 */
#ifndef CUPID_BLIP_BUF_H
#define CUPID_BLIP_BUF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct blip_t blip_t;

typedef struct {
    uint64_t factor;
    uint64_t offset;
    int32_t available;
    int32_t size;
    int32_t integrator;
} BlipStateHeader;

blip_t *blip_new(int sample_count);
void blip_delete(blip_t *buffer);
void blip_set_rates(blip_t *buffer, double clock_rate, double sample_rate);
void blip_clear(blip_t *buffer);
void blip_add_delta(blip_t *buffer, unsigned clock_time, int delta);
void blip_end_frame(blip_t *buffer, unsigned clock_duration);
int blip_samples_avail(const blip_t *buffer);
int blip_read_samples(blip_t *buffer, short out[], int count, int stereo);
size_t blip_state_sample_count(const blip_t *buffer);
bool blip_state_export(const blip_t *buffer, BlipStateHeader *header,
                       int32_t *samples, size_t sample_count);
bool blip_state_import(blip_t *buffer, const BlipStateHeader *header,
                       const int32_t *samples, size_t sample_count);

#ifdef __cplusplus
}
#endif

#endif
