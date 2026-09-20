/*
 * epsm.h - EPSM expansion sound hardware
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator, licensed under the GNU General
 * Public License, version 3 or any later version.
 */
#ifndef CUPID_EPSM_H
#define CUPID_EPSM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "../state/state.h"
#include "../state/state_io.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { EPSM_ADPCM_ROM_SIZE = 0x2000, EPSM_CLOCK_RATE = 8000000 };
typedef struct EpsmDevice EpsmDevice;

// Firmware changes affect the next prepared device, preserving the active one.
bool epsm_set_adpcm_rom(const uint8_t *data, size_t size);
bool epsm_load_adpcm_file(const char *path);
EpsmDevice *epsm_create(void);
void epsm_destroy(EpsmDevice *device);
void epsm_activate(EpsmDevice *device);
bool epsm_enabled(void);
bool epsm_has_adpcm_rom(void);

void epsm_power_on(void);
void epsm_clear_irq_source(void);
void epsm_clock_master(unsigned clocks, uint32_t master_hz);
uint64_t epsm_clock_count(void);
bool epsm_irq_pending(void);
void epsm_write_4016(uint8_t data_bus, uint8_t out_pins);
void epsm_write_port(uint16_t address, uint8_t value);
void epsm_sample_stereo(float *left, float *right);
bool epsm_state_capture(NesStateWriter *writer);
bool epsm_state_validate(NesStateReader *reader);
bool epsm_state_apply(NesStateReader *reader);

typedef struct EpsmStateRestore EpsmStateRestore;
NesStateResult epsm_state_prepare(NesStateReader *reader, EpsmStateRestore **out_restore);
void epsm_state_apply_prepared(EpsmStateRestore *restore);
void epsm_state_restore_free(EpsmStateRestore *restore);

#ifdef __cplusplus
}
#endif
#endif
