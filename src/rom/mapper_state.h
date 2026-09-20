/*
 * mapper_state.h - Transactional cartridge mapper save-state service
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
#ifndef CUPID_MAPPER_STATE_H
#define CUPID_MAPPER_STATE_H

#include "../state/state.h"
#include "../state/state_io.h"

typedef struct MapperStateRestore MapperStateRestore;

bool mapper_state_capture(NesStateWriter *writer);
bool mapper_hardware_state_capture(NesStateWriter *writer);
NesStateResult mapper_state_prepare(NesStateReader *reader, MapperStateRestore **out_restore);
void mapper_state_apply_prepared(MapperStateRestore *restore);
void mapper_state_restore_free(MapperStateRestore *restore);

#endif
