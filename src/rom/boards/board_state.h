/*
 * board_state.h - Transactional cartridge board save-state service
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
#ifndef CUPID_BOARD_STATE_H
#define CUPID_BOARD_STATE_H

#include "../board.h"
#include "../../state/state.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "../../state/state_io.h"

typedef struct BoardStateRestore BoardStateRestore;

NesStateResult board_state_capture(const CartridgeBoard *board, NesStateWriter *writer);
NesStateResult board_state_validate(const CartridgeBoard *board, NesStateReader *reader,
                                    BoardStateRestore **out_restore);
void board_state_apply(CartridgeBoard *board, const BoardStateRestore *restore);
void board_state_restore_free(BoardStateRestore *restore);

#ifdef __cplusplus
}
#endif

#endif
