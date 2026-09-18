/*
 * unif.h - UNIF board-name resolution shared by loaders and the game database
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#ifndef CUPID_UNIF_H
#define CUPID_UNIF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    UNIF_BOARD_UNKNOWN = 32768,
    UNIF_BOARD_MALEE,
    UNIF_BOARD_GS2013,
    UNIF_BOARD_GHOSTBUSTERS_63IN1,
    UNIF_BOARD_SUPER24IN1_SC03,
    UNIF_BOARD_CC21,
    UNIF_BOARD_AC08,
    UNIF_BOARD_PUZZLE,
    UNIF_BOARD_FK23C,
    UNIF_BOARD_FK23CA,
    UNIF_BOARD_255IN1,
    UNIF_BOARD_VRC7,
    UNIF_BOARD_8237A,
    UNIF_BOARD_SSS_NROM_256
};

int32_t unif_board_mapper_id(const char *board_name);

#ifdef __cplusplus
}
#endif

#endif
