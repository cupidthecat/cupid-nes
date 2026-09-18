/*
 * board.h - Cartridge board interface
 *
 * Author: @frankischilling
 *
 * Boards share the production CPU and PPU buses while owning their register,
 * interrupt, and cartridge RAM state. ROM buffers remain owned by the loader.
 *
 * This file is part of Cupid NES Emulator.
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#ifndef CUPID_CARTRIDGE_BOARD_H
#define CUPID_CARTRIDGE_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif
#include "rom.h"

typedef struct CartridgeBoard CartridgeBoard;

bool board_handles_mapper(unsigned mapper);
CartridgeBoard *board_create(const iNESHeader *header, uint8_t *prg, size_t prg_bytes,
                             uint8_t *chr, size_t chr_bytes);
void board_destroy(CartridgeBoard *board);
uint8_t board_cpu_read(CartridgeBoard *board, uint16_t address, uint8_t open_bus);
void board_cpu_write(CartridgeBoard *board, uint16_t address, uint8_t value);
uint8_t board_ppu_read(CartridgeBoard *board, uint16_t address, unsigned fetch_source);
void board_ppu_write(CartridgeBoard *board, uint16_t address, uint8_t value);
void board_clock_cpu(CartridgeBoard *board, bool write_cycle);
void board_notify_ppu_address(CartridgeBoard *board, uint16_t address, uint64_t cycle);
void board_reset(CartridgeBoard *board, bool soft_reset);
void board_after_reset(CartridgeBoard *board);
bool board_irq_pending(const CartridgeBoard *board);
void board_irq_ack(CartridgeBoard *board);
float board_audio(const CartridgeBoard *board);
Mirroring board_mirroring(const CartridgeBoard *board);
void board_set_mirroring(CartridgeBoard *board, Mirroring mirroring);
void board_apply_trainer(CartridgeBoard *board, const uint8_t trainer[512]);
void board_battery_configure(CartridgeBoard *board, const char *rom_path);
void board_battery_flush(CartridgeBoard *board);

#ifdef __cplusplus
}
#endif
#endif
