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
#include "replay_memory.h"

typedef struct CartridgeBoard CartridgeBoard;

enum { BOARD_FCNS_MAPPER_ID = 65532, BOARD_STUDYBOX_MAPPER_ID = 65533 };

bool board_handles_mapper(unsigned mapper);
bool board_is_fcns_header(const iNESHeader *header);
bool board_handles_header(const iNESHeader *header);
CartridgeBoard *board_create(const iNESHeader *header, uint8_t *prg, size_t prg_bytes,
                             uint8_t *chr, size_t chr_bytes);
CartridgeBoard *board_create_with_metadata(const iNESHeader *header,
                                           uint8_t *prg, size_t prg_bytes,
                                           uint8_t *chr, size_t chr_bytes,
                                           const RomDatabaseInfo *database);
CartridgeBoard *board_create_studybox(const uint8_t *bios, size_t bios_size,
                                      const uint8_t *media, size_t media_size);
bool board_set_fcns_kanji_firmware(const uint8_t *data, size_t size);
void board_destroy(CartridgeBoard *board);
uint8_t *board_cpu_ram_8k(CartridgeBoard *board);
uint8_t board_cpu_read(CartridgeBoard *board, uint16_t address, uint8_t open_bus);
uint8_t board_cpu_peek(const CartridgeBoard *board, uint16_t address, uint8_t open_bus);
void board_cpu_write(CartridgeBoard *board, uint16_t address, uint8_t value);
bool board_read_cpu_register(CartridgeBoard *board, uint16_t address, uint8_t *value);
void board_observe_cpu_write(CartridgeBoard *board, uint16_t address, uint8_t value);
uint8_t board_ppu_read(CartridgeBoard *board, uint16_t address, unsigned fetch_source);
uint8_t board_ppu_peek(const CartridgeBoard *board, uint16_t address);
bool board_debug_write_ppu(CartridgeBoard *board, uint16_t address, uint8_t value);
void board_ppu_write(CartridgeBoard *board, uint16_t address, uint8_t value);
void board_clock_cpu(CartridgeBoard *board, bool write_cycle);
void board_notify_ppu_address(CartridgeBoard *board, uint16_t address, uint64_t cycle);
void board_reset(CartridgeBoard *board, bool soft_reset);
void board_after_reset(CartridgeBoard *board);
bool board_irq_pending(const CartridgeBoard *board);
void board_irq_ack(CartridgeBoard *board);
float board_audio(const CartridgeBoard *board);
unsigned board_audio_mix_channel(const CartridgeBoard *board);
bool board_set_mapper_input(CartridgeBoard *board, unsigned input, bool pressed);
Mirroring board_mirroring(const CartridgeBoard *board);
void board_set_mirroring(CartridgeBoard *board, Mirroring mirroring);
void board_apply_trainer(CartridgeBoard *board, const uint8_t trainer[512]);
void board_battery_configure(CartridgeBoard *board, const char *rom_path);
bool board_battery_flush(CartridgeBoard *board);
void board_replay_initialize_memory(CartridgeBoard *board, CartReplayMemoryInitializer initialize,
                                     void *context);
size_t board_replay_save_ram_size(const CartridgeBoard *board);
bool board_replay_set_save_ram(CartridgeBoard *board, const uint8_t *bytes, size_t size);

#ifdef __cplusplus
}
#endif
#endif
