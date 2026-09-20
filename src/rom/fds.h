/*
 * fds.h - Famicom Disk System device interface
 *
 * Author: @frankischilling
 *
 * This file defines the disk-system image, controller, memory, IRQ, audio, media, and
 * persistence interfaces used by the cartridge layer and production loader.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef FDS_H
#define FDS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "rom.h"
#include "../state/state_io.h"

typedef struct FdsImage FdsImage;

typedef struct {
    bool insert_automatically;
    bool fast_forward_loading;
} FdsAutomationOptions;

FdsImage *fds_image_create(const uint8_t *disk, size_t disk_size,
                           const uint8_t *bios, size_t bios_size,
                           const char *disk_path, bool write_protected);
FdsImage *fds_image_create_options(const uint8_t *disk, size_t disk_size,
                                   const uint8_t *bios, size_t bios_size,
                                   const char *disk_path, const FdsLoadOptions *options);
void fds_image_destroy(FdsImage *image);

// Takes ownership of a prepared image. Preparation is fallible; activation is not.
void fds_activate(FdsImage *image);
void fds_shutdown(void);
bool fds_active(void);

uint8_t fds_cpu_read_bus(uint16_t addr, uint8_t open_bus);
uint8_t fds_cpu_peek_bus(uint16_t addr, uint8_t open_bus);
void fds_cpu_write(uint16_t addr, uint8_t value);
uint8_t fds_ppu_read(uint16_t addr);
void fds_ppu_write(uint16_t addr, uint8_t value);
void fds_clock_cpu(int cpu_cycles);
void fds_reset(void);
Mirroring fds_mirroring(void);
bool fds_irq_pending(void);
void fds_irq_ack(void);
float fds_expansion_audio(void);
void fds_nsf_audio_reset(void);
void fds_nsf_audio_clock(int cpu_cycles);
uint8_t fds_nsf_audio_read(uint16_t addr, uint8_t open_bus);
void fds_nsf_audio_write(uint16_t addr, uint8_t value);
float fds_nsf_audio_output(void);

// Disk persistence leaves the in-memory image dirty if writing or replacement fails.
bool fds_flush(void);
bool fds_disk_dirty(void);
FdsSaveMode fds_save_mode(void);
const char *fds_save_path(void);

size_t fds_side_count(void);
bool fds_disk_inserted(void);
size_t fds_current_side(void);
bool fds_insert_disk(size_t side);
void fds_eject_disk(void);
void fds_set_write_protected(bool protected_media);
bool fds_write_protected(void);
void fds_set_automation_options(FdsAutomationOptions options);
FdsAutomationOptions fds_automation_options(void);
bool fds_automatic_insert_active(void);
bool fds_automatic_insert_ambiguous(void);
bool fds_loading_fast_forward(void);
// Called by the disk clock when the PPU reaches a new frame. Repeated calls
// for the same frame are harmless; no CPU, PPU, or disk cycles are skipped.
void fds_automation_frame(uint64_t frame);

bool fds_state_capture(NesStateWriter *writer);
bool fds_state_validate(NesStateReader *reader);
bool fds_state_apply(NesStateReader *reader);

#endif
