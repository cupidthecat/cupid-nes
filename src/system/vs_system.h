/*
 * vs_system.h - Nintendo VS System hardware interface
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
#ifndef NES_VS_SYSTEM_H
#define NES_VS_SYSTEM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "../apu/apu.h"
#include "../rom/rom.h"

typedef enum {
    VS_TYPE_DEFAULT = 0,
    VS_TYPE_RBI_BASEBALL = 1,
    VS_TYPE_TKO_BOXING = 2,
    VS_TYPE_SUPER_XEVIOUS = 3,
    VS_TYPE_ICE_CLIMBER = 4,
    VS_TYPE_DUAL = 5,
    VS_TYPE_RAID_ON_BUNGELING_BAY = 6
} VsSystemType;

typedef enum {
    VS_PPU_2C03 = 0,
    VS_PPU_2C04_0001,
    VS_PPU_2C04_0002,
    VS_PPU_2C04_0003,
    VS_PPU_2C04_0004,
    VS_PPU_2C05_01,
    VS_PPU_2C05_02,
    VS_PPU_2C05_03,
    VS_PPU_2C05_04,
    VS_PPU_2C05_05
} VsPpuModel;

typedef enum {
    VS_INPUT_STANDARD = 4,
    VS_INPUT_SWAPPED = 5,
    VS_INPUT_SWAP_AB = 6
} VsInputType;

typedef struct {
    bool enabled;
    bool dual;
    VsSystemType type;
    VsPpuModel ppu_model;
    VsInputType input_type;
} VsRomConfig;

bool vs_decode_header(const iNESHeader *header, int mapper, size_t prg_bytes,
                      size_t chr_bytes, VsRomConfig *config,
                      char *reason, size_t reason_size);
void vs_commit_config(const VsRomConfig *config);
void vs_clear_config(void);
bool vs_enabled(void);
bool vs_dual_system(void);
VsSystemType vs_system_type(void);
VsPpuModel vs_ppu_model(void);
unsigned vs_active_side(void);

void vs_power_on_secondary(void);
// Called after resetting the main CPU/PPU/APU.
void vs_soft_reset(void);
int vs_cpu_step(void);
void vs_start_frame(void);
uint64_t vs_side_cpu_cycles(unsigned side);
uint64_t vs_side_frame_count(unsigned side);
unsigned vs_video_width(void);
const uint32_t *vs_video_framebuffer(void);
APU *vs_side_apu(unsigned side);
void vs_audio_init(int sample_rate);
void vs_audio_callback(void *userdata, uint8_t *stream, int len);

void vs_write_4016(uint8_t value);
uint8_t vs_read_controller_port(unsigned port);
bool vs_protection_read(uint16_t address, uint8_t *value);
uint8_t vs_prg_chr_select_bit(void);
bool vs_shared_ram_access_allowed(void);
bool vs_external_irq_pending(void);

bool vs_set_dip_switches(uint16_t value);
uint16_t vs_dip_switches(void);
bool vs_set_coin(unsigned slot, bool pressed);
bool vs_set_service(unsigned side, bool pressed);

bool vs_ppu_is_2c05(void);
bool vs_ppu_status_signature(uint8_t *signature);
bool vs_ppu_rgb_color(uint8_t color, uint8_t mask, uint32_t *argb);

#endif
