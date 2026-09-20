/*
 * fds.c - Famicom Disk System hardware emulation
 *
 * Author: @frankischilling
 *
 * This file implements FDS BIOS and RAM mapping, disk media and transfer timing, timer
 * and transfer IRQs, CRC handling, writable media persistence, and the FDS wavetable and
 * modulation audio unit.
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

#include "fds.h"
#include "../system/hardware.h"
#include "../system/execution_policy.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../media/patch.h"
#include "../util/file_io.h"
#include "game_db.h"
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FDS_BIOS_SIZE 0x2000u
#define FDS_WORK_RAM_SIZE 0x8000u
#define FDS_CHR_RAM_SIZE 0x2000u
#define FDS_SIDE_SIZE 65500u
#define QD_SIDE_SIZE 65536u
#define FDS_INITIAL_GAP_BYTES (28300u / 8u)
#define FDS_BLOCK_GAP_BYTES (976u / 8u)
#define FDS_NO_SIDE SIZE_MAX

typedef struct {
    uint8_t *raw;
    uint8_t *drive;
    size_t drive_size;
    bool dirty;
    uint8_t identity_header[10];
} FdsSide;

typedef struct {
    uint8_t speed;
    uint8_t gain;
    bool envelope_off;
    bool increase;
    uint16_t frequency;
    uint32_t timer;
    uint8_t master_speed;
} FdsEnvelope;

typedef struct {
    FdsEnvelope envelope;
    int8_t counter;
    bool disabled;
    uint8_t table[64];
    uint8_t table_position;
    uint16_t overflow;
    int32_t output;
} FdsMod;

typedef struct {
    uint8_t wave[64];
    bool wave_write_enabled;
    FdsEnvelope volume;
    FdsMod mod;
    bool disable_envelopes;
    bool halt_waveform;
    uint8_t master_volume;
    uint16_t wave_overflow;
    uint8_t wave_position;
    uint8_t output;
} FdsAudio;

struct FdsImage {
    uint8_t bios[FDS_BIOS_SIZE];
    FdsSide *sides;
    size_t side_count;
    size_t side_capacity;
    bool headered;
    bool qd_format;
    uint8_t header[16];
    char *disk_path;
    bool write_protected;
    FdsSaveMode save_mode;
    uint8_t *original_disk;
    size_t original_disk_size;
};

typedef struct {
    uint64_t last_frame;
    uint64_t last_check_frame;
    uint32_t successive_checks;
    int32_t eject_frames;
    int32_t switch_frames;
    int32_t retry_frames;
    size_t previous_side;
    bool game_started;
    bool ambiguous;
} FdsAutomationState;

typedef struct {
    FdsImage *image;
    uint8_t work_ram[FDS_WORK_RAM_SIZE];
    uint8_t chr_ram[FDS_CHR_RAM_SIZE];
    size_t current_side;
    size_t disk_position;
    uint32_t transfer_delay;
    uint16_t crc;
    uint16_t timer_reload;
    uint16_t timer_counter;
    uint8_t write_data;
    uint8_t read_data;
    uint8_t ext_connector;
    bool timer_enabled;
    bool timer_repeat;
    bool timer_irq;
    bool disk_irq;
    bool disk_regs_enabled;
    bool sound_regs_enabled;
    bool motor_on;
    bool reset_transfer;
    bool read_mode;
    bool crc_control;
    bool disk_ready;
    bool transfer_irq_enabled;
    bool transfer_complete;
    bool end_of_head;
    bool scanning;
    bool gap_ended;
    bool previous_crc_control;
    bool bad_crc;
    bool dirty;
    Mirroring mirroring;
    FdsAudio audio;
    FdsAutomationState automation;
} FdsState;

static FdsState fds;

#include "fds_automation.h"

typedef struct {
    uint8_t *data;
    size_t size;
    size_t capacity;
} DriveBuilder;

static char *fds_strdup(const char *text) {
    if (!text) return NULL;
    size_t len = strlen(text) + 1;
    char *copy = (char *)malloc(len);
    if (copy) memcpy(copy, text, len);
    return copy;
}

static bool builder_reserve(DriveBuilder *builder, size_t extra) {
    if (extra > SIZE_MAX - builder->size) return false;
    size_t need = builder->size + extra;
    if (need <= builder->capacity) return true;
    size_t capacity = builder->capacity ? builder->capacity : FDS_SIDE_SIZE;
    while (capacity < need) {
        if (capacity > SIZE_MAX / 2) { capacity = need; break; }
        capacity *= 2;
    }
    uint8_t *data = (uint8_t *)realloc(builder->data, capacity);
    if (!data) return false;
    builder->data = data;
    builder->capacity = capacity;
    return true;
}

static bool builder_append(DriveBuilder *builder, uint8_t value) {
    if (!builder_reserve(builder, 1)) return false;
    builder->data[builder->size] = value;
    builder->size++;
    return true;
}

static bool builder_fill(DriveBuilder *builder, uint8_t value, size_t count) {
    if (!builder_reserve(builder, count)) return false;
    memset(builder->data + builder->size, value, count);
    builder->size += count;
    return true;
}

static size_t fds_block_length(const uint8_t *raw, size_t pos, bool qd_format) {
    switch (raw[pos]) {
        case 1: return 56;
        case 2: return 2;
        case 3: return 16;
        case 4:
            if (pos < (qd_format ? 5u : 3u)) return 0;
            return 1u + raw[pos - (qd_format ? 5u : 3u)]
                + ((size_t)raw[pos - (qd_format ? 4u : 2u)] << 8);
        default: return 0;
    }
}

static bool build_drive_side(FdsSide *side, size_t side_capacity, bool qd_format) {
    DriveBuilder builder = {0};
    if (!builder_fill(&builder, 0, FDS_INITIAL_GAP_BYTES)) goto fail;

    size_t pos = 0;
    while (pos < side_capacity) {
        size_t block_length = fds_block_length(side->raw, pos, qd_format);
        size_t stored_length = block_length + (qd_format && block_length ? 2u : 0u);
        if (!block_length || stored_length > side_capacity - pos) {
            if (!builder_append(&builder, 0x80)) goto fail;
            for (; pos < side_capacity; ++pos)
                if (!builder_append(&builder, side->raw[pos])) goto fail;
            break;
        }

        if (!builder_append(&builder, 0x80)) goto fail;
        for (size_t i = 0; i < stored_length; ++i)
            if (!builder_append(&builder, side->raw[pos + i])) goto fail;
        if (!qd_format && (!builder_append(&builder, 0x4D) || !builder_append(&builder, 0x62))) goto fail;
        if (!builder_fill(&builder, 0, FDS_BLOCK_GAP_BYTES)) goto fail;
        pos += stored_length;
    }
    if (builder.size < side_capacity && !builder_fill(&builder, 0, side_capacity - builder.size)) goto fail;
    side->drive = builder.data;
    side->drive_size = builder.size;
    return true;

fail:
    free(builder.data);
    return false;
}

static void free_side(FdsSide *side) {
    if (!side) return;
    free(side->raw);
    free(side->drive);
    memset(side, 0, sizeof(*side));
}

static FdsImage *fds_image_create_data(const uint8_t *disk, size_t disk_size,
                                       const uint8_t *bios, size_t bios_size,
                                       const char *disk_path, bool write_protected) {
    if (!disk || !bios || bios_size != FDS_BIOS_SIZE) return NULL;

    bool headered = disk_size >= 16 && memcmp(disk, "FDS\x1A", 4) == 0;
    size_t side_count;
    size_t offset;
    size_t side_capacity;
    bool qd_format;
    if (headered) {
        side_count = disk[4];
        offset = 16;
        if (!side_count) return NULL;
        bool fds_size = side_count <= (SIZE_MAX - offset) / FDS_SIDE_SIZE
            && disk_size == offset + side_count * FDS_SIDE_SIZE;
        bool qd_size = side_count <= (SIZE_MAX - offset) / QD_SIDE_SIZE
            && disk_size == offset + side_count * QD_SIDE_SIZE;
        if (!fds_size && !qd_size) return NULL;
        qd_format = qd_size;
        side_capacity = qd_format ? QD_SIDE_SIZE : FDS_SIDE_SIZE;
    } else {
        bool fds_size = disk_size && disk_size % FDS_SIDE_SIZE == 0;
        bool qd_size = disk_size && disk_size % QD_SIDE_SIZE == 0;
        if (!fds_size && !qd_size) return NULL;
        qd_format = qd_size && !fds_size;
        side_capacity = qd_format ? QD_SIDE_SIZE : FDS_SIDE_SIZE;
        side_count = disk_size / side_capacity;
        offset = 0;
        if (!side_count || side_count > 255) return NULL;
    }

    FdsImage *image = (FdsImage *)calloc(1, sizeof(*image));
    if (!image) return NULL;
    image->sides = (FdsSide *)calloc(side_count, sizeof(*image->sides));
    if (!image->sides) { free(image); return NULL; }
    image->disk_path = fds_strdup(disk_path);
    if (disk_path && !image->disk_path) { fds_image_destroy(image); return NULL; }
    memcpy(image->bios, bios, FDS_BIOS_SIZE);
    image->side_count = side_count;
    image->side_capacity = side_capacity;
    image->headered = headered;
    image->qd_format = qd_format;
    image->write_protected = write_protected;
    if (headered) memcpy(image->header, disk, sizeof(image->header));

    for (size_t side = 0; side < side_count; ++side) {
        image->sides[side].raw = (uint8_t *)malloc(side_capacity);
        if (!image->sides[side].raw) { fds_image_destroy(image); return NULL; }
        memcpy(image->sides[side].raw, disk + offset + side * side_capacity, side_capacity);
        memcpy(image->sides[side].identity_header, image->sides[side].raw + 14,
               sizeof(image->sides[side].identity_header));
        if (!build_drive_side(&image->sides[side], side_capacity, qd_format)) {
            fds_image_destroy(image);
            return NULL;
        }
    }
    return image;
}

void fds_image_destroy(FdsImage *image) {
    if (!image) return;
    for (size_t i = 0; i < image->side_count; ++i) free_side(&image->sides[i]);
    free(image->sides);
    free(image->disk_path);
    free(image->original_disk);
    free(image);
}

#include "fds_image_options.h"

static void envelope_reset_timer(FdsEnvelope *channel) {
    channel->timer = 8u * ((uint32_t)channel->speed + 1u) * ((uint32_t)channel->master_speed + 1u);
}

static void envelope_write(FdsEnvelope *channel, uint16_t addr, uint8_t value) {
    switch (addr & 3u) {
        case 0:
            channel->speed = value & 0x3F;
            channel->increase = (value & 0x40) != 0;
            channel->envelope_off = (value & 0x80) != 0;
            envelope_reset_timer(channel);
            if (channel->envelope_off) channel->gain = channel->speed;
            break;
        case 2:
            channel->frequency = (uint16_t)((channel->frequency & 0x0F00) | value);
            break;
        case 3:
            channel->frequency = (uint16_t)((channel->frequency & 0x00FF) | ((uint16_t)(value & 0x0F) << 8));
            break;
    }
}

static bool envelope_tick(FdsEnvelope *channel) {
    if (channel->envelope_off || !channel->master_speed) return false;
    channel->timer--;
    if (channel->timer) return false;
    envelope_reset_timer(channel);
    if (channel->increase) {
        if (channel->gain < 32) channel->gain++;
    } else if (channel->gain) {
        channel->gain--;
    }
    return true;
}

static void mod_set_counter(FdsMod *mod, int value) {
    value &= 0x7F;
    if (value >= 64) value -= 128;
    mod->counter = (int8_t)value;
}

static void mod_update_output(FdsMod *mod, uint16_t volume_pitch) {
    int32_t temp = (int32_t)mod->counter * mod->envelope.gain;
    int32_t remainder = temp & 0x0F;
    temp >>= 4;
    if (remainder > 0 && (temp & 0x80) == 0) temp += mod->counter < 0 ? -1 : 2;
    if (temp >= 192) temp -= 256;
    else if (temp < -64) temp += 256;
    temp *= volume_pitch;
    remainder = temp & 0x3F;
    temp >>= 6;
    if (remainder >= 32) temp++;
    mod->output = temp;
}

static void mod_write(FdsMod *mod, uint16_t addr, uint8_t value) {
    if (addr == 0x4084 || addr == 0x4086) {
        envelope_write(&mod->envelope, addr, value);
    } else if (addr == 0x4085) {
        mod_set_counter(mod, value);
    } else if (addr == 0x4087) {
        envelope_write(&mod->envelope, addr, value);
        mod->disabled = (value & 0x80) != 0;
        if (mod->disabled) mod->overflow = 0;
    }
}

static bool mod_tick(FdsMod *mod) {
    if (mod->disabled || !mod->envelope.frequency) return false;
    uint16_t frequency = mod->envelope.frequency;
    mod->overflow = (uint16_t)(mod->overflow + frequency);
    if (mod->overflow >= frequency) return false;
    static const int8_t lut[8] = {0, 1, 2, 4, 0, -4, -2, -1};
    uint8_t entry = mod->table[mod->table_position];
    if (entry == 4) mod_set_counter(mod, 0);
    else mod_set_counter(mod, mod->counter + lut[entry]);
    mod->table_position = (uint8_t)((mod->table_position + 1) & 0x3F);
    return true;
}

static void audio_update_output(FdsAudio *audio) {
    if (audio->wave_write_enabled) return;
    static const unsigned volume_table[4] = {36, 24, 17, 14};
    unsigned gain = audio->volume.gain > 32 ? 32 : audio->volume.gain;
    unsigned level = gain * volume_table[audio->master_volume & 3];
    audio->output = (uint8_t)((audio->wave[audio->wave_position] * level) / 1152u);
}

static void audio_clock(FdsAudio *audio) {
    int frequency = audio->volume.frequency;
    if (!audio->halt_waveform && !audio->disable_envelopes) {
        (void)envelope_tick(&audio->volume);
        if (envelope_tick(&audio->mod.envelope)) mod_update_output(&audio->mod, audio->volume.frequency);
    }
    if (mod_tick(&audio->mod)) mod_update_output(&audio->mod, audio->volume.frequency);
    audio_update_output(audio);
    int pitch = frequency + audio->mod.output;
    if (!audio->halt_waveform && pitch > 0) {
        uint16_t add = (uint16_t)pitch;
        audio->wave_overflow = (uint16_t)(audio->wave_overflow + add);
        if (audio->wave_overflow < add)
            audio->wave_position = (uint8_t)((audio->wave_position + 1) & 0x3F);
    }
}

static uint8_t audio_read(FdsAudio *audio, uint16_t addr, uint8_t open_bus) {
    if (addr >= 0x4040 && addr <= 0x407F) {
        uint8_t sample = audio->wave_write_enabled ? audio->wave[addr & 0x3F]
                                                   : audio->wave[audio->wave_position];
        return (uint8_t)((open_bus & 0xC0) | sample);
    }
    switch (addr) {
        case 0x4090: return (uint8_t)((open_bus & 0xC0) | (audio->volume.gain & 0x3F));
        case 0x4091: return (uint8_t)(((uint32_t)audio->wave_position << 6 | (audio->wave_overflow >> 12)) & 0xFF);
        case 0x4092: return (uint8_t)((open_bus & 0xC0) | (audio->mod.envelope.gain & 0x3F));
        case 0x4093: return (uint8_t)((open_bus & 0x80) | ((audio->mod.overflow >> 5) & 0x7F));
        case 0x4094: return (uint8_t)(((int)audio->mod.counter * audio->mod.envelope.gain >> 4) & 0xFF);
        case 0x4095: {
            static const int8_t lut[8] = {0, 1, 2, 4, 12, -4, -2, -1};
            return (uint8_t)((open_bus & 0xC0) | (lut[audio->mod.table[audio->mod.table_position]] & 0x0F));
        }
        case 0x4096: return (uint8_t)((open_bus & 0xC0) | (audio->wave[audio->wave_position] & 0x3F));
        case 0x4097: return (uint8_t)((open_bus & 0x80) | ((uint8_t)audio->mod.counter & 0x7F));
        default: return open_bus;
    }
}

static void audio_write(FdsAudio *audio, uint16_t addr, uint8_t value) {
    if (addr >= 0x4040 && addr <= 0x407F) {
        if (audio->wave_write_enabled) audio->wave[addr & 0x3F] = value & 0x3F;
        return;
    }
    switch (addr) {
        case 0x4080:
        case 0x4082:
            envelope_write(&audio->volume, addr, value);
            mod_update_output(&audio->mod, audio->volume.frequency);
            break;
        case 0x4083:
            audio->disable_envelopes = (value & 0x40) != 0;
            audio->halt_waveform = (value & 0x80) != 0;
            if (audio->halt_waveform) audio->wave_position = 0;
            if (audio->disable_envelopes) {
                envelope_reset_timer(&audio->volume);
                envelope_reset_timer(&audio->mod.envelope);
            }
            envelope_write(&audio->volume, addr, value);
            mod_update_output(&audio->mod, audio->volume.frequency);
            break;
        case 0x4084:
        case 0x4085:
        case 0x4086:
        case 0x4087:
            mod_write(&audio->mod, addr, value);
            mod_update_output(&audio->mod, audio->volume.frequency);
            break;
        case 0x4088:
            if (audio->mod.disabled) {
                uint8_t data = value & 7;
                audio->mod.table[audio->mod.table_position] = data;
                audio->mod.table[(audio->mod.table_position + 1) & 0x3F] = data;
                audio->mod.table_position = (uint8_t)((audio->mod.table_position + 2) & 0x3F);
            }
            break;
        case 0x4089:
            audio->master_volume = value & 3;
            audio->wave_write_enabled = (value & 0x80) != 0;
            break;
        case 0x408A:
            audio->volume.master_speed = value;
            audio->mod.envelope.master_speed = value;
            break;
    }
}

static void audio_reset(FdsAudio *audio) {
    memset(audio, 0, sizeof(*audio));
    audio->volume.master_speed = 0xE8;
    audio->mod.envelope.master_speed = 0xE8;
}

void fds_activate(FdsImage *image) {
    fds_shutdown();
    memset(&fds, 0, sizeof(fds));
    fds.image = image;
    if (image) {
        nes_initialize_power_on_ram(fds.work_ram, sizeof(fds.work_ram), 0);
        nes_initialize_power_on_ram(fds.chr_ram, sizeof(fds.chr_ram), 0);
    }

    fds.current_side = image && image->side_count ? 0 : FDS_NO_SIDE;
    fds.mirroring = MIRROR_VERTICAL;
    fds.disk_regs_enabled = true;
    fds.sound_regs_enabled = true;
    fds.read_mode = true;
    fds.reset_transfer = true;
    fds.end_of_head = true;
    fds.ext_connector = 0;
    fds.gap_ended = true;
    audio_reset(&fds.audio);
    fds_automation_reset();
}

bool fds_active(void) { return fds.image != NULL; }

static bool rebuild_side(const FdsImage *image, const FdsSide *side, uint8_t *output) {
    if (!image || !side || !output) return false;
    size_t capacity = image->side_capacity;
    if (!side->dirty) {
        memcpy(output, side->raw, capacity);
        return true;
    }

    memset(output, 0, capacity);
    bool in_gap = true;
    size_t input = 0, written = 0;
    size_t file_size = 0;
    while (input < side->drive_size && written < capacity) {
        if (in_gap) {
            if (side->drive[input] == 0x80) in_gap = false;
            input++;
            continue;
        }

        size_t block_length;
        switch (side->drive[input]) {
            case 1:
                block_length = 56;
                break;
            case 2:
                block_length = 2;
                break;
            case 3:
                block_length = 16;
                if (input + 14 >= side->drive_size) return false;
                file_size = side->drive[input + 13] | ((size_t)side->drive[input + 14] << 8);
                break;
            case 4:
                block_length = 1u + file_size;
                break;
            default: {
                size_t remaining = side->drive_size - input;
                if (remaining > capacity - written) remaining = capacity - written;
                memcpy(output + written, side->drive + input, remaining);
                return true;
            }
        }

        size_t stored_length = block_length + (image->qd_format ? 2u : 0u);
        if (stored_length > side->drive_size - input) return false;
        if (stored_length > capacity - written) break;
        memcpy(output + written, side->drive + input, stored_length);
        written += stored_length;
        input += stored_length;
        if (!image->qd_format) {
            if (side->drive_size - input < 2) return false;
            input += 2; // Skip the synthetic CRC bytes in the drive stream.
        }
        in_gap = true;
    }
    return true;
}

bool fds_flush(void) {
    if (!nes_execution_allows_persistence()) return true;
    if (!fds.image || !fds.dirty) return true;
    if (fds.image->write_protected || !fds.image->disk_path) return false;
    size_t prefix = fds.image->headered ? 16u : 0u;
    if (fds.image->side_count > (SIZE_MAX - prefix) / fds.image->side_capacity) return false;
    size_t size = prefix + fds.image->side_count * fds.image->side_capacity;
    uint8_t *output = (uint8_t *)malloc(size);
    if (!output) return false;
    if (prefix) memcpy(output, fds.image->header, 16);
    for (size_t side = 0; side < fds.image->side_count; ++side) {
        uint8_t *side_output = output + prefix + side * fds.image->side_capacity;
        if (!rebuild_side(fds.image, &fds.image->sides[side], side_output)) {
            free(output);
            return false;
        }
    }
    bool ok;
    if (fds.image->save_mode == FDS_SAVE_OVERLAY) {
        uint8_t *patch = NULL;
        size_t patch_size = 0;
        NesPatchResult patched = nes_patch_create_ips(fds.image->original_disk,
                                                       fds.image->original_disk_size,
                                                       output, size, 32u * 1024u * 1024u,
                                                       &patch, &patch_size);
        ok = patched == NES_PATCH_OK
            && nes_file_write_atomic(fds.image->disk_path, patch, patch_size) == NES_FILE_OK;
        free(patch);
    } else {
        ok = nes_file_write_atomic(fds.image->disk_path, output, size) == NES_FILE_OK;
    }
    if (ok) {
        for (size_t side = 0; side < fds.image->side_count; ++side) {
            FdsSide *disk_side = &fds.image->sides[side];
            if (!disk_side->dirty) continue;
            memcpy(disk_side->raw, output + prefix + side * fds.image->side_capacity,
                   fds.image->side_capacity);
            disk_side->dirty = false;
        }
    }
    free(output);
    if (ok) fds.dirty = false;
    return ok;
}

void fds_shutdown(void) {
    if (!fds.image) return;
    if (!fds_flush() && fds.dirty)
        fprintf(stderr, "Failed to flush modified FDS disk image; unsaved media changes are being discarded\n");
    fds_image_destroy(fds.image);
    memset(&fds, 0, sizeof(fds));
    fds.current_side = FDS_NO_SIDE;
}

bool fds_disk_dirty(void) { return fds.dirty; }
FdsSaveMode fds_save_mode(void) { return fds.image ? fds.image->save_mode : FDS_SAVE_IN_PLACE; }
const char *fds_save_path(void) { return fds.image ? fds.image->disk_path : NULL; }
size_t fds_side_count(void) { return fds.image ? fds.image->side_count : 0; }
bool fds_disk_inserted(void) { return fds.image && fds.current_side < fds.image->side_count; }
size_t fds_current_side(void) { return fds_disk_inserted() ? fds.current_side : FDS_NO_SIDE; }

bool fds_insert_disk(size_t side) {
    if (!fds.image || side >= fds.image->side_count) return false;
    fds.current_side = side;
    fds.end_of_head = true;
    fds.scanning = false;
    fds.disk_position = 0;
    return true;
}

void fds_eject_disk(void) {
    fds.current_side = FDS_NO_SIDE;
}

void fds_set_write_protected(bool protected_media) {
    if (fds.image) fds.image->write_protected = protected_media;
}

bool fds_write_protected(void) { return !fds.image || fds.image->write_protected; }

static void update_crc(uint8_t value) {
    fds.crc ^= value;
    for (unsigned bit = 0; bit < 8; ++bit) {
        bool carry = (fds.crc & 1) != 0;
        fds.crc >>= 1;
        if (carry) fds.crc ^= 0x8408;
    }
}

static uint8_t disk_read_byte(void) {
    if (!fds_disk_inserted()) return 0;
    FdsSide *side = &fds.image->sides[fds.current_side];
    return fds.disk_position < side->drive_size ? side->drive[fds.disk_position] : 0;
}

static void disk_write_byte(uint8_t value) {
    if (!fds_disk_inserted() || fds.image->write_protected) return;
    FdsSide *side = &fds.image->sides[fds.current_side];
    if (fds.disk_position >= side->drive_size || side->drive[fds.disk_position] == value) return;
    side->drive[fds.disk_position] = value;
    side->dirty = true;
    fds.dirty = true;
}

static void clock_timer(void) {
    if (!fds.timer_enabled) return;
    if (fds.timer_counter == 0) {
        fds.timer_irq = true;
        fds.timer_counter = fds.timer_reload;
        if (!fds.timer_repeat) fds.timer_enabled = false;
    } else {
        fds.timer_counter--;
    }
}

static void clock_disk(void) {
    if (!fds_disk_inserted() || !fds.motor_on) {
        fds.end_of_head = true;
        fds.scanning = false;
        if (fds.automation.eject_frames < 0) fds.automation.eject_frames = 77;
        return;
    }
    if (fds.reset_transfer && !fds.scanning) return;
    if (fds.end_of_head) {
        fds.transfer_delay = 50000;
        fds.end_of_head = false;
        fds.disk_position = 0;
        fds.gap_ended = false;
        return;
    }
    if (fds.transfer_delay) {
        fds.transfer_delay--;
        return;
    }

    fds.scanning = true;
    fds.automation.eject_frames = -1;
    fds.automation.switch_frames = -1;
    uint8_t data = 0;
    bool need_irq = fds.transfer_irq_enabled;
    if (fds.read_mode) {
        data = disk_read_byte();
        if (!fds.previous_crc_control) update_crc(data);
        if (!fds.disk_ready) {
            fds.gap_ended = false;
            fds.crc = 0;
            fds.bad_crc = false;
        } else if (data && !fds.gap_ended) {
            fds.gap_ended = true;
            need_irq = false;
        }
        if (fds.gap_ended) {
            fds.transfer_complete = true;
            fds.read_data = data;
            if (need_irq) fds.disk_irq = true;
        }
        if (!fds.previous_crc_control && fds.crc_control) fds.bad_crc = fds.crc != 0;
    } else {
        if (!fds.crc_control) {
            fds.transfer_complete = true;
            data = fds.write_data;
            if (need_irq) fds.disk_irq = true;
        }
        if (!fds.disk_ready) {
            data = 0;
            fds.crc = 0;
        }
        if (!fds.crc_control) update_crc(data);
        else {
            data = (uint8_t)(fds.crc & 0xFF);
            fds.crc >>= 8;
        }
        disk_write_byte(data);
        fds.gap_ended = false;
        fds.bad_crc = false;
    }
    fds.previous_crc_control = fds.crc_control;

    FdsSide *side = &fds.image->sides[fds.current_side];
    fds.disk_position++;
    if (fds.disk_position >= side->drive_size) {
        fds.motor_on = false;
        fds.end_of_head = true;
        if (fds.transfer_irq_enabled) fds.disk_irq = true;
    } else {
        // A delay value of 149 produces the next transfer on the 150th CPU clock.
        fds.transfer_delay = 149;
    }
}

void fds_clock_cpu(int cpu_cycles) {
    for (int cycle = 0; cycle < cpu_cycles; ++cycle) {
        fds_automation_frame(ppu.frame_count);
        clock_timer();
        audio_clock(&fds.audio);
        clock_disk();
    }
}

void fds_reset(void) {
    // Console reset does not reset FDS controller or audio state. Fresh device
    // state is established when fds_activate() installs a new disk image.
}

bool fds_irq_pending(void) { return fds.timer_irq || fds.disk_irq; }
void fds_irq_ack(void) {
    fds.timer_irq = false;
    fds.disk_irq = false;
}
Mirroring fds_mirroring(void) { return fds.mirroring; }
float fds_expansion_audio(void) { return -(float)fds.audio.output * (20.0f / 5000.0f); }
void fds_nsf_audio_reset(void) { audio_reset(&fds.audio); }
void fds_nsf_audio_clock(int cpu_cycles) {
    for (int cycle = 0; cycle < cpu_cycles; ++cycle) audio_clock(&fds.audio);
}
uint8_t fds_nsf_audio_read(uint16_t addr, uint8_t open_bus) {
    return audio_read(&fds.audio, addr, open_bus);
}
void fds_nsf_audio_write(uint16_t addr, uint8_t value) { audio_write(&fds.audio, addr, value); }
float fds_nsf_audio_output(void) { return -(float)fds.audio.output * (20.0f / 5000.0f); }

uint8_t fds_ppu_read(uint16_t addr) {
    return fds.chr_ram[addr & 0x1FFF];
}

void fds_ppu_write(uint16_t addr, uint8_t value) {
    fds.chr_ram[addr & 0x1FFF] = value;
}

uint8_t fds_cpu_read_bus(uint16_t addr, uint8_t open_bus) {
    if (!fds.image) return open_bus;
    if (addr >= 0x6000 && addr <= 0xDFFF) return fds.work_ram[addr - 0x6000];
    if (addr >= 0xE000) {
        fds_automation_bios_read(addr);
        return fds.image->bios[addr - 0xE000];
    }
    if (addr >= 0x4040 && addr <= 0x4097) return audio_read(&fds.audio, addr, open_bus);
    switch (addr) {
        case 0x4030: {
            uint8_t value = open_bus & 0x24;
            if (fds.timer_irq) value |= 0x01;
            if (fds.mirroring == MIRROR_HORIZONTAL) value |= 0x08;
            if (fds.image->qd_format && fds.bad_crc) value |= 0x10;
            if (fds.transfer_complete) value |= 0x80;
            fds.timer_irq = false;
            return value;
        }
        case 0x4031:
            fds.transfer_complete = false;
            fds.disk_irq = false;
            return fds.read_data;
        case 0x4032: {
            uint8_t value = open_bus & 0xF8;
            if (!fds_disk_inserted()) value |= 0x01;
            if (!fds_disk_inserted() || !fds.scanning) value |= 0x02;
            if (!fds_disk_inserted() || fds.image->write_protected) value |= 0x04;
            fds_automation_status_read();
            return value;
        }
        case 0x4033:
            return (uint8_t)(fds.ext_connector | (fds.motor_on ? 0x80 : 0));
        default:
            return open_bus;
    }
}

void fds_cpu_write(uint16_t addr, uint8_t value) {
    if (!fds.image) return;
    if (addr >= 0x6000 && addr <= 0xDFFF) {
        fds.work_ram[addr - 0x6000] = value;
        return;
    }
    if (!fds.disk_regs_enabled && addr >= 0x4024 && addr <= 0x4026) return;
    if (!fds.sound_regs_enabled && (addr == 0x4080 || addr == 0x4085 || addr == 0x4088)) return;
    if (addr >= 0x4040 && addr <= 0x408A) {
        audio_write(&fds.audio, addr, value);
        return;
    }
    switch (addr) {
        case 0x4020:
            fds.timer_reload = (uint16_t)((fds.timer_reload & 0xFF00) | value);
            break;
        case 0x4021:
            fds.timer_reload = (uint16_t)((fds.timer_reload & 0x00FF) | ((uint16_t)value << 8));
            break;
        case 0x4022:
            fds.timer_repeat = (value & 1) != 0;
            fds.timer_enabled = (value & 2) != 0 && fds.disk_regs_enabled;
            if (fds.timer_enabled) fds.timer_counter = fds.timer_reload;
            else fds.timer_irq = false;
            break;
        case 0x4023:
            fds.disk_regs_enabled = (value & 1) != 0;
            fds.sound_regs_enabled = (value & 2) != 0;
            if (!fds.disk_regs_enabled) {
                fds.timer_enabled = false;
                fds.timer_irq = false;
                fds.disk_irq = false;
                fds.reset_transfer = true;
                fds.motor_on = false;
                fds.read_mode = true;
                fds.mirroring = MIRROR_VERTICAL;
                fds.crc_control = false;
                fds.disk_ready = false;
                fds.transfer_irq_enabled = false;
                fds.ext_connector = 0x7F;
            }
            if (!fds.sound_regs_enabled) {
                audio_write(&fds.audio, 0x4080, 0x80);
                audio_write(&fds.audio, 0x4085, 0x00);
            }
            break;
        case 0x4024:
            fds.write_data = value;
            fds.transfer_complete = false;
            fds.disk_irq = false;
            break;
        case 0x4025:
            fds.reset_transfer = (value & 0x01) == 0;
            fds.motor_on = (value & 0x02) == 0;
            fds.read_mode = (value & 0x04) != 0;
            fds.mirroring = (value & 0x08) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
            fds.crc_control = (value & 0x10) != 0;
            fds.disk_ready = (value & 0x40) != 0;
            fds.transfer_irq_enabled = (value & 0x80) != 0;
            fds.disk_irq = false;
            break;
        case 0x4026:
            fds.ext_connector = value & 0x7F;
            break;
    }
}
