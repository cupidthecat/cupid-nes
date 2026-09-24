/* Movie startup profile. SPDX-License-Identifier: GPL-3.0-or-later */
#include "tas_startup.h"
#include "input_event.h"
#include "../cpu/cpu.h"
#include "../apu/apu.h"
#include "../ppu/ppu.h"
#include "../rom/mapper.h"
#include "../rom/replay_memory.h"
#include "../rom/fds.h"
#include "../system/hardware.h"
#include "../system/execution_policy.h"
#include "../system/timing.h"
#include "../system/vs_system.h"
#include "../util/md5.h"
#include "../cheats/cheats.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint32_t option;
    uint64_t random[2];
} StartupMemory;

static uint64_t rotate64(uint64_t value, unsigned count) {
    return (value << count) | (value >> (64 - count));
}

static uint64_t splitmix64(uint64_t *seed) {
    uint64_t value = (*seed += UINT64_C(0x9e3779b97f4a7c15));
    value = (value ^ (value >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    value = (value ^ (value >> 27)) * UINT64_C(0x94d049bb133111eb);
    return value ^ (value >> 31);
}

static uint8_t next_random(StartupMemory *memory) {
    uint64_t first = memory->random[0], second = memory->random[1];
    uint8_t value = (uint8_t)(first + second);
    second ^= first;
    memory->random[0] = rotate64(first, 55) ^ second ^ (second << 14);
    memory->random[1] = rotate64(second, 36);
    return value;
}

static void initialize(void *data, size_t size, bool default_zero, void *context) {
    if (!data) {
        return;
    }
    StartupMemory *memory = context;
    uint8_t *bytes = data;
    for (size_t i = 0; i < size; ++i) {
        switch (memory->option) {
        case 1:
            bytes[i] = 0xFF;
            break;
        case 2:
            bytes[i] = 0;
            break;
        case 3:
            bytes[i] = next_random(memory);
            break;
        default:
            bytes[i] = !default_zero && (i & 4u) ? 0xFF : 0;
            break;
        }
    }
}

bool nes_tas_rom_md5(uint8_t digest[16]) {
    if (!digest || !cart || rom_is_nsf() || rom_is_studybox()) {
        return false;
    }
    if (fds_active()) {
        return fds_movie_md5(digest);
    }
    NesMd5 hash;
    nes_md5_init(&hash);
    return nes_md5_update(&hash, prg_rom, prg_size) &&
           (!cart_has_chr_rom() || nes_md5_update(&hash, chr_rom, chr_size)) && nes_md5_final(&hash, digest);
}

bool nes_tas_pin_cheats(NesFm2Movie *movie) {
    if (!movie) {
        return false;
    }
    for (size_t i = 0; i < movie->extension_count; ++i) {
        if (!strcmp(movie->extensions[i].key, "cupidCheatsHash")) {
            return true;
        }
    }
    if (movie->extension_count >= SIZE_MAX / sizeof(*movie->extensions)) {
        return false;
    }
    char *key = malloc(sizeof("cupidCheatsHash")), *value = malloc(9);
    if (!key || !value) {
        free(key);
        free(value);
        return false;
    }
    memcpy(key, "cupidCheatsHash", sizeof("cupidCheatsHash"));
    snprintf(value, 9, "%08x", cheats_compatibility_hash());
    NesFm2HeaderField *fields = realloc(movie->extensions, (movie->extension_count + 1) * sizeof(*fields));
    if (!fields) {
        free(key);
        free(value);
        return false;
    }
    movie->extensions = fields;
    fields[movie->extension_count++] = (NesFm2HeaderField){key, value};
    return true;
}

static bool matching_cheats(const NesFm2Movie *movie) {
    char expected[9];
    snprintf(expected, sizeof(expected), "%08x", cheats_compatibility_hash());
    for (size_t i = 0; i < movie->extension_count; ++i) {
        if (!strcmp(movie->extensions[i].key, "cupidCheatsHash") && strcmp(movie->extensions[i].value, expected)) {
            return false;
        }
    }
    return true;
}

static uint32_t read32(const uint8_t *bytes) {
    return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 | (uint32_t)bytes[2] << 16 | (uint32_t)bytes[3] << 24;
}

/* Standard cartridge save RAM is one non-empty slot in the source container.
 * Other slots must be explicitly empty; do not concatenate unlike save chips. */
static bool saveram(const NesFm2Movie *movie, const uint8_t **bytes, size_t *size) {
    *bytes = NULL;
    *size = 0;
    if (!movie->saveram.size) {
        return true;
    }
    const uint8_t *data = movie->saveram.data;
    size_t remaining = movie->saveram.size;
    if (!data || remaining < 4 || read32(data) > 1 || (read32(data) != 0) != ((ines_header.flags6 & 2u) != 0)) {
        return false;
    }
    data += 4;
    remaining -= 4;
    while (remaining) {
        if (remaining < 4) {
            return false;
        }
        size_t length = read32(data);
        data += 4;
        remaining -= 4;
        if (length > remaining) {
            return false;
        }
        if (length) {
            if (*bytes || length != cart_replay_save_ram_size()) {
                return false;
            }
            *bytes = data;
            *size = length;
        }
        data += length;
        remaining -= length;
    }
    return *size == cart_replay_save_ram_size();
}

NesMovieResult nes_tas_startup_validate(const NesFm2Movie *movie, char *error, size_t capacity) {
    const char *reason = NULL;
    NesMovieResult result = NES_MOVIE_INCOMPATIBLE;
    uint8_t digest[16];
    const uint8_t *save_bytes;
    size_t save_size;
    if (!movie || !nes_tas_rom_md5(digest)) {
        reason = "Load a cartridge or disk image before opening a TAS movie.";
        result = NES_MOVIE_NO_IMAGE;
    } else if (memcmp(digest, movie->rom_md5, sizeof(digest))) {
        reason = "Movie ROM checksum does not match the loaded PRG and CHR data.";
    } else if (!matching_cheats(movie)) {
        reason = "The saved movie uses a different enabled cheat configuration. Restore its cheats before playback.";
    } else if (movie->savestate.size) {
        reason = "This movie starts from a foreign save state. Export a power-on movie to play it here.";
    } else if (movie->fds != fds_active() || (fds_active() && fds_disk_dirty())) {
        reason = "Movie disk configuration differs from the loaded image, or that image has unsaved disk changes.";
    } else if (vs_dual_system()) {
        reason = "FM2 does not describe both machines of a VS Dual System recording.";
    } else if (movie->pal && (fds_active() || vs_enabled())) {
        reason = "The loaded disk or arcade hardware requires an NTSC movie.";
    } else if (movie->ram_init_option > 3 || movie->ports[2] != NES_FM2_PORT_NONE) {
        reason = "Movie uses an unsupported RAM initialization or expansion controller profile.";
    } else if (!saveram(movie, &save_bytes, &save_size)) {
        reason = "Movie save RAM does not match this cartridge's save memory layout.";
    } else {
        for (size_t i = 0; i < movie->frame_count; ++i) {
            uint8_t commands = movie->frames[i].commands;
            if ((commands & 0x80u) ||
                ((commands & (NES_FM2_COMMAND_FDS_INSERT | NES_FM2_COMMAND_FDS_SELECT)) && !fds_active()) ||
                ((commands & (NES_FM2_COMMAND_VS_COIN_1 | NES_FM2_COMMAND_VS_COIN_2 | NES_FM2_COMMAND_VS_SERVICE)) &&
                 !vs_enabled())) {
                reason = "Movie contains a command that is unavailable on the loaded hardware.";
                break;
            }
        }
    }
    if (error && capacity) {
        snprintf(error, capacity, "%s", reason ? reason : "");
    }
    return reason ? result : NES_MOVIE_OK;
}

bool nes_tas_startup_configure(const NesFm2Movie *movie) {
    if (!movie || !nes_execution_allows_host_configuration()) {
        return false;
    }
    nes_set_region(movie->pal ? NES_REGION_PAL : NES_REGION_NTSC);
    cpu_use_default_startup_alignment();
    nes_set_ram_power_on_state(NES_RAM_POWER_DEFAULT);
    nes_set_randomize_vblank(false);
    nes_set_console_model(movie->microphone ? NES_CONSOLE_HVC001 : NES_CONSOLE_NES001);
    cpu_set_test_mode(false);
    apu_set_cpu_revision(APU_CPU_REVISION_EARLY_2A03);
    apu_set_disable_noise_mode(false);
    apu_set_swap_duty_cycles(false);
    ppu_set_revision(PPU_REVISION_2C02_E_PLUS);
    ppu_set_oam_row_corruption_worst_case(false);
    ppu_set_startup_write_restriction(false);
    ppu_set_oam_decay(false);
    ppu_set_oamdata_read_disabled(false);
    ppu_set_palette_readback_disabled(false);
    ppu_set_reset_suppression(false);
    ppu_set_sprite_eval_wrap_bug(false);
    const NesPortDevice ports[] = {NES_PORT_NONE, NES_PORT_GAMEPAD, NES_PORT_ZAPPER};
    if ((unsigned)movie->ports[0] > NES_FM2_PORT_ZAPPER || (unsigned)movie->ports[1] > NES_FM2_PORT_ZAPPER) {
        return false;
    }
    NesInputConfiguration config = {.adapter = movie->fourscore ? NES_ADAPTER_FOUR_SCORE : NES_ADAPTER_NONE,
                                    .ports = {ports[movie->ports[0]], ports[movie->ports[1]]},
                                    .expansion = NES_EXPANSION_NONE};
    return joypad_apply_configuration(&config);
}

bool nes_tas_startup_power(const NesFm2Movie *movie) {
    if (!movie || nes_execution_allows_persistence()) {
        return false;
    }
    NesInputEvent power = {.type = NES_INPUT_EVENT_POWER_CYCLE};
    if (!nes_input_event_apply(&power)) {
        return false;
    }
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        Joypad *pad = joypad_player(player);
        if (pad) {
            memset(pad, 0, sizeof(*pad));
        }
    }
    uint64_t seed = movie->ram_init_seed;
    StartupMemory memory = {.option = movie->ram_init_option};
    memory.random[0] = splitmix64(&seed);
    memory.random[1] = splitmix64(&seed);
    uint8_t bytes[0x2000];
    size_t cpu_size = cart_cpu_ram_8k() ? sizeof(bytes) : 0x800u;
    initialize(bytes, cpu_size, false, &memory);
    if (!cpu_replay_set_ram(bytes, cpu_size)) {
        return false;
    }
    initialize(ppu_vram, 0x800, true, &memory);
    initialize(ppu_palette, PPU_PALETTE_SIZE, true, &memory);
    initialize(ppu.oam, sizeof(ppu.oam), true, &memory);
    for (size_t i = 0; i < PPU_PALETTE_SIZE; ++i) {
        ppu_palette[i] &= 0x3F;
    }
    cart_replay_initialize_memory(initialize, &memory);
    if (fds_active()) {
        fds_replay_initialize_memory(initialize, &memory);
    }
    rom_reapply_trainer();
    const uint8_t *save_bytes;
    size_t save_size;
    if (!saveram(movie, &save_bytes, &save_size)) {
        return false;
    }
    return (!save_size || cart_replay_set_save_ram(save_bytes, save_size)) && ppu_begin_tas_timing();
}
