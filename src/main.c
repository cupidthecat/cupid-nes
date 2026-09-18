/*
 * main.c - Main entry point for Cupid NES Emulator
 *
 * Author: @frankischilling
 *
 * This file initializes the loaded cartridge, CPU, PPU, APU, SDL video and audio, input,
 * palette tools, frame pacing, and the main emulation loop. It also handles controller
 * input, reset events, palette file loading, and final emulator shutdown.
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

#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include "rom/rom.h"
#include "rom/fds.h"
#include "cpu/cpu.h"
#include "ppu/ppu.h"
#include "joypad/joypad.h"
#include "joypad/family_basic.h"
#include "../include/globals.h"
#include "apu/apu.h"
#include <time.h>
#include "rom/mapper.h"
#include <math.h>
#include "ui/palette_tool.h"
#include "system/timing.h"
#include "system/hardware.h"
#include "system/vs_system.h"

#define AUDIO_SAMPLE_RATE 44100
#define AUDIO_BUFFER_SAMPLES 1024

// SDL presents the framebuffer that the PPU fills.
uint32_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];

Joypad pad1 = {0}, pad2 = {0};

static SDL_GameController *controllers[NES_INPUT_PLAYERS];

static void open_controller(int device) {
    if (!SDL_IsGameController(device)) return;
    SDL_JoystickID id = SDL_JoystickGetDeviceInstanceID(device);
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        if (controllers[player] &&
            SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(controllers[player])) == id)
            return;
    }
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        if (!controllers[player]) {
            controllers[player] = SDL_GameControllerOpen(device);
            return;
        }
    }
}

static void controller_event(const SDL_Event *event) {
    if (event->type == SDL_CONTROLLERDEVICEADDED) {
        open_controller(event->cdevice.which);
        return;
    }
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        if (!controllers[player]) continue;
        SDL_JoystickID id = SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(controllers[player]));
        if (event->type == SDL_CONTROLLERDEVICEREMOVED && event->cdevice.which == id) {
            SDL_GameControllerClose(controllers[player]);
            controllers[player] = NULL;
            joypad_player(player)->buttons = 0;
        } else if ((event->type == SDL_CONTROLLERBUTTONDOWN || event->type == SDL_CONTROLLERBUTTONUP)
                   && event->cbutton.which == id) {
            int button;
            switch (event->cbutton.button) {
                case SDL_CONTROLLER_BUTTON_A: button = BTN_A; break;
                case SDL_CONTROLLER_BUTTON_B: button = BTN_B; break;
                case SDL_CONTROLLER_BUTTON_BACK: button = BTN_SELECT; break;
                case SDL_CONTROLLER_BUTTON_START: button = BTN_START; break;
                case SDL_CONTROLLER_BUTTON_DPAD_UP: button = BTN_UP; break;
                case SDL_CONTROLLER_BUTTON_DPAD_DOWN: button = BTN_DOWN; break;
                case SDL_CONTROLLER_BUTTON_DPAD_LEFT: button = BTN_LEFT; break;
                case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: button = BTN_RIGHT; break;
                default: continue;
            }
            joypad_set_player(player, button, event->type == SDL_CONTROLLERBUTTONDOWN);
        }
    }
}

static bool mat_key_event(const SDL_KeyboardEvent *event) {
    static const SDL_Keycode keys[] = {
        SDLK_1, SDLK_2, SDLK_3, SDLK_4,
        SDLK_q, SDLK_w, SDLK_e, SDLK_r,
        SDLK_a, SDLK_s, SDLK_d, SDLK_f
    };
    bool handled = false;
    for (unsigned pad = 0; pad < 12; ++pad) {
        if (event->keysym.sym != keys[pad]) continue;
        for (unsigned slot = 0; slot < 3; ++slot) {
            bool active = slot == 2
                ? joypad_expansion_device() == NES_EXPANSION_FAMILY_TRAINER_A
                    || joypad_expansion_device() == NES_EXPANSION_FAMILY_TRAINER_B
                : joypad_port_device(slot) == NES_PORT_POWER_PAD_A
                    || joypad_port_device(slot) == NES_PORT_POWER_PAD_B;
            if (active) {
                joypad_set_mat_pad(slot, pad, event->type == SDL_KEYDOWN);
                handled = true;
            }
        }
        break;
    }
    return handled;
}

static bool tape_capture_pending;

static bool finish_tape_capture(const char *path) {
    family_basic_tape_stop();
    if (!tape_capture_pending) return true;
    if (!path || !family_basic_tape_save_file(path)) {
        fprintf(stderr, "Could not save tape; the captured signal remains in memory\n");
        return false;
    }
    tape_capture_pending = false;
    return true;
}

static bool family_basic_key_event(const SDL_KeyboardEvent *event,
                                   const char *play_path, const char *record_path) {
    if (joypad_expansion_device() != NES_EXPANSION_FAMILY_BASIC) return false;
    static const SDL_Scancode keys[] = {
        SDL_SCANCODE_F8, SDL_SCANCODE_RETURN, SDL_SCANCODE_LEFTBRACKET, SDL_SCANCODE_RIGHTBRACKET,
        SDL_SCANCODE_RALT, SDL_SCANCODE_RSHIFT, SDL_SCANCODE_BACKSLASH, SDL_SCANCODE_F12,
        SDL_SCANCODE_F7, SDL_SCANCODE_GRAVE, SDL_SCANCODE_APOSTROPHE, SDL_SCANCODE_SEMICOLON,
        SDL_SCANCODE_F9, SDL_SCANCODE_SLASH, SDL_SCANCODE_MINUS, SDL_SCANCODE_EQUALS,
        SDL_SCANCODE_F6, SDL_SCANCODE_O, SDL_SCANCODE_L, SDL_SCANCODE_K,
        SDL_SCANCODE_PERIOD, SDL_SCANCODE_COMMA, SDL_SCANCODE_P, SDL_SCANCODE_0,
        SDL_SCANCODE_F5, SDL_SCANCODE_I, SDL_SCANCODE_U, SDL_SCANCODE_J,
        SDL_SCANCODE_M, SDL_SCANCODE_N, SDL_SCANCODE_9, SDL_SCANCODE_8,
        SDL_SCANCODE_F4, SDL_SCANCODE_Y, SDL_SCANCODE_G, SDL_SCANCODE_H,
        SDL_SCANCODE_B, SDL_SCANCODE_V, SDL_SCANCODE_7, SDL_SCANCODE_6,
        SDL_SCANCODE_F3, SDL_SCANCODE_T, SDL_SCANCODE_R, SDL_SCANCODE_D,
        SDL_SCANCODE_F, SDL_SCANCODE_C, SDL_SCANCODE_5, SDL_SCANCODE_4,
        SDL_SCANCODE_F2, SDL_SCANCODE_W, SDL_SCANCODE_S, SDL_SCANCODE_A,
        SDL_SCANCODE_X, SDL_SCANCODE_Z, SDL_SCANCODE_E, SDL_SCANCODE_3,
        SDL_SCANCODE_F1, SDL_SCANCODE_ESCAPE, SDL_SCANCODE_Q, SDL_SCANCODE_LCTRL,
        SDL_SCANCODE_LSHIFT, SDL_SCANCODE_LALT, SDL_SCANCODE_1, SDL_SCANCODE_2,
        SDL_SCANCODE_HOME, SDL_SCANCODE_UP, SDL_SCANCODE_RIGHT, SDL_SCANCODE_LEFT,
        SDL_SCANCODE_DOWN, SDL_SCANCODE_SPACE, SDL_SCANCODE_DELETE, SDL_SCANCODE_INSERT
    };
    _Static_assert(sizeof(keys) / sizeof(keys[0]) == FB_KEY_COUNT, "Complete keyboard matrix");
    bool down = event->type == SDL_KEYDOWN;
    if (event->keysym.scancode == SDL_SCANCODE_F10 && down && !event->repeat) {
        if (record_path) {
            if (family_basic_tape_mode() != FB_TAPE_RECORDING && finish_tape_capture(record_path)) {
                family_basic_tape_record(cpu_total_cycles);
                tape_capture_pending = true;
            }
        } else if (play_path) {
            family_basic_tape_play(cpu_total_cycles);
        }
    } else if (event->keysym.scancode == SDL_SCANCODE_F11 && down && !event->repeat) {
        finish_tape_capture(record_path);
    } else if (event->keysym.scancode == SDL_SCANCODE_BACKSPACE) {
        family_basic_set_key(FB_KEY_DELETE, down);
    } else {
        for (unsigned key = 0; key < FB_KEY_COUNT; ++key) {
            if (event->keysym.scancode == keys[key]) {
                family_basic_set_key((FamilyBasicKey)key, down);
                break;
            }
        }
    }
    return true;
}

int main(int argc, char *argv[]) {
    SDL_AudioSpec want;
    SDL_AudioSpec have;
    SDL_AudioDeviceID audio_dev = 0;

    const char *rom_path = NULL;
    const char *barcode = NULL;
    const char *tape_play_path = NULL;
    const char *tape_record_path = NULL;
    const char *fds_bios_path = NULL;
    size_t fds_frontend_side = 0;
    bool fds_side_set = false;
    bool fds_start_ejected = false;
    bool fds_start_write_protected = false;
    bool vs_dip_set = false;
    uint16_t vs_dips = 0;
    bool startup_phase_set = false;
    bool startup_seed_set = false;
    unsigned startup_cpu_offset = 0, startup_ppu_phase = 0;
    uint32_t startup_seed = 0;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--console") == 0) {
            if (++i == argc || !nes_set_console_model_name(argv[i])) {
                fprintf(stderr, "Console must be nes-001, nes-101, famicom, or av-famicom\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--cpu-revision") == 0) {
            if (++i == argc) {
                fprintf(stderr, "CPU revision must be early-2a03 or late-2a03\n");
                return 1;
            }
            if (strcmp(argv[i], "early-2a03") == 0)
                apu_set_cpu_revision(APU_CPU_REVISION_EARLY_2A03);
            else if (strcmp(argv[i], "late-2a03") == 0)
                apu_set_cpu_revision(APU_CPU_REVISION_LATE_2A03);
            else {
                fprintf(stderr, "CPU revision must be early-2a03 or late-2a03\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--startup-phase") == 0) {
            if (++i == argc || startup_phase_set || startup_seed_set) {
                fprintf(stderr, "Choose one startup phase CPU:PPU or startup seed\n");
                return 1;
            }
            char *end;
            errno = 0;
            unsigned long cpu_offset = strtoul(argv[i], &end, 10);
            if (errno || argv[i][0] < '0' || argv[i][0] > '9' || *end != ':' || cpu_offset > 15) {
                fprintf(stderr, "Startup phase must be CPU:PPU in regional master clocks\n");
                return 1;
            }
            const char *ppu_text = end + 1;
            errno = 0;
            unsigned long ppu_phase = strtoul(ppu_text, &end, 10);
            if (errno || ppu_text[0] < '0' || ppu_text[0] > '9' || *end || ppu_phase > 4) {
                fprintf(stderr, "Startup phase must be CPU:PPU in regional master clocks\n");
                return 1;
            }
            startup_cpu_offset = (unsigned)cpu_offset;
            startup_ppu_phase = (unsigned)ppu_phase;
            startup_phase_set = true;
        } else if (strcmp(argv[i], "--startup-seed") == 0) {
            if (++i == argc || startup_phase_set || startup_seed_set) {
                fprintf(stderr, "Choose one startup phase CPU:PPU or startup seed\n");
                return 1;
            }
            char *end;
            errno = 0;
            unsigned long long seed = strtoull(argv[i], &end, 10);
            if (errno || argv[i][0] < '0' || argv[i][0] > '9' || *end || seed > UINT32_MAX) {
                fprintf(stderr, "Startup seed must be an integer from 0 to 4294967295\n");
                return 1;
            }
            startup_seed = (uint32_t)seed;
            startup_seed_set = true;
        } else if (strcmp(argv[i], "--ppu-revision") == 0) {
            if (++i == argc || !ppu_set_revision_name(argv[i])) {
                fprintf(stderr, "PPU revision must be 2c02-pre-e or 2c02e-plus\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--ppu-oam-row-corruption") == 0) {
            ppu_set_oam_row_corruption_worst_case(true);
        } else if (strcmp(argv[i], "--ppu-startup-restriction") == 0) {
            ppu_set_startup_write_restriction(true);
        } else if (strcmp(argv[i], "--ppu-oam-decay") == 0) {
            ppu_set_oam_decay(true);
        } else if (strcmp(argv[i], "--adapter") == 0) {
            if (++i == argc || !joypad_set_adapter_name(argv[i])) {
                fprintf(stderr, "Adapter must be none, four-score, famicom-2, or famicom-4\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--port1") == 0 || strcmp(argv[i], "--port2") == 0) {
            unsigned port = argv[i][6] == '2' ? 1 : 0;
            if (++i == argc || !joypad_set_port_device_name(port, argv[i])) {
                fprintf(stderr, "Port device must be pad, none, arkanoid, power-pad-a, power-pad-b, or zapper\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--expansion") == 0) {
            if (++i == argc || !joypad_set_expansion_device_name(argv[i])) {
                fprintf(stderr, "Expansion device must be none, arkanoid, family-trainer-a, family-trainer-b, zapper, or family-basic\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--zapper-radius") == 0) {
            if (++i == argc) {
                fprintf(stderr, "Zapper radius must be an integer from 0 to 255\n");
                return 1;
            }
            char *end;
            unsigned long radius = strtoul(argv[i], &end, 10);
            if (end == argv[i] || *end || radius > NES_ZAPPER_MAX_RADIUS
                || !joypad_set_zapper_radius((unsigned)radius)) {
                fprintf(stderr, "Zapper radius must be an integer from 0 to 255\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--vs-dip") == 0) {
            if (++i == argc) {
                fprintf(stderr, "VS DIP value must be an integer from 0 to 65535\n");
                return 1;
            }
            char *end;
            unsigned long value = strtoul(argv[i], &end, 0);
            if (end == argv[i] || *end || value > 0xFFFFu) {
                fprintf(stderr, "VS DIP value must be an integer from 0 to 65535\n");
                return 1;
            }
            vs_dips = (uint16_t)value;
            vs_dip_set = true;
        } else if (strcmp(argv[i], "--barcode") == 0) {
            if (++i == argc) {
                fprintf(stderr, "Barcode requires 8 or 13 decimal digits\n");
                return 1;
            }
            barcode = argv[i];
        } else if (strcmp(argv[i], "--tape-play") == 0 || strcmp(argv[i], "--tape-record") == 0) {
            bool record = strcmp(argv[i], "--tape-record") == 0;
            if (++i == argc || tape_play_path || tape_record_path) {
                fprintf(stderr, "Choose one tape file with --tape-play or --tape-record\n");
                return 1;
            }
            if (record) tape_record_path = argv[i];
            else tape_play_path = argv[i];
        } else if (strcmp(argv[i], "--fds-bios") == 0) {
            if (++i == argc) {
                fprintf(stderr, "--fds-bios requires an 8KB BIOS file\n");
                return 1;
            }
            fds_bios_path = argv[i];
        } else if (strcmp(argv[i], "--fds-side") == 0) {
            if (++i == argc) {
                fprintf(stderr, "--fds-side requires a side number starting at 1\n");
                return 1;
            }
            char *end = NULL;
            unsigned long side = strtoul(argv[i], &end, 10);
            if (!side || *end) {
                fprintf(stderr, "FDS side must be a positive number\n");
                return 1;
            }
            fds_frontend_side = (size_t)(side - 1);
            fds_side_set = true;
        } else if (strcmp(argv[i], "--fds-eject") == 0) {
            fds_start_ejected = true;
        } else if (strcmp(argv[i], "--fds-write-protect") == 0) {
            fds_start_write_protected = true;
        } else if (argv[i][0] == '-' || rom_path) {
            fprintf(stderr, "Unexpected argument: %s\n", argv[i]);
            return 1;
        } else {
            rom_path = argv[i];
        }
    }
    if (!rom_path) {
        printf("Usage: %s [--console MODEL] [--cpu-revision REVISION] "
               "[--startup-phase CPU:PPU | --startup-seed SEED] "
               "[--ppu-revision REVISION] [--ppu-oam-row-corruption] "
               "[--ppu-startup-restriction] [--ppu-oam-decay] "
               "[--adapter TYPE] [--port1 DEVICE] [--port2 DEVICE] "
               "[--expansion DEVICE] [--barcode DIGITS] "
               "[--zapper-radius PIXELS] [--vs-dip VALUE] [--tape-play FILE | --tape-record FILE] "
               "[--fds-bios BIOS] [--fds-side N] "
               "[--fds-eject] [--fds-write-protect] <rom-file>\n", argv[0]);
        return 1;
    }
    if (!fds_bios_path && (fds_side_set || fds_start_ejected || fds_start_write_protected)) {
        fprintf(stderr, "FDS media options require --fds-bios\n");
        return 1;
    }
    if (!joypad_configuration_valid()) {
        fprintf(stderr, "An adapter and another device cannot share the same connector\n");
        return 1;
    }
    if ((tape_play_path || tape_record_path) && joypad_expansion_device() != NES_EXPANSION_FAMILY_BASIC) {
        fprintf(stderr, "Tape input requires --expansion family-basic\n");
        return 1;
    }
    
    printf("Console: %s\n", nes_console_model_name());
    printf("CPU revision: %s\n", apu_get_cpu_revision() == APU_CPU_REVISION_EARLY_2A03
           ? "early-2a03" : "late-2a03");
    printf("PPU revision: %s\n", ppu_revision_name());
    printf("PPU OAM row corruption: %s\n",
           ppu_oam_row_corruption_worst_case() ? "worst-case" : "compatibility");
    printf("PPU startup write restriction: %s\n",
           ppu_startup_write_restriction_enabled() ? "enabled" : "compatibility");
    printf("PPU OAM decay: %s\n", ppu_oam_decay_enabled() ? "enabled" : "compatibility");
    printf("Input adapter: %s\n", joypad_adapter_name());
    printf("Loading ROM: %s\n", rom_path);
    int load_result = fds_bios_path
        ? load_fds(rom_path, fds_bios_path, fds_start_write_protected)
        : load_rom(rom_path);
    if(load_result != 0) {
        fprintf(stderr, "Failed to load ROM\n");
        return 1;
    }
    if (startup_phase_set && !cpu_set_startup_alignment(startup_cpu_offset, startup_ppu_phase)) {
        fprintf(stderr, "Startup phase must be CPU 0..%u and PPU 0..%u for this image\n",
                (unsigned)nes_timing()->cpu_divider - 1, (unsigned)nes_timing()->ppu_divider - 1);
        unload_rom();
        return 1;
    }
    if (startup_seed_set) cpu_seed_startup_alignment(startup_seed);
    if (vs_dip_set && !vs_set_dip_switches(vs_dips)) {
        fprintf(stderr, "--vs-dip requires a VS System image\n");
        unload_rom();
        return 1;
    }
    if (barcode && !cart_set_barcode(barcode)) {
        fprintf(stderr, "Barcode input requires a Datach cartridge and 8 or 13 decimal digits\n");
        unload_rom();
        return 1;
    }
    if (barcode) printf("Press F8 to scan the configured barcode\n");
    if (rom_is_fds()) {
        if (fds_side_set && !fds_insert_disk(fds_frontend_side)) {
            fprintf(stderr, "FDS side is outside the loaded disk image\n");
            unload_rom();
            return 1;
        }
        if (fds_start_ejected) fds_eject_disk();
    }
    cpu_total_cycles = 0;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    // Print ROM metadata at startup so mapper selection can be checked from the log.
    if (!rom_is_fds()) {
        printf("=== ROM Header Info ===\n");
        printf("Signature: %c%c%c 0x%02X\n",
               ines_header.signature[0],
               ines_header.signature[1],
               ines_header.signature[2],
               ines_header.signature[3]);
        printf("PRG-ROM Chunks: %d\n", ines_header.prg_rom_chunks);
        printf("CHR-ROM Chunks: %d\n", ines_header.chr_rom_chunks);
        printf("Flags6: 0x%02X\n", ines_header.flags6);
        printf("Flags7: 0x%02X\n", ines_header.flags7);
        printf("Mirroring: %s\n", (mirroring_mode == 0 ? "Horizontal" : "Vertical"));
        printf("=======================\n");
    }
    
    if (!rom_is_fds() && (ines_header.prg_rom_chunks > 1 || (ines_header.flags6 & 0xF0))) {
        printf("WARNING: This ROM likely uses a mapper (mapper number: %d).\n",
            (ines_header.flags7 & 0xF0) | ((ines_header.flags6 & 0xF0) >> 4));
    }
    
    if (!rom_is_fds())
        printf("Mapper detected: %d\n", ((ines_header.flags7 & 0xF0) | ((ines_header.flags6 & 0xF0) >> 4)));


    printf("Resetting CPU...\n");
    if (!cpu_power_on(&cpu)) {
        fprintf(stderr, "Invalid CPU startup alignment\n");
        unload_rom();
        return 1;
    }
    CpuStartupAlignment alignment = cpu_get_startup_alignment();
    printf("Startup alignment: CPU %u, PPU %u%s\n", (unsigned)alignment.cpu_offset,
           (unsigned)alignment.ppu_phase, startup_seed_set ? " (seeded)" : "");
    if (startup_seed_set) printf("Startup seed: %llu\n", (unsigned long long)startup_seed);
    vs_power_on_secondary();
    if (vs_enabled()) {
        printf("VS System: %s, PPU model %u, DIP $%04X\n",
               vs_dual_system() ? "dual" : "single", (unsigned)vs_ppu_model(),
               (unsigned)vs_dip_switches());
        printf("VS controls: 5-8 coin slots, F1/F2 service buttons\n");
    }
    if (tape_play_path && !family_basic_tape_load_file(tape_play_path)) {
        fprintf(stderr, "Could not load tape: %s\n", tape_play_path);
        unload_rom();
        return 1;
    }
    if (tape_play_path || tape_record_path)
        printf("Family BASIC tape: F10 starts the tape; F11 stops and saves a recording\n");
    printf("CPU state after reset:\n");
    printf("  PC: 0x%04X\n", cpu.pc);
    printf("  SP: 0x%02X\n", cpu.sp);
    printf("  A: 0x%02X\n", cpu.a);
    printf("  X: 0x%02X\n", cpu.x);
    printf("  Y: 0x%02X\n", cpu.y);
    printf("  Status: 0x%02X\n", cpu.status);

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_GAMECONTROLLER) != 0) {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        return 1;
    }
    for (int device = 0; device < SDL_NumJoysticks(); ++device) open_controller(device);

    // Configure audio before starting the main loop.
    memset(&want, 0, sizeof want);
    memset(&have, 0, sizeof have);
    want.freq = AUDIO_SAMPLE_RATE;
    want.format = AUDIO_F32;     // float32 mono
    want.channels = 1;
    want.samples = AUDIO_BUFFER_SAMPLES;
    want.callback = vs_audio_callback;

    audio_dev = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    if (!audio_dev) {
        fprintf(stderr, "Warning: audio disabled (%s)\n", SDL_GetError());
    } else {
        printf("=== Audio Info ===\n");
        printf("Requested: %d Hz, Got: %d Hz\n", want.freq, have.freq);
        printf("Requested: %d samples buffer, Got: %d samples\n", want.samples, have.samples);
        printf("Cycles per sample: %.6f\n", nes_timing()->cpu_hz / have.freq);
        printf("==================\n");
        vs_audio_init(have.freq);
        SDL_PauseAudioDevice(audio_dev, 0);
    }

    int video_width = (int)vs_video_width();
    SDL_Window *window = SDL_CreateWindow("Cupid NES Emulator",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, video_width * 2, SCREEN_HEIGHT * 2, SDL_WINDOW_SHOWN);
    if(!window) {
        fprintf(stderr, "SDL_CreateWindow Error: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if(!renderer) {
        fprintf(stderr, "SDL_CreateRenderer Error: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Texture *texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, video_width, SCREEN_HEIGHT);
    if(!texture) {
        fprintf(stderr, "SDL_CreateTexture Error: %s\n", SDL_GetError());
        return 1;
    }

    bool running = true;
    SDL_Event e;

    palette_tool_init();
    const double performance_frequency = (double)SDL_GetPerformanceFrequency();
    double frame_deadline = (double)SDL_GetPerformanceCounter();
    
    while (running) {
        Uint32 frameStart = SDL_GetTicks();
        uint64_t frame_start_cycles = cpu_total_cycles;
    
        while (SDL_PollEvent(&e)) {
            controller_event(&e);
            bool main_mouse_event = e.type == SDL_MOUSEMOTION
                ? e.motion.windowID == SDL_GetWindowID(window)
                : (e.type == SDL_MOUSEBUTTONDOWN || e.type == SDL_MOUSEBUTTONUP)
                    && e.button.windowID == SDL_GetWindowID(window);
            if (main_mouse_event) {
                int mouse_x, mouse_y, window_width, window_height;
                uint32_t buttons = SDL_GetMouseState(&mouse_x, &mouse_y);
                SDL_GetWindowSize(window, &window_width, &window_height);
                int position = window_width > 0 ? 0x54 + 160 * mouse_x / window_width : 0x54;
                bool on_screen = mouse_x >= 0 && mouse_y >= 0 && mouse_x < window_width
                    && mouse_y < window_height && !(buttons & SDL_BUTTON_RMASK);
                int aim_x = on_screen ? 256 * mouse_x / window_width : -1;
                int aim_y = on_screen ? 240 * mouse_y / window_height : -1;
                bool trigger = (buttons & (SDL_BUTTON_LMASK | SDL_BUTTON_RMASK)) != 0;
                for (unsigned slot = 0; slot < 3; ++slot) {
                    joypad_set_paddle(slot, position, (buttons & SDL_BUTTON_LMASK) != 0);
                    joypad_set_zapper(slot, aim_x, aim_y, trigger);
                }
            }
            if (e.type == SDL_QUIT) {
                if (rom_is_fds() && !fds_flush()) {
                    fprintf(stderr, "Failed to save modified FDS media; keeping the emulator open\n");
                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "FDS Save Error",
                        "The modified disk image could not be saved. The emulator will remain open so the media changes are not discarded.", window);
                } else {
                    running = false;
                }
            }
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && family_basic_key_event(&e.key, tape_play_path, tape_record_path)) continue;
            palette_tool_handle_event(&e, renderer);
            
            if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) {
                if (mat_key_event(&e.key)) continue;
                int down = (e.type == SDL_KEYDOWN);
    
                switch (e.key.keysym.sym) {
                    case SDLK_F10:
                        if (down && rom_is_fds()) fds_set_write_protected(!fds_write_protected());
                        break;
                    case SDLK_F9:
                        if (down && rom_is_fds() && fds_side_count()) {
                            fds_frontend_side = (fds_frontend_side + 1) % fds_side_count();
                            (void)fds_insert_disk(fds_frontend_side);
                        }
                        break;
                    case SDLK_F8:
                        if (down && !e.key.repeat && barcode) cart_set_barcode(barcode);
                        if (down && !e.key.repeat && rom_is_fds()) {
                            if (fds_disk_inserted()) fds_eject_disk();
                            else (void)fds_insert_disk(fds_frontend_side);
                        }
                        break;
                    case SDLK_F7:
                        if (down) { palette_tool_toggle_overlay(); }
                        break;
                    case SDLK_F6:
                        if (down) { ppu_palette_reset_default(); palette_tool_flash(true); }
                        break;
                    case SDLK_r:
                        if (down) {
                            if (audio_dev) SDL_LockAudioDevice(audio_dev);
                            ppu_soft_reset(&ppu);
                            apu_soft_reset(&apu);
                            cpu_soft_reset(&cpu);
                            vs_soft_reset();
                            if (audio_dev) SDL_UnlockAudioDevice(audio_dev);
                        }
                        break;
                    case SDLK_5: if (vs_enabled()) vs_set_coin(0, down != 0); break;
                    case SDLK_6: if (vs_enabled()) vs_set_coin(1, down != 0); break;
                    case SDLK_7: if (vs_dual_system()) vs_set_coin(2, down != 0); break;
                    case SDLK_8: if (vs_dual_system()) vs_set_coin(3, down != 0); break;
                    case SDLK_F1: if (vs_enabled()) vs_set_service(0, down != 0); break;
                    case SDLK_F2: if (vs_dual_system()) vs_set_service(1, down != 0); break;
                    case SDLK_z:        joypad_set(&pad1, BTN_A,      down); break;
                    case SDLK_x:        joypad_set(&pad1, BTN_B,      down); break;
                    case SDLK_RSHIFT:   joypad_set(&pad1, BTN_SELECT, down); break;
                    case SDLK_RETURN:   joypad_set(&pad1, BTN_START,  down); break;
                    case SDLK_UP:       joypad_set(&pad1, BTN_UP,     down); break;
                    case SDLK_DOWN:     joypad_set(&pad1, BTN_DOWN,   down); break;
                    case SDLK_LEFT:     joypad_set(&pad1, BTN_LEFT,   down); break;
                    case SDLK_RIGHT:    joypad_set(&pad1, BTN_RIGHT,  down); break;
                    case SDLK_m:        joypad_set_microphone(down != 0); break;
                    default: break;
                }

                // Ctrl+V accepts the palette text formats handled by the parser.
                if (down && (e.key.keysym.sym == SDLK_v)) {
                    const SDL_Keymod mods = SDL_GetModState();
                    if ((mods & KMOD_CTRL) != 0) {
                        if (SDL_HasClipboardText()) {
                            char *txt = SDL_GetClipboardText();
                            if (txt) {
                                int rc = ppu_palette_load_hex_string(txt);
                                palette_tool_flash(rc == 0);
                                SDL_free(txt);
                                if (rc != 0) {
                                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_WARNING, "Palette Paste Error",
                                        "Clipboard text did not contain a valid palette.\n\nAccepts: 64 x RRGGBB tokens, or raw 192/1536 hex bytes.", NULL);
                                }
                            }
                        }
                    }
                }
            }

            // Dropped .pal files use the same validation as explicit palette loads.
            if (e.type == SDL_DROPFILE) {
                char *dropped_f = e.drop.file;
                if (dropped_f) {
                    int rc = ppu_palette_load_pal_file(dropped_f);
                    palette_tool_flash(rc == 0);
                    if (rc != 0) {
                        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Palette Load Error",
                            "Failed to load .pal file. Expected 192 or 1536 bytes.", NULL);
                    }
                    SDL_free(dropped_f);
                }
            }
        }

        // A successful quit flush must be the last chance for emulation to mutate
        // writable disk media. Do not run another frame after accepting SDL_QUIT.
        if (!running) break;
    
        // Run CPU steps until the PPU completes the current frame.
        vs_start_frame();
        while (!ppu.frame_complete) {
            vs_cpu_step();
        }

        // Present the frame, then draw the palette UI on top.
        SDL_UpdateTexture(texture, NULL, vs_video_framebuffer(), video_width * sizeof(uint32_t));
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        int ww = 0, hh = 0; SDL_GetRendererOutputSize(renderer, &ww, &hh);
        if (palette_tool_is_visible()) { palette_tool_draw(renderer, ww, hh); }
        SDL_RenderPresent(renderer);
    
        Uint32 frameTime = SDL_GetTicks() - frameStart;
        palette_tool_tick(frameTime);
        frame_deadline += (double)(cpu_total_cycles - frame_start_cycles)
                        * performance_frequency / nes_timing()->cpu_hz;
        double current_ticks = (double)SDL_GetPerformanceCounter();
        if (frame_deadline > current_ticks) {
            // Carry fractional milliseconds into the next deadline instead of
            // running every frame early after truncating SDL's delay argument.
            SDL_Delay((Uint32)((frame_deadline - current_ticks) * 1000.0 / performance_frequency));
        } else if (current_ticks - frame_deadline > performance_frequency * 0.25) {
            frame_deadline = current_ticks;
        }
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    if (audio_dev) SDL_CloseAudioDevice(audio_dev);
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player)
        if (controllers[player]) SDL_GameControllerClose(controllers[player]);
    bool tape_saved = finish_tape_capture(tape_record_path);
    bool tape_failed = family_basic_tape_failed();
    if (tape_failed) fprintf(stderr, "Tape recording stopped because the capture buffer could not grow\n");
    family_basic_shutdown();
    if (!unload_rom()) {
        fprintf(stderr, "Failed to unload modified FDS media\n");
        SDL_Quit();
        return 1;
    }
    SDL_Quit();
    return tape_saved && !tape_failed ? 0 : 1;
}
