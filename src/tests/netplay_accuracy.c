/*
 * netplay_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Two-process netplay regression fixture. SPDX-License-Identifier: GPL-3.0-or-later */
#include "../replay/netplay.h"
#include "../replay/netplay_hash.h"
#include "../rom/rom.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../apu/apu.h"
#include "../joypad/joypad.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include "../system/vs_system.h"
#include "../system/execution_policy.h"
#include "../ui/machine_actions.h"
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REQUIRE(x) do { if (!(x)) { fprintf(stderr, "Netplay check failed at %d: %s\n", __LINE__, #x); goto fail; } } while (0)

int test_netplay_peer(const char *role, unsigned port, const char *scenario) {
    bool host = strcmp(role, "host") == 0;
    bool mismatch = strcmp(scenario, "mismatch") == 0;
    bool profile = strcmp(scenario, "profile") == 0;
    bool interrupted = strcmp(scenario, "interrupted") == 0;
    bool malformed = strcmp(scenario, "malformed") == 0;
    bool desync = strcmp(scenario, "desync") == 0;
    uint8_t image[16 + 32768] = {0};
    memcpy(image, "NES\032", 4); image[4] = 2;
    const uint8_t program[] = {
        0xA9, 1, 0x8D, 0x16, 0x40, 0xA9, 0, 0x8D, 0x16, 0x40,
        0xAD, 0x16, 0x40, 0x85, 0x20, 0xAD, 0x17, 0x40, 0x85, 0x21,
        0xE6, 0x22, 0x4C, 0x00, 0x80
    };
    memcpy(image + 16, program, sizeof(program));
    image[16 + 0x7ffd] = 0x80;
    if (mismatch && !host) image[16 + 0x100] = 0x71;
    NesNetplaySession *session = NULL;
    nes_set_ram_power_on_state(NES_RAM_POWER_ZERO);
    REQUIRE(nes_set_region_mode(!strcmp(scenario, "pal") ? NES_REGION_MODE_PAL : NES_REGION_MODE_NTSC));
    REQUIRE(load_rom_memory(image, sizeof(image)) == 0);
    REQUIRE(frontend_machine_power_cycle());
    apu_audio_init(host ? 44100 : 48000);
    write_mem(0x10, host ? 0x5A : 0xA5);
    if (profile && !host) apu_set_disable_noise_mode(true);
    session = nes_netplay_create(); REQUIRE(session);
    NesNetplayResult result;
    if (host) {
        REQUIRE(nes_netplay_host_listen(session, (uint16_t)port, 1, 2) == NES_NETPLAY_OK);
        printf("LISTENING %u\n", nes_netplay_port(session)); fflush(stdout);
        result = nes_netplay_host_accept(session, 15000);
    } else result = nes_netplay_join(session, "127.0.0.1", (uint16_t)port, 15000);
    if (mismatch || profile || malformed) {
        REQUIRE(result == ((mismatch || profile) ? NES_NETPLAY_INCOMPATIBLE : NES_NETPLAY_PROTOCOL_ERROR));
        REQUIRE(nes_execution_policy() == NES_EXECUTION_LIVE);
        REQUIRE(read_mem(0x10) == (host ? 0x5A : 0xA5));
        goto done;
    }
    if (result != NES_NETPLAY_OK) fprintf(stderr, "Handshake: %s\n", nes_netplay_result_string(result));
    REQUIRE(result == NES_NETPLAY_OK);
    REQUIRE(read_mem(0x10) == 0x5A);
    for (unsigned frame = 0; frame < 24; ++frame) {
        /* Each process attempts both slots. Only its assigned slot may change. */
        (void)joypad_set_player(host ? 0 : 1, 0, (frame & 1u) != 0);
        (void)joypad_set_player(host ? 1 : 0, 0, (frame & 1u) == 0);
        if (!host && frame % 4 == 0) SDL_Delay(20);
        if (host && frame == 4 && !interrupted && !desync) {
            REQUIRE(nes_netplay_set_paused(session, true) == NES_NETPLAY_PAUSED);
            SDL_Delay(60);
            REQUIRE(nes_netplay_set_paused(session, false) == NES_NETPLAY_OK);
        }
        if (!host && desync && frame == 6) write_mem(0x11, 0xB7);
        if (!host && interrupted && frame == 6) {
            /* Simulate abrupt termination without cleanup or a disconnect packet. */
            _Exit(0);
        }
        do {
            result = nes_netplay_frame_begin(session, 3000);
            if (result == NES_NETPLAY_PAUSED) SDL_Delay(1);
        } while (result == NES_NETPLAY_PAUSED);
        if ((interrupted && host && frame == 6) || (desync && frame == 6)) {
            REQUIRE(result == (desync ? NES_NETPLAY_DESYNC : NES_NETPLAY_NETWORK_ERROR));
            REQUIRE(nes_execution_policy() == NES_EXECUTION_LIVE);
            REQUIRE(read_mem(0x10) == (host ? 0x5A : 0xA5));
            goto done;
        }
        if (result != NES_NETPLAY_OK) fprintf(stderr, "Begin frame %u: %s\n", frame, nes_netplay_result_string(result));
        REQUIRE(result == NES_NETPLAY_OK);
        vs_start_frame();
        while (!ppu.frame_complete) vs_cpu_step();
        float samples[128];
        apu_audio_pull_stereo(&apu, samples, host ? 17 : 61);
        result = nes_netplay_frame_complete(session, true);
        if (result != NES_NETPLAY_OK) fprintf(stderr, "End frame %u: %s\n", frame, nes_netplay_result_string(result));
        REQUIRE(result == NES_NETPLAY_OK);
        REQUIRE((read_mem(0x20) & 1u) == (frame & 1u));
        REQUIRE((read_mem(0x21) & 1u) == (frame & 1u));
        NesNetplayHardwareHash hash;
        REQUIRE(nes_netplay_hardware_hash(&hash));
        printf("FRAME %u %016llx %016llx\n", frame,
               (unsigned long long)hash.first, (unsigned long long)hash.second);
        fflush(stdout);
    }
    REQUIRE(nes_netplay_disconnect(session) == NES_NETPLAY_OK);
    REQUIRE(nes_execution_policy() == NES_EXECUTION_LIVE);
    REQUIRE(read_mem(0x10) == (host ? 0x5A : 0xA5));
done:
    nes_netplay_destroy(session);
    session = NULL;
    apu_audio_shutdown_state(&apu);
    REQUIRE(unload_rom());
    puts("NETPLAY PASS");
    return 0;
fail:
    nes_netplay_destroy(session);
    session = NULL;
    apu_audio_shutdown_state(&apu);
    (void)unload_rom();
    return 1;
}
