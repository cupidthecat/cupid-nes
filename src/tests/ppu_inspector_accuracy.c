/* PPU viewer decoding and side-effect regressions. SPDX-License-Identifier: GPL-3.0-or-later */
#include "board_tests.h"
#include "../debugger/ppu_inspector.h"
#include "../apu/apu.h"
#include "../state/state.h"
#include "../system/execution_policy.h"

static DebugPpuImage snapshot;
static uint32_t pixels[512 * 480];

static int decoding(void) {
    memset(&snapshot, 0, sizeof(snapshot));
    for (unsigned n = 0; n < 64; ++n) {
        snapshot.colors[n] = 0xFF000000u | n;
    }
    for (unsigned n = 0; n < 32; ++n) {
        snapshot.memory[0x3F00 + n] = (uint8_t)n;
    }
    uint8_t planes[16] = {0xA0, 0, 0, 0, 0, 0, 0, 0, 0x60};
    BOARD_CHECK(debug_ppu_pixel(planes, 0, 0) == 1);
    BOARD_CHECK(debug_ppu_pixel(planes, 1, 0) == 2);
    BOARD_CHECK(debug_ppu_pixel(planes, 2, 0) == 3);
    BOARD_CHECK(debug_ppu_pixel(planes, 3, 0) == 0);
    BOARD_CHECK(debug_ppu_color(&snapshot, 3, 0, false) == snapshot.colors[0]);
    BOARD_CHECK(debug_ppu_color(&snapshot, 4, 0, true) == 0);
    memcpy(snapshot.memory, planes, 16);
    memcpy(snapshot.memory + 0x1000, planes, 16);
    debug_ppu_patterns(&snapshot, snapshot.memory, 2, pixels);
    BOARD_CHECK(pixels[0] == snapshot.colors[9] && pixels[128] == pixels[0]);
    BOARD_CHECK(pixels[1] == snapshot.colors[10] && pixels[2] == snapshot.colors[11]);
    snapshot.nt_palettes[3 * 960] = 3;
    snapshot.memory[0x2C00] = 0x21;
    memcpy(snapshot.nt_planes[3 * 960], planes, 16);
    DebugPpuSelection n = debug_ppu_nametable(&snapshot, 256, 240);
    BOARD_CHECK(n.nametable == 0x2C00 && n.attribute == 0x2FC0 && n.pattern == 0x210 && n.palette == 3);
    debug_ppu_nametables(&snapshot, false, pixels);
    BOARD_CHECK(pixels[240 * 512 + 256] == snapshot.colors[13]);
    debug_ppu_nametables(&snapshot, true, pixels);
    BOARD_CHECK(pixels[240 * 512 + 260] == snapshot.colors[13]);
    BOARD_CHECK(pixels[244 * 512 + 256] == snapshot.colors[14]);
    BOARD_CHECK(pixels[244 * 512 + 260] == snapshot.colors[15]);
    memset(snapshot.oam, 255, sizeof(snapshot.oam));
    snapshot.state.ctrl = 0x20;
    snapshot.oam[0] = 9;
    snapshot.oam[1] = 3;
    snapshot.oam[2] = 0xE2;
    snapshot.oam[3] = 250;
    DebugPpuSelection s = debug_ppu_sprite(&snapshot, 0);
    BOARD_CHECK(s.pattern == 0x1020 && s.height == 16 && s.flip_x && s.flip_y && s.behind);
    BOARD_CHECK(s.x == 250 && s.y == 10 && s.palette == 6);
    snapshot.sprites[0x1037] = 1; /* Bottom-right becomes top-left after both flips. */
    debug_ppu_sprites(&snapshot, true, pixels);
    BOARD_CHECK(pixels[10 * 256 + 250] == snapshot.colors[25]);
    BOARD_CHECK(pixels[10 * 256] != snapshot.colors[25]);
    snapshot.oam[0] = 255;
    debug_ppu_sprites(&snapshot, true, pixels);
    BOARD_CHECK(pixels[250] != snapshot.colors[25]);
    snapshot.state.ctrl = 0;
    snapshot.oam[0] = snapshot.oam[4] = 0;
    snapshot.oam[1] = snapshot.oam[5] = 0;
    snapshot.oam[2] = 0;
    snapshot.oam[6] = 1;
    snapshot.oam[3] = snapshot.oam[7] = 0;
    snapshot.sprites[0] = 0x80;
    debug_ppu_sprites(&snapshot, true, pixels);
    BOARD_CHECK(pixels[256] == snapshot.colors[17]);
    return 0;
}

static int capture_unchanged(void) {
    NesStateBlob before = {0}, after = {0};
    BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
    debug_ppu_capture(&snapshot, true);
    BOARD_CHECK(nes_state_capture(&after) == NES_STATE_OK);
    bool same = before.size == after.size && !memcmp(before.data, after.data, before.size);
    nes_state_blob_free(&before);
    nes_state_blob_free(&after);
    BOARD_CHECK(same);
    return 0;
}

static int writable_memory(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0, false));
    apu_power_on(&apu);
    BOARD_CHECK(board_image_load(&image) == 0);
    apu_power_on(&apu);
    debugger_init();
    BOARD_CHECK(capture_unchanged() == 0);
    BOARD_CHECK(!debug_ppu_write(false, 0, 0));
    debugger_pause();
    BOARD_CHECK(debug_ppu_write(false, 0, 0x55));
    BOARD_CHECK(debug_ppu_write(false, 8, 0xAA));
    BOARD_CHECK(debug_ppu_paint(0, 0, 0, 3));
    BOARD_CHECK(debugger_peek_ppu(0) == 0xD5 && debugger_peek_ppu(8) == 0xAA);
    BOARD_CHECK(debug_ppu_paint(0, 7, 0, 2));
    BOARD_CHECK(debugger_peek_ppu(0) == 0xD4 && debugger_peek_ppu(8) == 0xAB);
    BOARD_CHECK(!debug_ppu_paint(1, 0, 0, 1) && !debug_ppu_paint(0, 8, 0, 1));
    BOARD_CHECK(debug_ppu_write(false, 0x3F10, 0xFF));
    BOARD_CHECK(debugger_peek_ppu(0x3F00) == 0x3F && debugger_peek_ppu(0x3F30) == 0x3F);
    BOARD_CHECK(debug_ppu_write(true, 2, 0xFF) && ppu.oam[2] == 0xE3);
    BOARD_CHECK(!debug_ppu_write(true, 256, 1) && !debug_ppu_write(false, 0x4000, 1));
    BOARD_CHECK(debug_ppu_write(false, 0x2000, 0x12));
    BOARD_CHECK(debugger_peek_ppu(0x3000) == 0x12);
    BOARD_CHECK(debug_ppu_write(false, 0x23C0, 0xE4));
    debug_ppu_capture(&snapshot, true);
    BOARD_CHECK(snapshot.nt_palettes[0] == 0 && snapshot.nt_palettes[2] == 1);
    BOARD_CHECK(snapshot.nt_palettes[64] == 2 && snapshot.nt_palettes[66] == 3);
    for (unsigned policy = 1; policy <= NES_EXECUTION_REWIND; policy <<= 1) {
        BOARD_CHECK(nes_execution_set_policy(policy));
        BOARD_CHECK(!debug_ppu_write(false, 0, 0) && !debug_ppu_paint(0, 0, 0, 0));
    }
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0x2000, false));
    apu_power_on(&apu);
    BOARD_CHECK(board_image_load(&image) == 0);
    debugger_init();
    debugger_pause();
    BOARD_CHECK(!debug_ppu_paint(0, 0, 0, 3));
    BOARD_CHECK(debugger_peek_ppu(0) == 0);
    board_image_free(&image);
    debugger_shutdown();
    return 0;
}

static bool store(uint16_t address, uint8_t value) {
    write_mem(0x200, 0xA9);
    write_mem(0x201, value);
    write_mem(0x202, 0x8D);
    write_mem(0x203, (uint8_t)address);
    write_mem(0x204, (uint8_t)(address >> 8));
    cpu.pc = 0x200;
    return cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4;
}

static int extended_banks(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 5, 0x8000, 0x8000, true));
    apu_power_on(&apu);
    BOARD_CHECK(board_image_load(&image) == 0);
    apu_power_on(&apu);
    debugger_init();
    BOARD_CHECK(store(0x5104, 2) && store(0x5C05, 0x83) && store(0x5104, 1));
    ppu_vram[5] = 0x2A;
    (void)ppu_read(0x2000);
    (void)ppu_read(0x2000);
    (void)ppu_read(0x2000);
    (void)ppu_read(0x23C0);
    (void)ppu_read(0x2001);
    BOARD_CHECK(ppu_read(0x2005) == 0x2A && ppu_read(0x23C0) == 0xAA);
    BOARD_CHECK(capture_unchanged() == 0);
    BOARD_CHECK(snapshot.nt_palettes[5] == 2 && snapshot.nt_planes[5][0] == 12);
    BOARD_CHECK(ppu_read(0x0123) == 12 && ppu_read(0x012B) == 12);
    BOARD_CHECK(ppu_read(0x0123) == 0);
    BOARD_CHECK(store(0x5104, 0) && store(0x5101, 3));
    BOARD_CHECK(store(0x5120, 2) && store(0x5128, 7));
    cart_notify_ppu_ctrl_write(0x20);
    BOARD_CHECK(cart_debug_chr(0, CART_PPU_FETCH_SPRITE) == 2);
    BOARD_CHECK(cart_debug_chr(0, CART_PPU_FETCH_BG) == 7);
    BOARD_CHECK(capture_unchanged() == 0);
    board_image_free(&image);
    debugger_shutdown();
    return 0;
}

static int mapped_writes(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 111, 0x8000, 0, false));
    apu_power_on(&apu);
    BOARD_CHECK(board_image_load(&image) == 0);
    debugger_init();
    debugger_pause();
    BOARD_CHECK(debug_ppu_write(false, 0x2000, 0x12));
    BOARD_CHECK(debug_ppu_write(false, 0x3000, 0x34));
    BOARD_CHECK(debugger_peek_ppu(0x2000) == 0x12 && debugger_peek_ppu(0x3000) == 0x34);
    board_image_free(&image);
    debugger_shutdown();
    BOARD_CHECK(board_image_create(&image, 209, 0x8000, 0x2000, false));
    apu_power_on(&apu);
    BOARD_CHECK(board_image_load(&image) == 0);
    debugger_init();
    debugger_pause();
    cart_cpu_write(0xD000, 0x60);
    cart_cpu_write(0xB000, 0);
    cart_cpu_write(0xB004, 0);
    ppu_vram[0x123] = 0x5A;
    BOARD_CHECK(!debug_ppu_write(false, 0x2123, 0xA5));
    BOARD_CHECK(ppu_vram[0x123] == 0x5A && debugger_peek_ppu(0x2123) == 0);
    cart_cpu_write(0xD000, 0x20);
    BOARD_CHECK(debug_ppu_write(false, 0x2123, 0xA5) && ppu_vram[0x123] == 0xA5);
    board_image_free(&image);
    debugger_shutdown();
    return 0;
}

int test_ppu_inspector_accuracy(void) {
    int failures = decoding() + writable_memory() + extended_banks() + mapped_writes();
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    printf("PPU inspector: %s (%d groups failed)\n", failures ? "FAIL" : "PASS", failures);
    return failures;
}
