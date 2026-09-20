/*
 * video_trace_accuracy.c - Presentation tracing preserves machine execution
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../rom/fds.h"
#include "../state/state.h"
#include "../system/hardware.h"
#include "../system/vs_system.h"
#include "../ui/frontend_execution.h"
#include "../video/video_trace.h"
#include "../../include/globals.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        failed = 1; \
        goto cleanup; \
    } \
} while (0)

static bool prepare_display(NesRegion region, bool dual) {
    BoardImage image = {0};
    size_t prg_size = dual ? 0x10000u : 0x8000u;
    size_t chr_size = dual ? 0x8000u : 0x2000u;
    if (!board_image_create(&image, dual ? 99 : 0, prg_size, chr_size, !dual)) return false;
    if (dual) {
        image.data[6] = 0x30;
        image.data[7] = 0x61;
    } else {
        image.data[12] = region == NES_REGION_PAL ? 1 : region == NES_REGION_DENDY ? 3 : 0;
    }
    uint8_t *prg = image.data + 16;
    memset(prg, 0xEA, prg_size);
    for (size_t side = 0; side < (dual ? 2u : 1u); ++side) {
        size_t base = side * 0x8000;
        const uint8_t program[] = {0xA9,0x1E,0x8D,1,0x20,0x4C,5,0x80};
        memcpy(prg + base, program, sizeof(program));
        for (size_t vector = 0x7FFA; vector <= 0x7FFE; vector += 2) {
            prg[base + vector] = 0;
            prg[base + vector + 1] = 0x80;
        }
    }
    uint8_t *chr = prg + prg_size;
    for (size_t offset = 0; offset < chr_size; offset += 16) {
        for (unsigned row = 0; row < 8; ++row) {
            chr[offset + row] = (uint8_t)((offset / 16) & 1u ? 0x81u >> (row & 3u) : 0xFF);
            chr[offset + row + 8] = (uint8_t)((offset / 16) & 1u ? 0x7E : 0);
        }
    }
    bool ok = nes_set_region_mode(NES_REGION_MODE_AUTO) && load_rom_memory(image.data, image.size) == 0;
    board_image_free(&image);
    if (!ok) return false;
    cpu_select_machine(NULL);
    cpu_use_default_startup_alignment();
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    if (!cpu_power_on(&cpu)) return false;
    vs_power_on_secondary();
    vs_audio_init(44100);
    memset(ppu_vram, 0, NT_RAM_SIZE);
    memset(ppu.oam, 0xFF, sizeof(ppu.oam));
    for (unsigned i = 0; i < 4; ++i) {
        ppu.oam[i * 4] = 20;
        ppu.oam[i * 4 + 1] = 1;
        ppu.oam[i * 4 + 2] = (uint8_t)(i == 0 ? 0x20 : i == 2 ? 0x40 : i == 3 ? 0x80 : 0);
        ppu.oam[i * 4 + 3] = (uint8_t)(30 + i * 20);
    }
    ppu_write(0x3F00, 0x0F);
    ppu_write(0x3F01, 0x21);
    ppu_write(0x3F11, 0x16);
    ppu_write(0x3F12, 0x26);
    ppu_write(0x3F13, 0x36);
    write_mem(0x2005, 3);
    write_mem(0x2005, 0);
    return true;
}

static unsigned tile_pixel(const NesVideoTile *tile, bool sprite) {
    unsigned x = tile->x;
    if (sprite && (tile->attributes & 0x40)) x = 7u - x;
    unsigned bit = 7u - x;
    return ((tile->chr.bytes[tile->y] >> bit) & 1u)
        | (((tile->chr.bytes[tile->y + 8u] >> bit) & 1u) << 1);
}

static int test_hardware_invariance(void) {
    int failed = 0;
    NesStateBlob initial = {0}, ordinary = {0}, traced = {0};
    FrontendExecutionRuntime execution;
    for (unsigned configuration = 0; configuration < 4; ++configuration) {
        bool dual = configuration == 3;
        NesRegion region = dual ? NES_REGION_NTSC : (NesRegion)configuration;
        nes_video_trace_shutdown();
        CHECK(prepare_display(region, dual));
        frontend_execution_init(&execution, NULL, 44100, "trace.nes", NULL, NULL, NULL);
        CHECK(frontend_execution_run_frame(&execution));
        CHECK(nes_state_capture(&initial) == NES_STATE_OK);
        for (unsigned frame = 0; frame < 3; ++frame) CHECK(frontend_execution_run_frame(&execution));
        CHECK(nes_state_capture(&ordinary) == NES_STATE_OK);
        CHECK(nes_state_restore(initial.data, initial.size) == NES_STATE_OK);
        CHECK(nes_video_trace_use(NES_VIDEO_TRACE_HD, true));
        CHECK(nes_video_trace_use(NES_VIDEO_TRACE_LAYERS, true));
        for (unsigned frame = 0; frame < 3; ++frame) CHECK(frontend_execution_run_frame(&execution));
        CHECK(nes_state_capture(&traced) == NES_STATE_OK);
        CHECK(ordinary.size == traced.size && !memcmp(ordinary.data, traced.data, ordinary.size));
        const uint32_t *raw = vs_video_framebuffer();
        unsigned background_pixels = 0, sprite_pixels = 0;
        for (unsigned side = 0; side < (dual ? 2u : 1u); ++side) {
            const NesVideoTraceFrame *frame = nes_video_trace_frame(side);
            CHECK(frame && frame->complete && frame->frame_number == vs_side_frame_count(side));
            for (unsigned y = 0; y < 240; ++y) {
                for (unsigned x = 0; x < 256; ++x) {
                    const NesVideoPixel *pixel = frame->pixels + y * 256u + x;
                    CHECK(pixel->original_rgb == raw[y * vs_video_width() + side * 256u + x]);
                    CHECK(nes_video_trace_layer_rgb(pixel, true, true) == pixel->original_rgb);
                    CHECK(nes_video_trace_layer_signal(pixel, true, true) == pixel->original_signal);
                    CHECK(nes_video_trace_layer_rgb(pixel, false, false) == pixel->backdrop_rgb);
                    if (pixel->background.chr.valid) {
                        CHECK(pixel->background.x < 8 && pixel->background.y < 8);
                        CHECK(tile_pixel(&pixel->background, false) == pixel->background.color_index);
                        ++background_pixels;
                    }
                    for (unsigned i = 0; i < pixel->sprite_count; ++i) {
                        const NesVideoTile *sprite = &pixel->sprites[i];
                        if (!sprite->chr.valid) continue;
                        CHECK(sprite->x < 8 && sprite->y < 8);
                        CHECK(tile_pixel(sprite, true) == sprite->color_index);
                        ++sprite_pixels;
                    }
                }
            }
            if (side == 0) {
                const NesVideoPixel *behind = frame->pixels + 21u * 256u + 31u;
                const NesVideoPixel *front = frame->pixels + 21u * 256u + 51u;
                CHECK(behind->background.color_index && behind->selected_sprite_index);
                CHECK(behind->selected_sprite_attributes & 0x20);
                CHECK(behind->original_rgb == behind->background.rgb);
                CHECK(nes_video_trace_layer_rgb(behind, false, true) == behind->selected_sprite_rgb);
                CHECK(front->selected_sprite_index && !(front->selected_sprite_attributes & 0x20));
                CHECK(front->original_rgb == front->selected_sprite_rgb);
                CHECK(nes_video_trace_layer_rgb(front, true, false) == front->background.rgb);
            }
        }
        CHECK(background_pixels >= 60000 && sprite_pixels >= 100);
        CHECK(nes_video_trace_use(NES_VIDEO_TRACE_HD, false) && nes_video_trace_active);
        CHECK(nes_video_trace_use(NES_VIDEO_TRACE_LAYERS, false) && !nes_video_trace_active);
        CHECK(!nes_video_trace_frame(0));
        nes_state_blob_free(&initial); nes_state_blob_free(&ordinary); nes_state_blob_free(&traced);
    }
cleanup:
    nes_state_blob_free(&initial); nes_state_blob_free(&ordinary); nes_state_blob_free(&traced);
    nes_video_trace_shutdown();
    (void)unload_rom();
    return failed;
}

static int test_chr_banking(void) {
    int failed = 0;
    BoardImage image = {0};
    CHECK(nes_video_trace_use(NES_VIDEO_TRACE_HD, true));
    const unsigned mappers[] = {3, 4, 31};
    for (unsigned i = 0; i < sizeof(mappers) / sizeof(mappers[0]); ++i) {
        CHECK(board_image_create(&image, mappers[i], 0x8000, 0x8000, true));
        memset(image.data + 16, 0xFF, 0x8000);
        CHECK(load_rom_memory(image.data, image.size) == 0);
        nes_video_trace_begin_read(0);
        uint16_t address = 0x0123;
        size_t offset = address;
        if (mappers[i] == 3) {
            cart_cpu_write(0x8000, 2);
            offset += 0x4000;
        } else if (mappers[i] == 4) {
            cart_cpu_write(0x8000, 2);
            cart_cpu_write(0x8001, 7);
            address = 0x1023;
            offset = 7 * 0x400 + 0x23;
        }
        uint8_t value = cart_ppu_read(address);
        const NesVideoChr *tile = nes_video_trace_last_read(0);
        CHECK(tile && tile->valid && !tile->ram && tile->index == offset / 16);
        CHECK(tile->offset == (offset & 15u) && tile->bytes[tile->offset] == value);
        CHECK(!memcmp(tile->bytes, image.data + 16 + 0x8000 + (offset & ~(size_t)15), 16));
        board_image_free(&image);
    }
    CHECK(board_image_create(&image, 0, 0x8000, 0, true));
    CHECK(load_rom_memory(image.data, image.size) == 0);
    for (unsigned i = 0; i < 16; ++i) cart_ppu_write((uint16_t)(0x120 + i), (uint8_t)(0x81 + i));
    nes_video_trace_begin_read(0);
    CHECK(cart_ppu_read(0x127) == 0x88);
    const NesVideoChr *tile = nes_video_trace_last_read(0);
    CHECK(tile && tile->valid && tile->ram && tile->index == 0x12);
    for (unsigned i = 0; i < 16; ++i) CHECK(tile->bytes[i] == 0x81 + i);
    uint8_t partial[15] = {0};
    nes_video_trace_chr_read(partial, sizeof(partial), 0, true);
    CHECK(!nes_video_trace_last_read(0)->valid);
    nes_video_trace_chr_read(NULL, SIZE_MAX, SIZE_MAX, false);
    CHECK(!nes_video_trace_last_read(0)->valid);
cleanup:
    board_image_free(&image);
    nes_video_trace_shutdown();
    (void)unload_rom();
    return failed;
}

static int test_disk_trace(void) {
    int failed = 0;
    uint8_t *disk = calloc(65516, 1);
    uint8_t bios[0x2000] = {0};
    CHECK(disk);
    memcpy(disk, "FDS\x1A", 4);
    disk[4] = 1;
    CHECK(load_fds_memory(disk, 65516, bios, sizeof(bios), NULL, false) == 0);
    CHECK(nes_video_trace_use(NES_VIDEO_TRACE_EXPORT, true));
    for (unsigned i = 0; i < 16; ++i) fds_ppu_write((uint16_t)(0x1230 + i), (uint8_t)(i + 10));
    nes_video_trace_begin_read(0);
    CHECK(fds_ppu_read(0x1234) == 14);
    const NesVideoChr *tile = nes_video_trace_last_read(0);
    CHECK(tile && tile->valid && tile->ram && tile->index == 0x123 && tile->offset == 4);
    for (unsigned i = 0; i < 16; ++i) CHECK(tile->bytes[i] == i + 10);
    uint64_t generation = nes_video_trace_generation(0);
    CHECK(!nes_video_trace_use(0x80, true));
    nes_video_trace_reset_side(0);
    CHECK(nes_video_trace_generation(0) != generation && !nes_video_trace_frame(0)->complete);
cleanup:
    free(disk);
    nes_video_trace_shutdown();
    (void)unload_rom();
    return failed;
}

int test_video_trace_accuracy(void) {
    NesRegionMode previous_region = nes_region_mode();
    NesRamPowerOnState previous_ram = nes_ram_power_on_state();
    bool previous_restriction = ppu_startup_write_restriction_enabled();
    (void)nes_set_region_mode(NES_REGION_MODE_AUTO);
    (void)nes_set_ram_power_on_state(NES_RAM_POWER_ZERO);
    ppu_set_startup_write_restriction(false);
    int failures = test_hardware_invariance() + test_chr_banking() + test_disk_trace();
    (void)nes_set_region_mode(previous_region);
    (void)nes_set_ram_power_on_state(previous_ram);
    ppu_set_startup_write_restriction(previous_restriction);
    printf("Video presentation trace: 3 groups, %d failures\n", failures);
    return failures;
}
