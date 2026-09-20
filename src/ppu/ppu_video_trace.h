/*
 * ppu_video_trace.h - Copy completed fetch metadata without changing PPU state
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_PPU_VIDEO_TRACE_H
#define CUPID_PPU_VIDEO_TRACE_H

typedef struct {
    uint64_t generation;
    NesVideoTile pending;
    NesVideoTile background[16];
    NesVideoTile sprites[8];
    uint8_t sprite_x[8];
} PpuPresentationFetches;

static PpuPresentationFetches presentation_fetches[2];

static PpuPresentationFetches *ppu_trace_fetches(void) {
    unsigned side = vs_active_side();
    PpuPresentationFetches *fetches = &presentation_fetches[side];
    uint64_t generation = nes_video_trace_generation(side);
    if (fetches->generation != generation) {
        memset(fetches, 0, sizeof(*fetches));
        fetches->generation = generation;
    }
    return fetches;
}

static void ppu_trace_pattern(bool sprite, unsigned index, bool high, uint8_t value) {
    PpuPresentationFetches *fetches = ppu_trace_fetches();
    NesVideoTile *tile = sprite ? &fetches->sprites[index & 7u] : &fetches->pending;
    const NesVideoChr *read = nes_video_trace_last_read(vs_active_side());
    if (!high) {
        memset(tile, 0, sizeof(*tile));
        if (read) tile->chr = *read;
        tile->address = sprite ? ppu.sprite_fetch_addr : ppu.bg_tile_addr;
        tile->y = read && read->valid ? read->offset & 7u : tile->address & 7u;
        if (sprite) {
            tile->attributes = ppu.sprite_fetch_attr;
            fetches->sprite_x[index & 7u] = 0;
            tile->chr.valid = tile->chr.valid && ppu.sprite_fetch_valid;
        }
    } else if (!read || !read->valid || read->index != tile->chr.index || read->ram != tile->chr.ram
               || memcmp(read->bytes, tile->chr.bytes, 16)) {
        /* Two planes from different banks do not identify one replacement
         * tile. The original pixels still render normally. */
        tile->chr.valid = false;
    }
    if (read && read->valid && value != read->bytes[read->offset]) tile->chr.valid = false;
}

static void ppu_trace_shift_background(void) {
    PpuPresentationFetches *fetches = ppu_trace_fetches();
    memmove(fetches->background + 1, fetches->background, 15 * sizeof(NesVideoTile));
    memset(&fetches->background[0], 0, sizeof(NesVideoTile));
}

static void ppu_trace_load_background(void) {
    PpuPresentationFetches *fetches = ppu_trace_fetches();
    for (unsigned bit = 0; bit < 8; ++bit) {
        fetches->background[bit] = fetches->pending;
        fetches->background[bit].x = (uint8_t)(7u - bit);
    }
}

static uint32_t ppu_trace_palette(unsigned offset, bool sprite) {
    return ((uint32_t)(sprite ? 0xFFu : active_ppu_palette[0]) << 24)
        | ((uint32_t)active_ppu_palette[offset + 1] << 16)
        | ((uint32_t)active_ppu_palette[offset + 2] << 8)
        | active_ppu_palette[offset + 3];
}

static NesVideoPixel *ppu_trace_begin_pixel(int x, int y, uint8_t background, uint8_t palette) {
    NesVideoPixel *pixel = nes_video_trace_pixel(vs_active_side(), (unsigned)x, (unsigned)y);
    if (!pixel) return NULL;
    PpuPresentationFetches *fetches = ppu_trace_fetches();
    pixel->background = fetches->background[15u - ppu.x];
    pixel->background.color_index = background;
    pixel->background.palette_offset = (uint8_t)(palette * 4u);
    pixel->background.palette = ppu_trace_palette(palette * 4u, false);
    pixel->background.color = active_ppu_palette[palette * 4u + background];
    pixel->background.rgb = get_color(pixel->background.color);
    if (!(ppu.mask & 8) || (x < 8 && !(ppu.mask & 2))) pixel->background.chr.valid = false;
    pixel->backdrop = active_ppu_palette[0];
    pixel->backdrop_rgb = get_color(pixel->backdrop);
    pixel->mask = ppu.mask;
    pixel->scroll_address = ppu.t;
    pixel->fine_x = ppu.x;
    return pixel;
}

static void ppu_trace_sprite_pixel(NesVideoPixel *pixel, unsigned index, uint8_t color, bool visible) {
    PpuPresentationFetches *fetches = ppu_trace_fetches();
    if (visible && pixel && pixel->sprite_count < NES_VIDEO_TRACE_SPRITES) {
        NesVideoTile *sprite = &pixel->sprites[pixel->sprite_count++];
        *sprite = fetches->sprites[index];
        if (fetches->sprite_x[index] >= 8) sprite->chr.valid = false;
        sprite->x = fetches->sprite_x[index] & 7u;
        sprite->color_index = color;
        sprite->attributes = ppu.sprite_attributes[index];
        sprite->palette_offset = (uint8_t)(0x10u + (sprite->attributes & 3u) * 4u);
        sprite->palette = ppu_trace_palette(sprite->palette_offset, true);
        sprite->color = active_ppu_palette[sprite->palette_offset + color];
        sprite->rgb = get_color(sprite->color);
    }
    if (fetches->sprite_x[index] < 8) ++fetches->sprite_x[index];
}

static void ppu_trace_blank_sprite(NesVideoPixel *pixel, unsigned index, int x, bool visible) {
    if (!visible || !pixel || !(ppu.sprite_valid_mask & (1u << index))) return;
    int offset = x + 1 - ppu.sprite_start_dot[index];
    if (offset < 0 || offset >= 8 || pixel->sprite_count >= NES_VIDEO_TRACE_SPRITES) return;
    PpuPresentationFetches *fetches = ppu_trace_fetches();
    unsigned previous = fetches->sprite_x[index];
    fetches->sprite_x[index] = (uint8_t)offset;
    ppu_trace_sprite_pixel(pixel, index, 0, true);
    fetches->sprite_x[index] = (uint8_t)previous;
}

#endif
