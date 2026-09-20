/*
 * ppu_state_impl.h - Private PPU save-state serialization
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_PPU_STATE_IMPL_H
#define CUPID_PPU_STATE_IMPL_H

static bool ppu_state_write_core(NesStateWriter *writer, const PPU *state) {
    if (!writer || !state) return false;
    if (!nes_state_write_u8(writer, state->ctrl)
        || !nes_state_write_u8(writer, state->mask)
        || !nes_state_write_u8(writer, state->status)
        || !nes_state_write_u8(writer, state->oam_addr)
        || !nes_state_write_u8(writer, state->scroll_x)
        || !nes_state_write_u8(writer, state->scroll_y)
        || !nes_state_write_u16(writer, state->v)
        || !nes_state_write_u16(writer, state->t)
        || !nes_state_write_u8(writer, state->x)
        || !nes_state_write_u8(writer, state->w)
        || !nes_state_write_u8(writer, state->ppudata_buffer)
        || !nes_state_write_u8(writer, state->open_bus)
        || !nes_state_write_u16(writer, state->bus_address)
        || !nes_state_write_u8(writer, state->vram_address_latch)
        || !nes_state_write_u8(writer, state->vram_bus_data)
        || !nes_state_write_bool(writer, state->bus_ale_this_dot)
        || !nes_state_write_bool(writer, state->bus_read_this_dot)
        || !nes_state_write_u16(writer, state->address_write_value)
        || !nes_state_write_u8(writer, state->address_write_delay)
        || !nes_state_write_u8(writer, state->data_read_delay)
        || !nes_state_write_u8(writer, state->data_write_delay)
        || !nes_state_write_u8(writer, state->data_write_value)
        || !nes_state_write_u8(writer, state->data_read_cooldown)
        || !nes_state_write_bool(writer, state->data_increment_pending)
        || !nes_state_write_bytes(writer, state->oam, sizeof(state->oam))
        || !nes_state_write_bytes(writer, state->secondary_oam, sizeof(state->secondary_oam))
        || !nes_state_write_u8(writer, state->sprite_count)
        || !nes_state_write_bytes(writer, state->sprite_positions, sizeof(state->sprite_positions))
        || !nes_state_write_bytes(writer, state->sprite_pattern_lo, sizeof(state->sprite_pattern_lo))
        || !nes_state_write_bytes(writer, state->sprite_pattern_hi, sizeof(state->sprite_pattern_hi))
        || !nes_state_write_bytes(writer, state->sprite_attributes, sizeof(state->sprite_attributes))) return false;
    for (unsigned i = 0; i < 8; ++i)
        if (!nes_state_write_u16(writer, state->sprite_start_dot[i])) return false;
    if (!nes_state_write_u8(writer, state->sprite_valid_mask)
        || !nes_state_write_u8(writer, state->sprite_active_mask)
        || !nes_state_write_u8(writer, state->sprite_counting_mask)
        || !nes_state_write_u8(writer, state->sprite_expired_mask)
        || !nes_state_write_u8(writer, state->sprite_skip_clocks)
        || !nes_state_write_u8(writer, state->sprite_status_pending)
        || !nes_state_write_bool(writer, state->sprite_zero_hit)
        || !nes_state_write_bool(writer, state->sprite_zero_on_line)
        || !nes_state_write_bool(writer, state->nmi_out)
        || !nes_state_write_u8(writer, state->oam_bus)
        || !nes_state_write_u8(writer, state->oam_read_latch)) return false;
    for (unsigned i = 0; i < 32; ++i)
        if (!nes_state_write_u64(writer, state->oam_decay_cycles[i])) return false;
    if (!nes_state_write_u8(writer, state->secondary_index)
        || !nes_state_write_u8(writer, state->overflow_count)
        || !nes_state_write_bool(writer, state->eval_in_range)
        || !nes_state_write_bool(writer, state->eval_done)
        || !nes_state_write_bool(writer, state->secondary_oam_full)
        || !nes_state_write_bool(writer, state->secondary_oam_overflowed)
        || !nes_state_write_bool(writer, state->secondary_sprite_zero)
        || !nes_state_write_u8(writer, state->sprite_fetch_y)
        || !nes_state_write_u8(writer, state->sprite_fetch_tile)
        || !nes_state_write_u8(writer, state->sprite_fetch_attr)
        || !nes_state_write_u8(writer, state->sprite_fetch_x)
        || !nes_state_write_u16(writer, state->sprite_fetch_addr)
        || !nes_state_write_bool(writer, state->sprite_fetch_valid)
        || !nes_state_write_bool(writer, state->oam_corruption_pending)
        || !nes_state_write_u8(writer, state->oam_corruption_source_row)
        || !nes_state_write_u8(writer, state->oam_corruption_dest_row)
        || !nes_state_write_bool(writer, state->suppress_vblank)
        || !nes_state_write_bool(writer, state->rendering_enabled)
        || !nes_state_write_bool(writer, state->fetches_enabled)
        || !nes_state_write_bool(writer, state->startup_writes_restricted)
        || !nes_state_write_bool(writer, state->skipped_frame_dot)
        || !nes_state_write_u32(writer, (uint32_t)(int32_t)state->scanline)
        || !nes_state_write_u32(writer, (uint32_t)(int32_t)state->dot)
        || !nes_state_write_bool(writer, state->odd_frame)
        || !nes_state_write_bool(writer, state->frame_complete)
        || !nes_state_write_u64(writer, state->frame_count)
        || !nes_state_write_u64(writer, state->total_cycles)
        || !nes_state_write_u32(writer, state->cpu_clock_phase)
        || !nes_state_write_u8(writer, state->frame_video_phase)
        || !nes_state_write_u8(writer, state->completed_video_phase)
        || !nes_state_write_u8(writer, state->nt_byte)
        || !nes_state_write_u8(writer, state->at_byte)
        || !nes_state_write_u8(writer, state->pt_lo)
        || !nes_state_write_u8(writer, state->pt_hi)
        || !nes_state_write_u16(writer, state->bg_tile_addr)
        || !nes_state_write_u16(writer, state->bg_shift_lo)
        || !nes_state_write_u16(writer, state->bg_shift_hi)
        || !nes_state_write_u16(writer, state->at_shift_lo)
        || !nes_state_write_u16(writer, state->at_shift_hi)
        || !nes_state_write_u8(writer, state->at_latch_lo)
        || !nes_state_write_u8(writer, state->at_latch_hi)
        || !nes_state_write_bytes(writer, state->pixel_indices, sizeof(state->pixel_indices))) return false;
    for (size_t i = 0; i < sizeof(state->pixel_signal) / sizeof(state->pixel_signal[0]); ++i)
        if (!nes_state_write_u16(writer, state->pixel_signal[i])) return false;
    return true;
}

static bool ppu_state_read_core(NesStateReader *reader, PPU *state) {
    uint32_t scanline, dot;
    memset(state, 0, sizeof(*state));
    if (!nes_state_read_u8(reader, &state->ctrl)
        || !nes_state_read_u8(reader, &state->mask)
        || !nes_state_read_u8(reader, &state->status)
        || !nes_state_read_u8(reader, &state->oam_addr)
        || !nes_state_read_u8(reader, &state->scroll_x)
        || !nes_state_read_u8(reader, &state->scroll_y)
        || !nes_state_read_u16(reader, &state->v)
        || !nes_state_read_u16(reader, &state->t)
        || !nes_state_read_u8(reader, &state->x)
        || !nes_state_read_u8(reader, &state->w)
        || !nes_state_read_u8(reader, &state->ppudata_buffer)
        || !nes_state_read_u8(reader, &state->open_bus)
        || !nes_state_read_u16(reader, &state->bus_address)
        || !nes_state_read_u8(reader, &state->vram_address_latch)
        || !nes_state_read_u8(reader, &state->vram_bus_data)
        || !nes_state_read_bool(reader, &state->bus_ale_this_dot)
        || !nes_state_read_bool(reader, &state->bus_read_this_dot)
        || !nes_state_read_u16(reader, &state->address_write_value)
        || !nes_state_read_u8(reader, &state->address_write_delay)
        || !nes_state_read_u8(reader, &state->data_read_delay)
        || !nes_state_read_u8(reader, &state->data_write_delay)
        || !nes_state_read_u8(reader, &state->data_write_value)
        || !nes_state_read_u8(reader, &state->data_read_cooldown)
        || !nes_state_read_bool(reader, &state->data_increment_pending)
        || !nes_state_read_bytes(reader, state->oam, sizeof(state->oam))
        || !nes_state_read_bytes(reader, state->secondary_oam, sizeof(state->secondary_oam))
        || !nes_state_read_u8(reader, &state->sprite_count)
        || !nes_state_read_bytes(reader, state->sprite_positions, sizeof(state->sprite_positions))
        || !nes_state_read_bytes(reader, state->sprite_pattern_lo, sizeof(state->sprite_pattern_lo))
        || !nes_state_read_bytes(reader, state->sprite_pattern_hi, sizeof(state->sprite_pattern_hi))
        || !nes_state_read_bytes(reader, state->sprite_attributes, sizeof(state->sprite_attributes))) return false;
    for (unsigned i = 0; i < 8; ++i)
        if (!nes_state_read_u16(reader, &state->sprite_start_dot[i])) return false;
    if (!nes_state_read_u8(reader, &state->sprite_valid_mask)
        || !nes_state_read_u8(reader, &state->sprite_active_mask)
        || !nes_state_read_u8(reader, &state->sprite_counting_mask)
        || !nes_state_read_u8(reader, &state->sprite_expired_mask)
        || !nes_state_read_u8(reader, &state->sprite_skip_clocks)
        || !nes_state_read_u8(reader, &state->sprite_status_pending)
        || !nes_state_read_bool(reader, &state->sprite_zero_hit)
        || !nes_state_read_bool(reader, &state->sprite_zero_on_line)
        || !nes_state_read_bool(reader, &state->nmi_out)
        || !nes_state_read_u8(reader, &state->oam_bus)
        || !nes_state_read_u8(reader, &state->oam_read_latch)) return false;
    for (unsigned i = 0; i < 32; ++i)
        if (!nes_state_read_u64(reader, &state->oam_decay_cycles[i])) return false;
    if (!nes_state_read_u8(reader, &state->secondary_index)
        || !nes_state_read_u8(reader, &state->overflow_count)
        || !nes_state_read_bool(reader, &state->eval_in_range)
        || !nes_state_read_bool(reader, &state->eval_done)
        || !nes_state_read_bool(reader, &state->secondary_oam_full)
        || !nes_state_read_bool(reader, &state->secondary_oam_overflowed)
        || !nes_state_read_bool(reader, &state->secondary_sprite_zero)
        || !nes_state_read_u8(reader, &state->sprite_fetch_y)
        || !nes_state_read_u8(reader, &state->sprite_fetch_tile)
        || !nes_state_read_u8(reader, &state->sprite_fetch_attr)
        || !nes_state_read_u8(reader, &state->sprite_fetch_x)
        || !nes_state_read_u16(reader, &state->sprite_fetch_addr)
        || !nes_state_read_bool(reader, &state->sprite_fetch_valid)
        || !nes_state_read_bool(reader, &state->oam_corruption_pending)
        || !nes_state_read_u8(reader, &state->oam_corruption_source_row)
        || !nes_state_read_u8(reader, &state->oam_corruption_dest_row)
        || !nes_state_read_bool(reader, &state->suppress_vblank)
        || !nes_state_read_bool(reader, &state->rendering_enabled)
        || !nes_state_read_bool(reader, &state->fetches_enabled)
        || !nes_state_read_bool(reader, &state->startup_writes_restricted)
        || !nes_state_read_bool(reader, &state->skipped_frame_dot)
        || !nes_state_read_u32(reader, &scanline)
        || !nes_state_read_u32(reader, &dot)
        || !nes_state_read_bool(reader, &state->odd_frame)
        || !nes_state_read_bool(reader, &state->frame_complete)
        || !nes_state_read_u64(reader, &state->frame_count)
        || !nes_state_read_u64(reader, &state->total_cycles)
        || !nes_state_read_u32(reader, &state->cpu_clock_phase)
        || !nes_state_read_u8(reader, &state->frame_video_phase)
        || !nes_state_read_u8(reader, &state->completed_video_phase)
        || !nes_state_read_u8(reader, &state->nt_byte)
        || !nes_state_read_u8(reader, &state->at_byte)
        || !nes_state_read_u8(reader, &state->pt_lo)
        || !nes_state_read_u8(reader, &state->pt_hi)
        || !nes_state_read_u16(reader, &state->bg_tile_addr)
        || !nes_state_read_u16(reader, &state->bg_shift_lo)
        || !nes_state_read_u16(reader, &state->bg_shift_hi)
        || !nes_state_read_u16(reader, &state->at_shift_lo)
        || !nes_state_read_u16(reader, &state->at_shift_hi)
        || !nes_state_read_u8(reader, &state->at_latch_lo)
        || !nes_state_read_u8(reader, &state->at_latch_hi)
        || !nes_state_read_bytes(reader, state->pixel_indices, sizeof(state->pixel_indices))) return false;
    for (size_t i = 0; i < sizeof(state->pixel_signal) / sizeof(state->pixel_signal[0]); ++i)
        if (!nes_state_read_u16(reader, &state->pixel_signal[i])) return false;
    state->scanline = (int)(int32_t)scanline;
    state->dot = (int)(int32_t)dot;
    if (state->scanline < -1 || state->scanline >= (int)nes_timing()->scanlines
        || state->dot < 0 || state->dot > 340
        || state->cpu_clock_phase >= nes_timing()->cpu_divider) return false;
    return true;
}

bool ppu_machine_state_capture(NesStateWriter *writer, const PpuMachineContext *context,
                               const uint32_t *framebuffer_data) {
    if (!writer || !context || !framebuffer_data || !ppu_state_write_core(writer, &context->state)
        || !nes_state_write_bytes(writer, context->vram, sizeof(context->vram))
        || !nes_state_write_bytes(writer, context->palette, sizeof(context->palette))
        || !nes_state_write_bytes(writer, context->bg_opaque, sizeof(context->bg_opaque))) return false;
    for (unsigned i = 0; i < 8; ++i)
        if (!nes_state_write_u64(writer, context->open_bus_expire[i])) return false;
    for (size_t i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; ++i)
        if (!nes_state_write_u32(writer, framebuffer_data[i])) return false;
    return true;
}

bool ppu_machine_state_decode(NesStateReader *reader, PpuMachineContext *context,
                              uint32_t *framebuffer_data) {
    if (!reader || !context || !framebuffer_data || !ppu_state_read_core(reader, &context->state)
        || !nes_state_read_bytes(reader, context->vram, sizeof(context->vram))
        || !nes_state_read_bytes(reader, context->palette, sizeof(context->palette))
        || !nes_state_read_bytes(reader, context->bg_opaque, sizeof(context->bg_opaque))) return false;
    for (unsigned i = 0; i < 8; ++i)
        if (!nes_state_read_u64(reader, &context->open_bus_expire[i])) return false;
    for (size_t i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; ++i)
        if (!nes_state_read_u32(reader, &framebuffer_data[i])) return false;
    return true;
}

typedef struct {
    PpuMachineContext context;
    uint32_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];
    PpuRevision revision;
    bool oam_row_corruption;
    bool startup_restriction;
    bool oam_decay;
    bool reset_suppression;
    bool sprite_wrap_bug;
    bool oamdata_disabled;
    bool palette_disabled;
} PpuSavedState;

static bool ppu_state_decode(NesStateReader *reader, PpuSavedState *saved) {
    uint8_t revision;
    if (!ppu_machine_state_decode(reader, &saved->context, saved->framebuffer)
        || !nes_state_read_u8(reader, &revision)
        || !nes_state_read_bool(reader, &saved->oam_row_corruption)
        || !nes_state_read_bool(reader, &saved->startup_restriction)
        || !nes_state_read_bool(reader, &saved->oam_decay)
        || !nes_state_read_bool(reader, &saved->reset_suppression)
        || !nes_state_read_bool(reader, &saved->sprite_wrap_bug)
        || !nes_state_read_bool(reader, &saved->oamdata_disabled)
        || !nes_state_read_bool(reader, &saved->palette_disabled)
        || revision > PPU_REVISION_2C02_E_PLUS
        || nes_state_reader_remaining(reader) != 0) return false;
    saved->revision = (PpuRevision)revision;
    return true;
}

bool ppu_state_capture(NesStateWriter *writer) {
    if (!writer || active_ppu != main_ppu) return false;
    PpuMachineContext context;
    context.state = *main_ppu;
    memcpy(context.vram, ppu_vram, sizeof(context.vram));
    memcpy(context.palette, ppu_palette, sizeof(context.palette));
    memcpy(context.bg_opaque, bg_opaque, sizeof(context.bg_opaque));
    memcpy(context.open_bus_expire, main_ppu_ob_expire, sizeof(context.open_bus_expire));
    return ppu_machine_state_capture(writer, &context, framebuffer)
        && nes_state_write_u8(writer, (uint8_t)active_ppu_revision)
        && nes_state_write_bool(writer, oam_row_corruption_worst_case)
        && nes_state_write_bool(writer, startup_write_restriction)
        && nes_state_write_bool(writer, oam_decay)
        && nes_state_write_bool(writer, reset_suppression)
        && nes_state_write_bool(writer, sprite_eval_wrap_bug)
        && nes_state_write_bool(writer, oamdata_read_disabled)
        && nes_state_write_bool(writer, palette_readback_disabled);
}

bool ppu_state_validate(NesStateReader *reader) {
    static PpuSavedState saved;
    return reader && ppu_state_decode(reader, &saved);
}

bool ppu_state_apply(NesStateReader *reader) {
    static PpuSavedState saved;
    if (!reader || !ppu_state_decode(reader, &saved)) return false;
    ppu_select_machine(NULL, framebuffer);
    *main_ppu = saved.context.state;
    memcpy(ppu_vram, saved.context.vram, sizeof(ppu_vram));
    memcpy(ppu_palette, saved.context.palette, sizeof(ppu_palette));
    memcpy(bg_opaque, saved.context.bg_opaque, sizeof(bg_opaque));
    memcpy(main_ppu_ob_expire, saved.context.open_bus_expire, sizeof(main_ppu_ob_expire));
    memcpy(framebuffer, saved.framebuffer, sizeof(saved.framebuffer));
    active_ppu_revision = saved.revision;
    oam_row_corruption_worst_case = saved.oam_row_corruption;
    startup_write_restriction = saved.startup_restriction;
    oam_decay = saved.oam_decay;
    reset_suppression = saved.reset_suppression;
    sprite_eval_wrap_bug = saved.sprite_wrap_bug;
    oamdata_read_disabled = saved.oamdata_disabled;
    palette_readback_disabled = saved.palette_disabled;
    return true;
}

#endif
