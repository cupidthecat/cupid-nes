/*
 * fds_state_impl.h - Private FDS save-state serialization
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_FDS_STATE_IMPL_H
#define CUPID_FDS_STATE_IMPL_H

static uint32_t fds_state_crc32(const uint8_t *data, size_t size) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

static bool fds_state_write_envelope(NesStateWriter *writer, const FdsEnvelope *value) {
    return nes_state_write_u8(writer, value->speed)
        && nes_state_write_u8(writer, value->gain)
        && nes_state_write_bool(writer, value->envelope_off)
        && nes_state_write_bool(writer, value->increase)
        && nes_state_write_u16(writer, value->frequency)
        && nes_state_write_u32(writer, value->timer)
        && nes_state_write_u8(writer, value->master_speed);
}

static bool fds_state_read_envelope(NesStateReader *reader, FdsEnvelope *value) {
    return nes_state_read_u8(reader, &value->speed)
        && nes_state_read_u8(reader, &value->gain)
        && nes_state_read_bool(reader, &value->envelope_off)
        && nes_state_read_bool(reader, &value->increase)
        && nes_state_read_u16(reader, &value->frequency)
        && nes_state_read_u32(reader, &value->timer)
        && nes_state_read_u8(reader, &value->master_speed);
}

static bool fds_state_write_audio(NesStateWriter *writer, const FdsAudio *audio) {
    return nes_state_write_bytes(writer, audio->wave, sizeof(audio->wave))
        && nes_state_write_bool(writer, audio->wave_write_enabled)
        && fds_state_write_envelope(writer, &audio->volume)
        && fds_state_write_envelope(writer, &audio->mod.envelope)
        && nes_state_write_u8(writer, (uint8_t)audio->mod.counter)
        && nes_state_write_bool(writer, audio->mod.disabled)
        && nes_state_write_bytes(writer, audio->mod.table, sizeof(audio->mod.table))
        && nes_state_write_u8(writer, audio->mod.table_position)
        && nes_state_write_u16(writer, audio->mod.overflow)
        && nes_state_write_u32(writer, (uint32_t)audio->mod.output)
        && nes_state_write_bool(writer, audio->disable_envelopes)
        && nes_state_write_bool(writer, audio->halt_waveform)
        && nes_state_write_u8(writer, audio->master_volume)
        && nes_state_write_u16(writer, audio->wave_overflow)
        && nes_state_write_u8(writer, audio->wave_position)
        && nes_state_write_u8(writer, audio->output);
}

static bool fds_state_read_audio(NesStateReader *reader, FdsAudio *audio) {
    uint8_t counter;
    uint32_t output;
    memset(audio, 0, sizeof(*audio));
    if (!nes_state_read_bytes(reader, audio->wave, sizeof(audio->wave))
        || !nes_state_read_bool(reader, &audio->wave_write_enabled)
        || !fds_state_read_envelope(reader, &audio->volume)
        || !fds_state_read_envelope(reader, &audio->mod.envelope)
        || !nes_state_read_u8(reader, &counter)
        || !nes_state_read_bool(reader, &audio->mod.disabled)
        || !nes_state_read_bytes(reader, audio->mod.table, sizeof(audio->mod.table))
        || !nes_state_read_u8(reader, &audio->mod.table_position)
        || !nes_state_read_u16(reader, &audio->mod.overflow)
        || !nes_state_read_u32(reader, &output)
        || !nes_state_read_bool(reader, &audio->disable_envelopes)
        || !nes_state_read_bool(reader, &audio->halt_waveform)
        || !nes_state_read_u8(reader, &audio->master_volume)
        || !nes_state_read_u16(reader, &audio->wave_overflow)
        || !nes_state_read_u8(reader, &audio->wave_position)
        || !nes_state_read_u8(reader, &audio->output)) return false;
    audio->mod.counter = (int8_t)counter;
    audio->mod.output = (int32_t)output;
    if (audio->mod.table_position >= 64 || audio->master_volume > 3
        || audio->wave_position >= 64) return false;
    for (unsigned i = 0; i < 64; ++i)
        if (audio->wave[i] > 0x3F || audio->mod.table[i] > 7) return false;
    return true;
}

typedef struct {
    uint8_t work_ram[FDS_WORK_RAM_SIZE];
    uint8_t chr_ram[FDS_CHR_RAM_SIZE];
    bool disk_inserted;
    uint32_t current_side;
    uint64_t disk_position;
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
} FdsMutableSavedState;

static bool fds_state_write_automation(NesStateWriter *writer,
                                       const FdsAutomationState *automation) {
    uint64_t previous_side = automation->previous_side == FDS_NO_SIDE
                           ? UINT64_MAX : (uint64_t)automation->previous_side;
    return nes_state_write_u64(writer, automation->last_frame)
        && nes_state_write_u64(writer, automation->last_check_frame)
        && nes_state_write_u32(writer, automation->successive_checks)
        && nes_state_write_u32(writer, (uint32_t)automation->eject_frames)
        && nes_state_write_u32(writer, (uint32_t)automation->switch_frames)
        && nes_state_write_u32(writer, (uint32_t)automation->retry_frames)
        && nes_state_write_u64(writer, previous_side)
        && nes_state_write_bool(writer, automation->game_started)
        && nes_state_write_bool(writer, automation->ambiguous);
}

static bool fds_state_read_automation(NesStateReader *reader,
                                      FdsAutomationState *automation,
                                      const FdsImage *image) {
    uint32_t eject_frames, switch_frames, retry_frames;
    uint64_t previous_side;
    memset(automation, 0, sizeof(*automation));
    if (!nes_state_read_u64(reader, &automation->last_frame)
        || !nes_state_read_u64(reader, &automation->last_check_frame)
        || !nes_state_read_u32(reader, &automation->successive_checks)
        || !nes_state_read_u32(reader, &eject_frames)
        || !nes_state_read_u32(reader, &switch_frames)
        || !nes_state_read_u32(reader, &retry_frames)
        || !nes_state_read_u64(reader, &previous_side)
        || !nes_state_read_bool(reader, &automation->game_started)
        || !nes_state_read_bool(reader, &automation->ambiguous)) return false;
    automation->eject_frames = (int32_t)eject_frames;
    automation->switch_frames = (int32_t)switch_frames;
    automation->retry_frames = (int32_t)retry_frames;
    if (automation->eject_frames < -1 || automation->eject_frames > 77
        || automation->switch_frames < -1 || automation->switch_frames > 77
        || automation->retry_frames < -1 || automation->retry_frames > 200) return false;
    if (previous_side == UINT64_MAX) {
        automation->previous_side = FDS_NO_SIDE;
    } else {
        if (previous_side > SIZE_MAX || !image || previous_side >= image->side_count) return false;
        automation->previous_side = (size_t)previous_side;
    }
    return true;
}

static bool fds_state_write_mutable(NesStateWriter *writer, const FdsState *state) {
    bool inserted = state->image && state->current_side < state->image->side_count;
    if (!nes_state_write_bytes(writer, state->work_ram, sizeof(state->work_ram))
        || !nes_state_write_bytes(writer, state->chr_ram, sizeof(state->chr_ram))
        || !nes_state_write_bool(writer, inserted)
        || !nes_state_write_u32(writer, inserted ? (uint32_t)state->current_side : 0)
        || !nes_state_write_u64(writer, state->disk_position)
        || !nes_state_write_u32(writer, state->transfer_delay)
        || !nes_state_write_u16(writer, state->crc)
        || !nes_state_write_u16(writer, state->timer_reload)
        || !nes_state_write_u16(writer, state->timer_counter)
        || !nes_state_write_u8(writer, state->write_data)
        || !nes_state_write_u8(writer, state->read_data)
        || !nes_state_write_u8(writer, state->ext_connector)
        || !nes_state_write_bool(writer, state->timer_enabled)
        || !nes_state_write_bool(writer, state->timer_repeat)
        || !nes_state_write_bool(writer, state->timer_irq)
        || !nes_state_write_bool(writer, state->disk_irq)
        || !nes_state_write_bool(writer, state->disk_regs_enabled)
        || !nes_state_write_bool(writer, state->sound_regs_enabled)
        || !nes_state_write_bool(writer, state->motor_on)
        || !nes_state_write_bool(writer, state->reset_transfer)
        || !nes_state_write_bool(writer, state->read_mode)
        || !nes_state_write_bool(writer, state->crc_control)
        || !nes_state_write_bool(writer, state->disk_ready)
        || !nes_state_write_bool(writer, state->transfer_irq_enabled)
        || !nes_state_write_bool(writer, state->transfer_complete)
        || !nes_state_write_bool(writer, state->end_of_head)
        || !nes_state_write_bool(writer, state->scanning)
        || !nes_state_write_bool(writer, state->gap_ended)
        || !nes_state_write_bool(writer, state->previous_crc_control)
        || !nes_state_write_bool(writer, state->bad_crc)
        || !nes_state_write_bool(writer, state->dirty)
        || !nes_state_write_u8(writer, (uint8_t)state->mirroring)
        || !fds_state_write_audio(writer, &state->audio)
        || !fds_state_write_automation(writer, &state->automation)) return false;
    return true;
}

static bool fds_state_read_mutable(NesStateReader *reader, FdsMutableSavedState *saved,
                                   const FdsImage *image) {
    uint8_t mirroring;
    memset(saved, 0, sizeof(*saved));
    if (!nes_state_read_bytes(reader, saved->work_ram, sizeof(saved->work_ram))
        || !nes_state_read_bytes(reader, saved->chr_ram, sizeof(saved->chr_ram))
        || !nes_state_read_bool(reader, &saved->disk_inserted)
        || !nes_state_read_u32(reader, &saved->current_side)
        || !nes_state_read_u64(reader, &saved->disk_position)
        || !nes_state_read_u32(reader, &saved->transfer_delay)
        || !nes_state_read_u16(reader, &saved->crc)
        || !nes_state_read_u16(reader, &saved->timer_reload)
        || !nes_state_read_u16(reader, &saved->timer_counter)
        || !nes_state_read_u8(reader, &saved->write_data)
        || !nes_state_read_u8(reader, &saved->read_data)
        || !nes_state_read_u8(reader, &saved->ext_connector)
        || !nes_state_read_bool(reader, &saved->timer_enabled)
        || !nes_state_read_bool(reader, &saved->timer_repeat)
        || !nes_state_read_bool(reader, &saved->timer_irq)
        || !nes_state_read_bool(reader, &saved->disk_irq)
        || !nes_state_read_bool(reader, &saved->disk_regs_enabled)
        || !nes_state_read_bool(reader, &saved->sound_regs_enabled)
        || !nes_state_read_bool(reader, &saved->motor_on)
        || !nes_state_read_bool(reader, &saved->reset_transfer)
        || !nes_state_read_bool(reader, &saved->read_mode)
        || !nes_state_read_bool(reader, &saved->crc_control)
        || !nes_state_read_bool(reader, &saved->disk_ready)
        || !nes_state_read_bool(reader, &saved->transfer_irq_enabled)
        || !nes_state_read_bool(reader, &saved->transfer_complete)
        || !nes_state_read_bool(reader, &saved->end_of_head)
        || !nes_state_read_bool(reader, &saved->scanning)
        || !nes_state_read_bool(reader, &saved->gap_ended)
        || !nes_state_read_bool(reader, &saved->previous_crc_control)
        || !nes_state_read_bool(reader, &saved->bad_crc)
        || !nes_state_read_bool(reader, &saved->dirty)
        || !nes_state_read_u8(reader, &mirroring)
        || mirroring > MIRROR_FOUR
        || !fds_state_read_audio(reader, &saved->audio)
        || !fds_state_read_automation(reader, &saved->automation, image)) return false;
    saved->mirroring = (Mirroring)mirroring;
    if (saved->disk_inserted) {
        if (!image || saved->current_side >= image->side_count) return false;
        if (saved->disk_position > image->sides[saved->current_side].drive_size) return false;
    }
    return saved->ext_connector <= 0x7F;
}

static void fds_state_apply_mutable(const FdsMutableSavedState *saved) {
    FdsImage *image = fds.image;
    memcpy(fds.work_ram, saved->work_ram, sizeof(fds.work_ram));
    memcpy(fds.chr_ram, saved->chr_ram, sizeof(fds.chr_ram));
    fds.current_side = saved->disk_inserted ? saved->current_side : FDS_NO_SIDE;
    fds.disk_position = (size_t)saved->disk_position;
    fds.transfer_delay = saved->transfer_delay;
    fds.crc = saved->crc;
    fds.timer_reload = saved->timer_reload;
    fds.timer_counter = saved->timer_counter;
    fds.write_data = saved->write_data;
    fds.read_data = saved->read_data;
    fds.ext_connector = saved->ext_connector;
    fds.timer_enabled = saved->timer_enabled;
    fds.timer_repeat = saved->timer_repeat;
    fds.timer_irq = saved->timer_irq;
    fds.disk_irq = saved->disk_irq;
    fds.disk_regs_enabled = saved->disk_regs_enabled;
    fds.sound_regs_enabled = saved->sound_regs_enabled;
    fds.motor_on = saved->motor_on;
    fds.reset_transfer = saved->reset_transfer;
    fds.read_mode = saved->read_mode;
    fds.crc_control = saved->crc_control;
    fds.disk_ready = saved->disk_ready;
    fds.transfer_irq_enabled = saved->transfer_irq_enabled;
    fds.transfer_complete = saved->transfer_complete;
    fds.end_of_head = saved->end_of_head;
    fds.scanning = saved->scanning;
    fds.gap_ended = saved->gap_ended;
    fds.previous_crc_control = saved->previous_crc_control;
    fds.bad_crc = saved->bad_crc;
    fds.dirty = fds.dirty || saved->dirty;
    fds.mirroring = saved->mirroring;
    fds.audio = saved->audio;
    fds.automation = saved->automation;
    fds.image = image;
}

static bool fds_state_process(NesStateReader *reader, bool apply) {
    bool has_image;
    uint32_t bios_crc, original_crc, side_count, side_capacity;
    uint64_t original_size;
    uint8_t save_mode;
    bool headered, qd_format;
    if (!reader || !nes_state_read_bool(reader, &has_image)) return false;
    if (has_image != (fds.image != NULL)) return false;

    if (!has_image) {
        FdsAudio audio;
        if (!fds_state_read_audio(reader, &audio) || nes_state_reader_remaining(reader) != 0)
            return false;
        if (apply) fds.audio = audio;
        return true;
    }

    if (!nes_state_read_u32(reader, &bios_crc)
        || !nes_state_read_u32(reader, &original_crc)
        || !nes_state_read_u64(reader, &original_size)
        || !nes_state_read_u32(reader, &side_count)
        || !nes_state_read_u32(reader, &side_capacity)
        || !nes_state_read_bool(reader, &headered)
        || !nes_state_read_bool(reader, &qd_format)
        || !nes_state_read_u8(reader, &save_mode)
        || side_count != fds.image->side_count
        || side_capacity != fds.image->side_capacity
        || headered != fds.image->headered
        || qd_format != fds.image->qd_format
        || save_mode > FDS_SAVE_OVERLAY
        || save_mode != (uint8_t)fds.image->save_mode
        || original_size != fds.image->original_disk_size
        || original_crc != fds_state_crc32(fds.image->original_disk,
                                           fds.image->original_disk_size)
        || bios_crc != fds_state_crc32(fds.image->bios, sizeof(fds.image->bios))) return false;

    FdsMutableSavedState saved;
    if (!fds_state_read_mutable(reader, &saved, fds.image)) return false;
    for (size_t side = 0; side < fds.image->side_count; ++side) {
        uint32_t drive_size;
        bool dirty;
        if (!nes_state_read_u32(reader, &drive_size)
            || drive_size != fds.image->sides[side].drive_size
            || nes_state_reader_remaining(reader) < sizeof(fds.image->sides[side].identity_header)
                                                 + fds.image->side_capacity + drive_size + 1)
            return false;
        uint8_t identity_header[sizeof(fds.image->sides[side].identity_header)];
        if (!nes_state_read_bytes(reader, identity_header, sizeof(identity_header))) return false;
        const uint8_t *raw = reader->data + reader->offset;
        reader->offset += fds.image->side_capacity;
        const uint8_t *drive = reader->data + reader->offset;
        reader->offset += drive_size;
        if (!nes_state_read_bool(reader, &dirty)) return false;
        if (apply) {
            memcpy(fds.image->sides[side].identity_header, identity_header,
                   sizeof(identity_header));
            memcpy(fds.image->sides[side].raw, raw, fds.image->side_capacity);
            memcpy(fds.image->sides[side].drive, drive, drive_size);
            fds.image->sides[side].dirty = fds.image->sides[side].dirty || dirty;
        }
    }
    if (nes_state_reader_remaining(reader) != 0) return false;
    if (apply) fds_state_apply_mutable(&saved);
    return true;
}

bool fds_state_capture(NesStateWriter *writer) {
    if (!writer || !nes_state_write_bool(writer, fds.image != NULL)) return false;
    if (!fds.image) return fds_state_write_audio(writer, &fds.audio);
    if (fds.image->side_count > UINT32_MAX || fds.image->side_capacity > UINT32_MAX
        || !nes_state_write_u32(writer, fds_state_crc32(fds.image->bios, sizeof(fds.image->bios)))
        || !nes_state_write_u32(writer, fds_state_crc32(fds.image->original_disk,
                                                        fds.image->original_disk_size))
        || !nes_state_write_u64(writer, (uint64_t)fds.image->original_disk_size)
        || !nes_state_write_u32(writer, (uint32_t)fds.image->side_count)
        || !nes_state_write_u32(writer, (uint32_t)fds.image->side_capacity)
        || !nes_state_write_bool(writer, fds.image->headered)
        || !nes_state_write_bool(writer, fds.image->qd_format)
        || !nes_state_write_u8(writer, (uint8_t)fds.image->save_mode)
        || !fds_state_write_mutable(writer, &fds)) return false;
    for (size_t side = 0; side < fds.image->side_count; ++side) {
        const FdsSide *value = &fds.image->sides[side];
        if (value->drive_size > UINT32_MAX
            || !nes_state_write_u32(writer, (uint32_t)value->drive_size)
            || !nes_state_write_bytes(writer, value->identity_header, sizeof(value->identity_header))
            || !nes_state_write_bytes(writer, value->raw, fds.image->side_capacity)
            || !nes_state_write_bytes(writer, value->drive, value->drive_size)
            || !nes_state_write_bool(writer, value->dirty)) return false;
    }
    return true;
}

bool fds_state_validate(NesStateReader *reader) {
    return fds_state_process(reader, false);
}

bool fds_state_apply(NesStateReader *reader) {
    return fds_state_process(reader, true);
}

#endif
