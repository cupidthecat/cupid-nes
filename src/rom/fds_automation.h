/*
 * fds_automation.h - Optional BIOS disk selection and loading controls
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_FDS_AUTOMATION_H
#define CUPID_FDS_AUTOMATION_H

static FdsAutomationOptions automatic_options;

static void fds_automation_reset(void) {
    memset(&fds.automation, 0, sizeof(fds.automation));
    fds.automation.switch_frames = -1;
    fds.automation.previous_side = FDS_NO_SIDE;
    fds.automation.last_frame = ppu.frame_count;
}

void fds_set_automation_options(FdsAutomationOptions options) {
    automatic_options = options;
}

FdsAutomationOptions fds_automation_options(void) {
    return automatic_options;
}

bool fds_automatic_insert_active(void) {
    return fds.image && automatic_options.insert_automatically
        && !fds.automation.ambiguous && nes_execution_allows_automatic_media();
}

bool fds_automatic_insert_ambiguous(void) {
    return fds.image && fds.automation.ambiguous;
}

bool fds_loading_fast_forward(void) {
    if (!fds.image || !nes_execution_allows_automatic_media()) return false;
    bool changing_side = fds_automatic_insert_active() && fds.automation.eject_frames > 0;
    return changing_side || (automatic_options.fast_forward_loading
                             && (fds.scanning || !fds.automation.game_started));
}

void fds_automation_frame(uint64_t frame) {
    if (!fds.image || frame == fds.automation.last_frame) return;
    fds.automation.last_frame = frame;
    if (!fds_automatic_insert_active()) return;

    if (fds.automation.eject_frames > 0) {
        fds.automation.eject_frames--;
    } else if (fds.automation.switch_frames > 0) {
        fds.automation.switch_frames--;
        if (fds.automation.switch_frames == 0) {
            // The BIOS can now read the requested header. That read chooses
            // the matching side before any file bytes are transferred.
            (void)fds_insert_disk(0);
            fds.automation.retry_frames = 200;
        }
    } else if (fds.automation.retry_frames > 0) {
        fds.automation.retry_frames--;
        if (fds.automation.retry_frames == 0) {
            fds.automation.previous_side = FDS_NO_SIDE;
            fds.automation.eject_frames = 34;
            fds.automation.switch_frames = -1;
        }
    }
}

static bool fds_automation_peek(uint16_t address, uint8_t *value) {
    if (address < 0x2000) {
        *value = cpu_peek_internal_ram(address);
        return true;
    }

    if (address >= 0x6000 && address <= 0xDFFF) {
        *value = fds.work_ram[address - 0x6000];
        return true;
    }

    if (address >= 0xE000) {
        *value = fds.image->bios[address - 0xE000];
        return true;
    }

    // The BIOS expects a memory pointer. An invalid I/O pointer must not
    // acknowledge an IRQ, shift a controller, or recursively read the BIOS.
    return false;
}

static void fds_automation_select_requested_side(void) {
    uint16_t pointer = (uint16_t)(cpu_peek_internal_ram(0)
                                 | ((uint16_t)cpu_peek_internal_ram(1) << 8));
    uint8_t wanted[10];
    for (unsigned i = 0; i < sizeof(wanted); i++) {
        uint16_t address = (uint16_t)(pointer + i);
        if (!fds_automation_peek(address, &wanted[i])) return;
        // A pointer into this hook is malformed. Treating that byte as zero
        // matches the BIOS helper's nonrecursive debug-read behavior.
        if (address == 0xE445) wanted[i] = 0;
    }

    size_t selected = FDS_NO_SIDE;
    unsigned matches = 0;
    for (size_t side = 0; side < fds.image->side_count; side++) {
        bool match = true;
        for (unsigned i = 0; i < sizeof(wanted); i++) {
            if (wanted[i] != 0xFF && wanted[i] != fds.image->sides[side].identity_header[i]) {
                match = false;
                break;
            }
        }

        if (match) {
            selected = side;
            matches++;
        }
    }

    if (matches == 1) {
        fds.current_side = selected;
        if (selected > 0) fds.automation.game_started = true;
    } else if (matches > 1) {
        fds.automation.ambiguous = true;
        fprintf(stderr, "Automatic disk insertion stopped: several sides match the requested header\n");
    }

    fds.automation.switch_frames = -1;
    fds.automation.retry_frames = -1;
}

static void fds_automation_bios_read(uint16_t address) {
    if (address == 0xE18C && !fds.automation.game_started
        && (cpu_peek_internal_ram(0x100) & 0xC0)) {
        fds.automation.game_started = true;
    }

    if (address == 0xE445 && fds_automatic_insert_active()) {
        fds_automation_select_requested_side();
    }
}

static void fds_automation_status_read(void) {
    if (!fds_automatic_insert_active()) return;
    uint64_t frame = ppu.frame_count;
    uint64_t previous = fds.automation.last_check_frame;
    if (frame >= previous && frame - previous < 100) {
        if (fds.automation.successive_checks < UINT32_MAX) fds.automation.successive_checks++;
    } else {
        fds.automation.successive_checks = 0;
    }

    fds.automation.last_check_frame = frame;
    if (fds.automation.successive_checks > 20 && fds.automation.eject_frames == 0
        && fds.automation.switch_frames == -1) {
        fds.automation.switch_frames = 77;
        fds.automation.previous_side = fds.current_side;
        fds_eject_disk();
        fds.automation.successive_checks = 0;
        fds.automation.last_check_frame = 0;
    }
}

#endif
