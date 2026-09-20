/*
 * input_event.c - Host input observation and deterministic injection
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "input_event.h"
#include "../joypad/joypad.h"
#include "../joypad/family_basic.h"
#include "../cpu/cpu.h"
#include "../system/vs_system.h"
#include "../rom/mapper.h"
#include "../rom/fds.h"
#include "../ui/machine_actions.h"

#include <string.h>

static NesInputEventObserver active_observer;
static void *active_observer_userdata;
static unsigned injection_depth;
static unsigned suppression_depth;

void nes_input_event_set_observer(NesInputEventObserver observer, void *userdata) {
    active_observer = observer;
    active_observer_userdata = userdata;
}

void nes_input_event_clear_observer(void) {
    active_observer = NULL;
    active_observer_userdata = NULL;
}

bool nes_input_event_submit(const NesInputEvent *event) {
    if (!event) return false;
    if (injection_depth || suppression_depth || !active_observer) return true;
    return active_observer(event, active_observer_userdata);
}

void nes_input_event_suppress_begin(void) { ++suppression_depth; }
void nes_input_event_suppress_end(void) {
    if (suppression_depth) --suppression_depth;
}

static bool apply_event(const NesInputEvent *event) {
    switch (event->type) {
        case NES_INPUT_EVENT_PLAYER_BUTTON:
            return joypad_set_player((unsigned)event->a, event->b, event->c != 0);
        case NES_INPUT_EVENT_MICROPHONE:
            joypad_set_microphone(event->a != 0);
            return true;
        case NES_INPUT_EVENT_PADDLE:
            return joypad_set_paddle((unsigned)event->a, event->b, event->c != 0);
        case NES_INPUT_EVENT_MAT:
            return joypad_set_mat_pad((unsigned)event->a, (unsigned)event->b, event->c != 0);
        case NES_INPUT_EVENT_ZAPPER:
            return joypad_set_zapper((unsigned)event->a, event->b, event->c, event->d != 0);
        case NES_INPUT_EVENT_SUBOR_KEY:
            return joypad_set_subor_key((SuborKey)event->a, event->b != 0);
        case NES_INPUT_EVENT_SUBOR_MOUSE_MOTION:
            return joypad_add_subor_mouse_motion(event->a, event->b);
        case NES_INPUT_EVENT_SUBOR_MOUSE_BUTTONS:
            return joypad_set_subor_mouse_buttons(event->a != 0, event->b != 0);
        case NES_INPUT_EVENT_SNES_BUTTON:
            return joypad_set_snes_button((unsigned)event->a, (SnesButton)event->b, event->c != 0);
        case NES_INPUT_EVENT_SNES_MOUSE_MOTION:
            return joypad_add_snes_mouse_motion((unsigned)event->a, event->b, event->c);
        case NES_INPUT_EVENT_SNES_MOUSE_BUTTONS:
            return joypad_set_snes_mouse_buttons((unsigned)event->a, event->b != 0, event->c != 0);
        case NES_INPUT_EVENT_NTT_KEY:
            return joypad_set_ntt_key((unsigned)event->a, (NttKey)event->b, event->c != 0);
        case NES_INPUT_EVENT_FCNS_KEY:
            return joypad_set_fcns_key((FcnsKey)event->a, event->b != 0);
        case NES_INPUT_EVENT_VIRTUAL_BOY_BUTTON:
            return joypad_set_virtual_boy_button((unsigned)event->a,
                                                  (VirtualBoyButton)event->b,
                                                  event->c != 0);
        case NES_INPUT_EVENT_HORI_TRACK_MOTION:
            return joypad_add_hori_track_motion(event->a, event->b);
        case NES_INPUT_EVENT_PARTY_TAP:
            return joypad_set_party_tap_button((unsigned)event->a, event->b != 0);
        case NES_INPUT_EVENT_PACHINKO:
            return joypad_set_pachinko_controls(event->a != 0, event->b != 0);
        case NES_INPUT_EVENT_BOXING:
            return joypad_set_boxing_sensor((unsigned)event->a, event->b != 0);
        case NES_INPUT_EVENT_JISSEN:
            return joypad_set_jissen_key((JissenKey)event->a, event->b != 0);
        case NES_INPUT_EVENT_BARCODE_BATTLER:
            return joypad_scan_barcode_battler(event->text);
        case NES_INPUT_EVENT_OEKA_KIDS:
            return joypad_set_oeka_kids_tablet(event->a, event->b,
                                               event->c != 0, event->d != 0);
        case NES_INPUT_EVENT_VS_COIN:
            return vs_set_coin((unsigned)event->a, event->b != 0);
        case NES_INPUT_EVENT_VS_SERVICE:
            return vs_set_service((unsigned)event->a, event->b != 0);
        case NES_INPUT_EVENT_CART_BARCODE:
            return cart_set_barcode(event->text);
        case NES_INPUT_EVENT_FAMILY_BASIC_KEY:
            return family_basic_set_key((FamilyBasicKey)event->a, event->b != 0);
        case NES_INPUT_EVENT_FAMILY_BASIC_TAPE_PLAY:
            return family_basic_tape_play(cpu_total_cycles);
        case NES_INPUT_EVENT_FAMILY_BASIC_TAPE_RECORD:
            family_basic_tape_record(cpu_total_cycles);
            return true;
        case NES_INPUT_EVENT_FAMILY_BASIC_TAPE_STOP:
            family_basic_tape_stop();
            return true;
        case NES_INPUT_EVENT_SOFT_RESET:
            return frontend_machine_soft_reset();
        case NES_INPUT_EVENT_POWER_CYCLE:
            return frontend_machine_power_cycle();
        case NES_INPUT_EVENT_FDS_INSERT:
            return event->a >= 0 && fds_insert_disk((size_t)event->a);
        case NES_INPUT_EVENT_FDS_EJECT:
            fds_eject_disk();
            return true;
        case NES_INPUT_EVENT_CART_KARAOKE:
            return cart_set_karaoke_input((CartKaraokeInput)event->a, event->b != 0);
        default:
            return false;
    }
}

bool nes_input_event_apply(const NesInputEvent *event) {
    if (!event) return false;
    ++injection_depth;
    bool ok = apply_event(event);
    --injection_depth;
    return ok;
}
