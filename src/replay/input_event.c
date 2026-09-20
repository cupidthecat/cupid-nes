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
#include "../rom/fds.h"
#include "../rom/mapper.h"
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

static bool bool_arg(int32_t value) {
    return value == 0 || value == 1;
}

static bool digits_valid(const char *digits) {
    size_t count = digits ? strlen(digits) : 0;
    if (count != 8 && count != 13) return false;
    for (size_t i = 0; i < count; ++i)
        if (digits[i] < '0' || digits[i] > '9') return false;
    return true;
}

bool nes_input_event_validate(const NesInputEvent *event) {
    if (!event || event->type < NES_INPUT_EVENT_PLAYER_BUTTON
        || event->type > NES_INPUT_EVENT_LAST
        || !memchr(event->text, '\0', sizeof(event->text))) return false;
    switch (event->type) {
        case NES_INPUT_EVENT_PLAYER_BUTTON:
            return event->a >= 0 && event->a < NES_INPUT_PLAYERS
                && event->b >= BTN_A && event->b <= BTN_RIGHT && bool_arg(event->c);
        case NES_INPUT_EVENT_MICROPHONE:
            return bool_arg(event->a);
        case NES_INPUT_EVENT_PADDLE:
            return event->a >= 0 && event->a < 3
                && event->b >= 0x54 && event->b <= 0xF4 && bool_arg(event->c);
        case NES_INPUT_EVENT_MAT:
            return event->a >= 0 && event->a < 3 && event->b >= 0 && event->b < 12
                && bool_arg(event->c);
        case NES_INPUT_EVENT_ZAPPER:
            return event->a >= 0 && event->a < 3 && bool_arg(event->d);
        case NES_INPUT_EVENT_SUBOR_KEY:
            return event->a >= 0 && event->a < SUBOR_KEY_COUNT && bool_arg(event->b);
        case NES_INPUT_EVENT_SUBOR_MOUSE_MOTION:
            return joypad_port_device(1) == NES_PORT_SUBOR_MOUSE;
        case NES_INPUT_EVENT_SUBOR_MOUSE_BUTTONS:
            return joypad_port_device(1) == NES_PORT_SUBOR_MOUSE
                && bool_arg(event->a) && bool_arg(event->b);
        case NES_INPUT_EVENT_SNES_BUTTON:
            return event->a >= 0 && event->a < 2
                && (joypad_port_device((unsigned)event->a) == NES_PORT_SNES_CONTROLLER
                    || joypad_port_device((unsigned)event->a) == NES_PORT_NTT_KEYPAD)
                && event->b >= 0 && event->b < SNES_BUTTON_COUNT && bool_arg(event->c);
        case NES_INPUT_EVENT_SNES_MOUSE_MOTION:
            return event->a >= 0 && event->a < 2
                && joypad_port_device((unsigned)event->a) == NES_PORT_SNES_MOUSE;
        case NES_INPUT_EVENT_SNES_MOUSE_BUTTONS:
            return event->a >= 0 && event->a < 2
                && joypad_port_device((unsigned)event->a) == NES_PORT_SNES_MOUSE
                && bool_arg(event->b) && bool_arg(event->c);
        case NES_INPUT_EVENT_NTT_KEY:
            return event->a >= 0 && event->a < 2
                && joypad_port_device((unsigned)event->a) == NES_PORT_NTT_KEYPAD
                && event->b >= 0 && event->b < NTT_KEY_COUNT && bool_arg(event->c);
        case NES_INPUT_EVENT_FCNS_KEY:
            return joypad_expansion_device() == NES_EXPANSION_FCNS_CONTROLLER
                && event->a >= 0 && event->a < FCNS_KEY_COUNT && bool_arg(event->b);
        case NES_INPUT_EVENT_VIRTUAL_BOY_BUTTON:
            return event->a >= 0 && event->a < 2
                && joypad_port_device((unsigned)event->a) == NES_PORT_VIRTUAL_BOY
                && event->b >= 0 && event->b < VB_BUTTON_COUNT && bool_arg(event->c);
        case NES_INPUT_EVENT_HORI_TRACK_MOTION:
            return joypad_expansion_device() == NES_EXPANSION_HORI_TRACK;
        case NES_INPUT_EVENT_PARTY_TAP:
            return joypad_expansion_device() == NES_EXPANSION_PARTY_TAP
                && event->a >= 0 && event->a < 6 && bool_arg(event->b);
        case NES_INPUT_EVENT_PACHINKO:
            return joypad_expansion_device() == NES_EXPANSION_PACHINKO
                && bool_arg(event->a) && bool_arg(event->b);
        case NES_INPUT_EVENT_BOXING:
            return joypad_expansion_device() == NES_EXPANSION_EXCITING_BOXING
                && event->a >= 0 && event->a < 8 && bool_arg(event->b);
        case NES_INPUT_EVENT_JISSEN:
            return joypad_expansion_device() == NES_EXPANSION_JISSEN_MAHJONG
                && event->a >= 0 && event->a < JISSEN_KEY_COUNT && bool_arg(event->b);
        case NES_INPUT_EVENT_BARCODE_BATTLER:
            return joypad_expansion_device() == NES_EXPANSION_BARCODE_BATTLER
                && digits_valid(event->text);
        case NES_INPUT_EVENT_OEKA_KIDS:
            return joypad_expansion_device() == NES_EXPANSION_OEKA_KIDS_TABLET
                && bool_arg(event->c) && bool_arg(event->d);
        case NES_INPUT_EVENT_VS_COIN:
            return vs_enabled() && event->a >= 0
                && event->a < (vs_dual_system() ? 4 : 2) && bool_arg(event->b);
        case NES_INPUT_EVENT_VS_SERVICE:
            return vs_enabled() && event->a >= 0
                && event->a < (vs_dual_system() ? 2 : 1) && bool_arg(event->b);
        case NES_INPUT_EVENT_CART_BARCODE:
            return cart_barcode_supported() && digits_valid(event->text);
        case NES_INPUT_EVENT_FAMILY_BASIC_KEY:
            return joypad_expansion_device() == NES_EXPANSION_FAMILY_BASIC
                && event->a >= 0 && event->a < FB_KEY_COUNT && bool_arg(event->b);
        case NES_INPUT_EVENT_FAMILY_BASIC_TAPE_PLAY:
        case NES_INPUT_EVENT_FAMILY_BASIC_TAPE_RECORD:
        case NES_INPUT_EVENT_FAMILY_BASIC_TAPE_STOP:
            return joypad_expansion_device() == NES_EXPANSION_FAMILY_BASIC;
        case NES_INPUT_EVENT_SOFT_RESET:
        case NES_INPUT_EVENT_POWER_CYCLE:
            return true;
        case NES_INPUT_EVENT_FDS_INSERT:
            return fds_active() && event->a >= 0 && (uint64_t)event->a < fds_side_count();
        case NES_INPUT_EVENT_FDS_EJECT:
            return fds_active();
        case NES_INPUT_EVENT_CART_KARAOKE:
            return event->a >= 0 && event->a < CART_KARAOKE_INPUT_COUNT
                && bool_arg(event->b);
        default:
            return false;
    }
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
    if (!nes_input_event_validate(event)) return false;
    ++injection_depth;
    bool ok = apply_event(event);
    --injection_depth;
    return ok;
}
