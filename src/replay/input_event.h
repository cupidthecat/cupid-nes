/*
 * input_event.h - Host input events shared by movies and netplay
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_REPLAY_INPUT_EVENT_H
#define CUPID_REPLAY_INPUT_EVENT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NES_INPUT_EVENT_PLAYER_BUTTON = 1,
    NES_INPUT_EVENT_MICROPHONE,
    NES_INPUT_EVENT_PADDLE,
    NES_INPUT_EVENT_MAT,
    NES_INPUT_EVENT_ZAPPER,
    NES_INPUT_EVENT_SUBOR_KEY,
    NES_INPUT_EVENT_SUBOR_MOUSE_MOTION,
    NES_INPUT_EVENT_SUBOR_MOUSE_BUTTONS,
    NES_INPUT_EVENT_SNES_BUTTON,
    NES_INPUT_EVENT_SNES_MOUSE_MOTION,
    NES_INPUT_EVENT_SNES_MOUSE_BUTTONS,
    NES_INPUT_EVENT_NTT_KEY,
    NES_INPUT_EVENT_FCNS_KEY,
    NES_INPUT_EVENT_VIRTUAL_BOY_BUTTON,
    NES_INPUT_EVENT_HORI_TRACK_MOTION,
    NES_INPUT_EVENT_PARTY_TAP,
    NES_INPUT_EVENT_PACHINKO,
    NES_INPUT_EVENT_BOXING,
    NES_INPUT_EVENT_JISSEN,
    NES_INPUT_EVENT_BARCODE_BATTLER,
    NES_INPUT_EVENT_OEKA_KIDS,
    NES_INPUT_EVENT_VS_COIN,
    NES_INPUT_EVENT_VS_SERVICE,
    NES_INPUT_EVENT_CART_BARCODE,
    NES_INPUT_EVENT_FAMILY_BASIC_KEY,
    NES_INPUT_EVENT_FAMILY_BASIC_TAPE_PLAY,
    NES_INPUT_EVENT_FAMILY_BASIC_TAPE_RECORD,
    NES_INPUT_EVENT_FAMILY_BASIC_TAPE_STOP,
    NES_INPUT_EVENT_SOFT_RESET,
    NES_INPUT_EVENT_POWER_CYCLE,
    NES_INPUT_EVENT_FDS_INSERT,
    NES_INPUT_EVENT_FDS_EJECT,
    NES_INPUT_EVENT_CART_KARAOKE,
    NES_INPUT_EVENT_LAST = NES_INPUT_EVENT_CART_KARAOKE
} NesInputEventType;

enum { NES_INPUT_EVENT_TEXT_MAX = 16 };

typedef struct {
    NesInputEventType type;
    int32_t a;
    int32_t b;
    int32_t c;
    int32_t d;
    char text[NES_INPUT_EVENT_TEXT_MAX];
} NesInputEvent;

typedef bool (*NesInputEventObserver)(const NesInputEvent *event, void *userdata);

/* Public host-input setters submit here before mutating emulated input state.
 * Returning false blocks the live mutation (movie playback/netplay ownership). */
void nes_input_event_set_observer(NesInputEventObserver observer, void *userdata);
void nes_input_event_clear_observer(void);
bool nes_input_event_submit(const NesInputEvent *event);

/* Replay injection uses the ordinary public setters with observation bypassed,
 * keeping validation and device-specific behavior in one place. */
bool nes_input_event_apply(const NesInputEvent *event);

/* Nested public setters (for example SNES buttons mapped onto NES buttons) use
 * this to avoid recording the same physical host action twice. */
void nes_input_event_suppress_begin(void);
void nes_input_event_suppress_end(void);

#ifdef __cplusplus
}
#endif

#endif
