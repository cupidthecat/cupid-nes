/*
 * family_basic.h - Family BASIC keyboard and cassette interface
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FAMILY_BASIC_H
#define FAMILY_BASIC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "../state/state.h"
#include "../state/state_io.h"

/* Keys follow the keyboard's nine scan rows, low half before high half. */
typedef enum {
    FB_KEY_F8, FB_KEY_RETURN, FB_KEY_LEFT_BRACKET, FB_KEY_RIGHT_BRACKET,
    FB_KEY_KANA, FB_KEY_RIGHT_SHIFT, FB_KEY_YEN, FB_KEY_STOP,
    FB_KEY_F7, FB_KEY_AT, FB_KEY_COLON, FB_KEY_SEMICOLON,
    FB_KEY_UNDERSCORE, FB_KEY_SLASH, FB_KEY_MINUS, FB_KEY_CARET,
    FB_KEY_F6, FB_KEY_O, FB_KEY_L, FB_KEY_K,
    FB_KEY_PERIOD, FB_KEY_COMMA, FB_KEY_P, FB_KEY_0,
    FB_KEY_F5, FB_KEY_I, FB_KEY_U, FB_KEY_J,
    FB_KEY_M, FB_KEY_N, FB_KEY_9, FB_KEY_8,
    FB_KEY_F4, FB_KEY_Y, FB_KEY_G, FB_KEY_H,
    FB_KEY_B, FB_KEY_V, FB_KEY_7, FB_KEY_6,
    FB_KEY_F3, FB_KEY_T, FB_KEY_R, FB_KEY_D,
    FB_KEY_F, FB_KEY_C, FB_KEY_5, FB_KEY_4,
    FB_KEY_F2, FB_KEY_W, FB_KEY_S, FB_KEY_A,
    FB_KEY_X, FB_KEY_Z, FB_KEY_E, FB_KEY_3,
    FB_KEY_F1, FB_KEY_ESCAPE, FB_KEY_Q, FB_KEY_CONTROL,
    FB_KEY_LEFT_SHIFT, FB_KEY_GRPH, FB_KEY_1, FB_KEY_2,
    FB_KEY_CLEAR_HOME, FB_KEY_UP, FB_KEY_RIGHT, FB_KEY_LEFT,
    FB_KEY_DOWN, FB_KEY_SPACE, FB_KEY_DELETE, FB_KEY_INSERT,
    FB_KEY_COUNT
} FamilyBasicKey;

typedef enum {
    FB_TAPE_STOPPED, FB_TAPE_PLAYING, FB_TAPE_RECORDING
} FamilyBasicTapeMode;

enum { FAMILY_BASIC_TAPE_SAMPLE_CYCLES = 88 };

void family_basic_reset(void);
void family_basic_shutdown(void);
bool family_basic_set_key(FamilyBasicKey key, bool pressed);
void family_basic_write(uint8_t value, uint64_t cpu_cycles);
uint8_t family_basic_read(unsigned port, uint64_t cpu_cycles);

/* Tape files pack consecutive 88-CPU-cycle samples, least significant bit first. */
bool family_basic_tape_load(const uint8_t *data, size_t size);
bool family_basic_tape_load_file(const char *path);
bool family_basic_tape_save_file(const char *path);
bool family_basic_tape_play(uint64_t cpu_cycles);
void family_basic_tape_record(uint64_t cpu_cycles);
void family_basic_tape_stop(void);
FamilyBasicTapeMode family_basic_tape_mode(void);
bool family_basic_tape_failed(void);

bool family_basic_state_capture(NesStateWriter *writer);
bool family_basic_state_validate(NesStateReader *reader);
bool family_basic_state_apply(NesStateReader *reader);

typedef struct FamilyBasicStateRestore FamilyBasicStateRestore;
NesStateResult family_basic_state_prepare(NesStateReader *reader, FamilyBasicStateRestore **out_restore);
void family_basic_state_apply_prepared(FamilyBasicStateRestore *restore);
void family_basic_state_restore_free(FamilyBasicStateRestore *restore);

#endif
