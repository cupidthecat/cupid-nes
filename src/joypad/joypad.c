#include "joypad.h"

void joypad_set(Joypad* jp, int btn, int pressed) {
    if (pressed) jp->buttons |=  (1u << btn);
    else         jp->buttons &= ~(1u << btn);
}

void joypad_write_strobe(Joypad* jp, uint8_t value) {
    jp->strobe = value & 1;
    if (jp->strobe) {
        // While strobe=1, hardware continuously latches current buttons.
        jp->shift = jp->buttons;
    } else {
        // On 1->0 transition, a new snapshot is taken (already done above if strobe was 1).
        jp->shift = jp->buttons;
    }
}

uint8_t joypad_read(Joypad* jp) {
    // Bit0 returns next button in A,B,Select,Start,Up,Down,Left,Right order.
    // Other bits are "open bus"; returning 1 in bit6 is a common, safe choice.
    uint8_t ret = (jp->shift & 1u) | 0x40;

    if (!jp->strobe) {
        // Shift only when strobe==0. After 8 reads, real hardware returns 1s;
        // do that by shifting in 1s from the top.
        jp->shift = (jp->shift >> 1) | 0x80;
    } else {
        // If strobe==1, always return current A state; keep shift synced.
        jp->shift = jp->buttons;
    }
    return ret;
}