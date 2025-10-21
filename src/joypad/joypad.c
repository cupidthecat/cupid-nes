#include "joypad.h"

void joypad_set(Joypad* jp, int btn, int pressed){
    if (pressed) jp->buttons |=  (1u << btn);
    else         jp->buttons &= ~(1u << btn);
}
  
void joypad_write_strobe(Joypad* jp, uint8_t v){
    jp->strobe = v & 1;
    if (jp->strobe) jp->shift = jp->buttons; // while strobe=1, continually latch
    else            jp->shift = jp->buttons; // snapshot on 1->0 (OK to write again)
}
  
uint8_t joypad_read(Joypad* jp){
    uint8_t ret = (jp->shift & 1u) | 0x40;     // LSB first, bit6=1 (open bus)
    if (!jp->strobe) jp->shift = (jp->shift >> 1) | 0x80; // shift in 1s after 8 reads
    else             jp->shift = jp->buttons;             // strobe=1 -> always A
    return ret;
}