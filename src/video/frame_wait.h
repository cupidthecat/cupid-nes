/*
 * frame_wait.h - Host deadline wait with bounded display servicing
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_FRAME_WAIT_H
#define CUPID_FRAME_WAIT_H
#include "frame_timing.h"
#include <SDL2/SDL.h>
#include <math.h>

/* Counter and delay arguments let regressions exercise the production wait
 * with a deterministic clock, including the final fraction of a millisecond. */
static void frame_wait(double deadline, double frequency, void (*service)(void *), void *context,
                       Uint64(SDLCALL *counter)(void), void(SDLCALL *sleep_ms)(Uint32)) {
    if (!isfinite(deadline) || !isfinite(frequency) || frequency <= 0) {
        return;
    }

    for (;;) {
        /* Service even an expired deadline: late frames may skip uploading,
         * but GTK still needs to finish paints and deliver input. */
        if (service) {
            service(context);
        }

        double now = (double)counter();
        double remaining = (deadline - now) * 1000.0 / frequency;
        if (remaining <= 0) {
            return;
        }

        unsigned delay = nes_frame_timing_sleep_ms(remaining);
        if (service && delay > 1) {
            delay = 1;
        }

        if (delay) {
            sleep_ms(delay);
        } else {
            double next_service = now + frequency * 0.00025;
            while ((now = (double)counter()) < deadline) {
                if (service && now >= next_service) {
                    service(context);
                    next_service = (double)counter() + frequency * 0.00025;
                }

                SDL_CPUPauseInstruction();
            }

            return;
        }
    }
}
#endif
