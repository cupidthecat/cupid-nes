/* capture_tools.h - RIFF, encoding and movie preference panels
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef NES_CAPTURE_TOOLS_H
#define NES_CAPTURE_TOOLS_H
#include <stdbool.h>
typedef struct NesCaptureFrontend NesCaptureFrontend;

enum {
    CAPTURE_RIFF_PANEL = 0x2500,
    CAPTURE_MOVIE_PANEL = 0x2501,
    CAPTURE_ENCODING_PANEL = 0x2502,
    CAPTURE_COMMAND_GIF = 0x2503
};

bool nes_capture_tools_init(NesCaptureFrontend *frontend);
void nes_capture_tools_shutdown(NesCaptureFrontend *frontend);
#endif
