/* capture_riff.h - Bounded read-only RIFF inspection
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef NES_CAPTURE_RIFF_H
#define NES_CAPTURE_RIFF_H
#include "capture_writer.h"

enum { NES_RIFF_NODE_LIMIT = 8192, NES_RIFF_DEPTH_LIMIT = 16 };

typedef struct {
    uint64_t offset;
    uint32_t size;
    unsigned depth;
    char tag[5];
    char kind[5];
    char detail[192];
    bool warning;
} NesRiffNode;

typedef struct {
    NesRiffNode *nodes;
    size_t count;
    unsigned warnings;
    uint64_t file_size;
    uint32_t width, height, video_frames, streams;
} NesRiffReport;

/* Initialize the report to zero; success replaces it, even for damaged input.
 * Malformed chunks are warnings. Open/read/allocation failures leave it intact. */
NesFileResult nes_capture_riff_inspect(const char *path, NesRiffReport *report);
void nes_capture_riff_free(NesRiffReport *report);
#endif
