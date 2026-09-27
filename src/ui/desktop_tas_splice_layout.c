/*
 * desktop_tas_splice_layout.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* TAS splice range and length preview. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_tas_internal.h"

#include <stdio.h>

static void text(FrontendDesktopUi *ui, const char *label) {
    CLAY_TEXT(desktop_clay_string(ui->clay, label),
              CLAY_TEXT_CONFIG({.fontSize = 11, .textColor = {232, 237, 247, 255}, .wrapMode = CLAY_TEXT_WRAP_WORDS}));
}

static void button(FrontendDesktopUi *ui, const char *label, unsigned action, bool checked, bool enabled, float width) {
    Clay_ElementId id = desktop_clay_hit(ui->clay, enabled ? HIT_TAS_ACTION : HIT_NONE, (int)action, 0);
    CLAY(id, {.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_FIXED(23)},
                         .padding = {4, 4, 3, 3},
                         .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
              .backgroundColor = checked ? (Clay_Color){58, 70, 104, 255} : (Clay_Color){39, 47, 64, 255},
              .border = {.color = checked ? (Clay_Color){183, 196, 255, 255} : (Clay_Color){73, 87, 115, 255},
                         .width = {.left = 1, .right = 1, .top = 1, .bottom = 1}},
              .clip = {.horizontal = true, .vertical = true}}) {
        CLAY_TEXT(desktop_clay_string(ui->clay, label),
                  CLAY_TEXT_CONFIG(
                      {.fontSize = 11,
                       .textColor = enabled ? (Clay_Color){232, 237, 247, 255} : (Clay_Color){153, 168, 192, 255},
                       .wrapMode = CLAY_TEXT_WRAP_NONE}));
    }
}

static void range(FrontendDesktopUi *ui, size_t first, size_t count, unsigned action, bool first_enabled,
                  bool count_enabled, float width) {
    char label[64];
    CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
        snprintf(label, sizeof(label), "First: %zu", first);
        button(ui, label, action, false, first_enabled, (width - 4) / 2);
        snprintf(label, sizeof(label), "Count: %zu", count);
        button(ui, label, action + 1, false, count_enabled, (width - 4) / 2);
    }
}

void desktop_tas_splice_layout(FrontendDesktopUi *ui, const NesTasProgress *progress, bool writable, float width) {
    desktop_tas_splice_observe(ui);
    DesktopTasSplicer *splice = &ui->tas_editor->splice;
    bool ready = progress->active && !progress->recording && !progress->frame_in_progress && !progress->seeking &&
                 frontend_execution_paused(ui->execution) && !nes_tas_edit_active(splice->destination);
    char label[256];
    /* Keep the existing two tab rows and their available bookmark space. The
     * file toolbar opens this additional sidebar without moving other tabs. */
    snprintf(label, sizeof(label), "Source: %s", splice->source_name);
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_FIXED(16)}},
                  .clip = {.horizontal = true, .vertical = true}}) {
        text(ui, label);
    }
    CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
        button(ui, "Load file", TAS_ACTION_SPLICE_LOAD, false, ready, 67);
        button(ui, "Current", TAS_ACTION_SPLICE_CURRENT, !splice->source, ready, 63);
        button(ui, "Selection", TAS_ACTION_SPLICE_SOURCE_SELECTION, false, ready && !splice->source, width - 138);
    }
    range(ui, splice->options.source_first, splice->options.source_count, TAS_ACTION_SPLICE_SOURCE_FIRST, ready, ready,
          width);
    CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
        static const char *const modes[] = {"Append", "Insert", "Replace", "Extract"};
        for (unsigned i = 0; i < 4; ++i) {
            button(ui, modes[i], TAS_ACTION_SPLICE_APPEND + i, (unsigned)splice->options.mode == i, ready,
                   (width - 12) / 4);
        }
    }
    text(ui, "Destination range (zero-based)");
    bool location = splice->options.mode == NES_TAS_SPLICE_INSERT || splice->options.mode == NES_TAS_SPLICE_REPLACE;
    size_t first = splice->options.mode == NES_TAS_SPLICE_APPEND    ? nes_tas_project_frame_count(splice->destination)
                   : splice->options.mode == NES_TAS_SPLICE_EXTRACT ? 0
                                                                    : splice->options.destination_first;
    size_t count = splice->options.mode == NES_TAS_SPLICE_EXTRACT   ? nes_tas_project_frame_count(splice->destination)
                   : splice->options.mode == NES_TAS_SPLICE_REPLACE ? splice->options.destination_count
                                                                    : 0;
    range(ui, first, count, TAS_ACTION_SPLICE_DESTINATION_FIRST, ready && location,
          ready && splice->options.mode == NES_TAS_SPLICE_REPLACE, width);
    button(ui, "Use destination selection", TAS_ACTION_SPLICE_DESTINATION_SELECTION, false, ready && location, width);
    button(ui, splice->options.copy_markers ? "Copy source markers: yes" : "Copy source markers: no",
           TAS_ACTION_SPLICE_MARKERS, splice->options.copy_markers, ready, width);
    if (splice->preview_valid) {
        snprintf(label, sizeof(label), "Current: %zu frames", splice->preview.destination_frames);
        text(ui, label);
        snprintf(label, sizeof(label), "Remove %zu, insert %zu", splice->preview.removed, splice->preview.inserted);
        text(ui, label);
        snprintf(label, sizeof(label), "Result: %zu frames, %zu markers", splice->preview.resulting_frames,
                 splice->preview.resulting_markers);
        text(ui, label);
    } else {
        text(ui, splice->error);
    }
    button(ui, "Apply splice", TAS_ACTION_SPLICE_APPLY, false, ready && writable && splice->preview_valid, width);
    text(ui, splice->options.mode == NES_TAS_SPLICE_EXTRACT
                 ? "Extract replaces all input. Playback starts from the destination's original startup state."
                 : "Surviving markers follow their input. Replaced markers are removed.");
}
