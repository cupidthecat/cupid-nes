/*
 * desktop_tas_splice.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Native TAS splice controls. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_tas_internal.h"
#include "../system/vs_system.h"

#include <stdio.h>
#include <string.h>

static uint64_t generation(FrontendDesktopUi *ui) {
    NesTasProgress progress = {0};
    nes_tas_session_progress(desktop_tas_session(ui), &progress);
    return progress.generation;
}

static bool current(FrontendDesktopUi *ui) {
    DesktopTasSplicer *splice = &ui->tas_editor->splice;
    const NesTasProject *project = desktop_tas_project_const(ui);
    return project && splice->initialized && splice->destination == project && splice->generation == generation(ui) &&
           splice->revision == nes_tas_project_revision(project);
}

static bool ready(FrontendDesktopUi *ui) {
    NesTasProgress progress = {0};
    nes_tas_session_progress(desktop_tas_session(ui), &progress);
    if (!progress.active || progress.recording || progress.frame_in_progress || progress.seeking ||
        !frontend_execution_paused(ui->execution) || nes_tas_edit_active(desktop_tas_project_const(ui))) {
        desktop_tas_status(ui, "Pause playback and finish the current take, frame or edit before splicing.");
        return false;
    }
    return true;
}

void desktop_tas_splice_observe(FrontendDesktopUi *ui) {
    DesktopTasSplicer *splice = &ui->tas_editor->splice;
    const NesTasProject *project = desktop_tas_project_const(ui);
    uint64_t active_generation = generation(ui);
    if (!splice->initialized || splice->destination != project || splice->generation != active_generation) {
        nes_tas_project_destroy(splice->source);
        memset(splice, 0, sizeof(*splice));
        splice->initialized = true;
        splice->destination = project;
        splice->generation = active_generation;
        splice->options.mode = NES_TAS_SPLICE_APPEND;
        splice->options.copy_markers = true;
        splice->options.source_count = nes_tas_project_frame_count(project);
        splice->options.destination_first = ui->tas_editor->cursor;
        snprintf(splice->source_name, sizeof(splice->source_name), "Current project");
    }
    splice->revision = nes_tas_project_revision(project);
    splice->options.allowed_commands = NES_FM2_COMMAND_RESET | NES_FM2_COMMAND_POWER;
    const NesFm2Movie *movie = nes_tas_project_movie(project);
    if (movie && movie->fds) {
        splice->options.allowed_commands |= NES_FM2_COMMAND_FDS_INSERT | NES_FM2_COMMAND_FDS_SELECT;
    }
    if (vs_enabled()) {
        splice->options.allowed_commands |=
            NES_FM2_COMMAND_VS_COIN_1 | NES_FM2_COMMAND_VS_COIN_2 | NES_FM2_COMMAND_VS_SERVICE;
    }
    splice->preview_valid =
        nes_tas_splice_preview(project, splice->source ? splice->source : project, &splice->options, &splice->preview,
                               splice->error, sizeof(splice->error)) == NES_TAS_OK;
}

static bool selection(FrontendDesktopUi *ui, size_t *first, size_t *count) {
    const NesTasProject *project = desktop_tas_project_const(ui);
    size_t last;
    if (!nes_tas_selection_bounds(project, first, &last) || nes_tas_selection_count(project) != last - *first + 1) {
        desktop_tas_status(ui, "Select one continuous frame range in the timeline.");
        return false;
    }
    *count = last - *first + 1;
    return true;
}

static void load_source(FrontendDesktopUi *ui) {
    char path[4096] = {0}, error[256] = {0};
    if (!frontend_open_movie_dialog(path, sizeof(path), error, sizeof(error))) {
        if (*error) {
            desktop_tas_status(ui, error);
        }
        return;
    }
    if (!ready(ui) || !current(ui)) {
        desktop_tas_status(ui, "The destination changed while choosing a source. Load the source again.");
        return;
    }
    NesTasProject *source = NULL;
    NesTasResult result = nes_tas_splice_load(path, &source, error, sizeof(error));
    DesktopTasSplicer *splice = &ui->tas_editor->splice;
    NesTasSpliceOptions options = splice->options;
    options.mode = NES_TAS_SPLICE_APPEND;
    options.source_first = 0;
    options.source_count = nes_tas_project_frame_count(source);
    NesTasSplicePreview preview;
    if (result == NES_TAS_OK) {
        result = nes_tas_splice_preview(splice->destination, source, &options, &preview, error, sizeof(error));
    }
    if (result != NES_TAS_OK) {
        nes_tas_project_destroy(source);
        desktop_tas_status(ui, error);
        return;
    }
    nes_tas_project_destroy(splice->source);
    splice->source = source;
    splice->options.source_first = 0;
    splice->options.source_count = options.source_count;
    const char *name = path;
    for (const char *p = path; *p; ++p) {
        if (*p == '/' || *p == '\\') {
            name = p + 1;
        }
    }
    snprintf(splice->source_name, sizeof(splice->source_name), "%s", name);
    ++splice->settings_serial;
    desktop_tas_splice_observe(ui);
    desktop_tas_status(ui, "Source loaded. Choose ranges and review the resulting length before applying.");
}

static void start_range_edit(FrontendDesktopUi *ui, unsigned action) {
    DesktopTasSplicer *splice = &ui->tas_editor->splice;
    unsigned control = TAS_EDIT_SPLICE_SOURCE_FIRST + action - TAS_ACTION_SPLICE_SOURCE_FIRST;
    size_t values[] = {splice->options.source_first, splice->options.source_count, splice->options.destination_first,
                       splice->options.destination_count};
    char text[32];
    snprintf(text, sizeof(text), "%zu", values[control - TAS_EDIT_SPLICE_SOURCE_FIRST]);
    splice->edit_destination = splice->destination;
    splice->edit_revision = splice->revision;
    splice->edit_generation = splice->generation;
    splice->edit_settings_serial = splice->settings_serial;
    splice->edit_control = control;
    desktop_start_text_edit(ui, control, text);
}

void desktop_tas_splice_action(FrontendDesktopUi *ui, unsigned action) {
    DesktopTasSplicer *splice = &ui->tas_editor->splice;
    if (action == TAS_ACTION_SPLICE_OPEN) {
        ui->tas_editor->sidebar_tab = 6;
        desktop_tas_splice_observe(ui);
        return;
    }
    if (!current(ui)) {
        desktop_tas_status(ui, "The timeline changed. Review the refreshed splice preview.");
        return;
    }
    if (!ready(ui)) {
        return;
    }
    if (action == TAS_ACTION_SPLICE_LOAD) {
        load_source(ui);
        return;
    }
    if (action >= TAS_ACTION_SPLICE_SOURCE_FIRST && action <= TAS_ACTION_SPLICE_DESTINATION_COUNT) {
        start_range_edit(ui, action);
        return;
    }
    if (action == TAS_ACTION_SPLICE_APPLY) {
        NesTasProject *project = desktop_tas_project_writable(ui, true);
        if (!project) {
            return;
        }
        char error[256] = {0};
        NesTasResult result = nes_tas_splice_apply(project, splice->source ? splice->source : project, &splice->options,
                                                   error, sizeof(error));
        if (result != NES_TAS_OK) {
            desktop_tas_status(ui, error);
            return;
        }
        ui->tas_editor->cursor = splice->preview.first;
        ui->tas_editor->anchor_valid = false;
        desktop_tas_after_model_edit(ui, result, "Splice applied. Undo restores the complete previous timeline.");
        desktop_tas_splice_observe(ui);
        return;
    }
    if (action == TAS_ACTION_SPLICE_CURRENT) {
        nes_tas_project_destroy(splice->source);
        splice->source = NULL;
        splice->options.source_first = 0;
        splice->options.source_count = nes_tas_project_frame_count(splice->destination);
        snprintf(splice->source_name, sizeof(splice->source_name), "Current project");
    } else if (action == TAS_ACTION_SPLICE_SOURCE_SELECTION || action == TAS_ACTION_SPLICE_DESTINATION_SELECTION) {
        size_t first, count;
        if ((action == TAS_ACTION_SPLICE_SOURCE_SELECTION && splice->source) || !selection(ui, &first, &count)) {
            return;
        }
        if (action == TAS_ACTION_SPLICE_SOURCE_SELECTION) {
            splice->options.source_first = first;
            splice->options.source_count = count;
        } else {
            splice->options.destination_first = first;
            splice->options.destination_count = count;
        }
    } else if (action == TAS_ACTION_SPLICE_MARKERS) {
        splice->options.copy_markers = !splice->options.copy_markers;
    } else if (action >= TAS_ACTION_SPLICE_APPEND && action <= TAS_ACTION_SPLICE_EXTRACT) {
        splice->options.mode = (NesTasSpliceMode)(action - TAS_ACTION_SPLICE_APPEND);
    } else {
        return;
    }
    ++splice->settings_serial;
    desktop_tas_splice_observe(ui);
}

bool desktop_tas_splice_commit(FrontendDesktopUi *ui, const char *text, char *error, size_t size) {
    DesktopTasSplicer *splice = &ui->tas_editor->splice;
    const NesTasProject *project = desktop_tas_project_const(ui);
    if (!ready(ui) || !current(ui) || splice->edit_destination != project ||
        splice->edit_generation != generation(ui) || splice->edit_revision != nes_tas_project_revision(project) ||
        splice->edit_settings_serial != splice->settings_serial || splice->edit_control != ui->edit_control) {
        snprintf(error, size, "The project or splice options changed. Open the range field again.");
        return false;
    }
    size_t value;
    if (!tas_frontend_parse_frame(text, &value)) {
        snprintf(error, size, "Enter a nonnegative whole number.");
        return false;
    }
    size_t source_frames = nes_tas_project_frame_count(splice->source ? splice->source : project);
    size_t destination_frames = nes_tas_project_frame_count(project);
    unsigned control = ui->edit_control;
    bool source = control <= TAS_EDIT_SPLICE_SOURCE_COUNT;
    if (value > (source ? source_frames : destination_frames) || (control == TAS_EDIT_SPLICE_SOURCE_COUNT && !value)) {
        snprintf(error, size, "%s",
                 source ? "Choose a source frame or nonzero count inside the source movie."
                        : "Choose a destination frame or count inside the current timeline.");
        return false;
    }
    if (control == TAS_EDIT_SPLICE_SOURCE_FIRST) {
        splice->options.source_first = value;
    } else if (control == TAS_EDIT_SPLICE_SOURCE_COUNT) {
        splice->options.source_count = value;
    } else if (control == TAS_EDIT_SPLICE_DESTINATION_FIRST) {
        splice->options.destination_first = value;
    } else if (control == TAS_EDIT_SPLICE_DESTINATION_COUNT) {
        splice->options.destination_count = value;
    } else {
        snprintf(error, size, "Unknown splice range field.");
        return false;
    }
    ++splice->settings_serial;
    splice->edit_control = 0;
    desktop_tas_splice_observe(ui);
    return true;
}
