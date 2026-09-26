/* capture_tools.c - Production capture inspector and preference controls
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "capture_tools.h"
#include "capture_frontend.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "../capture/capture_riff.h"
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    NesRiffReport riff;
    char path[CAPTURE_PATH_CAPACITY];
    char summary[256];
    char **rows;
    size_t row_count;
    char values[NES_MOVIE_PREFERENCE_COUNT][NES_MOVIE_BACKUP_PATH];
    char level[16];
    unsigned registered;
    bool gif_command;
} CaptureTools;

static bool error_text(char *error, size_t size, const char *text) {
    if (error && size) {
        snprintf(error, size, "%s", text);
    }
    return false;
}

static bool add(FrontendPanelModel *model, unsigned id, FrontendPanelControlType type, const char *label,
                const char *value, int selected, const char *const *items, size_t count, bool enabled) {
    FrontendPanelControl control = {.id = id,
                                    .type = type,
                                    .label = label,
                                    .value = value,
                                    .selected = selected,
                                    .items = items,
                                    .item_count = count,
                                    .enabled = enabled};
    return frontend_panel_add_control(model, &control);
}

static void clear_rows(CaptureTools *tools) {
    for (size_t i = 0; i < tools->row_count; ++i) {
        free(tools->rows[i]);
    }
    free(tools->rows);
    tools->rows = NULL;
    tools->row_count = 0;
}

static bool inspect(CaptureTools *tools, char *error, size_t size) {
    NesRiffReport next = {0};
    NesFileResult result = nes_capture_riff_inspect(tools->path, &next);
    if (result != NES_FILE_OK) {
        return error_text(error, size, nes_file_result_message(result));
    }
    char **rows = calloc(next.count, sizeof(*rows));
    if (!rows) {
        nes_capture_riff_free(&next);
        return error_text(error, size, "Not enough memory for inspector rows");
    }
    for (size_t i = 0; i < next.count; ++i) {
        rows[i] = malloc(320);
        if (!rows[i]) {
            for (size_t j = 0; j < i; ++j) {
                free(rows[j]);
            }
            free(rows);
            nes_capture_riff_free(&next);
            return error_text(error, size, "Not enough memory for inspector rows");
        }
        const NesRiffNode *n = &next.nodes[i];
        snprintf(rows[i], 320, "%*s%s %s @%" PRIu64 " size=%u%s %s", (int)n->depth * 2, "", n->tag, n->kind, n->offset,
                 n->size, n->warning ? " WARNING:" : "", n->detail);
    }
    clear_rows(tools);
    nes_capture_riff_free(&tools->riff);
    tools->riff = next;
    tools->rows = rows;
    tools->row_count = next.count;
    snprintf(tools->summary, sizeof(tools->summary),
             "%" PRIu64 " bytes; %zu rows; %u warnings; %ux%u; %u frames; %u streams", next.file_size, next.count,
             next.warnings, next.width, next.height, next.video_frames, next.streams);
    return true;
}

static bool riff_snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    NesCaptureFrontend *f = context;
    CaptureTools *tools = f->tools;
    model->status = tools->summary;
    bool ok = add(model, 1, FRONTEND_PANEL_TEXT, "AVI/RIFF file (read only)", tools->path, 0, NULL, 0, true) &&
              add(model, 2, FRONTEND_PANEL_ACTION, "Inspect file", "", 0, NULL, 0, true) &&
              add(model, 3, FRONTEND_PANEL_LIST, "Chunk hierarchy and index entries", "", -1,
                  (const char *const *)tools->rows, tools->row_count, true);
    return ok || error_text(error, size, "Inspector requires three controls");
}

static bool riff_action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    (void)selected;
    NesCaptureFrontend *f = context;
    CaptureTools *tools = f->tools;
    if (id == 1 && value && strlen(value) < sizeof(tools->path)) {
        strcpy(tools->path, value);
        return true;
    }
    if (id == 2) {
        return inspect(tools, error, size);
    }
    return error_text(error, size, "Enter an AVI file path, then choose Inspect file");
}

static bool preferences_snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    NesCaptureFrontend *f = context;
    CaptureTools *tools = f->tools;
    static const char *const labels[] = {"At movie end",
                                         "Read-only playback default",
                                         "Insert recorded TAS frames",
                                         "Start recordings from power-on",
                                         "Show movie subtitles",
                                         "Subtitle duration (movie frames)",
                                         "Game overlays: 1=input, 4=frame, 8=status (add values)",
                                         "Capture overlays: 1=input, 2=subtitles, 4=frame, 8=status",
                                         "Input/frame/status position",
                                         "Automatic movie and TAS backups",
                                         "Retained backups (1-100)",
                                         "Backup directory (empty = beside movie)"};
    static const char *const endings[] = {"Pause at final frame", "Stop and restore previous game"};
    static const char *const positions[] = {"Top left", "Top right", "Bottom left", "Bottom right"};
    for (unsigned i = 0; i < NES_MOVIE_PREFERENCE_COUNT; ++i) {
        if (i == 6 || i == 7) {
            continue;
        }
        if (!nes_movie_preferences_get(&f->preferences, nes_movie_preferences_key(i), tools->values[i],
                                       sizeof(tools->values[i]))) {
            return false;
        }
        FrontendPanelControlType type = FRONTEND_PANEL_TEXT;
        const char *const *items = NULL;
        size_t count = 0;
        if ((i >= 1 && i <= 4) || i == 9) {
            type = FRONTEND_PANEL_CHECKBOX;
        }
        if (i == 0 || i == 8) {
            type = FRONTEND_PANEL_CHOICE;
            items = i == 0 ? endings : positions;
            count = i == 0 ? 2 : 4;
        }
        if (!add(model, i + 1, type, labels[i], tools->values[i], atoi(tools->values[i]), items, count, true)) {
            return error_text(error, size, "Movie preferences require twelve controls");
        }
    }
    static const char *const overlay_labels[] = {
        "Show controller inputs",  "Show frame counter",    "Show movie/TAS status",   "Capture controller inputs",
        "Capture movie subtitles", "Capture frame counter", "Capture movie/TAS status"};
    static const unsigned bits[] = {1, 4, 8, 1, 2, 4, 8};
    for (unsigned i = 0; i < 7; ++i) {
        unsigned mask = i < 3 ? f->preferences.display_overlays : f->preferences.capture_overlays;
        if (!add(model, 20 + i, FRONTEND_PANEL_CHECKBOX, overlay_labels[i], "", (mask & bits[i]) != 0, NULL, 0, true)) {
            return error_text(error, size, "Movie preferences require seventeen controls");
        }
    }
    model->status = f->effective_preferences;
    return true;
}

static bool preferences_action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    NesCaptureFrontend *f = context;
    if (id >= 20 && id <= 26) {
        const unsigned bits[] = {1, 4, 8, 1, 2, 4, 8};
        if (id < 23) {
            f->preferences.display_overlays ^= bits[id - 20];
        } else {
            f->preferences.capture_overlays ^= bits[id - 20];
        }
        return true;
    }
    if (!id || id > NES_MOVIE_PREFERENCE_COUNT) {
        return false;
    }
    unsigned index = id - 1;
    char number[16];
    if ((index >= 1 && index <= 4) || index == 9) {
        if (!nes_movie_preferences_get(&f->preferences, nes_movie_preferences_key(index), number, sizeof(number))) {
            return false;
        }
        strcpy(number, number[0] == '0' ? "1" : "0");
        value = number;
    } else if (index == 0 || index == 8) {
        snprintf(number, sizeof(number), "%d", selected);
        value = number;
    }
    NesMoviePreferences next = f->preferences;
    if (!nes_movie_preferences_set(&next, nes_movie_preferences_key(index), value)) {
        return error_text(error, size, "Movie preference value is outside the supported range");
    }
    if (f->apply_preferences && !f->apply_preferences(f->overlay_context, &next, error, size)) {
        return false;
    }
    f->preferences = next;
    nes_movie_backup_set_policy(&next.backup);
    return true;
}

static bool encoding_snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    NesCaptureFrontend *f = context;
    CaptureTools *tools = f->tools;
    static const char *const formats[] = {"AVI video with stereo audio", "Animated GIF (no audio)"};
    static const char *const codecs[] = {"Uncompressed RGB24", "ZMBV lossless RGB32"};
    static const char *const scales[] = {"1x", "2x", "3x", "4x"};
    bool enabled = !f->session.info.recording;
    snprintf(tools->level, sizeof(tools->level), "%u", f->options.compression_level);
    bool ok = add(model, 1, FRONTEND_PANEL_CHOICE, "Video format", "", f->options.format, formats, 2, enabled) &&
              add(model, 2, FRONTEND_PANEL_CHOICE, "AVI codec", "", f->options.codec, codecs, 2, enabled) &&
              add(model, 3, FRONTEND_PANEL_TEXT, "Compression level (0-9)", tools->level, 0, NULL, 0, enabled) &&
              add(model, 4, FRONTEND_PANEL_CHOICE, "GIF nearest-neighbor scale", "", (int)f->options.gif_scale - 1,
                  scales, 4, enabled) &&
              add(model, 5, FRONTEND_PANEL_TEXT, "GIF output path", f->gif_path, 0, NULL, 0, enabled) &&
              add(model, CAPTURE_COMMAND_GIF, FRONTEND_PANEL_ACTION, "Record animated GIF", "", 0, NULL, 0,
                  enabled && f->hooks.has_image(f->hooks.context));
    model->status = "AVI and GIF advance only with completed emulated frames. Select overlays in Movie preferences.";
    return ok || error_text(error, size, "Encoding preferences require six controls");
}

static bool record_gif(void *context, char *error, size_t size) {
    NesCaptureFrontend *f = context;
    if (f->session.info.recording) {
        return error_text(error, size, "Stop recording first");
    }
    f->options.format = NES_CAPTURE_FORMAT_GIF;
    return frontend_command_invoke(CAPTURE_COMMAND_VIDEO, error, size);
}

static bool encoding_action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    NesCaptureFrontend *f = context;
    if (f->session.info.recording) {
        return error_text(error, size, "Stop recording before changing encoding settings");
    }
    if (id == CAPTURE_COMMAND_GIF) {
        return record_gif(context, error, size);
    }
    if (id == 1 && selected >= 0 && selected <= 1) {
        f->options.format = (NesCaptureFormat)selected;
    } else if (id == 2 && selected >= 0 && selected <= 1) {
        f->options.codec = (NesCaptureCodec)selected;
    } else if (id == 3 && value && value[0] >= '0' && value[0] <= '9' && !value[1]) {
        f->options.compression_level = (unsigned)(value[0] - '0');
    } else if (id == 4 && selected >= 0 && selected <= 3) {
        f->options.gif_scale = (unsigned)selected + 1;
    } else if (id == 5 && value && strlen(value) < sizeof(f->gif_path)) {
        if (*value && !f->hooks.validate_path(f->hooks.context, value, error, size)) {
            return false;
        }
        strcpy(f->gif_path, value);
    } else {
        return error_text(error, size, "Invalid encoder setting");
    }
    return true;
}

bool nes_capture_tools_init(NesCaptureFrontend *f) {
    CaptureTools *tools = calloc(1, sizeof(*tools));
    if (!tools) {
        return false;
    }
    f->tools = tools;
    const FrontendPanelSpec panels[] = {{.id = CAPTURE_RIFF_PANEL,
                                         .title = "AVI / RIFF Inspector",
                                         .category = "Tools",
                                         .snapshot = riff_snapshot,
                                         .action = riff_action,
                                         .userdata = f},
                                        {.id = CAPTURE_MOVIE_PANEL,
                                         .title = "Movie preferences",
                                         .category = "Settings",
                                         .snapshot = preferences_snapshot,
                                         .action = preferences_action,
                                         .userdata = f},
                                        {.id = CAPTURE_ENCODING_PANEL,
                                         .title = "Capture encoding",
                                         .category = "Tools",
                                         .snapshot = encoding_snapshot,
                                         .action = encoding_action,
                                         .userdata = f}};
    for (unsigned i = 0; i < 3; ++i) {
        if (!frontend_panel_register(&panels[i])) {
            nes_capture_tools_shutdown(f);
            return false;
        }
        ++tools->registered;
    }
    FrontendCommandSpec command = {.id = CAPTURE_COMMAND_GIF,
                                   .label = "Record animated GIF",
                                   .menu = "Tools",
                                   .flags = FRONTEND_COMMAND_NEEDS_SESSION,
                                   .handler = record_gif,
                                   .userdata = f};
    if (!frontend_command_register(&command)) {
        nes_capture_tools_shutdown(f);
        return false;
    }
    tools->gif_command = true;
    return true;
}

void nes_capture_tools_shutdown(NesCaptureFrontend *f) {
    CaptureTools *tools = f ? f->tools : NULL;
    if (!tools) {
        return;
    }
    for (unsigned i = 0; i < tools->registered; ++i) {
        (void)frontend_panel_unregister(CAPTURE_RIFF_PANEL + i);
    }
    if (tools->gif_command) {
        (void)frontend_command_unregister(CAPTURE_COMMAND_GIF);
    }
    clear_rows(tools);
    nes_capture_riff_free(&tools->riff);
    free(tools);
    f->tools = NULL;
}
