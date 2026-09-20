/* Typed settings controls and validated numeric entry. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_internal.h"
#include "frontend_panels.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum { NUMBER_UNSIGNED, NUMBER_SIGNED, NUMBER_SHORT, NUMBER_WIDE, NUMBER_DOUBLE, NUMBER_FLOAT } NumberType;

typedef struct {
    void *value;
    NumberType type;
    double low, high;
} Number;

#define NUMBER(field, type_, low_, high_)                                                                              \
    return (Number) {                                                                                                  \
        &s->field, type_, low_, high_                                                                                  \
    }

static Number number(FrontendDesktopUi *ui, int row) {
    FrontendSettings *s = &ui->staged;
    switch (ui->settings_category) {
    case 0:
        if (row == 2) {
            NUMBER(recent_file_limit, NUMBER_UNSIGNED, 0, FRONTEND_RECENT_MAX);
        }
        if (row == 6) {
            NUMBER(window_width, NUMBER_UNSIGNED, 640, 16384);
        }
        if (row == 7) {
            NUMBER(window_height, NUMBER_UNSIGNED, 480, 16384);
        }
        break;
    case 1:
        if (row == 1) {
            NUMBER(speed, NUMBER_DOUBLE, .1, 16);
        }
        if (row == 2) {
            NUMBER(fast_forward_speed, NUMBER_DOUBLE, .1, 16);
        }
        if (row == 3) {
            NUMBER(rewind_seconds, NUMBER_UNSIGNED, 0, 60);
        }
        if (row == 5) {
            NUMBER(rewind_step_frames, NUMBER_UNSIGNED, 1, 30);
        }
        break;
    case 2:
        if (row >= 7) {
            NesVideoOverscan *o = &s->presentation.overscan[(row - 7) / 4];
            unsigned *edges[] = {&o->left, &o->right, &o->top, &o->bottom};
            return (Number){edges[(row - 7) % 4], NUMBER_UNSIGNED, 0, (row - 7) % 4 < 2 ? 255 : 239};
        }
        break;
    case 3:
        if (row == 1) {
            NUMBER(audio_mix.master_volume, NUMBER_UNSIGNED, 0, 100);
        }
        if (row == 3) {
            NUMBER(audio_sample_rate, NUMBER_UNSIGNED, 8000, 192000);
        }
        if (row == 4) {
            NUMBER(audio_buffer_samples, NUMBER_UNSIGNED, 64, 8192);
        }
        if (row >= 5) {
            unsigned channel = (unsigned)(row - 5) / 2;
            if ((row - 5) % 2) {
                NUMBER(audio_mix.pan[channel], NUMBER_SIGNED, -100, 100);
            }
            NUMBER(audio_mix.volume[channel], NUMBER_UNSIGNED, 0, 200);
        }
        break;
    case 4:
        if (row == 5) {
            NUMBER(zapper_radius, NUMBER_UNSIGNED, 0, 255);
        }
        break;
    case 5:
        if (row == 15) {
            NUMBER(nsf_player.silence_ms, NUMBER_UNSIGNED, 10, 600000);
        }
        if (row == 16) {
            NUMBER(nsf_player.silence_threshold, NUMBER_FLOAT, 0, .1);
        }
        break;
    case 6:
        if (row == 6) {
            NUMBER(capture.sample_rate, NUMBER_UNSIGNED, 8000, 192000);
        }
        if (row == 7) {
            NUMBER(capture.byte_limit, NUMBER_WIDE, 1024, UINT32_MAX);
        }
        break;
    case 7:
        if (row == 16) {
            NUMBER(cart_dips, NUMBER_UNSIGNED, 0, 255);
        }
        if (row == 17) {
            NUMBER(vs_dips, NUMBER_SHORT, 0, 65535);
        }
        if (row == 19) {
            NUMBER(startup_cpu_offset, NUMBER_UNSIGNED, 0, 15);
        }
        if (row == 20) {
            NUMBER(startup_ppu_phase, NUMBER_UNSIGNED, 0, 4);
        }
        if (row == 22) {
            NUMBER(startup_seed, NUMBER_UNSIGNED, 0, UINT32_MAX);
        }
        if (row == 24) {
            NUMBER(power_on_seed, NUMBER_UNSIGNED, 0, UINT32_MAX);
        }
        break;
    }
    return (Number){0};
}

#undef NUMBER

DesktopSettingKind desktop_setting_kind(const FrontendDesktopUi *ui, int row) {
    switch (ui->settings_category) {
    case 0:
        return row == 2 || row >= 6 ? SETTING_NUMBER : SETTING_TOGGLE;
    case 1:
        return row == 0 || row == 4 ? SETTING_CHOICE : SETTING_NUMBER;
    case 2:
        return row == 3 ? SETTING_CHOICE : row < 7 ? SETTING_TOGGLE : SETTING_NUMBER;
    case 3:
        return row == 0 ? SETTING_TOGGLE : row == 2 ? SETTING_TEXT : SETTING_NUMBER;
    case 4:
        return row == 5 ? SETTING_NUMBER : row == 7 ? SETTING_TEXT : row >= 8 ? SETTING_BINDING : SETTING_CHOICE;
    case 5:
        return row <= 4 || row == 9 || row == 10 ? SETTING_TEXT
               : row == 5                        ? SETTING_CHOICE
               : row >= 15                       ? SETTING_NUMBER
                                                 : SETTING_TOGGLE;
    case 6:
        return row == 0 ? SETTING_CHOICE : row <= 4 ? SETTING_TEXT : row == 5 ? SETTING_TOGGLE : SETTING_NUMBER;
    case 7:
        return row == 1                                          ? SETTING_READONLY
               : row < 5 || row == 15                            ? SETTING_CHOICE
               : row < 16 || row == 18 || row == 21 || row == 23 ? SETTING_TOGGLE
                                                                 : SETTING_NUMBER;
    default:
        return SETTING_READONLY;
    }
}

int desktop_setting_choices(FrontendDesktopUi *ui, int row, int *selected) {
    FrontendSettings *s = &ui->staged;
    *selected = 0;
    switch (ui->settings_category) {
    case 1:
        if (row == 0) {
            *selected = s->region_mode;
            return 4;
        }
        if (row == 4) {
            *selected = (int)s->run_ahead_frames;
            return 5;
        }
        break;
    case 2:
        if (row == 3) {
            *selected = s->aspect_mode;
            return 2;
        }
        break;
    case 4:
        if (row == 0) {
            for (size_t i = 0; i < s->profile_count; ++i) {
                if (!strcmp(s->profiles[i].name, s->active_profile)) {
                    *selected = (int)i;
                }
            }
            return (int)s->profile_count;
        }
        if (row == 1) {
            *selected = s->input.adapter;
            return 4;
        }
        if (row == 2 || row == 3) {
            *selected = s->input.ports[row - 2];
            return 11;
        }
        if (row == 4) {
            *selected = s->input.expansion;
            return 19;
        }
        if (row == 6) {
            *selected = (int)ui->settings_player;
            return NES_INPUT_PLAYERS;
        }
        break;
    case 5:
        if (row == 5) {
            *selected = s->disk_save_mode;
            return 2;
        }
        break;
    case 6:
        if (row == 0) {
            *selected = (int)s->state_slot;
            return NES_STATE_SLOT_COUNT;
        }
        break;
    case 7:
        if (row == 0) {
            *selected = s->console_model;
            return 4;
        }
        if (row == 2) {
            *selected = s->cpu_revision;
            return 2;
        }
        if (row == 3) {
            *selected = s->ppu_revision;
            return 2;
        }
        if (row == 4) {
            *selected = s->ram_power_state;
            return 4;
        }
        if (row == 15) {
            *selected = s->mmc3_revision_a;
            return 2;
        }
        break;
    }
    return 0;
}

void desktop_setting_choose(FrontendDesktopUi *ui, int row, int option) {
    int selected, count = desktop_setting_choices(ui, row, &selected);
    if (option < 0 || option >= count) {
        return;
    }
    for (int guard = 0; selected != option && guard < count; ++guard) {
        desktop_adjust_setting(ui, row, option > selected ? 1 : -1);
        (void)desktop_setting_choices(ui, row, &selected);
    }
}

void desktop_setting_choice_text(FrontendDesktopUi *ui, int row, int option, char *text, size_t size) {
    FrontendDesktopUi preview = *ui;
    char name[96];
    desktop_setting_choose(&preview, row, option);
    desktop_setting_text(&preview, row, name, sizeof(name), text, size);
}

static double number_value(Number n) {
    switch (n.type) {
    case NUMBER_UNSIGNED:
        return *(unsigned *)n.value;
    case NUMBER_SIGNED:
        return *(int *)n.value;
    case NUMBER_SHORT:
        return *(uint16_t *)n.value;
    case NUMBER_WIDE:
        return (double)*(uint64_t *)n.value;
    case NUMBER_DOUBLE:
        return *(double *)n.value;
    case NUMBER_FLOAT:
        return *(float *)n.value;
    }
    return 0;
}

bool desktop_setting_commit_number(FrontendDesktopUi *ui, int row, const char *text) {
    Number n = number(ui, row);
    if (!n.value) {
        return false;
    }
    errno = 0;
    char *end;
    double value = strtod(text, &end);
    bool parsed = end != text;
    while (*end == ' ' || *end == '\t') {
        ++end;
    }
    bool integer = n.type != NUMBER_DOUBLE && n.type != NUMBER_FLOAT;
    if (!parsed || *end || errno || !isfinite(value) || value < n.low || value > n.high ||
        (integer && floor(value) != value)) {
        char message[160];
        snprintf(message, sizeof(message), "Enter %s from %.10g to %.10g.", integer ? "a whole number" : "a number",
                 n.low, n.high);
        desktop_copy_status(ui, message);
        return false;
    }
    switch (n.type) {
    case NUMBER_UNSIGNED:
        *(unsigned *)n.value = (unsigned)value;
        break;
    case NUMBER_SIGNED:
        *(int *)n.value = (int)value;
        break;
    case NUMBER_SHORT:
        *(uint16_t *)n.value = (uint16_t)value;
        break;
    case NUMBER_WIDE:
        *(uint64_t *)n.value = (uint64_t)value;
        break;
    case NUMBER_DOUBLE:
        *(double *)n.value = value;
        break;
    case NUMBER_FLOAT:
        *(float *)n.value = (float)value;
        break;
    }
    return true;
}

void desktop_activate_setting(FrontendDesktopUi *ui, int row) {
    ui->settings_row = row;
    DesktopSettingKind kind = desktop_setting_kind(ui, row);
    if (kind == SETTING_READONLY) {
        return;
    }
    if (kind == SETTING_CHOICE) {
        ui->choice_open = true;
        ui->choice_panel = false;
        ui->choice_row = row;
        (void)desktop_setting_choices(ui, row, &ui->choice_index);
        ui->choice_page = ui->choice_index / 20;
    } else if (kind == SETTING_NUMBER) {
        Number n = number(ui, row);
        char text[64];
        if (!n.value) {
            return;
        }
        snprintf(text, sizeof(text), "%.10g", number_value(n));
        desktop_begin_edit(ui, (unsigned)row, text);
        ui->edit_number = true;
    } else {
        desktop_adjust_setting(ui, row, 1);
    }
}

static bool panel_choice(FrontendDesktopUi *ui, FrontendPanelModel *model) {
    return frontend_panel_snapshot(ui->panel_id, model, NULL, 0) && ui->choice_row >= 0 &&
           (size_t)ui->choice_row < model->count;
}

int desktop_choice_count(FrontendDesktopUi *ui) {
    if (!ui->choice_panel) {
        int selected;
        return desktop_setting_choices(ui, ui->choice_row, &selected);
    }
    FrontendPanelControl controls[64];
    FrontendPanelModel model = {.controls = controls, .capacity = 64};
    return panel_choice(ui, &model) ? (int)controls[ui->choice_row].item_count : 0;
}

void desktop_choice_text(FrontendDesktopUi *ui, int option, char *text, size_t size) {
    text[0] = 0;
    if (!ui->choice_panel) {
        desktop_setting_choice_text(ui, ui->choice_row, option, text, size);
        return;
    }
    FrontendPanelControl controls[64];
    FrontendPanelModel model = {.controls = controls, .capacity = 64};
    if (panel_choice(ui, &model) && option >= 0 && (size_t)option < controls[ui->choice_row].item_count) {
        snprintf(text, size, "%s", controls[ui->choice_row].items[option]);
    }
}

void desktop_choice_select(FrontendDesktopUi *ui, int option) {
    if (option < 0 || option >= desktop_choice_count(ui)) {
        return;
    }
    if (!ui->choice_panel) {
        desktop_setting_choose(ui, ui->choice_row, option);
    } else {
        FrontendPanelControl controls[64];
        FrontendPanelModel model = {.controls = controls, .capacity = 64};
        char error[256] = {0};
        if (panel_choice(ui, &model) &&
            !frontend_panel_action(ui->panel_id, controls[ui->choice_row].id, NULL, option, error, sizeof(error))) {
            desktop_copy_status(ui, error);
        }
    }
    ui->choice_open = false;
}
