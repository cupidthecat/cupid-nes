/*
 * desktop_memory.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Memory search result tables. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_memory.h"
#include "desktop_internal.h"
#include "frontend_panels.h"
#include "memory_search_frontend.h"
#include "watch_frontend.h"

#include <stdio.h>
#include <string.h>

static const Clay_Color surface = {24, 28, 39, 255}, raised = {33, 39, 54, 255};
static const Clay_Color ink = {232, 237, 247, 255}, muted = {151, 165, 190, 255};
static const Clay_Color selected = {52, 57, 85, 255}, accent = {170, 180, 255, 255};

static void text(FrontendDesktopUi *ui, const char *value, unsigned size, Clay_Color color) {
    CLAY_TEXT(desktop_clay_string(ui->clay, value ? value : ""),
              CLAY_TEXT_CONFIG({.fontSize = (uint16_t)size, .textColor = color, .wrapMode = CLAY_TEXT_WRAP_NONE}));
}

static int index_of(const FrontendPanelModel *model, unsigned id) {
    for (size_t i = 0; i < model->count; ++i) {
        if (model->controls[i].id == id) {
            return (int)i;
        }
    }

    return -1;
}

static const char *control_value(const FrontendPanelControl *control) {
    if (control->type == FRONTEND_PANEL_CHOICE && control->selected >= 0 &&
        (size_t)control->selected < control->item_count) {
        return control->items[control->selected];
    }

    if (control->type == FRONTEND_PANEL_CHECKBOX) {
        return control->selected ? "On" : "Off";
    }

    return control->value ? control->value : "";
}

static void control(FrontendDesktopUi *ui, const FrontendPanelModel *model, unsigned id, float fraction) {
    int index = index_of(model, id);
    if (index < 0) {
        return;
    }

    const FrontendPanelControl *c = &model->controls[index];
    bool action = c->type == FRONTEND_PANEL_ACTION;
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_PERCENT(fraction)},
                             .layoutDirection = CLAY_TOP_TO_BOTTOM, .childGap = 3}}) {
        if (!action) {
            text(ui, c->label, 11, muted);
        }

        Clay_ElementId hit = desktop_clay_hit(ui->clay, c->enabled ? HIT_PANEL : HIT_NONE, index, 0);
        CLAY(hit, {.layout = {.sizing = {.width = CLAY_SIZING_PERCENT(1), .height = CLAY_SIZING_FIXED(26)},
                             .padding = {7, 7, 4, 4}, .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
                    .backgroundColor = ui->panel_row == index || Clay_PointerOver(hit) ? selected : raised,
                    .clip = {.horizontal = true, .vertical = true}}) {
            text(ui, action ? c->label : control_value(c), 12, c->enabled ? ink : muted);
        }
    }
}

static void control_row(FrontendDesktopUi *ui, const FrontendPanelModel *model, const unsigned *ids, size_t count) {
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}, .childGap = 8}}) {
        for (size_t i = 0; i < count; ++i) {
            control(ui, model, ids[i], 1.0f / (float)count);
        }
    }
}

static void note(FrontendDesktopUi *ui, const char *value, Clay_Color color) {
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(20)}},
                  .clip = {.horizontal = true, .vertical = true}}) {
        text(ui, value, 12, color);
    }
}

static void wrapped_note(FrontendDesktopUi *ui, const char *value, Clay_Color color, float height) {
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(height)}}}) {
        CLAY_TEXT(desktop_clay_string(ui->clay, value ? value : ""),
                  CLAY_TEXT_CONFIG({.fontSize = 12, .textColor = color, .wrapMode = CLAY_TEXT_WRAP_WORDS}));
    }
}

void desktop_memory_focus(FrontendDesktopUi *ui, const FrontendPanelModel *model, int direction) {
    if (!model->count) {
        return;
    }

    int count = (int)model->count;
    int index = ui->panel_row >= 0 && ui->panel_row < count ? ui->panel_row : 0;
    int step = direction < 0 ? -1 : 1;
    if (direction) {
        index = (index + count + step) % count;
    }

    for (int i = 0; i < count; ++i) {
        const FrontendPanelControl *c = &model->controls[index];
        if (c->enabled && !c->read_only) {
            ui->panel_row = index;
            return;
        }

        index = (index + count + step) % count;
    }
}

bool desktop_memory_layout(FrontendDesktopUi *ui, float width, float height) {
    if (width < 760 || height < 660) {
        return false;
    }

    FrontendPanelControl controls[64];
    FrontendPanelModel model = {.controls = controls, .capacity = 64};
    char error[256] = {0};
    if (!frontend_panel_snapshot(ui->panel_id, &model, error, sizeof(error))) {
        note(ui, error, muted);
        return true;
    }

    desktop_memory_focus(ui, &model, 0);
    static const unsigned setup[] = {MEMORY_SEARCH_SPACE, MEMORY_SEARCH_WIDTH, MEMORY_SEARCH_FORMAT, MEMORY_SEARCH_ALIGNED};
    static const unsigned filter[] = {MEMORY_SEARCH_FIRST, MEMORY_SEARCH_LAST, MEMORY_SEARCH_COMPARISON,
                                      MEMORY_SEARCH_REFERENCE, MEMORY_SEARCH_VALUE};
    static const unsigned actions[] = {MEMORY_SEARCH_START, MEMORY_SEARCH_FILTER, MEMORY_SEARCH_REFRESH, MEMORY_SEARCH_UNDO};
    static const unsigned pages[] = {MEMORY_SEARCH_PREV_PAGE, MEMORY_SEARCH_NEXT_PAGE, MEMORY_SEARCH_WATCH, MEMORY_SEARCH_COPY};
    static const unsigned draft[] = {MEMORY_SEARCH_BYTE, MEMORY_SEARCH_REPLACEMENT, MEMORY_SEARCH_COMPARE, MEMORY_SEARCH_DESCRIPTION};
    static const unsigned send[] = {MEMORY_SEARCH_SEND, MEMORY_SEARCH_TEST};
    ui->visible_rows = MEMORY_SEARCH_PAGE_SIZE;
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_GROW()},
                             .layoutDirection = CLAY_TOP_TO_BOTTOM, .childGap = 7}}) {
        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}, .childGap = 12}}) {
            CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}}}) {
                text(ui, ui->panel_id == CHEAT_FINDER_PANEL ? "Cheat Finder" : "Memory Search", 22, ink);
            }

            if (!ui->parent) {
                Clay_ElementId close = desktop_clay_hit(ui->clay, HIT_CLOSE, 0, 0);
                CLAY(close, {.layout = {.padding = {8, 8, 4, 4}}, .backgroundColor = raised}) {
                    text(ui, "Close", 12, ink);
                }
            }
        }

        control_row(ui, &model, setup, sizeof(setup) / sizeof(*setup));
        control_row(ui, &model, filter, sizeof(filter) / sizeof(*filter));
        control_row(ui, &model, actions, sizeof(actions) / sizeof(*actions));
        int info = index_of(&model, MEMORY_SEARCH_INFO);
        note(ui, info >= 0 ? controls[info].value : "", muted);
        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}, .childGap = 2}}) {
            for (unsigned col = 0; col < 5; ++col) {
                int at = index_of(&model, MEMORY_SEARCH_SORT + col);
                if (at < 0) {
                    continue;
                }

                const FrontendPanelControl *c = &controls[at];
                Clay_ElementId hit = desktop_clay_hit(ui->clay, c->enabled ? HIT_PANEL : HIT_NONE, at, 0);
                CLAY(hit, {.layout = {.sizing = {.width = CLAY_SIZING_PERCENT(.2f), .height = CLAY_SIZING_FIXED(26)},
                                     .padding = {8, 8, 4, 4}}, .backgroundColor = raised,
                            .clip = {.horizontal = true, .vertical = true}}) {
                    char title[64];
                    snprintf(title, sizeof(title), "%s%s", c->label,
                             c->value && *c->value ? (!strcmp(c->value, "descending") ? " v" : " ^") : "");
                    text(ui, title, 12, ui->panel_row == at ? accent : ink);
                }
            }
        }

        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()},
                                 .layoutDirection = CLAY_TOP_TO_BOTTOM}}) {
            for (unsigned i = 0; i < MEMORY_SEARCH_PAGE_SIZE; ++i) {
                int at = index_of(&model, MEMORY_SEARCH_ROW + i);
                if (at < 0) {
                    continue;
                }

                const FrontendPanelControl *c = &controls[at];
                Clay_ElementId hit = desktop_clay_hit(ui->clay, c->enabled ? HIT_PANEL : HIT_NONE, at, 0);
                bool changed = !strcmp(c->label, "Changed result");
                CLAY(hit, {.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(22)},
                                     .childGap = 2},
                            .backgroundColor = c->selected || ui->panel_row == at ? selected : i % 2 ? raised : surface}) {
                    for (unsigned col = 0; col < 5; ++col) {
                        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_PERCENT(.2f), .height = CLAY_SIZING_GROW()},
                                                 .padding = {8, 8, 3, 3}},
                                      .clip = {.horizontal = true, .vertical = true}}) {
                            text(ui, col < c->item_count ? c->items[col] : "", 12, changed && col == 1 ? accent : ink);
                        }
                    }
                }
            }
        }

        control_row(ui, &model, pages, sizeof(pages) / sizeof(*pages));
        int selection = index_of(&model, MEMORY_SEARCH_SELECTION);
        note(ui, selection >= 0 ? controls[selection].value : "", accent);
        control_row(ui, &model, draft, sizeof(draft) / sizeof(*draft));
        control_row(ui, &model, send, ui->panel_id == CHEAT_FINDER_PANEL ? 2 : 1);
        note(ui, model.status, muted);
    }

    return true;
}

bool desktop_watch_layout(FrontendDesktopUi *ui, float width, float height) {
    if (width < 760 || height < 710) return false;
    FrontendPanelControl controls[64];
    FrontendPanelModel model = {.controls = controls, .capacity = 64};
    char error[256] = {0};
    if (!frontend_panel_snapshot(ui->panel_id, &model, error, sizeof(error))) {
        note(ui, error, muted);
        return true;
    }
    desktop_memory_focus(ui, &model, 0);
    static const unsigned setup[] = {WATCH_SPACE, WATCH_EXPRESSION, WATCH_WIDTH, WATCH_FORMAT};
    static const unsigned editor[] = {WATCH_LABEL, WATCH_COUNT, WATCH_ADD, WATCH_UPDATE};
    static const unsigned history[] = {WATCH_PREVIOUS, WATCH_FREEZE, WATCH_COPY};
    static const unsigned actions[] = {WATCH_PREV_PAGE, WATCH_NEXT_PAGE, WATCH_TOGGLE, WATCH_REMOVE,
                                       WATCH_MOVE_UP, WATCH_MOVE_DOWN};
    static const unsigned export[] = {WATCH_EXPORT_PATH, WATCH_EXPORT};
    ui->visible_rows = WATCH_PAGE_SIZE;
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_GROW()},
                             .layoutDirection = CLAY_TOP_TO_BOTTOM, .childGap = 7}}) {
        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}, .childGap = 12}}) {
            CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}}}) {
                text(ui, "Memory Watches", 22, ink);
            }
            if (!ui->parent) {
                Clay_ElementId close = desktop_clay_hit(ui->clay, HIT_CLOSE, 0, 0);
                CLAY(close, {.layout = {.padding = {8, 8, 4, 4}}, .backgroundColor = raised}) {
                    text(ui, "Close", 12, ink);
                }
            }
        }
        control_row(ui, &model, setup, sizeof(setup) / sizeof(*setup));
        control_row(ui, &model, editor, sizeof(editor) / sizeof(*editor));
        control_row(ui, &model, history, sizeof(history) / sizeof(*history));
        int info = index_of(&model, WATCH_INFO);
        note(ui, info >= 0 ? controls[info].value : "", muted);
        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}, .childGap = 2}}) {
            for (unsigned col = 0; col < 5; ++col) {
                int at = index_of(&model, WATCH_SORT + col);
                if (at < 0) continue;
                const FrontendPanelControl *c = &controls[at];
                Clay_ElementId hit = desktop_clay_hit(ui->clay, c->enabled ? HIT_PANEL : HIT_NONE, at, 0);
                CLAY(hit, {.layout = {.sizing = {.width = CLAY_SIZING_PERCENT(.2f), .height = CLAY_SIZING_FIXED(26)},
                                     .padding = {8, 8, 4, 4}}, .backgroundColor = raised,
                            .clip = {.horizontal = true, .vertical = true}}) {
                    char title[64];
                    snprintf(title, sizeof(title), "%s%s", c->label,
                             c->value && *c->value ? (!strcmp(c->value, "descending") ? " v" : " ^") : "");
                    text(ui, title, 12, ui->panel_row == at ? accent : ink);
                }
            }
        }
        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()},
                                 .layoutDirection = CLAY_TOP_TO_BOTTOM}}) {
            for (unsigned i = 0; i < WATCH_PAGE_SIZE; ++i) {
                int at = index_of(&model, WATCH_ROW + i);
                if (at < 0) continue;
                const FrontendPanelControl *c = &controls[at];
                Clay_ElementId hit = desktop_clay_hit(ui->clay, c->enabled ? HIT_PANEL : HIT_NONE, at, 0);
                bool changed = !strcmp(c->label, "Changed watch");
                bool unavailable = !strcmp(c->label, "Unavailable watch");
                CLAY(hit, {.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(22)},
                                     .childGap = 2},
                            .backgroundColor = c->selected || ui->panel_row == at ? selected : i % 2 ? raised : surface}) {
                    for (unsigned col = 0; col < 5; ++col) {
                        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_PERCENT(.2f), .height = CLAY_SIZING_GROW()},
                                                 .padding = {8, 8, 3, 3}},
                                      .clip = {.horizontal = true, .vertical = true}}) {
                            text(ui, col < c->item_count ? c->items[col] : "", 12,
                                 unavailable ? muted : changed && col == 1 ? accent : ink);
                        }
                    }
                }
            }
        }
        control_row(ui, &model, actions, sizeof(actions) / sizeof(*actions));
        int selection = index_of(&model, WATCH_SELECTION), values = index_of(&model, WATCH_VALUES);
        wrapped_note(ui, selection >= 0 ? controls[selection].value : "", accent, 48);
        wrapped_note(ui, values >= 0 ? controls[values].value : "", ink, 32);
        control_row(ui, &model, export, sizeof(export) / sizeof(*export));
        note(ui, model.status, muted);
    }
    return true;
}
