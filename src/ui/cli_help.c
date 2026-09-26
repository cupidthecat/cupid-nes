/*
 * cli_help.c - Searchable command-line reference
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "cli_help.h"
#include "frontend_panels.h"
#include <ctype.h>
#include <string.h>

static bool contains(const char *text, const char *needle) {
    if (!text) {
        return false;
    }
    if (!*needle) {
        return true;
    }
    for (; *text; ++text) {
        size_t i = 0;
        while (needle[i] && text[i] && tolower((unsigned char)needle[i]) == tolower((unsigned char)text[i])) {
            ++i;
        }
        if (!needle[i]) {
            return true;
        }
    }
    return false;
}

bool frontend_cli_help_search(FrontendCliHelp *h, const char *query) {
    if (!h || !query || strlen(query) >= sizeof(h->search)) {
        return false;
    }
    memmove(h->search, query, strlen(query) + 1);
    h->count = 0;
    h->selected = 0;
    for (size_t i = 0; i < frontend_cli_count(); ++i) {
        const FrontendCliOption *o = frontend_cli_at(i);
        if (contains(o->name, h->search) || contains(o->alias, h->search) || contains(o->description, h->search) ||
            contains(o->values, h->search) || contains(o->default_value, h->search)) {
            h->matches[h->count] = i;
            h->items[h->count++] = o->name;
        }
    }
    return true;
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    FrontendCliHelp *h = context;
    const FrontendCliOption *o = h->count ? frontend_cli_at(h->matches[h->selected]) : NULL;
    FrontendPanelControl controls[] = {
        {1, FRONTEND_PANEL_TEXT, "Search switches, values and descriptions", h->search, NULL, 0, 0, true, false},
        {2, FRONTEND_PANEL_LIST, "Application options", NULL, h->items, h->count, h->selected, true, true},
        {3, FRONTEND_PANEL_TEXT, "Option", o ? o->name : "No matches", NULL, 0, 0, true, true},
        {4, FRONTEND_PANEL_TEXT, "Alias", o && o->alias ? o->alias : "None", NULL, 0, 0, true, true},
        {5, FRONTEND_PANEL_TEXT, "Accepted values", o ? o->values : "", NULL, 0, 0, true, true},
        {6, FRONTEND_PANEL_TEXT, "Default", o ? o->default_value : "", NULL, 0, 0, true, true},
        {7, FRONTEND_PANEL_TEXT, "Description", o ? o->description : "", NULL, 0, 0, true, true},
        {8, FRONTEND_PANEL_ACTION, "Previous option", NULL, NULL, 0, 0, h->count && h->selected > 0, false},
        {9, FRONTEND_PANEL_ACTION, "Next option", NULL, NULL, 0, 0, h->count && (size_t)(h->selected + 1) < h->count,
         false}};
    for (size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); ++i) {
        if (!frontend_panel_add_control(model, &controls[i])) {
            return false;
        }
    }
    model->status = "Cupid [options] [image]. Tab selects controls; arrows navigate the list; Enter edits search.";
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    FrontendCliHelp *h = context;
    (void)error;
    (void)size;
    if (id == 1) {
        return frontend_cli_help_search(h, value);
    }
    if (id == 2 && selected >= 0 && (size_t)selected < h->count) {
        h->selected = selected;
        return true;
    }
    if (id == 8 && h->selected > 0) {
        --h->selected;
        return true;
    }
    if (id == 9 && (size_t)(h->selected + 1) < h->count) {
        ++h->selected;
        return true;
    }
    return false;
}

bool frontend_cli_help_register(FrontendCliHelp *h) {
    if (!h) {
        return false;
    }
    memset(h, 0, sizeof(*h));
    frontend_cli_help_search(h, "");
    FrontendPanelSpec spec = {CLI_HELP_PANEL, "Command Line Help", "Help", 0, snapshot, action, h};
    return frontend_panel_register(&spec);
}

void frontend_cli_help_unregister(void) {
    frontend_panel_unregister(CLI_HELP_PANEL);
}
