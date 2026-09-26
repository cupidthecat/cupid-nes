/*
 * cli_help.h - Searchable command-line reference
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_CLI_HELP_H
#define CUPID_CLI_HELP_H
#include "cli_options.h"

enum { CLI_HELP_PANEL = 0x2680 };

typedef struct {
    char search[128];
    const char *items[FRONTEND_CLI_COUNT];
    size_t matches[FRONTEND_CLI_COUNT], count;
    int selected;
} FrontendCliHelp;

bool frontend_cli_help_search(FrontendCliHelp *help, const char *query);
bool frontend_cli_help_register(FrontendCliHelp *help);
void frontend_cli_help_unregister(void);
#endif
