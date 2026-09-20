/* Desktop network sessions. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_NETPLAY_FRONTEND_H
#define CUPID_NETPLAY_FRONTEND_H
#include <stdbool.h>
#include <stddef.h>
struct FrontendExecutionRuntime;
typedef struct FrontendNetplay FrontendNetplay;
FrontendNetplay *frontend_netplay_create(struct FrontendExecutionRuntime *execution);
void frontend_netplay_destroy(FrontendNetplay *frontend);
bool frontend_netplay_register(FrontendNetplay *frontend);
bool frontend_netplay_before_frame(FrontendNetplay *frontend);
void frontend_netplay_after_frame(FrontendNetplay *frontend, bool completed);
bool frontend_netplay_pause(FrontendNetplay *frontend, bool paused, char *error, size_t size);
const char *frontend_netplay_status(FrontendNetplay *frontend);
#endif
