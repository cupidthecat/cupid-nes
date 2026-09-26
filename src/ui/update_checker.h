/*
 * update_checker.h - Asynchronous release checks
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_UPDATE_CHECKER_H
#define CUPID_UPDATE_CHECKER_H
#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stddef.h>
#ifndef CUPID_VERSION
#define CUPID_VERSION "0.0.0-dev"
#endif
enum { UPDATE_PANEL = 0x2640, UPDATE_CHECK_COMMAND = 0x2641, UPDATE_METADATA_LIMIT = 256 * 1024 };

typedef enum { UPDATE_IDLE, UPDATE_CHECKING, UPDATE_CURRENT, UPDATE_AVAILABLE, UPDATE_ERROR } UpdateStatus;

typedef struct {
    char version[96], url[512];
} UpdateRelease;

typedef bool (*UpdateFetch)(void *context, char *body, size_t capacity, unsigned *status, char *error,
                            size_t error_size);

typedef struct {
    char current[96], preferences[2048], acknowledged[96], message[256];
    bool automatic, automatic_request, notify, attempted;
    UpdateRelease release;
    UpdateStatus status;
    SDL_Thread *thread;
    SDL_atomic_t finished;
    UpdateFetch fetch;
    void *fetch_context;
    UpdateStatus worker_status;
    UpdateRelease worker_release;
    char worker_message[256];
} UpdateChecker;

bool update_version_compare(const char *a, const char *b, int *order);
bool update_metadata_parse(const char *text, size_t size, UpdateRelease *release);
bool update_fetch_native(void *context, char *body, size_t capacity, unsigned *status, char *error, size_t error_size);
bool update_checker_init(UpdateChecker *checker, const char *version, const char *preferences, char *error,
                         size_t error_size);
bool update_checker_start(UpdateChecker *checker, bool automatic, char *error, size_t error_size);
void update_checker_poll(UpdateChecker *checker);
bool update_checker_acknowledge(UpdateChecker *checker, char *error, size_t error_size);
bool update_checker_set_automatic(UpdateChecker *checker, bool automatic, char *error, size_t error_size);
bool update_checker_register_ui(UpdateChecker *checker);
void update_checker_shutdown(UpdateChecker *checker);
#endif
