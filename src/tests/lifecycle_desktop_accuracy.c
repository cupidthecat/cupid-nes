/*
 * lifecycle_desktop_accuracy.c - Standalone production idle-startup acceptance
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifdef CUPID_LIFECYCLE_DESKTOP_TEST_MAIN
#define main cupid_lifecycle_application_entry
#include "../main.c"
#undef main
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif
static Uint32 finish_idle(Uint32 interval, void *context) {
    (void)interval;
    (void)context;
    SDL_Event event = {0};
    event.type = SDL_QUIT;
    SDL_PushEvent(&event);
    return 0;
}

int main(int argc, char **argv) {
    SDL_setenv("SDL_VIDEODRIVER", "dummy", 1);
    SDL_setenv("SDL_AUDIODRIVER", "dummy", 1);
    if (SDL_Init(SDL_INIT_TIMER) != 0) {
        return 1;
    }
    char directory[256];
    snprintf(directory, sizeof(directory), "build/idle-session-%llu", (unsigned long long)SDL_GetPerformanceCounter());
#ifdef _WIN32
    if (_mkdir(directory) != 0) {
        return 1;
    }
#else
    if (mkdir(directory, 0700) != 0) {
        return 1;
    }
#endif
    if (!SDL_AddTimer(1000, finish_idle, NULL)) {
        return 1;
    }
    bool missing = argc > 1 && !strcmp(argv[1], "missing-image");
    bool corrupt = argc > 1 && !strcmp(argv[1], "corrupt-resume");
    if (corrupt) {
        char path[512];
        snprintf(path, sizeof(path), "%s/session-tools.ini", directory);
        const char preferences[] = "version=1\nresume=1\nautomatic=0\ntime=0\ninterval=3600\nretain=10\n";
        if (nes_file_write_atomic(path, preferences, sizeof(preferences) - 1) != NES_FILE_OK) {
            return 1;
        }
        snprintf(path, sizeof(path), "%s/last-session", directory);
        if (nes_file_write_atomic(path, "broken", 6) != NES_FILE_OK) {
            return 1;
        }
    }
    char *arguments[] = {"cupid", "--data-dir", directory, "missing-image.nes"};
    uint64_t before = cpu_total_cycles;
    int result = application_main(missing ? 4 : 3, arguments);
    if (result || cpu_total_cycles != before) {
        fprintf(stderr, "Idle startup failed or executed machine cycles: %d\n", result);
        return 1;
    }
    printf("Production desktop startup (%s), event loop and clean exit: PASS\n", missing   ? "missing image"
                                                                                 : corrupt ? "corrupt resume"
                                                                                           : "no game");
    return 0;
}
#endif
