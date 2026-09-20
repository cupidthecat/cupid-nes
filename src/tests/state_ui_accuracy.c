/*
 * state_ui_accuracy.c - Save-state audio and file ownership through the desktop
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../debugger/debugger.h"
#include "../state/state.h"
#include "../system/execution_policy.h"
#include "../system/vs_system.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"
#include "../ui/output_guard.h"
#include "../ui/state_runtime.h"
#include "../util/file_io.h"
#include <SDL2/SDL.h>
#include <stdatomic.h>
#ifdef _WIN32
#include <direct.h>
#include <process.h>
#include <windows.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

#define CHECK(condition) do { if (!(condition)) { fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); ++failures; goto cleanup; } } while (0)

static void silence_callback(void *context, Uint8 *stream, int length) {
    (void)context;
    memset(stream, 0, (size_t)length);
}

static bool stop_capture_probe(void *context, char *error, size_t error_size) {
    unsigned *calls = context;
    ++*calls;
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool advance_frame(void) {
    uint64_t target = ppu.frame_count + 1;
    while (ppu.frame_count < target) if (cpu_step(&cpu) <= 0) return false;
    return true;
}

static bool same_state(const NesStateBlob *left, const NesStateBlob *right) {
    return left->size == right->size && !memcmp(left->data, right->data, left->size);
}

static bool create_fixture(BoardImage *image, const char *path) {
    if (!board_image_create(image, 0, 0x8000, 0x2000, true)) return false;
    const uint8_t program[] = {
        0xA9,0x01,0x8D,0x15,0x40, 0xA9,0xBF,0x8D,0x00,0x40,
        0xA9,0x40,0x8D,0x02,0x40, 0xA9,0x08,0x8D,0x03,0x40,
        0x4C,0x14,0x80
    };
    memcpy(image->data + 16, program, sizeof(program));
    for (unsigned offset = 0x7FFA; offset <= 0x7FFE; offset += 2) {
        image->data[16 + offset] = 0;
        image->data[16 + offset + 1] = 0x80;
    }
    if (nes_file_write_atomic(path, image->data, image->size) != NES_FILE_OK || load_rom(path)) return false;
    cpu_select_machine(NULL);
    cpu_use_default_startup_alignment();
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    if (!cpu_power_on(&cpu)) return false;
    vs_audio_init(44100);
    /* Power-on starts on the pre-render scanline. Cross that short startup
     * boundary, then generate a complete frame of nonzero pulse output. */
    return advance_frame() && advance_frame();
}

static int state_controls_preserve_audio(const char *directory) {
    int failures = 0;
    char rom_path[256], state_path[256], slot_path[256], missing_path[256], protected_path[256];
    snprintf(rom_path, sizeof(rom_path), "%s/game.nes", directory);
    snprintf(state_path, sizeof(state_path), "%s/resume.cstate", directory);
    snprintf(missing_path, sizeof(missing_path), "%s/missing/resume.cstate", directory);
    snprintf(protected_path, sizeof(protected_path), "%s/firmware.bin", directory);
    slot_path[0] = '\0';
    BoardImage image = {0};
    NesStateBlob expected = {0}, actual = {0};
    FrontendSettings settings;
    FrontendExecutionRuntime execution = {0};
    StateRuntime state = {0};
    SDL_AudioDeviceID audio = 0;
    bool audio_initialized = false;
    unsigned capture_stops = 0;
    char error[256] = {0};
    uint32_t old_policy = nes_execution_policy();
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    CHECK(create_fixture(&image, rom_path));
    CHECK(SDL_AudioInit("dummy") == 0);
    audio_initialized = true;
    SDL_AudioSpec want = {0}, have = {0};
    want.freq = 44100;
    want.format = AUDIO_F32;
    want.channels = 1;
    want.samples = 256;
    want.callback = silence_callback;
    audio = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    CHECK(audio != 0);
    CHECK(SDL_GetAudioDeviceStatus(audio) == SDL_AUDIO_PAUSED);
    frontend_settings_defaults(&settings);
    strcpy(settings.state_file_path, state_path);
    frontend_execution_init(&execution, &audio, have.freq, rom_path, NULL, NULL, NULL);
    execution_control_set_paused(&execution.execution, true);
    execution.before_machine_change = stop_capture_probe;
    execution.machine_change_context = &capture_stops;
    CHECK(frontend_execution_register_commands(&execution));
    state_runtime_init(&state, &settings, directory, &execution);
    CHECK(state_runtime_register_ui(&state));
    CHECK(atomic_load_explicit(&apu.ring_w, memory_order_relaxed) > 0);
    CHECK(apu.audio_transition_count > 0);
    CHECK(nes_state_capture(&expected) == NES_STATE_OK);
    CHECK(frontend_command_invoke(STATE_COMMAND_SAVE_FILE, error, sizeof(error)));
    CHECK(capture_stops == 0 && state.save_audio_device == 0);
    CHECK(nes_state_capture(&actual) == NES_STATE_OK && same_state(&expected, &actual));
    nes_state_blob_free(&actual);

    write_mem(0x0010, (uint8_t)(read_mem(0x0010) ^ 0xA5));
    CHECK(advance_frame());
    CHECK(frontend_command_invoke(STATE_COMMAND_LOAD_FILE, error, sizeof(error)));
    CHECK(capture_stops == 1 && !state.machine_locked);
    CHECK(frontend_execution_paused(&execution) && SDL_GetAudioDeviceStatus(audio) == SDL_AUDIO_PAUSED);
    CHECK(nes_state_capture(&actual) == NES_STATE_OK && same_state(&expected, &actual));
    nes_state_blob_free(&actual);

    /* Failed loading must leave the producer queue and filter history intact. */
    strcpy(settings.state_file_path, missing_path);
    CHECK(!frontend_command_invoke(STATE_COMMAND_LOAD_FILE, error, sizeof(error)));
    CHECK(!state.machine_locked && frontend_execution_paused(&execution));
    CHECK(nes_state_capture(&actual) == NES_STATE_OK && same_state(&expected, &actual));
    nes_state_blob_free(&actual);
    CHECK(!frontend_command_invoke(STATE_COMMAND_SAVE_FILE, error, sizeof(error)));
    CHECK(state.save_audio_device == 0);
    CHECK(nes_state_capture(&actual) == NES_STATE_OK && same_state(&expected, &actual));
    nes_state_blob_free(&actual);

    CHECK(nes_state_slot_path(directory, settings.state_slot, slot_path, sizeof(slot_path)) == NES_STATE_OK);
    char too_small[4];
    CHECK(nes_state_slot_path(directory, 0, too_small, sizeof(too_small)) == NES_STATE_ERROR_ARGUMENT);
    CHECK(!too_small[0]);
    CHECK(nes_state_slot_path(directory, NES_STATE_SLOT_COUNT, too_small, sizeof(too_small)) == NES_STATE_ERROR_ARGUMENT);
    CHECK(frontend_command_invoke(STATE_COMMAND_SAVE_SLOT, error, sizeof(error)));
    CHECK(nes_state_capture(&actual) == NES_STATE_OK && same_state(&expected, &actual));
    nes_state_blob_free(&actual);
    CHECK(advance_frame());
    CHECK(frontend_command_invoke(STATE_COMMAND_LOAD_SLOT, error, sizeof(error)));
    CHECK(nes_state_capture(&actual) == NES_STATE_OK && same_state(&expected, &actual));
    nes_state_blob_free(&actual);

    strcpy(settings.state_file_path, rom_path);
    CHECK(!frontend_command_invoke(STATE_COMMAND_SAVE_FILE, error, sizeof(error)));
    uint8_t *bytes = NULL;
    size_t size = 0;
    CHECK(nes_file_read_all(rom_path, image.size, &bytes, &size) == NES_FILE_OK);
    bool preserved = size == image.size && !memcmp(bytes, image.data, size);
    free(bytes);
    CHECK(preserved);
    const uint8_t firmware[] = {1, 2, 3, 4};
    CHECK(nes_file_write_atomic(protected_path, firmware, sizeof(firmware)) == NES_FILE_OK);
    const char *protected_paths[] = {protected_path};
    state_runtime_set_protected_paths(&state, protected_paths, 1);
    strcpy(settings.state_file_path, protected_path);
    CHECK(!frontend_command_invoke(STATE_COMMAND_SAVE_FILE, error, sizeof(error)));
    CHECK(state.save_audio_device == 0);
    strcpy(settings.state_file_path, state_path);
    CHECK(nes_execution_set_policy(NES_EXECUTION_MOVIE_PLAYBACK));
    unsigned before = capture_stops;
    CHECK(!frontend_command_invoke(STATE_COMMAND_LOAD_FILE, error, sizeof(error)));
    CHECK(capture_stops == before && !state.machine_locked);

cleanup:
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    state_runtime_shutdown(&state);
    frontend_execution_shutdown(&execution);
    frontend_commands_reset();
    frontend_panels_reset();
    if (audio) SDL_CloseAudioDevice(audio);
    if (audio_initialized) SDL_AudioQuit();
    nes_state_blob_free(&expected);
    nes_state_blob_free(&actual);
    (void)unload_rom();
    board_image_free(&image);
    if (slot_path[0]) (void)nes_file_remove(slot_path);
    (void)nes_file_remove(state_path);
    (void)nes_file_remove(rom_path);
    (void)nes_file_remove(protected_path);
    (void)nes_execution_set_policy(old_policy);
    return failures;
}

static int output_destinations(const char *directory) {
    int failures = 0;
    char rom[256], identity[256], saved[256], alias[256], safe[256];
    snprintf(rom, sizeof(rom), "%s/archive.zip", directory);
    snprintf(identity, sizeof(identity), "%s/member-patched.nes", directory);
    snprintf(alias, sizeof(alias), "%s/alias.cstate", directory);
    snprintf(safe, sizeof(safe), "%s/output.cstate", directory);
    saved[0] = '\0';
    const uint8_t original[] = {0x11, 0x22, 0x33};
    CHECK(nes_file_write_atomic(rom, original, sizeof(original)) == NES_FILE_OK);
    FrontendExecutionRuntime execution = {.rom_path = rom, .save_identity = identity};
    char error[160];
    CHECK(frontend_output_path_allowed(safe, &execution, NULL, 0, error, sizeof(error)));
    CHECK(!frontend_output_path_allowed(rom, &execution, NULL, 0, error, sizeof(error)));
    static const char *const suffixes[] = {
        ".sav", ".chr.sav", ".flash.sav", ".chr.flash.sav", ".eeprom128",
        ".eeprom256", ".turbofile.sav", ".battlebox.sav"
    };
    for (size_t i = 0; i < sizeof(suffixes) / sizeof(suffixes[0]); ++i) {
        snprintf(saved, sizeof(saved), "%s/member-patched%s", directory, suffixes[i]);
        CHECK(!frontend_output_path_allowed(saved, &execution, NULL, 0, error, sizeof(error)));
        CHECK(nes_file_write_atomic(saved, original, sizeof(original)) == NES_FILE_OK);
        CHECK(!frontend_output_path_allowed(saved, &execution, NULL, 0, error, sizeof(error)));
#ifdef _WIN32
        CHECK(CreateHardLinkA(alias, saved, NULL) != 0);
#else
        CHECK(link(saved, alias) == 0);
#endif
        CHECK(!frontend_output_path_allowed(alias, &execution, NULL, 0, error, sizeof(error)));
        CHECK(nes_file_remove(alias) == NES_FILE_OK);
        CHECK(nes_file_remove(saved) == NES_FILE_OK);
        saved[0] = '\0';
    }

cleanup:
    if (saved[0]) (void)nes_file_remove(saved);
    (void)nes_file_remove(alias);
    (void)nes_file_remove(rom);
    return failures;
}

int test_state_ui_accuracy(void) {
    char directory[128];
#ifdef _WIN32
    snprintf(directory, sizeof(directory), "build/state-ui-%u", (unsigned)_getpid());
    if (_mkdir(directory) != 0) return 1;
#else
    snprintf(directory, sizeof(directory), "build/state-ui-%u", (unsigned)getpid());
    if (mkdir(directory, 0700) != 0) return 1;
#endif
    int failures = state_controls_preserve_audio(directory) + output_destinations(directory);
#ifdef _WIN32
    if (_rmdir(directory)) ++failures;
#else
    if (rmdir(directory)) ++failures;
#endif
    printf("Desktop state ownership: 2 groups, %d failures\n", failures);
    return failures;
}
