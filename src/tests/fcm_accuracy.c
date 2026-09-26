/*
 * fcm_accuracy.c - Legacy input conversion and transactional output checks
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "board_tests.h"
#include "../replay/fcm.h"
#include "../replay/tas_startup.h"
#include "../ui/fcm_frontend.h"
#include "../ui/frontend_panels.h"
#include "../ui/platform_frontend.h"
#include "../system/timing.h"
#include "../util/file_io.h"
#include "../../include/globals.h"

static void put_word(uint8_t *bytes, uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) {
        bytes[i] = (uint8_t)(value >> (i * 8));
    }
}

static size_t fixture(uint8_t data[128]) {
    memset(data, 0, 128);
    memcpy(data, "FCM\x1a", 4);
    put_word(data + 4, 2);
    data[8] = 8 | 16;
    put_word(data + 12, 6);
    put_word(data + 16, 17);
    put_word(data + 24, 80);
    put_word(data + 28, 80);
    memcpy(data + 52, "fixture\0author\0", 15);
    const uint8_t events[] = {0, 0x21, 2, 0xa1, 1, 0xa2, 1, 0x10, 0xa0, 2};
    put_word(data + 20, sizeof(events));
    memcpy(data + 80, events, sizeof(events));
    return 80 + sizeof(events);
}

static int codec(void) {
    uint8_t data[128];
    size_t size = fixture(data);
    NesFm2Movie movie;
    nes_fm2_movie_init(&movie);
    NesFm2Diagnostic diagnostic;
    BOARD_CHECK(nes_fcm_convert(data, size, NULL, &movie, &diagnostic) == NES_FM2_OK);
    BOARD_CHECK(movie.frame_count == 6 && movie.rerecord_count == 17 && movie.fourscore);
    BOARD_CHECK(movie.frames[0].pads[0] == 1 && movie.frames[1].pads[0] == 1);
    BOARD_CHECK(movie.frames[2].pads[0] == 3 && movie.frames[3].commands == NES_FM2_COMMAND_RESET);
    BOARD_CHECK(movie.frames[4].commands == NES_FM2_COMMAND_POWER && movie.frames[5].pads[2] == 1);
    BOARD_CHECK(!strcmp(movie.rom_filename, "fixture") && !strcmp(movie.comments.items[0], "author"));
    NesFm2Frame *retained = movie.frames;
    for (size_t prefix = 0; prefix < size; ++prefix) {
        BOARD_CHECK(nes_fcm_convert(data, prefix, NULL, &movie, &diagnostic) != NES_FM2_OK);
        BOARD_CHECK(movie.frames == retained && movie.frame_count == 6);
    }
    data[8] |= 4;
    BOARD_CHECK(nes_fcm_convert(data, size, NULL, &movie, &diagnostic) == NES_FM2_OK && movie.pal);
    data[8] = 2 | 16;
    BOARD_CHECK(nes_fcm_convert(data, size, NULL, &movie, &diagnostic) == NES_FM2_UNSUPPORTED);
    data[8] = 8;
    BOARD_CHECK(nes_fcm_convert(data, size, NULL, &movie, &diagnostic) == NES_FM2_UNSUPPORTED);
    fixture(data);
    data[80] = 0x83;
    BOARD_CHECK(nes_fcm_convert(data, size, NULL, &movie, &diagnostic) == NES_FM2_UNSUPPORTED);
    fixture(data);
    data[82] = 99;
    BOARD_CHECK(nes_fcm_convert(data, size, NULL, &movie, &diagnostic) == NES_FM2_CORRUPT);
    /* A frame stores one reset/power operation, not an ordered event list. */
    for (unsigned first = 1; first <= 2; ++first) {
        for (unsigned second = 1; second <= 2; ++second) {
            fixture(data);
            put_word(data + 20, 3);
            data[80] = (uint8_t)(0x80 | first);
            data[81] = 0x80; /* A no-op does not erase the preceding command. */
            data[82] = (uint8_t)(0x80 | second);
            retained = movie.frames;
            BOARD_CHECK(nes_fcm_convert(data, 83, NULL, &movie, &diagnostic) == NES_FM2_UNSUPPORTED);
            BOARD_CHECK(movie.frames == retained && movie.frame_count == 6);
            BOARD_CHECK(strstr(diagnostic.message, "Multiple system commands"));
        }
    }
    fixture(data);
    put_word(data + 20, 2);
    data[80] = 0x20;
    data[81] = 0;
    BOARD_CHECK(nes_fcm_convert(data, 82, NULL, &movie, &diagnostic) == NES_FM2_TRUNCATED);
    fixture(data);
    NesFm2Limits limits = nes_fm2_default_limits();
    limits.max_frames = 5;
    BOARD_CHECK(nes_fcm_convert(data, size, &limits, &movie, &diagnostic) == NES_FM2_LIMIT);
    nes_fm2_movie_free(&movie);
    return 0;
}

static int timestamp_boundaries(void) {
    uint8_t data[128];
    NesFm2Movie movie;
    NesFm2Diagnostic diagnostic;
    nes_fm2_movie_init(&movie);
    const uint8_t streams[][7] = {
        {0x20, 0, 1, 0xa0, 2, 0, 0},
        {0x40, 0, 0, 1, 0xa0, 2, 0},
        {0x60, 0, 0, 1, 0xa0, 2, 0},
    };
    const size_t sizes[] = {5, 6, 6};
    const size_t boundaries[] = {256, 65536, 65536};
    for (size_t i = 0; i < 3; ++i) {
        fixture(data);
        put_word(data + 12, (uint32_t)(boundaries[i] + 2));
        put_word(data + 20, (uint32_t)sizes[i]);
        memcpy(data + 80, streams[i], sizes[i]);
        BOARD_CHECK(nes_fcm_convert(data, 80 + sizes[i], NULL, &movie, &diagnostic) == NES_FM2_OK);
        BOARD_CHECK(movie.frames[boundaries[i] - 1].pads[0] == 0);
        BOARD_CHECK(movie.frames[boundaries[i]].pads[0] == 1);
        BOARD_CHECK(movie.frames[boundaries[i] + 1].pads[0] == 1);
        /* A terminal no-op is valid; input or reset at that boundary is not. */
        NesFm2Frame *retained = movie.frames;
        data[80 + sizes[i] - 2] = 0x20;
        BOARD_CHECK(nes_fcm_convert(data, 80 + sizes[i], NULL, &movie, &diagnostic) == NES_FM2_CORRUPT);
        BOARD_CHECK(movie.frames == retained);
        data[80 + sizes[i] - 2] = 0xa1;
        BOARD_CHECK(nes_fcm_convert(data, 80 + sizes[i], NULL, &movie, &diagnostic) == NES_FM2_CORRUPT);
        BOARD_CHECK(movie.frames == retained);
    }
    nes_fm2_movie_free(&movie);
    return 0;
}

static int frontend(void) {
    BoardImage image = {0};
    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0x2000, true));
    apu_power_on(&apu);
    BOARD_CHECK(board_image_load(&image) == 0);
    nes_set_region(NES_REGION_NTSC);
    const char *source = "build/fcm-fixture.fcm", *output = "build/fcm-converted.fm2";
    uint8_t data[128];
    size_t size = fixture(data);
    BOARD_CHECK(nes_tas_rom_md5(data + 32));
    BOARD_CHECK(nes_file_write_atomic(source, data, size) == NES_FILE_OK);
    FrontendExecutionRuntime execution = {0};
    char error[256];
    BOARD_CHECK(fcm_frontend_convert(&execution, source, output, error, sizeof(error)));
    uint8_t *bytes = NULL;
    size_t count = 0;
    BOARD_CHECK(nes_file_read_all(output, 8192, &bytes, &count) == NES_FILE_OK);
    NesFm2Movie movie;
    NesFm2Diagnostic diagnostic;
    nes_fm2_movie_init(&movie);
    BOARD_CHECK(nes_fm2_parse(bytes, count, NULL, &movie, &diagnostic) == NES_FM2_OK);
    free(bytes);
    BOARD_CHECK(movie.frame_count == 6 && movie.frames[3].commands == NES_FM2_COMMAND_RESET);
    BOARD_CHECK(!fcm_frontend_convert(&execution, source, source, error, sizeof(error)));
    BOARD_CHECK(nes_file_write_atomic(output, (const uint8_t *)"keep", 4) == NES_FILE_OK);
    data[32] ^= 1;
    BOARD_CHECK(nes_file_write_atomic(source, data, size) == NES_FILE_OK);
    BOARD_CHECK(!fcm_frontend_convert(&execution, source, output, error, sizeof(error)) && strstr(error, "checksum"));
    data[32] ^= 1;
    data[8] |= 4;
    BOARD_CHECK(nes_file_write_atomic(source, data, size) == NES_FILE_OK);
    BOARD_CHECK(!fcm_frontend_convert(&execution, source, output, error, sizeof(error)) && strstr(error, "timing"));
    BOARD_CHECK(nes_file_read_all(output, 8192, &bytes, &count) == NES_FILE_OK && count == 4 &&
                !memcmp(bytes, "keep", 4));
    free(bytes);
    frontend_panels_reset();
    BOARD_CHECK(fcm_frontend_register(&execution));
    FrontendPanelInfo info;
    BOARD_CHECK(frontend_panel_get(FCM_CONVERTER_PANEL, &info) && !info.enabled);
    frontend_panel_set_session_active(true);
    BOARD_CHECK(frontend_panel_get(FCM_CONVERTER_PANEL, &info) && info.enabled);
    BOARD_CHECK(frontend_panel_action(FCM_CONVERTER_PANEL, FCM_CONVERTER_SOURCE, source, 0, error, sizeof(error)));
    BOARD_CHECK(frontend_panel_action(FCM_CONVERTER_PANEL, FCM_CONVERTER_OUTPUT, output, 0, error, sizeof(error)));
    fixture(data);
    BOARD_CHECK(nes_tas_rom_md5(data + 32));
    put_word(data + 20, 2);
    data[80] = 0x81;
    data[81] = 0x82;
    BOARD_CHECK(nes_file_write_atomic(source, data, 82) == NES_FILE_OK);
    BOARD_CHECK(!frontend_panel_action(FCM_CONVERTER_PANEL, FCM_CONVERTER_CONVERT, "", 0, error, sizeof(error)));
    BOARD_CHECK(strstr(error, "Multiple system commands"));
    BOARD_CHECK(nes_file_read_all(output, 8192, &bytes, &count) == NES_FILE_OK && count == 4 &&
                !memcmp(bytes, "keep", 4));
    free(bytes);
    size = fixture(data);
    BOARD_CHECK(nes_tas_rom_md5(data + 32));
    BOARD_CHECK(nes_file_write_atomic(source, data, size) == NES_FILE_OK);
    BOARD_CHECK(frontend_panel_action(FCM_CONVERTER_PANEL, FCM_CONVERTER_CONVERT, "", 0, error, sizeof(error)));
    FrontendPanelControl controls[3];
    FrontendPanelModel model = {.controls = controls, .capacity = 3};
    BOARD_CHECK(frontend_panel_snapshot(FCM_CONVERTER_PANEL, &model, error, sizeof(error)));
    BOARD_CHECK(model.count == 3 && strstr(model.status, "FM2 saved"));
    BOARD_CHECK(controls[0].id == FCM_CONVERTER_SOURCE && controls[0].type == FRONTEND_PANEL_FILE_OPEN &&
                controls[0].selected == FRONTEND_OPEN_LEGACY_MOVIE);
    BOARD_CHECK(nes_file_read_all(output, 8192, &bytes, &count) == NES_FILE_OK);
    BOARD_CHECK(nes_fm2_parse(bytes, count, NULL, &movie, &diagnostic) == NES_FM2_OK);
    free(bytes);
    BOARD_CHECK(movie.frame_count == 6 && movie.frames[4].commands == NES_FM2_COMMAND_POWER);
    fcm_frontend_unregister();
    BOARD_CHECK(!frontend_panel_get(FCM_CONVERTER_PANEL, &info));
    nes_fm2_movie_free(&movie);
    BOARD_CHECK(unload_rom());
    board_image_free(&image);
    (void)nes_file_remove(source);
    (void)nes_file_remove(output);
    return 0;
}

int test_fcm_accuracy(void) {
    int failures = codec() + timestamp_boundaries() + frontend();
    printf("Legacy movie conversion: codec, rejected input, game identity and output checks, %d failures\n", failures);
    return failures;
}
