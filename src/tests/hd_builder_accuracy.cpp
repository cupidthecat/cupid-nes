/*
 * hd_builder_accuracy.cpp - Runtime-compatible HD draft authoring regressions
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../hd/hd_builder.h"
#include "../hd/hd_pack_loader.hpp"
#include "../util/file_io.h"
#include <cstdio>
#include <cstring>
#include <vector>
#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        if (!(x)) {                                                                                                    \
            std::fprintf(stderr, "HD builder %d: %s (%s)\n", __LINE__, #x, error);                                     \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

extern "C" int run_hd_builder_accuracy_tests(void) {
    char error[512] = {0};
    NesHdBuilder *builder = nes_hd_builder_create();
    CHECK(builder);
    CHECK(nes_hd_builder_line_count(builder) == 2);
    CHECK(nes_hd_builder_open(builder, "src/tests/fixtures/hd_pack_v109", error, sizeof(error)));
    size_t count = nes_hd_builder_line_count(builder);
    CHECK(count > 20);
    CHECK(nes_hd_builder_set_line(builder, count, "<fallback>2,3", error, sizeof(error)));
    CHECK(nes_hd_builder_validate(builder, error, sizeof(error)));
    CHECK(nes_hd_builder_export(builder, "build/presentation-builder.zip", error, sizeof(error)));
    auto loaded = cupid::hd::load_pack_zip("build/presentation-builder.zip");
    CHECK(loaded);
    CHECK(loaded.candidate->fallbacks.size() == 2 && loaded.candidate->audio_tracks.size() == 2 &&
          loaded.candidate->backgrounds.size() == 1 && loaded.candidate->additions.size() == 1);
    CHECK(nes_hd_builder_set_line(builder, count, "<tile>0,0,01020304,99999,0,1,Y", error, sizeof(error)));
    CHECK(!nes_hd_builder_validate(builder, error, sizeof(error)));
    CHECK(!nes_hd_builder_export(builder, "build/presentation-builder.zip", error, sizeof(error)));
    CHECK(cupid::hd::load_pack_zip("build/presentation-builder.zip"));
    CHECK(nes_hd_builder_set_line(builder, count, NULL, error, sizeof(error)));
    CHECK(!nes_hd_builder_import_asset(builder, "../escape.png", "src/tests/fixtures/hd_pack_v109/tiles.png", error,
                                       sizeof(error)));
    CHECK(nes_hd_builder_import_asset(builder, "replacement.png", "src/tests/fixtures/hd_pack_v109/tiles.png", error,
                                      sizeof(error)));
    CHECK(nes_hd_builder_set_line(builder, 3, "<img>replacement.png", error, sizeof(error)));
    CHECK(!nes_hd_builder_import_asset(builder, "replacement.png", "src/tests/fixtures/hd_pack_v109/hires.txt", error,
                                       sizeof(error)));
    CHECK(nes_hd_builder_validate(builder, error, sizeof(error)));
    const char *protected_file = "build/presentation-builder.zip";
    CHECK(nes_hd_builder_set_protected_paths(builder, &protected_file, 1));
    CHECK(!nes_hd_builder_export(builder, protected_file, error, sizeof(error)));
    CHECK(nes_hd_builder_new(builder, 1, error, sizeof(error)));
    CHECK(!nes_hd_builder_open(builder, "build/no-such-pack.zip", error, sizeof(error)) &&
          nes_hd_builder_line_count(builder) == 2);
    std::vector<NesVideoPixel> pixels(256 * 240);
    for (auto &p : pixels) {
        p.original_rgb = p.backdrop_rgb = 0xff234567;
    }
    NesVideoTraceFrame trace = {};
    trace.pixels = pixels.data();
    trace.complete = true;
    NesHdFrameSource source = {};
    source.screens = 1;
    source.trace[0] = &trace;
    source.show_background = source.show_sprites = true;
    NesHdFrame frame = {};
    CHECK(nes_hd_builder_preview(builder, &source, nullptr, nullptr, &frame, error, sizeof(error)));
    CHECK(frame.width == 256 && frame.height == 240 && frame.pixels[0] == 0xff234567);
    NesVideoTraceFrame other = trace;
    other.frame_number = trace.frame_number + 1;
    source.screens = 2;
    source.trace[1] = &other;
    CHECK(!nes_hd_builder_preview(builder, &source, nullptr, nullptr, &frame, error, sizeof(error)));
    source.screens = 1;
    trace.complete = false;
    CHECK(!nes_hd_builder_preview(builder, &source, nullptr, nullptr, &frame, error, sizeof(error)));
    nes_hd_builder_destroy(builder);
    CHECK(nes_file_remove(protected_file) == NES_FILE_OK);
    std::puts("HD draft editing, roundtrip, asset safety, failed-export preservation and preview passed");
    return 0;
}
