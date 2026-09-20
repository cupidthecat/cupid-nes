/*
 * hd_runtime_accuracy.cpp - Production HD runtime lifecycle and panel checks
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../hd/hd_pack_loader.hpp"
#include "../hd/hd_runtime.h"
#include "../audio/audio_observer.h"
#include "../cpu/cpu_observer.h"
#include "../system/execution_policy.h"
#include "../state/state.h"
extern "C" {
#include "../ui/frontend_panels.h"
#include "../ui/hd_pack_frontend.h"
#include "../ui/app_paths.h"
#include "board_tests.h"
}

#include <array>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {
int failures;
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "HD runtime check failed at %s:%d: %s\n", __FILE__, __LINE__, #x); ++failures; } } while (0)

std::string utf8(const fs::path &path) {
    auto text = path.u8string();
    return std::string(text.begin(), text.end());
}

fs::path fresh_root() {
    static unsigned serial;
    fs::path root = fs::path("build") / ("hd-runtime-acceptance-" + std::to_string(++serial));
    std::error_code ec;
    fs::remove_all(root, ec);
    fs::create_directories(root, ec);
    CHECK(!ec);
    return root;
}

void write_text(const fs::path &path, const char *text) {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    stream << text;
    CHECK(stream.good());
}

bool changed_from(float left, float right, float original) {
    return left != original || right != original;
}

std::uint8_t fixture_memory_reader(void *, unsigned, bool ppu_space, std::uint16_t address) {
    if (address == 2) return ppu_space ? 0x00u : 0x80u;
    if (address == 1) return 0x01u;
    return ppu_space ? 0x00u : 0x01u;
}

void fill_capture_trace(std::vector<NesVideoPixel> &pixels,
                        const std::array<std::uint8_t, 32> &chr,
                        bool chr_ram) {
    pixels.assign(NES_VIDEO_TRACE_WIDTH * NES_VIDEO_TRACE_HEIGHT, NesVideoPixel{});
    for (unsigned y = 0; y < NES_VIDEO_TRACE_HEIGHT; ++y) {
        for (unsigned x = 0; x < NES_VIDEO_TRACE_WIDTH; ++x) {
            auto &pixel = pixels[y * NES_VIDEO_TRACE_WIDTH + x];
            auto &tile = pixel.background;
            tile.chr.index = 0;
            std::memcpy(tile.chr.bytes, chr.data(), 16);
            tile.chr.ram = chr_ram;
            tile.chr.valid = true;
            tile.x = static_cast<std::uint8_t>(x & 7u);
            tile.y = static_cast<std::uint8_t>(y & 7u);
            tile.palette = 0x0F212730u;
            tile.color_index = (x & 7u) == 0 ? 1 : 0;
            tile.color = tile.color_index ? 0x21 : 0x0F;
            tile.rgb = tile.color_index ? 0xFF112233u : 0xFF000000u;
            pixel.backdrop = 0x0F;
            pixel.backdrop_rgb = 0xFF000000u;
            pixel.original_rgb = tile.color_index ? tile.rgb : pixel.backdrop_rgb;
        }
    }
    for (unsigned x = 0; x < 8; ++x) {
        auto &pixel = pixels[x];
        auto &sprite = pixel.sprites[0];
        sprite.chr.index = chr_ram ? 0 : 1;
        std::memcpy(sprite.chr.bytes, chr.data() + 16, 16);
        sprite.chr.ram = chr_ram;
        sprite.chr.valid = true;
        sprite.x = static_cast<std::uint8_t>(x);
        sprite.y = 0;
        sprite.palette = 0xFF162636u;
        sprite.color_index = x == 1 ? 2 : 0;
        sprite.color = sprite.color_index ? 0x26 : 0x0F;
        sprite.rgb = sprite.color_index ? 0xFF445566u : 0xFF000000u;
        pixel.sprite_count = 1;
        if (sprite.color_index) pixel.original_rgb = sprite.rgb;
    }
}

int fresh_capture_round_trip_and_failures() {
    BoardImage image{};
    CHECK(board_image_create(&image, 0, 0x8000, 0x2000, true));
    CHECK(load_rom_memory(image.data, image.size) == 0);
    board_image_free(&image);

    fs::path root = fresh_root();
    std::string root_text = utf8(root);
    char error[512] = {0};
    NesHdRuntime *runtime = nes_hd_runtime_create(root_text.c_str(), error, sizeof(error));
    CHECK(runtime != nullptr);
    if (!runtime) return failures;

    std::array<std::uint8_t, 32> chr{};
    for (unsigned y = 0; y < 8; ++y) chr[y] = 0x80;
    chr[16 + 8] = 0x40;
    NesHdGameInfo game = {
        "Capture Game.nes",
        "89ABCDEF0123456789ABCDEF0123456789ABCDEF",
        chr.data(), chr.size(), true
    };
    CHECK(nes_hd_runtime_set_game(runtime, &game, error, sizeof(error)));

    std::vector<NesVideoPixel> pixels;
    fill_capture_trace(pixels, chr, false);
    NesVideoTraceFrame trace{pixels.data(), 77, 1, true};
    std::vector<std::uint32_t> fallback(pixels.size());
    for (size_t i = 0; i < pixels.size(); ++i) fallback[i] = pixels[i].original_rgb;
    NesHdFrameSource source{};
    source.fallback_pixels = fallback.data();
    source.fallback_width = NES_VIDEO_TRACE_WIDTH;
    source.fallback_height = NES_VIDEO_TRACE_HEIGHT;
    source.screens = 1;
    source.trace[0] = &trace;
    source.show_background = true;
    source.show_sprites = true;

    fs::path firmware = root / "firmware.bin";
    write_text(firmware, "protected");
    std::string firmware_text = utf8(firmware);
    const char *protected_paths[] = {firmware_text.c_str()};
    CHECK(nes_hd_runtime_set_protected_paths(runtime, protected_paths, 1, error, sizeof(error)));
    CHECK(!nes_hd_runtime_capture(runtime, &source, firmware_text.c_str(), error, sizeof(error)));
    CHECK(std::strstr(error, "firmware") != nullptr || std::strstr(error, "output") != nullptr);
    CHECK(!nes_hd_runtime_capture(runtime, &source, game.rom_path, error, sizeof(error)));

    NesVideoTraceFrame malformed{nullptr, 77, 1, true};
    NesHdFrameSource malformed_source = source;
    malformed_source.trace[0] = &malformed;
    CHECK(!nes_hd_runtime_capture(runtime, &malformed_source,
                                  utf8(root / "malformed.zip").c_str(), error, sizeof(error)));
    CHECK(std::strstr(error, "complete") != nullptr);

    NesVideoTraceFrame other = trace;
    other.frame_number = 78;
    NesHdFrameSource mismatched = source;
    mismatched.screens = 2;
    mismatched.trace[1] = &other;
    CHECK(!nes_hd_runtime_capture(runtime, &mismatched,
                                  utf8(root / "mismatched.zip").c_str(), error, sizeof(error)));
    CHECK(std::strstr(error, "paired") != nullptr);

    std::vector<NesVideoPixel> oversize_pixels(pixels.size());
    for (size_t i = 0; i < 32769; ++i) {
        auto &tile = oversize_pixels[i].background;
        tile.chr.ram = true;
        tile.chr.valid = true;
        tile.palette = static_cast<std::uint32_t>(i);
    }
    NesVideoTraceFrame oversize_trace{oversize_pixels.data(), 77, 1, true};
    NesHdFrameSource oversize_source = source;
    oversize_source.trace[0] = &oversize_trace;
    CHECK(!nes_hd_runtime_capture(runtime, &oversize_source,
                                  utf8(root / "oversize.zip").c_str(), error, sizeof(error)));
    CHECK(std::strstr(error, "too many") != nullptr);

    NesStateBlob hardware_before{};
    CHECK(nes_state_capture(&hardware_before) == NES_STATE_OK);
    fs::path captured = root / "captured.zip";
    std::string captured_text = utf8(captured);
    CHECK(nes_hd_runtime_capture(runtime, &source, captured_text.c_str(), error, sizeof(error)));
    NesStateBlob hardware_after{};
    CHECK(nes_state_capture(&hardware_after) == NES_STATE_OK);
    CHECK(hardware_before.size == hardware_after.size
          && (!hardware_before.size
              || std::memcmp(hardware_before.data, hardware_after.data, hardware_before.size) == 0));
    nes_state_blob_free(&hardware_before);
    nes_state_blob_free(&hardware_after);

    auto captured_pack = cupid::hd::load_pack_zip(captured_text);
    CHECK(captured_pack && captured_pack.candidate->tiles.size() == 2);
    if (captured_pack && !captured_pack.candidate->tile_images.empty()) {
        const auto &atlas = captured_pack.candidate->tile_images[0];
        CHECK(atlas.width >= 16 && atlas.height >= 8);
        CHECK(atlas.argb32[0] == 0xFF112233u && atlas.argb32[1] == 0u);
        CHECK(atlas.argb32[8] == 0u && atlas.argb32[9] == 0xFF445566u);
    }

    CHECK(nes_hd_runtime_load(runtime, captured_text.c_str(), error, sizeof(error)));
    CHECK(!nes_hd_runtime_export(runtime, game.rom_path, error, sizeof(error)));
    CHECK(std::strstr(error, "active image") != nullptr || std::strstr(error, "output") != nullptr);
    CHECK(nes_hd_runtime_enable(runtime, true, error, sizeof(error)));
    NesHdFrame rendered{};
    CHECK(nes_hd_runtime_render(runtime, &source, &rendered, error, sizeof(error)));
    CHECK(rendered.hd_rendered && rendered.width == NES_VIDEO_TRACE_WIDTH
          && rendered.height == NES_VIDEO_TRACE_HEIGHT);
    CHECK(rendered.pixels[0] == 0xFF112233u);
    CHECK(rendered.pixels[1] == 0xFF445566u);
    CHECK(rendered.pixels[2] == 0xFF000000u);
    CHECK(nes_hd_runtime_enable(runtime, false, error, sizeof(error)));

    std::vector<NesVideoPixel> second_pixels;
    fill_capture_trace(second_pixels, chr, true);
    second_pixels[0].background.chr.bytes[0] = 0x80;
    second_pixels[0].background.palette = 0x0F313233u;
    second_pixels[0].background.color_index = 1;
    second_pixels[0].background.color = 0x31;
    second_pixels[0].background.rgb = 0xFF778899u;
    NesVideoTraceFrame second_trace{second_pixels.data(), 77, 2, true};
    NesHdFrameSource dual = source;
    dual.screens = 2;
    dual.trace[1] = &second_trace;
    fs::path dual_capture = root / "dual.zip";
    CHECK(nes_hd_runtime_capture(runtime, &dual, utf8(dual_capture).c_str(), error, sizeof(error)));
    auto dual_pack = cupid::hd::load_pack_zip(utf8(dual_capture));
    CHECK(dual_pack && dual_pack.candidate->tiles.size() >= 3);

    NesHdRuntimeInfo before_failed_write{};
    CHECK(nes_hd_runtime_info(runtime, &before_failed_write));
    fs::path failed = root / "missing-parent" / "failed.zip";
    CHECK(!nes_hd_runtime_capture(runtime, &source, utf8(failed).c_str(), error, sizeof(error)));
    CHECK(std::strstr(error, "write") != nullptr || std::strstr(error, "I/O") != nullptr);
    NesHdRuntimeInfo after_failed_write{};
    CHECK(nes_hd_runtime_info(runtime, &after_failed_write));
    CHECK(before_failed_write.pack_loaded == after_failed_write.pack_loaded
          && before_failed_write.enabled == after_failed_write.enabled
          && std::string(before_failed_write.active_path ? before_failed_write.active_path : "")
             == std::string(after_failed_write.active_path ? after_failed_write.active_path : ""));

    nes_hd_runtime_destroy(runtime);
    CHECK(unload_rom());
    std::error_code ec;
    fs::remove_all(root, ec);
    CHECK(!ec);
    return 0;
}

int lifecycle_render_audio_and_transaction() {
    BoardImage image{};
    CHECK(board_image_create(&image, 0, 0x8000, 0x2000, true));
    CHECK(load_rom_memory(image.data, image.size) == 0);
    board_image_free(&image);
    NesStateBlob hardware_before{};
    CHECK(nes_state_capture(&hardware_before) == NES_STATE_OK);

    fs::path root = fresh_root();
    std::string root_text = utf8(root);
    char error[512] = {0};
    NesHdRuntime *runtime = nes_hd_runtime_create(root_text.c_str(), error, sizeof(error));
    CHECK(runtime != nullptr);
    if (!runtime) return failures;

    std::array<std::uint8_t, 32> chr{};
    NesHdGameInfo game = {
        "Example Game.nes",
        "0123456789ABCDEF0123456789ABCDEF01234567",
        chr.data(), chr.size(), true
    };
    CHECK(nes_hd_runtime_set_game(runtime, &game, error, sizeof(error)));
    nes_hd_runtime_set_memory_reader(runtime, fixture_memory_reader, nullptr);

    const char *fixture = "src/tests/fixtures/hd_pack_v109";
    CHECK(nes_hd_runtime_load(runtime, fixture, error, sizeof(error)));
    NesHdRuntimeInfo before{};
    CHECK(nes_hd_runtime_info(runtime, &before));
    CHECK(before.pack_loaded && !before.enabled && before.active_path);
    std::string active_before = before.active_path ? before.active_path : "";
    CHECK(before.alternate_audio_registers);

    fs::path incompatible = root / "hardware-change";
    std::error_code ec;
    fs::create_directories(incompatible, ec);
    CHECK(!ec);
    write_text(incompatible / "hires.txt", "<ver>109\n<options>disableSpriteLimit\n");
    std::string incompatible_text = utf8(incompatible);
    CHECK(!nes_hd_runtime_load(runtime, incompatible_text.c_str(), error, sizeof(error)));
    CHECK(std::strstr(error, "hardware") != nullptr);
    NesHdRuntimeInfo after_failure{};
    CHECK(nes_hd_runtime_info(runtime, &after_failure));
    CHECK(after_failure.pack_loaded && after_failure.active_path
          && active_before == after_failure.active_path);

    CHECK(nes_hd_runtime_enable(runtime, true, error, sizeof(error)));
    NesHdRuntimeInfo enabled{};
    CHECK(nes_hd_runtime_info(runtime, &enabled) && enabled.enabled);

    std::uint32_t fallback_pixels[4] = {0xff010203u, 0xff040506u, 0xff070809u, 0xff0a0b0cu};
    NesHdFrameSource source{};
    source.fallback_pixels = fallback_pixels;
    source.fallback_width = 2;
    source.fallback_height = 2;
    source.screens = 1;
    source.show_background = true;
    source.show_sprites = true;
    source.allow_pack_overscan = true;
    NesHdFrame frame{};
    CHECK(nes_hd_runtime_render(runtime, &source, &frame, error, sizeof(error)));
    CHECK(!frame.hd_rendered && !frame.trace_ready && frame.pixels == fallback_pixels);
    CHECK(frame.width == 2 && frame.height == 2);

    std::vector<NesVideoPixel> pixels(NES_VIDEO_TRACE_WIDTH * NES_VIDEO_TRACE_HEIGHT);
    for (auto &pixel : pixels) {
        pixel.original_rgb = 0xff334455u;
        pixel.backdrop_rgb = 0xff334455u;
        pixel.backdrop = 0;
        pixel.mask = 0;
    }
    NesVideoTraceFrame trace{pixels.data(), 123, 1, true};
    source.trace[0] = &trace;
    CHECK(nes_hd_runtime_render(runtime, &source, &frame, error, sizeof(error)));
    CHECK(frame.hd_rendered && frame.trace_ready);
    CHECK(frame.width == 244 && frame.height == 226 && frame.scale == 1);
    CHECK(frame.overscan.left == 5 && frame.overscan.right == 7
          && frame.overscan.top == 8 && frame.overscan.bottom == 6);
    CHECK(frame.pixels && frame.pixels[0] != 0);

    /* The committed fixture uses alternate enhanced-audio registers. Writes are
     * observed after real bus writes, while sample mixing is host-only. */
    nes_cpu_write_observe(0x3042, 1);  // album 1
    nes_cpu_write_observe(0x3022, 255); // BGM volume
    nes_cpu_write_observe(0x3052, 2);  // BGM track 2
    NesHdRuntimeInfo audio_info{};
    CHECK(nes_hd_runtime_info(runtime, &audio_info) && audio_info.audio_active);
    bool mixed = false;
    for (unsigned i = 0; i < 2048 && !mixed; ++i) {
        float left = 0.125f, right = 0.125f;
        nes_audio_process(0, 48000.0, &left, &right);
        mixed = changed_from(left, right, 0.125f);
    }
    CHECK(mixed);

    nes_cpu_write_observe(0x3012, 0x02); // stop BGM
    CHECK(nes_hd_runtime_info(runtime, &audio_info) && !audio_info.audio_active);
    CHECK(nes_execution_set_policy(NES_EXECUTION_SPECULATIVE));
    nes_cpu_write_observe(0x3042, 1);
    nes_cpu_write_observe(0x3052, 2);
    CHECK(nes_hd_runtime_info(runtime, &audio_info) && !audio_info.audio_active);
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));

    nes_cpu_write_observe(0x3042, 3); // album 3
    nes_cpu_write_observe(0x3032, 255); // SFX volume
    nes_cpu_write_observe(0x3062, 4); // SFX track 4
    CHECK(nes_hd_runtime_info(runtime, &audio_info) && audio_info.audio_active);
    mixed = false;
    for (unsigned i = 0; i < 2048 && !mixed; ++i) {
        float left = 0.25f, right = 0.25f;
        nes_audio_process(0, 44100.0, &left, &right);
        mixed = changed_from(left, right, 0.25f);
    }
    CHECK(mixed);

    fs::path exported = root / "runtime-export.zip";
    std::string exported_text = utf8(exported);
    CHECK(nes_hd_runtime_export(runtime, exported_text.c_str(), error, sizeof(error)));
    auto exported_pack = cupid::hd::load_pack_zip(exported_text);
    CHECK(exported_pack && exported_pack.candidate->audio_tracks.size() == 2);
    nes_hd_runtime_reset_audio(runtime);
    CHECK(nes_hd_runtime_info(runtime, &audio_info) && !audio_info.audio_active && audio_info.enabled);

    CHECK(nes_hd_runtime_enable(runtime, false, error, sizeof(error)));
    CHECK(nes_hd_runtime_info(runtime, &enabled) && !enabled.enabled && !enabled.audio_active);
    NesStateBlob hardware_after{};
    CHECK(nes_state_capture(&hardware_after) == NES_STATE_OK);
    CHECK(hardware_before.size == hardware_after.size
          && (!hardware_before.size
              || std::memcmp(hardware_before.data, hardware_after.data, hardware_before.size) == 0));
    nes_state_blob_free(&hardware_before);
    nes_state_blob_free(&hardware_after);
    nes_hd_runtime_clear_game(runtime);
    CHECK(nes_hd_runtime_info(runtime, &enabled) && !enabled.game_attached && !enabled.pack_loaded);
    nes_hd_runtime_destroy(runtime);
    CHECK(unload_rom());

    fs::remove_all(root, ec);
    CHECK(!ec);
    return 0;
}

int install_discovery_switch_and_panel() {
    fs::path root = fresh_root();
    std::string root_text = utf8(root);
    char error[512] = {0};
    NesHdRuntime *runtime = nes_hd_runtime_create(root_text.c_str(), error, sizeof(error));
    CHECK(runtime != nullptr);
    if (!runtime) return failures;
    NesHdGameInfo game = {
        "Managed Game.nes",
        "0123456789ABCDEF0123456789ABCDEF01234567",
        nullptr, 0, false
    };
    CHECK(nes_hd_runtime_set_game(runtime, &game, error, sizeof(error)));

    frontend_panels_reset();
    NesHdFrontend *frontend = nes_hd_frontend_create(runtime, error, sizeof(error));
    CHECK(frontend != nullptr);
    CHECK(frontend_paths_init(root_text.c_str(),error,sizeof(error)));
    CHECK(nes_hd_frontend_restore_preferences(frontend,game.rom_sha1,error,sizeof(error)));
    FrontendPanelInfo panel_info{};
    CHECK(frontend_panel_get(HD_PACK_PANEL, &panel_info));
    CHECK((panel_info.flags & FRONTEND_PANEL_NEEDS_SESSION) != 0);
    CHECK(!panel_info.enabled);
    frontend_panel_set_session_active(true);
    CHECK(frontend_panel_get(HD_PACK_PANEL, &panel_info) && panel_info.enabled);

    FrontendPanelControl controls[16]{};
    FrontendPanelModel model{controls, 16, 0, nullptr};
    CHECK(frontend_panel_snapshot(HD_PACK_PANEL, &model, error, sizeof(error)));
    CHECK(model.count == 13);

    std::array<std::uint8_t, 32> capture_chr{};
    for (unsigned y = 0; y < 8; ++y) capture_chr[y] = 0x80;
    capture_chr[16 + 8] = 0x40;
    std::vector<NesVideoPixel> capture_pixels;
    fill_capture_trace(capture_pixels, capture_chr, true);
    NesVideoTraceFrame capture_trace{capture_pixels.data(), 42, 1, true};
    NesHdFrameSource capture_source{};
    capture_source.screens = 1;
    capture_source.trace[0] = &capture_trace;
    fs::path panel_capture_dir = root / "capture-output";
    std::error_code panel_ec;
    fs::create_directories(panel_capture_dir, panel_ec);
    CHECK(!panel_ec);
    std::string panel_capture_text = utf8(panel_capture_dir / "fresh-panel.zip");
    CHECK(frontend_panel_action(HD_PACK_PANEL, HD_CONTROL_CAPTURE_PATH,
                                panel_capture_text.c_str(), -1, error, sizeof(error)));
    CHECK(frontend_panel_action(HD_PACK_PANEL, HD_CONTROL_CAPTURE_ARMED,
                                nullptr, 1, error, sizeof(error)));
    CHECK(nes_video_trace_active);
    nes_hd_frontend_set_capture_source(frontend, &capture_source);
    model.count = 0;
    CHECK(frontend_panel_snapshot(HD_PACK_PANEL, &model, error, sizeof(error)));
    CHECK(model.count == 13);
    bool capture_action_enabled = false;
    for (size_t i = 0; i < model.count; ++i)
        if (controls[i].id == HD_CONTROL_CAPTURE) capture_action_enabled = controls[i].enabled;
    CHECK(capture_action_enabled);
    CHECK(frontend_panel_action(HD_PACK_PANEL, HD_CONTROL_CAPTURE,
                                nullptr, -1, error, sizeof(error)));
    auto panel_capture = cupid::hd::load_pack_zip(panel_capture_text);
    CHECK(panel_capture && panel_capture.candidate->tiles.size() == 2);
    CHECK(frontend_panel_action(HD_PACK_PANEL, HD_CONTROL_CAPTURE_ARMED,
                                nullptr, 0, error, sizeof(error)));
    CHECK(!nes_video_trace_active);

    CHECK(frontend_panel_action(HD_PACK_PANEL, HD_CONTROL_INSTALL_SOURCE,
                                "src/tests/fixtures/hd_pack_v109", -1, error, sizeof(error)));
    CHECK(frontend_panel_action(HD_PACK_PANEL, HD_CONTROL_INSTALL_NAME,
                                "managed-pack", -1, error, sizeof(error)));
    CHECK(frontend_panel_action(HD_PACK_PANEL, HD_CONTROL_INSTALL,
                                nullptr, -1, error, sizeof(error)));

    CHECK(nes_hd_runtime_candidate_count(runtime) == 1);
    NesHdPackCandidateInfo candidate{};
    CHECK(nes_hd_runtime_candidate_at(runtime, 0, &candidate));
    CHECK(candidate.archive && candidate.installed && candidate.compatible && candidate.active);
    CHECK(nes_hd_runtime_switch(runtime, 0, error, sizeof(error)));

    fs::path panel_export = root / "panel-export.zip";
    std::string panel_export_text = utf8(panel_export);
    CHECK(frontend_panel_action(HD_PACK_PANEL, HD_CONTROL_EXPORT_PATH,
                                panel_export_text.c_str(), -1, error, sizeof(error)));
    CHECK(frontend_panel_action(HD_PACK_PANEL, HD_CONTROL_EXPORT,
                                nullptr, -1, error, sizeof(error)));
    auto exported_pack = cupid::hd::load_pack_zip(panel_export_text);
    CHECK(exported_pack && exported_pack.candidate->tiles.size() == 2);

    CHECK(frontend_panel_action(HD_PACK_PANEL, HD_CONTROL_ENABLED,
                                nullptr, 1, error, sizeof(error)));
    NesHdRuntimeInfo info{};
    CHECK(nes_hd_runtime_info(runtime, &info) && info.enabled);
    CHECK(frontend_panel_action(HD_PACK_PANEL, HD_CONTROL_ENABLED,
                                nullptr, 0, error, sizeof(error)));
    CHECK(nes_hd_runtime_info(runtime, &info) && !info.enabled);
    CHECK(nes_hd_runtime_enable(runtime,true,error,sizeof(error)));
    CHECK(nes_hd_frontend_restore_preferences(frontend,game.rom_sha1,error,sizeof(error)));
    CHECK(nes_hd_runtime_info(runtime,&info) && !info.enabled && info.pack_loaded);
    CHECK(frontend_panel_action(HD_PACK_PANEL, HD_CONTROL_RESCAN,
                                nullptr, -1, error, sizeof(error)));

    model.count = 0;
    CHECK(frontend_panel_snapshot(HD_PACK_PANEL, &model, error, sizeof(error)));
    CHECK(model.count == 13 && model.status != nullptr);

    nes_hd_frontend_destroy(frontend);
    frontend_paths_shutdown();
    CHECK(!frontend_panel_get(HD_PACK_PANEL, &panel_info));
    frontend_panels_reset();
    nes_hd_runtime_destroy(runtime);
    std::error_code ec;
    fs::remove_all(root, ec);
    CHECK(!ec);
    return 0;
}
} // namespace

extern "C" int test_hd_runtime_accuracy(void) {
    failures = 0;
    fresh_capture_round_trip_and_failures();
    lifecycle_render_audio_and_transaction();
    install_discovery_switch_and_panel();
    std::printf("HD runtime: 3 groups, %d failures\n", failures);
    return failures;
}
