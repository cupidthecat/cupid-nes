/*
 * hd_pack_accuracy.cpp - HD pack format 109 loader regressions
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../hd/hd_pack_loader.hpp"
#include "../third_party/miniz/miniz.h"

#include <algorithm>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

using namespace cupid::hd;

namespace {
int failures;
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "HD pack check failed at %s:%d: %s\n", __FILE__, __LINE__, #x); ++failures; } } while (0)

const char *fixture_dir() { return "src/tests/fixtures/hd_pack_v109"; }

std::string definition_text(const Pack &pack) {
    for (const auto &asset : pack.assets)
        if (asset.path == "hires.txt") return std::string(asset.bytes.begin(), asset.bytes.end());
    return {};
}

void set_definition(Pack &pack, const std::string &text) {
    for (auto &asset : pack.assets) {
        if (asset.path == "hires.txt") {
            asset.bytes.assign(text.begin(), text.end());
            return;
        }
    }
}

bool replace_once(std::string &text, const std::string &from, const std::string &to) {
    const auto pos = text.find(from);
    if (pos == std::string::npos) return false;
    text.replace(pos, from.size(), to);
    return true;
}

LoadResult reload_modified(const Pack &base, const std::string &definition) {
    Pack changed = base;
    set_definition(changed, definition);
    std::vector<std::uint8_t> zip;
    std::string error;
    if (!create_archive(changed, zip, error)) {
        LoadResult result;
        result.error = error;
        return result;
    }
    return load_pack_zip_bytes(zip.data(), zip.size());
}

std::vector<std::uint8_t> raw_zip(const std::vector<std::pair<std::string, std::string>> &entries) {
    mz_zip_archive zip{};
    if (!mz_zip_writer_init_heap(&zip, 0, 1024)) return {};
    bool ok = true;
    for (const auto &entry : entries) {
        if (!mz_zip_writer_add_mem(&zip, entry.first.c_str(), entry.second.data(),
                                   entry.second.size(), MZ_NO_COMPRESSION)) {
            ok = false;
            break;
        }
    }
    void *heap = nullptr;
    size_t size = 0;
    if (ok) ok = mz_zip_writer_finalize_heap_archive(&zip, &heap, &size) != 0;
    std::vector<std::uint8_t> result;
    if (ok) result.assign(static_cast<std::uint8_t *>(heap), static_cast<std::uint8_t *>(heap) + size);
    if (heap) mz_free(heap);
    mz_zip_writer_end(&zip);
    return result;
}

const Condition *condition(const Pack &pack, const std::string &name) {
    for (const auto &entry : pack.conditions) if (entry.name == name) return &entry;
    return nullptr;
}

void check_successful_pack(const Pack &pack) {
    CHECK(pack.version == 109 && pack.scale == 1 && pack.supported_rom_sha1.size() == 1);
    CHECK(pack.tile_images.size() == 1);
    CHECK(pack.tile_images[0].width == 16 && pack.tile_images[0].height == 16);
    CHECK(pack.tile_images[0].argb32.size() == 256 && pack.tile_images[0].argb32[0] == 0x00ff00ffu);
    CHECK(pack.tiles.size() == 2);
    CHECK(pack.tiles[0].key.source == TileSource::ChrRom && pack.tiles[0].key.chr_rom_index == 0);
    CHECK(pack.tiles[0].key.palette == 0x01020304u && pack.tiles[0].default_tile);
    CHECK(pack.tiles[1].key.source == TileSource::ChrRam && pack.tiles[1].chr_bank_id == 2
          && pack.tiles[1].runtime_tile_index == 3 && pack.tiles[1].brightness == 191);
    CHECK(std::all_of(pack.tiles[1].key.chr_ram.begin(), pack.tiles[1].key.chr_ram.end(),
                      [](std::uint8_t b) { return b == 0; }));
    CHECK(pack.conditions.size() == 40);
    CHECK(condition(pack, "tilepos") && condition(pack, "!tilepos") && condition(pack, "sppalette3"));
    CHECK(condition(pack, "mem") && condition(pack, "mem")->type == ConditionType::MemoryCheck);
    CHECK(condition(pack, "ppumem") && condition(pack, "ppumem")->memory_space == MemorySpace::Ppu);
    CHECK(condition(pack, "memc") && condition(pack, "memc")->type == ConditionType::MemoryCheckConstant);
    CHECK(condition(pack, "frame") && condition(pack, "frame")->operand_a == 60);
    CHECK(condition(pack, "posx") && condition(pack, "posx")->operand_b == 0);
    CHECK(condition(pack, "posy") && condition(pack, "posy")->operand_b == 240);
    CHECK(condition(pack, "oposx") && condition(pack, "oposx")->operand_b == 4);
    CHECK(condition(pack, "oposy") && condition(pack, "oposy")->operand_b == 5);
    CHECK(pack.background_images.size() == 1 && pack.backgrounds.size() == 1);
    CHECK(pack.backgrounds[0].priority == 39 && pack.backgrounds[0].blend_mode == BlendMode::Add);
    CHECK(pack.backgrounds[0].horizontal_scroll_ratio == 0.5f
          && pack.backgrounds[0].vertical_scroll_ratio == -0.25f);
    CHECK(pack.backgrounds[0].left == 2 && pack.backgrounds[0].top == 3
          && pack.backgrounds[0].conditions.size() == 4);
    CHECK(pack.additions.size() == 1 && pack.additions[0].offset_x == -2 && pack.additions[0].ignore_palette);
    CHECK(pack.fallbacks.size() == 1 && pack.fallbacks[0].tile_index == 0
          && pack.fallbacks[0].fallback_tile_index == 1);
    CHECK(pack.overscan && pack.overscan->top == 8 && pack.overscan->right == 7
          && pack.overscan->bottom == 6 && pack.overscan->left == 5);
    CHECK((static_cast<std::uint32_t>(pack.render_options)
          & static_cast<std::uint32_t>(RenderOption::DisableCache)) != 0);
    CHECK((static_cast<std::uint32_t>(pack.hardware_options)
          & static_cast<std::uint32_t>(HardwareOption::AlternateRegisterRange)) != 0);
    CHECK(pack.palette && (*pack.palette)[0] == 0xff00070eu);
    CHECK(pack.audio_tracks.size() == 2);
    const AudioTrack *bgm = nullptr, *sfx = nullptr;
    for (const auto &track : pack.audio_tracks) {
        if (track.role == AudioRole::BackgroundMusic) bgm = &track;
        else sfx = &track;
    }
    CHECK(bgm && bgm->album == 1 && bgm->track == 2 && bgm->channels == 2
          && bgm->sample_rate == 32000 && bgm->loop_position == 1);
    CHECK(bgm && bgm->pcm.size() > 1000 && bgm->pcm[0] != bgm->pcm[1]);
    CHECK(sfx && sfx->album == 3 && sfx->track == 4 && sfx->channels == 2
          && sfx->sample_rate == 22050 && sfx->pcm.size() == 4410);
}

void expect_failure(const Pack &base, std::string text, const std::string &from,
                    const std::string &to, const char *needle) {
    CHECK(replace_once(text, from, to));
    auto result = reload_modified(base, text);
    CHECK(!result);
    CHECK(result.error.find(needle) != std::string::npos);
}
}  // namespace

extern "C" int test_hd_pack_accuracy(void) {
    failures = 0;
    auto loaded = load_pack_directory(fixture_dir());
    CHECK(loaded);
    if (!loaded) {
        std::fprintf(stderr, "HD fixture failed: %s\n", loaded.error.c_str());
        return failures + 1;
    }
    CHECK(loaded.warnings.size() == 1);
    const Pack &base = *loaded.candidate;
    check_successful_pack(base);

    std::vector<std::uint8_t> archive;
    std::string archive_error;
    CHECK(create_archive(base, archive, archive_error));
    CHECK(!archive.empty());
    auto roundtrip = load_pack_zip_bytes(archive.data(), archive.size());
    CHECK(roundtrip);
    if (roundtrip) check_successful_pack(*roundtrip.candidate);

    std::string text = definition_text(base);
    expect_failure(base, text, "<img>tiles.png", "<img>missing.png", "not found");
    expect_failure(base, text, "[tilepos&tilenear", "[unknown&tilenear", "not defined");
    expect_failure(base, text, "0,0,1.0,Y", "0,0,1e30,Y", "brightness");
    expect_failure(base, text, "<background>bg.png,1.0", "<background>bg.png,nan", "brightness");
    expect_failure(base, text, "<options>alternateRegisterRange", "<options>disableSpriteLimit", "changes emulated hardware");
    auto unknown = reload_modified(base, text + "<unknown>value\n");
    CHECK(!unknown && unknown.error.find("unsupported HD pack tag") != std::string::npos);
    auto patch = reload_modified(base, text + "<patch>game.ips,0123456789012345678901234567890123456789\n");
    CHECK(!patch && patch.error.find("ROM patching") != std::string::npos);

    LoadLimits count_limits;
    count_limits.max_tiles = 1;
    auto count_failure = load_pack_directory(fixture_dir(), count_limits);
    CHECK(!count_failure && count_failure.error.find("too many tile rules") != std::string::npos);
    LoadLimits size_limits;
    size_limits.max_member_bytes = 128;
    auto size_failure = load_pack_directory(fixture_dir(), size_limits);
    CHECK(!size_failure && size_failure.error.find("exceeds") != std::string::npos);

    Pack corrupt_png = base;
    for (auto &asset : corrupt_png.assets)
        if (asset.path == "tiles.png" && !asset.bytes.empty()) asset.bytes[0] ^= 0xff;
    std::vector<std::uint8_t> corrupt_png_zip;
    CHECK(create_archive(corrupt_png, corrupt_png_zip, archive_error));
    auto corrupt_png_result = load_pack_zip_bytes(corrupt_png_zip.data(), corrupt_png_zip.size());
    CHECK(!corrupt_png_result && corrupt_png_result.error.find("PNG asset") != std::string::npos);

    Pack no_palette = base;
    no_palette.assets.erase(std::remove_if(no_palette.assets.begin(), no_palette.assets.end(),
        [](const PackAsset &asset) { return asset.path == "palette.dat"; }), no_palette.assets.end());
    std::vector<std::uint8_t> no_palette_zip;
    CHECK(create_archive(no_palette, no_palette_zip, archive_error));
    auto optional_palette = load_pack_zip_bytes(no_palette_zip.data(), no_palette_zip.size());
    CHECK(optional_palette && !optional_palette.candidate->palette);
    Pack bad_palette = base;
    for (auto &asset : bad_palette.assets) if (asset.path == "palette.dat") asset.bytes.pop_back();
    std::vector<std::uint8_t> bad_palette_zip;
    CHECK(create_archive(bad_palette, bad_palette_zip, archive_error));
    auto malformed_palette = load_pack_zip_bytes(bad_palette_zip.data(), bad_palette_zip.size());
    CHECK(!malformed_palette && malformed_palette.error.find("exactly 192") != std::string::npos);

    Pack corrupt_audio = base;
    for (auto &asset : corrupt_audio.assets) if (asset.path == "bgm.ogg" && !asset.bytes.empty()) asset.bytes.resize(32);
    std::vector<std::uint8_t> corrupt_audio_zip;
    CHECK(create_archive(corrupt_audio, corrupt_audio_zip, archive_error));
    auto corrupt_audio_result = load_pack_zip_bytes(corrupt_audio_zip.data(), corrupt_audio_zip.size());
    CHECK(!corrupt_audio_result && corrupt_audio_result.error.find("Ogg Vorbis") != std::string::npos);

    auto traversal_zip = raw_zip({{"hires.txt", "<ver>109\n"}, {"../escape.png", "x"}});
    auto traversal = load_pack_zip_bytes(traversal_zip.data(), traversal_zip.size());
    CHECK(!traversal && traversal.error.find("unsafe ZIP entry") != std::string::npos);
    auto duplicate_zip = raw_zip({{"hires.txt", "<ver>109\n"}, {"Asset.png", "x"}, {"asset.png", "y"}});
    auto duplicate = load_pack_zip_bytes(duplicate_zip.data(), duplicate_zip.size());
    CHECK(!duplicate && duplicate.error.find("duplicate or case-aliased") != std::string::npos);
    std::string invalid_name(1, static_cast<char>(0xff));
    invalid_name += ".png";
    auto utf8_zip = raw_zip({{"hires.txt", "<ver>109\n"}, {invalid_name, "x"}});
    auto utf8_failure = load_pack_zip_bytes(utf8_zip.data(), utf8_zip.size());
    CHECK(!utf8_failure && utf8_failure.error.find("UTF-8") != std::string::npos);
    LoadLimits entry_limits;
    entry_limits.max_entries = 1;
    auto entry_failure = load_pack_zip_bytes(archive.data(), archive.size(), entry_limits);
    CHECK(!entry_failure && entry_failure.error.find("too many entries") != std::string::npos);
    auto corrupted = raw_zip({{"hires.txt", "<ver>109\n"}});
    const std::string marker = "<ver>109";
    auto marker_pos = std::search(corrupted.begin(), corrupted.end(), marker.begin(), marker.end());
    CHECK(marker_pos != corrupted.end());
    if (marker_pos != corrupted.end()) *marker_pos ^= 1;
    auto corruption = load_pack_zip_bytes(corrupted.data(), corrupted.size());
    CHECK(!corruption && !corruption.error.empty());

    std::puts(failures ? "HD pack regressions: FAIL" : "HD pack regressions: PASS");
    return failures;
}
