/*
 * hd_builder.cpp - Editable format-109 pack drafts
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "hd_builder.h"
#include "hd_pack_loader.hpp"
#include "hd_renderer.hpp"
#include "../util/file_io.h"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>
#include <sstream>

using namespace cupid::hd;

struct NesHdBuilder {
    std::vector<std::string> lines;
    std::vector<PackAsset> assets;
    std::vector<std::string> protected_paths;
    std::vector<uint8_t> chr;
    std::shared_ptr<const Pack> validated;
    std::unique_ptr<Renderer> renderer;
    RenderedFrame side[2];
    std::vector<uint32_t> pixels;
    std::string game_path;
};

namespace {
bool fail(char *error, size_t size, const std::string &message) {
    if (error && size) {
        std::snprintf(error, size, "%s", message.c_str());
    }
    return false;
}

bool ok(char *error, size_t size) {
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

bool definition(const NesHdBuilder &builder, Pack &draft, std::string &error) {
    std::string text;
    LoadLimits limits;
    for (const auto &line : builder.lines) {
        if (line.size() + 1 > limits.max_definition_bytes - text.size()) {
            error = "Pack definition exceeds 4 MiB";
            return false;
        }
        text += line;
        text += '\n';
    }
    draft.assets = builder.assets;
    draft.assets.push_back({"hires.txt", AssetKind::Definition, std::vector<uint8_t>(text.begin(), text.end())});
    return true;
}

bool validate(NesHdBuilder &builder, std::string &error) {
    Pack draft;
    if (!definition(builder, draft, error)) {
        return false;
    }
    std::vector<uint8_t> bytes;
    if (!create_archive(draft, bytes, error)) {
        return false;
    }
    auto result = load_pack_zip_bytes(bytes.data(), bytes.size());
    if (!result) {
        error = result.error;
        return false;
    }
    std::shared_ptr<const Pack> replacement(std::move(result.candidate));
    auto renderer = Renderer::create(replacement, builder.chr.data(), builder.chr.size(), error);
    if (!renderer) {
        return false;
    }
    builder.validated = std::move(replacement);
    builder.renderer = std::move(renderer);
    return true;
}

bool safe_export(const NesHdBuilder &builder, const char *path) {
    if (!path || !*path) {
        return false;
    }
    auto separate = [path](const std::string &other) {
        if (other.empty()) {
            return true;
        }
        bool same = false;
        NesFileResult result = nes_file_same(path, other.c_str(), &same);
        return !same && (result == NES_FILE_OK || result == NES_FILE_NOT_FOUND);
    };
    if (!separate(builder.game_path)) {
        return false;
    }
    return std::all_of(builder.protected_paths.begin(), builder.protected_paths.end(), separate);
}

struct MemoryContext {
    NesHdMemoryReader read;
    void *context;
    unsigned side;
};

uint8_t peek(void *context, MemorySpace space, uint16_t address) {
    auto *memory = static_cast<MemoryContext *>(context);
    return memory->read ? memory->read(memory->context, memory->side, space == MemorySpace::Ppu, address) : 0;
}
} // namespace

NesHdBuilder *nes_hd_builder_create(void) {
    auto *builder = new (std::nothrow) NesHdBuilder;
    if (builder && !nes_hd_builder_new(builder, 2, nullptr, 0)) {
        delete builder;
        return nullptr;
    }
    return builder;
}

void nes_hd_builder_destroy(NesHdBuilder *builder) {
    delete builder;
}

bool nes_hd_builder_new(NesHdBuilder *builder, unsigned scale, char *error, size_t size) {
    if (!builder || scale < 1 || scale > 10) {
        return fail(error, size, "Pack scale must be from 1 to 10");
    }
    try {
        NesHdBuilder next;
        next.lines = {"<ver>109", "<scale>" + std::to_string(scale)};
        next.chr = builder->chr;
        next.game_path = builder->game_path;
        next.protected_paths = builder->protected_paths;
        std::string why;
        if (!validate(next, why)) {
            return fail(error, size, why);
        }
        *builder = std::move(next);
        return ok(error, size);
    } catch (const std::exception &e) {
        return fail(error, size, e.what());
    }
}

bool nes_hd_builder_open(NesHdBuilder *builder, const char *path, char *error, size_t size) {
    if (!builder || !path || !*path) {
        return fail(error, size, "Pack path is required");
    }
    try {
        auto loaded = load_pack_zip(path);
        if (!loaded) {
            loaded = load_pack_directory(path);
        }
        if (!loaded) {
            return fail(error, size, loaded.error);
        }
        NesHdBuilder next;
        next.chr = builder->chr;
        next.game_path = builder->game_path;
        next.protected_paths = builder->protected_paths;
        for (auto &asset : loaded.candidate->assets) {
            if (asset.path != "hires.txt") {
                next.assets.push_back(std::move(asset));
                continue;
            }
            std::istringstream stream(std::string(asset.bytes.begin(), asset.bytes.end()));
            std::string line;
            while (std::getline(stream, line)) {
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }
                next.lines.push_back(line);
            }
        }
        std::string why;
        if (!validate(next, why)) {
            return fail(error, size, why);
        }
        *builder = std::move(next);
        return ok(error, size);
    } catch (const std::exception &e) {
        return fail(error, size, e.what());
    }
}

size_t nes_hd_builder_line_count(const NesHdBuilder *builder) {
    return builder ? builder->lines.size() : 0;
}

const char *nes_hd_builder_line(const NesHdBuilder *builder, size_t index) {
    return builder && index < builder->lines.size() ? builder->lines[index].c_str() : nullptr;
}

bool nes_hd_builder_set_line(NesHdBuilder *builder, size_t index, const char *line, char *error, size_t size) {
    if (!builder || index > builder->lines.size() || (!line && index == builder->lines.size())) {
        return fail(error, size, "Select an existing line or append at the end");
    }
    if (line && (std::strlen(line) > 16384 || std::strchr(line, '\n') || std::strchr(line, '\r'))) {
        return fail(error, size, "Enter one definition line of at most 16384 bytes");
    }
    try {
        size_t bytes = line ? std::strlen(line) + 1 : 0;
        for (size_t i = 0; i < builder->lines.size(); ++i) {
            if (i != index) {
                bytes += builder->lines[i].size() + 1;
            }
        }
        if (bytes > LoadLimits{}.max_definition_bytes || builder->lines.size() >= LoadLimits{}.max_lines) {
            return fail(error, size, "Draft definition exceeds loader limits");
        }
        if (!line) {
            builder->lines.erase(builder->lines.begin() + static_cast<ptrdiff_t>(index));
        } else if (index == builder->lines.size()) {
            builder->lines.emplace_back(line);
        } else {
            builder->lines[index] = line;
        }
        return ok(error, size);
    } catch (const std::exception &e) {
        return fail(error, size, e.what());
    }
}

bool nes_hd_builder_import_asset(NesHdBuilder *builder, const char *member, const char *path, char *error,
                                 size_t size) {
    if (!builder || !member || !*member || !path || !std::strcmp(member, "hires.txt")) {
        return fail(error, size, "Specify an asset path other than hires.txt");
    }
    uint8_t *data = nullptr;
    size_t count = 0;
    NesFileResult read = nes_file_read_all(path, LoadLimits{}.max_member_bytes, &data, &count);
    if (read != NES_FILE_OK) {
        return fail(error, size, nes_file_result_message(read));
    }
    std::unique_ptr<uint8_t, decltype(&std::free)> allocation(data, &std::free);
    try {
        auto assets = builder->assets;
        auto found =
            std::find_if(assets.begin(), assets.end(), [member](const PackAsset &a) { return a.path == member; });
        PackAsset replacement{member, AssetKind::TileImage, std::vector<uint8_t>(data, data + count)};
        size_t dot = replacement.path.find_last_of('.');
        std::string extension = dot == std::string::npos ? "" : replacement.path.substr(dot);
        std::transform(extension.begin(), extension.end(), extension.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        std::string asset_error;
        if (extension == ".png") {
            Image decoded;
            if (!decode_png(replacement.bytes, member, {}, decoded, asset_error)) {
                return fail(error, size, asset_error);
            }
        } else if (extension == ".wav" || extension == ".ogg") {
            AudioTrack decoded;
            if (!decode_audio(replacement.bytes, member, {}, decoded, asset_error)) {
                return fail(error, size, asset_error);
            }
            replacement.kind = AssetKind::Audio;
        } else if (replacement.path == "palette.dat" && count == 192) {
            replacement.kind = AssetKind::Palette;
        } else {
            return fail(error, size, "Import PNG, WAV, OGG, or a 192-byte palette.dat");
        }
        if (found != assets.end()) {
            *found = std::move(replacement);
        } else {
            assets.push_back(std::move(replacement));
        }
        size_t total = 0;
        for (const auto &asset : assets) {
            total += asset.bytes.size();
        }
        if (total > LoadLimits{}.max_total_asset_bytes - LoadLimits{}.max_definition_bytes ||
            assets.size() >= LoadLimits{}.max_entries) {
            return fail(error, size, "Draft assets exceed loader limits");
        }
        Pack probe;
        probe.assets = assets;
        probe.assets.push_back({"hires.txt", AssetKind::Definition, {'<', 'v', 'e', 'r', '>', '1', '0', '9', '\n'}});
        std::vector<uint8_t> bytes;
        std::string why;
        if (!create_archive(probe, bytes, why)) {
            return fail(error, size, why);
        }
        builder->assets = std::move(assets);
        return ok(error, size);
    } catch (const std::exception &e) {
        return fail(error, size, e.what());
    }
}

bool nes_hd_builder_validate(NesHdBuilder *builder, char *error, size_t size) {
    if (!builder) {
        return fail(error, size, "No HD draft");
    }
    try {
        std::string why;
        return validate(*builder, why) ? ok(error, size) : fail(error, size, why);
    } catch (const std::exception &e) {
        return fail(error, size, e.what());
    }
}

bool nes_hd_builder_export(NesHdBuilder *builder, const char *path, char *error, size_t size) {
    if (!builder || !safe_export(*builder, path)) {
        return fail(error, size, "Export would replace a protected file");
    }
    if (!nes_hd_builder_validate(builder, error, size)) {
        return false;
    }
    try {
        std::vector<uint8_t> bytes;
        std::string why;
        if (!create_archive(*builder->validated, bytes, why)) {
            return fail(error, size, why);
        }
        NesFileResult result = nes_file_write_atomic(path, bytes.data(), bytes.size());
        return result == NES_FILE_OK ? ok(error, size) : fail(error, size, nes_file_result_message(result));
    } catch (const std::exception &e) {
        return fail(error, size, e.what());
    }
}

bool nes_hd_builder_set_game(NesHdBuilder *builder, const NesHdGameInfo *game, char *error, size_t size) {
    if (!builder || !game || (game->chr_size && !game->chr_rom) || game->chr_size > 64u * 1024u * 1024u) {
        return fail(error, size, "Game CHR data is invalid or exceeds 64 MiB");
    }
    try {
        std::vector<uint8_t> chr;
        if (game->chr_is_rom && game->chr_size) {
            chr.assign(game->chr_rom, game->chr_rom + game->chr_size);
        }
        std::string game_path = game->rom_path ? game->rom_path : "";
        std::string why;
        auto renderer =
            builder->validated ? Renderer::create(builder->validated, chr.data(), chr.size(), why) : nullptr;
        if (builder->validated && !renderer) {
            return fail(error, size, why);
        }
        builder->chr = std::move(chr);
        builder->game_path = std::move(game_path);
        builder->renderer = std::move(renderer);
        return ok(error, size);
    } catch (const std::exception &e) {
        return fail(error, size, e.what());
    }
}

bool nes_hd_builder_set_protected_paths(NesHdBuilder *builder, const char *const *paths, size_t count) {
    if (!builder || (count && !paths) || count > 4096) {
        return false;
    }
    try {
        std::vector<std::string> replacement;
        for (size_t i = 0; i < count; ++i) {
            if (paths[i]) {
                replacement.emplace_back(paths[i]);
            }
        }
        builder->protected_paths = std::move(replacement);
        return true;
    } catch (...) {
        return false;
    }
}

void nes_hd_builder_clear_game(NesHdBuilder *builder) {
    if (!builder) {
        return;
    }
    builder->renderer.reset();
    std::vector<uint8_t>().swap(builder->chr);
    std::vector<uint32_t>().swap(builder->pixels);
    for (auto &side : builder->side) {
        side = RenderedFrame{};
    }
    builder->game_path.clear();
}

bool nes_hd_builder_preview(NesHdBuilder *builder, const NesHdFrameSource *source, NesHdMemoryReader memory,
                            void *context, NesHdFrame *frame, char *error, size_t size) {
    if (!builder || !builder->renderer || !source || !frame || source->screens < 1 || source->screens > 2) {
        return fail(error, size, "Validate a draft and attach a completed frame before previewing");
    }
    try {
        if (source->screens == 2 && (!source->trace[0] || !source->trace[1] ||
                                     source->trace[0]->frame_number != source->trace[1]->frame_number)) {
            return fail(error, size, "Dual-screen previews need observations from the same frame");
        }
        RenderSettings settings;
        settings.background = source->show_background;
        settings.sprites = source->show_sprites;
        settings.use_pack_overscan = source->allow_pack_overscan;
        settings.overscan = {source->overscan.top, source->overscan.right, source->overscan.bottom,
                             source->overscan.left};
        std::string why;
        for (unsigned side = 0; side < source->screens; ++side) {
            if (!source->trace[side] || !source->trace[side]->complete) {
                return fail(error, size, "No completed tile observations");
            }
            MemoryContext reader{memory, context, side};
            if (!builder->renderer->render(*source->trace[side], settings, peek, &reader, builder->side[side], why)) {
                return fail(error, size, why);
            }
        }
        const auto &first = builder->side[0];
        unsigned width = first.width * source->screens;
        if (source->screens == 2 &&
            (builder->side[1].width != first.width || builder->side[1].height != first.height)) {
            return fail(error, size, "Preview screens have inconsistent geometry");
        }
        builder->pixels.resize(static_cast<size_t>(width) * first.height);
        for (unsigned side = 0; side < source->screens; ++side) {
            for (unsigned y = 0; y < first.height; ++y) {
                std::copy_n(builder->side[side].pixels.data() + static_cast<size_t>(y) * first.width, first.width,
                            builder->pixels.data() + static_cast<size_t>(y) * width + side * first.width);
            }
        }
        *frame = {builder->pixels.data(),
                  width,
                  first.height,
                  source->screens,
                  first.scale,
                  {first.overscan.left, first.overscan.right, first.overscan.top, first.overscan.bottom},
                  true,
                  true};
        return ok(error, size);
    } catch (const std::exception &e) {
        return fail(error, size, e.what());
    }
}
