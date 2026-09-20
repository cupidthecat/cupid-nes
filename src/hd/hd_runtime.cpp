/*
 * hd_runtime.cpp - Host-side HD pack lifecycle, rendering, and enhanced audio
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "hd_runtime.h"

#include "hd_pack_loader.hpp"
#include "hd_renderer.hpp"
#include "../audio/audio_observer.h"
#include "../capture/capture_writer.h"
#include "../cpu/cpu_observer.h"
#include "../util/file_io.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace {
using cupid::hd::AudioRole;
using cupid::hd::AssetKind;
using cupid::hd::AudioTrack;
using cupid::hd::HardwareOption;
using cupid::hd::MemorySpace;
using cupid::hd::Pack;
using cupid::hd::PackAsset;
using cupid::hd::Renderer;
using cupid::hd::RenderedFrame;
using cupid::hd::RenderSettings;

struct Candidate {
    std::string path;
    std::string name;
    bool archive = false;
    bool installed = false;
    bool compatible = true;
};

struct Voice {
    const AudioTrack *track = nullptr;
    double position = 0.0;
};

struct NesHdRuntimeImpl {
    std::string pack_root;
    std::string rom_path;
    std::string rom_sha1;
    std::string game_name;
    std::vector<std::uint8_t> chr_copy;
    bool chr_is_rom = false;
    std::vector<std::string> protected_paths;

    std::vector<Candidate> candidates;
    std::shared_ptr<Pack> pack;
    std::unique_ptr<Renderer> renderer;
    std::string active_path;
    std::string status = "No game is attached";
    std::vector<std::uint32_t> output;
    bool enabled = false;
    bool trace_ready = false;

    NesHdMemoryReader memory_reader = nullptr;
    void *memory_context = nullptr;

    uint32_t cpu_observer = 0;
    uint32_t audio_processor = 0;
    std::uint8_t album = 0;
    std::uint8_t playback_options = 0;
    std::uint8_t bgm_volume = 128;
    std::uint8_t sfx_volume = 128;
    bool bgm_paused = false;
    Voice bgm;
    std::array<Voice, 16> sfx{};
    unsigned next_sfx = 0;
};

static void set_error(char *error, size_t error_size, const std::string &message) {
    if (error && error_size) std::snprintf(error, error_size, "%s", message.c_str());
}

static void clear_error(char *error, size_t error_size) {
    if (error && error_size) error[0] = '\0';
}

static bool has_hardware_option(const Pack &pack, HardwareOption option) {
    return (static_cast<std::uint32_t>(pack.hardware_options)
            & static_cast<std::uint32_t>(option)) != 0;
}

static std::string lowercase_ascii(std::string text) {
    for (char &c : text) if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    return text;
}

static std::string path_utf8(const fs::path &path) {
    auto value = path.u8string();
    return std::string(value.begin(), value.end());
}

static fs::path utf8_path(const std::string &path) {
    return fs::u8path(path.begin(), path.end());
}

static bool valid_sha1(const char *sha1, std::string &canonical) {
    canonical.clear();
    if (!sha1 || !*sha1) return true;
    if (std::strlen(sha1) != 40) return false;
    canonical.reserve(40);
    for (const unsigned char c : std::string(sha1)) {
        if (!std::isxdigit(c)) return false;
        canonical.push_back(static_cast<char>(std::toupper(c)));
    }
    return true;
}

static std::string game_stem(const std::string &rom_path) {
    if (rom_path.empty()) return {};
    fs::path path = utf8_path(rom_path);
    std::string stem = path_utf8(path.stem());
    if (stem.empty()) stem = "game";
    return stem;
}

static bool compatible_pack(const NesHdRuntimeImpl &runtime, const Pack &pack) {
    if (pack.supported_rom_sha1.empty() || runtime.rom_sha1.empty()) return true;
    return std::any_of(pack.supported_rom_sha1.begin(), pack.supported_rom_sha1.end(),
                       [&](const std::string &sha1) {
                           return lowercase_ascii(sha1) == lowercase_ascii(runtime.rom_sha1);
                       });
}

static cupid::hd::LoadResult load_path(const std::string &path) {
    std::error_code ec;
    fs::path native = utf8_path(path);
    if (fs::is_directory(native, ec) && !ec) return cupid::hd::load_pack_directory(path);
    return cupid::hd::load_pack_zip(path);
}

struct PreparedPack {
    std::shared_ptr<Pack> pack;
    std::unique_ptr<Renderer> renderer;
};

static bool prepare_pack(NesHdRuntimeImpl &runtime, const std::string &path,
                         PreparedPack &prepared, std::string &error) {
    auto loaded = load_path(path);
    if (!loaded) {
        error = loaded.error.empty() ? "HD pack could not be loaded" : loaded.error;
        return false;
    }
    if (!compatible_pack(runtime, *loaded.candidate)) {
        error = "HD pack does not declare support for the loaded game SHA-1";
        return false;
    }
    if (has_hardware_option(*loaded.candidate, HardwareOption::DisableSpriteLimit)) {
        error = "HD pack requests disableSpriteLimit, which changes emulated hardware";
        return false;
    }
    auto shared = std::shared_ptr<Pack>(loaded.candidate.release());
    std::string renderer_error;
    auto renderer = Renderer::create(shared,
        runtime.chr_is_rom && !runtime.chr_copy.empty() ? runtime.chr_copy.data() : nullptr,
        runtime.chr_is_rom ? runtime.chr_copy.size() : 0, renderer_error);
    if (!renderer) {
        error = renderer_error.empty() ? "HD renderer rejected the pack" : renderer_error;
        return false;
    }
    prepared.pack = std::move(shared);
    prepared.renderer = std::move(renderer);
    return true;
}

static const AudioTrack *find_track(const NesHdRuntimeImpl &runtime, AudioRole role,
                                    std::uint8_t album, std::uint8_t track) {
    if (!runtime.pack) return nullptr;
    for (const auto &entry : runtime.pack->audio_tracks)
        if (entry.role == role && entry.album == album && entry.track == track) return &entry;
    return nullptr;
}

static void stop_audio(NesHdRuntimeImpl &runtime) {
    runtime.bgm = {};
    for (auto &voice : runtime.sfx) voice = {};
    runtime.next_sfx = 0;
    runtime.bgm_paused = false;
}

static bool alternate_registers(const NesHdRuntimeImpl &runtime) {
    return runtime.pack && has_hardware_option(*runtime.pack, HardwareOption::AlternateRegisterRange);
}

static int audio_register(const NesHdRuntimeImpl &runtime, std::uint16_t address) {
    if (alternate_registers(runtime)) {
        if (address < 0x3002 || address > 0x3062 || (address & 0x0Fu) != 2) return -1;
        return (address - 0x3002) / 0x10;
    }
    return address >= 0x4100 && address <= 0x4106 ? address - 0x4100 : -1;
}

static void observe_cpu_write(void *context, std::uint16_t address, std::uint8_t value) {
    auto &runtime = *static_cast<NesHdRuntimeImpl *>(context);
    if (!runtime.enabled || !runtime.pack) return;
    int reg = audio_register(runtime, address);
    if (reg < 0) return;
    switch (reg) {
        case 0: runtime.playback_options = value; break;
        case 1:
            runtime.bgm_paused = (value & 0x01u) != 0;
            if (value & 0x02u) runtime.bgm = {};
            if (value & 0x04u) for (auto &voice : runtime.sfx) voice = {};
            break;
        case 2: runtime.bgm_volume = value; break;
        case 3: runtime.sfx_volume = value; break;
        case 4: runtime.album = value; break;
        case 5:
            runtime.bgm = {find_track(runtime, AudioRole::BackgroundMusic, runtime.album, value), 0.0};
            break;
        case 6: {
            const AudioTrack *track = find_track(runtime, AudioRole::SoundEffect, runtime.album, value);
            if (track) {
                auto free = std::find_if(runtime.sfx.begin(), runtime.sfx.end(),
                                         [](const Voice &voice) { return !voice.track; });
                if (free != runtime.sfx.end()) *free = {track, 0.0};
                else {
                    runtime.sfx[runtime.next_sfx] = {track, 0.0};
                    runtime.next_sfx = (runtime.next_sfx + 1) % runtime.sfx.size();
                }
            }
            break;
        }
    }
}

static bool sample_voice(Voice &voice, double output_rate, bool loop,
                         float &left, float &right) {
    const AudioTrack *track = voice.track;
    if (!track || !track->channels || !track->sample_rate || track->pcm.empty()) return false;
    const std::size_t frames = track->pcm.size() / track->channels;
    if (!frames) { voice = {}; return false; }
    if (voice.position >= frames) {
        if (loop && track->loop_position < frames) voice.position = track->loop_position;
        else { voice = {}; return false; }
    }
    std::size_t first = static_cast<std::size_t>(voice.position);
    std::size_t second = std::min(first + 1, frames - 1);
    float fraction = static_cast<float>(voice.position - first);
    auto channel = [&](std::size_t frame, unsigned ch) {
        unsigned source = track->channels == 1 ? 0u : ch;
        return static_cast<float>(track->pcm[frame * track->channels + source]) / 32768.0f;
    };
    left = channel(first, 0) + (channel(second, 0) - channel(first, 0)) * fraction;
    right = channel(first, 1) + (channel(second, 1) - channel(first, 1)) * fraction;
    voice.position += static_cast<double>(track->sample_rate) / output_rate;
    return true;
}

static void process_audio(void *context, unsigned machine, double sample_rate,
                          float *left, float *right) {
    auto &runtime = *static_cast<NesHdRuntimeImpl *>(context);
    if (machine || !runtime.enabled || !runtime.pack || !left || !right) return;
    float mixed_left = 0.0f, mixed_right = 0.0f;
    if (!runtime.bgm_paused) {
        float l = 0.0f, r = 0.0f;
        if (sample_voice(runtime.bgm, sample_rate, (runtime.playback_options & 1u) != 0, l, r)) {
            float gain = runtime.bgm_volume / 255.0f;
            mixed_left += l * gain;
            mixed_right += r * gain;
        }
    }
    for (auto &voice : runtime.sfx) {
        float l = 0.0f, r = 0.0f;
        if (sample_voice(voice, sample_rate, false, l, r)) {
            float gain = runtime.sfx_volume / 255.0f;
            mixed_left += l * gain;
            mixed_right += r * gain;
        }
    }
    *left += mixed_left;
    *right += mixed_right;
}

static bool attach_host_hooks(NesHdRuntimeImpl &runtime, std::string &error) {
    if (runtime.cpu_observer && runtime.audio_processor) return true;
    uint32_t cpu = nes_cpu_write_observer_add(observe_cpu_write, &runtime);
    if (!cpu) { error = "HD audio CPU-write observer could not be attached"; return false; }
    uint32_t audio = nes_audio_processor_add(process_audio, &runtime);
    if (!audio) {
        (void)nes_cpu_write_observer_remove(cpu);
        error = "HD audio output processor could not be attached";
        return false;
    }
    runtime.cpu_observer = cpu;
    runtime.audio_processor = audio;
    return true;
}

static void detach_host_hooks(NesHdRuntimeImpl &runtime) {
    if (runtime.cpu_observer) (void)nes_cpu_write_observer_remove(runtime.cpu_observer);
    if (runtime.audio_processor) (void)nes_audio_processor_remove(runtime.audio_processor);
    runtime.cpu_observer = runtime.audio_processor = 0;
    stop_audio(runtime);
}

static bool commit_pack(NesHdRuntimeImpl &runtime, PreparedPack prepared,
                        const std::string &path, std::string &error) {
    bool was_enabled = runtime.enabled;
    if (was_enabled && !attach_host_hooks(runtime, error)) return false;
    runtime.pack = std::move(prepared.pack);
    runtime.renderer = std::move(prepared.renderer);
    runtime.active_path = path;
    stop_audio(runtime);
    runtime.status = "HD pack loaded";
    runtime.trace_ready = false;
    return true;
}

static bool safe_install_name(const char *name, std::string &result) {
    if (!name || !*name) return false;
    result = name;
    if (result == "." || result == ".." || result.size() >= 240) return false;
    for (unsigned char c : result)
        if (c < 0x20u || c == 0x7fu || c == '/' || c == '\\' || c == ':' || c == '*'
            || c == '?' || c == '"' || c == '<' || c == '>' || c == '|') return false;
    if (result.back() == '.' || result.back() == ' ') return false;
    if (lowercase_ascii(result).size() < 4 || lowercase_ascii(result).substr(result.size() - 4) != ".zip")
        result += ".zip";
    return true;
}

static std::string candidate_name(const std::string &path) {
    fs::path native = utf8_path(path);
    std::string name = path_utf8(native.stem());
    return name.empty() ? path : name;
}

static bool output_path_allowed(const NesHdRuntimeImpl &runtime, const char *path,
                                std::string &error) {
    if (!path || !*path || std::strlen(path) >= NES_FILE_PATH_LIMIT) {
        error = "HD pack output path is missing or too long";
        return false;
    }
    auto conflicts = [&](const std::string &protected_path) {
        if (protected_path.empty()) return false;
        bool same = false;
        NesFileResult result = nes_file_same(path, protected_path.c_str(), &same);
        if (result == NES_FILE_OK) return same;
        if (result != NES_FILE_NOT_FOUND) {
            error = "Could not verify the HD pack output path";
            return true;
        }
        std::error_code left_error, right_error;
        fs::path left = fs::absolute(utf8_path(path), left_error).lexically_normal();
        fs::path right = fs::absolute(utf8_path(protected_path), right_error).lexically_normal();
        if (left_error || right_error) return false;
        std::string left_text = path_utf8(left), right_text = path_utf8(right);
#ifdef _WIN32
        left_text = lowercase_ascii(std::move(left_text));
        right_text = lowercase_ascii(std::move(right_text));
#endif
        return left_text == right_text;
    };
    if (conflicts(runtime.rom_path) || conflicts(runtime.active_path)) {
        if (error.empty()) error = "HD pack output cannot replace the active image or pack";
        return false;
    }
    for (const auto &protected_path : runtime.protected_paths) {
        if (conflicts(protected_path)) {
            if (error.empty()) error = "HD pack output cannot replace active firmware or save data";
            return false;
        }
    }
    return true;
}

struct CaptureTileKey {
    bool ram = false;
    std::uint32_t index = 0;
    std::array<std::uint8_t, 16> bytes{};
    std::uint32_t palette = 0;

    bool operator==(const CaptureTileKey &other) const {
        return ram == other.ram && index == other.index && bytes == other.bytes
            && palette == other.palette;
    }
};

struct CaptureTileHash {
    std::size_t operator()(const CaptureTileKey &key) const {
        std::size_t hash = (static_cast<std::size_t>(key.palette) << 1)
                         ^ static_cast<std::size_t>(key.index) ^ (key.ram ? 0x9e3779b9u : 0u);
        if (key.ram) {
            for (std::uint8_t byte : key.bytes)
                hash = (hash ^ byte) * static_cast<std::size_t>(16777619u);
        }
        return hash;
    }
};

struct CapturedTile {
    CaptureTileKey key;
    std::array<std::uint32_t, 4> rgb{};
    std::array<bool, 4> seen{};
};

static bool collect_capture_tile(const NesHdRuntimeImpl &runtime, const NesVideoTile &tile,
                                 std::unordered_map<CaptureTileKey, std::size_t, CaptureTileHash> &indices,
                                 std::vector<CapturedTile> &tiles, std::string &error) {
    if (!tile.chr.valid) return true;
    if (tile.x >= 8 || tile.y >= 8 || tile.color_index > 3) {
        error = "Completed video trace contains invalid CHR pixel coordinates";
        return false;
    }
    CaptureTileKey key;
    key.ram = tile.chr.ram;
    key.index = tile.chr.index;
    key.palette = tile.palette;
    std::copy_n(tile.chr.bytes, key.bytes.size(), key.bytes.begin());
    if (!key.ram) {
        const std::uint64_t offset = static_cast<std::uint64_t>(key.index) * 16u;
        if (!runtime.chr_is_rom || offset > runtime.chr_copy.size()
            || runtime.chr_copy.size() - static_cast<std::size_t>(offset) < key.bytes.size()) {
            error = "Completed video trace references a CHR-ROM tile outside the attached game";
            return false;
        }
        if (std::memcmp(runtime.chr_copy.data() + static_cast<std::size_t>(offset),
                        key.bytes.data(), key.bytes.size()) != 0) {
            error = "Completed video trace CHR-ROM bytes do not match the attached game";
            return false;
        }
        key.bytes.fill(0);
    }
    auto found = indices.find(key);
    std::size_t index;
    if (found == indices.end()) {
        constexpr std::size_t kMaxCapturedTiles = 32768;
        if (tiles.size() >= kMaxCapturedTiles) {
            error = "Completed video trace contains too many distinct CHR tile and palette combinations";
            return false;
        }
        index = tiles.size();
        indices.emplace(key, index);
        tiles.push_back({key, {}, {}});
    } else {
        index = found->second;
    }
    if (tile.color_index) {
        const unsigned shift = (3u - tile.color_index) * 8u;
        if (((tile.palette >> shift) & 0xffu) != tile.color) {
            error = "Completed video trace palette metadata is inconsistent";
            return false;
        }
        auto &captured = tiles[index];
        const unsigned color = tile.color_index;
        if (captured.seen[color] && captured.rgb[color] != tile.rgb) {
            error = "Completed video trace has inconsistent RGB output for one CHR palette";
            return false;
        }
        captured.rgb[color] = tile.rgb;
        captured.seen[color] = true;
    }
    return true;
}

static std::string capture_key_text(const CaptureTileKey &key) {
    std::ostringstream text;
    text << std::uppercase << std::hex << std::setfill('0');
    if (key.ram) {
        for (std::uint8_t byte : key.bytes) text << std::setw(2) << static_cast<unsigned>(byte);
    } else {
        text << key.index;
    }
    return text.str();
}

static bool build_capture_archive(const NesHdRuntimeImpl &runtime, const NesHdFrameSource &source,
                                  std::vector<std::uint8_t> &archive, std::string &error) {
    if (source.screens < 1 || source.screens > 2) {
        error = "HD pack capture requires one or two completed video traces";
        return false;
    }
    for (unsigned side = 0; side < source.screens; ++side) {
        if (!source.trace[side] || !source.trace[side]->complete || !source.trace[side]->pixels) {
            error = "HD pack capture requires a complete video trace for every screen";
            return false;
        }
    }
    if (source.screens == 2 && source.trace[0]->frame_number != source.trace[1]->frame_number) {
        error = "HD pack capture requires paired dual-screen traces from the same completed frame";
        return false;
    }

    std::unordered_map<CaptureTileKey, std::size_t, CaptureTileHash> indices;
    std::vector<CapturedTile> tiles;
    indices.reserve(4096);
    tiles.reserve(4096);
    for (unsigned side = 0; side < source.screens; ++side) {
        const NesVideoPixel *pixels = source.trace[side]->pixels;
        for (std::size_t i = 0; i < static_cast<std::size_t>(NES_VIDEO_TRACE_WIDTH) * NES_VIDEO_TRACE_HEIGHT; ++i) {
            const auto &pixel = pixels[i];
            if (!collect_capture_tile(runtime, pixel.background, indices, tiles, error)) return false;
            if (pixel.sprite_count > NES_VIDEO_TRACE_SPRITES) {
                error = "Completed video trace contains too many sprite samples for one pixel";
                return false;
            }
            for (unsigned sprite = 0; sprite < pixel.sprite_count; ++sprite)
                if (!collect_capture_tile(runtime, pixel.sprites[sprite], indices, tiles, error)) return false;
        }
    }
    if (tiles.empty()) {
        error = "Completed video trace contains no capturable CHR tiles";
        return false;
    }

    constexpr unsigned kAtlasColumns = 256;
    const unsigned columns = std::min<unsigned>(kAtlasColumns, static_cast<unsigned>(tiles.size()));
    const unsigned rows = static_cast<unsigned>((tiles.size() + columns - 1u) / columns);
    const unsigned width = columns * 8u;
    const unsigned height = rows * 8u;
    std::vector<std::uint32_t> atlas(static_cast<std::size_t>(width) * height, 0u);
    for (std::size_t i = 0; i < tiles.size(); ++i) {
        const CapturedTile &tile = tiles[i];
        const std::uint8_t *bytes = tile.key.ram
            ? tile.key.bytes.data()
            : runtime.chr_copy.data() + static_cast<std::size_t>(tile.key.index) * 16u;
        const unsigned origin_x = static_cast<unsigned>(i % columns) * 8u;
        const unsigned origin_y = static_cast<unsigned>(i / columns) * 8u;
        for (unsigned y = 0; y < 8; ++y) {
            for (unsigned x = 0; x < 8; ++x) {
                const unsigned bit = 7u - x;
                const unsigned color = ((bytes[y] >> bit) & 1u)
                                     | (((bytes[y + 8u] >> bit) & 1u) << 1u);
                if (!color || !tile.seen[color]) continue;
                atlas[static_cast<std::size_t>(origin_y + y) * width + origin_x + x] = tile.rgb[color];
            }
        }
    }

    NesCaptureFrame frame{atlas.data(), width, height, width};
    std::uint8_t *png_data = nullptr;
    std::size_t png_size = 0;
    NesFileResult encoded = nes_capture_png_memory(&frame, &png_data, &png_size);
    if (encoded != NES_FILE_OK) {
        error = std::string("Could not encode captured HD tile atlas: ") + nes_file_result_message(encoded);
        return false;
    }
    std::vector<std::uint8_t> png(png_data, png_data + png_size);
    std::free(png_data);

    std::ostringstream definition;
    definition << "<ver>109\n<scale>1\n";
    if (!runtime.rom_sha1.empty()) definition << "<supportedRom>" << runtime.rom_sha1 << "\n";
    definition << "<img>capture.png\n";
    definition << std::uppercase << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < tiles.size(); ++i) {
        const unsigned x = static_cast<unsigned>(i % columns) * 8u;
        const unsigned y = static_cast<unsigned>(i / columns) * 8u;
        definition << "<tile>0," << capture_key_text(tiles[i].key) << ","
                   << std::setw(8) << tiles[i].key.palette << std::dec
                   << "," << x << "," << y << ",1.0,N\n" << std::hex;
    }
    const std::string text = definition.str();
    if (text.size() > 4u * 1024u * 1024u) {
        error = "Captured HD definition exceeds the version-109 definition size limit";
        return false;
    }

    Pack pack;
    pack.assets.push_back(PackAsset{"hires.txt", AssetKind::Definition,
        std::vector<std::uint8_t>(text.begin(), text.end())});
    pack.assets.push_back(PackAsset{"capture.png", AssetKind::TileImage, std::move(png)});
    if (!cupid::hd::create_archive(pack, archive, error)) return false;
    auto validated = cupid::hd::load_pack_zip_bytes(archive.data(), archive.size());
    if (!validated) {
        error = validated.error.empty() ? "Captured HD pack failed reload validation" : validated.error;
        return false;
    }
    if (!compatible_pack(runtime, *validated.candidate)) {
        error = "Captured HD pack failed attached-game compatibility validation";
        return false;
    }
    std::string renderer_error;
    auto renderer = Renderer::create(std::shared_ptr<Pack>(validated.candidate.release()),
        runtime.chr_is_rom && !runtime.chr_copy.empty() ? runtime.chr_copy.data() : nullptr,
        runtime.chr_is_rom ? runtime.chr_copy.size() : 0, renderer_error);
    if (!renderer) {
        error = renderer_error.empty() ? "Captured HD pack failed renderer validation" : renderer_error;
        return false;
    }
    return true;
}

static bool add_candidate(NesHdRuntimeImpl &runtime, const std::string &path,
                          bool installed, bool strict, std::unordered_set<std::string> &seen,
                          std::string &error) {
    std::string alias = lowercase_ascii(path);
    if (!seen.insert(alias).second) return true;
    auto loaded = load_path(path);
    if (!loaded) {
        if (!strict) return true;
        error = "HD candidate '" + path + "' is invalid: " + loaded.error;
        return false;
    }
    bool compatible = compatible_pack(runtime, *loaded.candidate);
    bool stem_match = lowercase_ascii(candidate_name(path)) == lowercase_ascii(runtime.game_name);
    if (!compatible && !stem_match) return true;
    std::error_code ec;
    bool archive = !fs::is_directory(utf8_path(path), ec) || ec;
    runtime.candidates.push_back({path, candidate_name(path), archive, installed, compatible});
    if (runtime.candidates.size() > NES_HD_MAX_CANDIDATES) {
        error = "HD pack discovery exceeded the candidate limit";
        return false;
    }
    return true;
}

struct ReaderContext { NesHdRuntimeImpl *runtime; unsigned side; };
static std::uint8_t read_memory(void *context, MemorySpace space, std::uint16_t address) {
    auto &reader = *static_cast<ReaderContext *>(context);
    if (!reader.runtime->memory_reader) return 0;
    return reader.runtime->memory_reader(reader.runtime->memory_context, reader.side,
                                         space == MemorySpace::Ppu, address);
}
} // namespace

struct NesHdRuntime { NesHdRuntimeImpl impl; };

extern "C" NesHdRuntime *nes_hd_runtime_create(const char *pack_root,
                                                char *error, size_t error_size) {
    clear_error(error, error_size);
    if (!pack_root || !*pack_root || std::strlen(pack_root) >= NES_FILE_PATH_LIMIT) {
        set_error(error, error_size, "HD pack root is missing or too long");
        return nullptr;
    }
    try {
        auto runtime = std::make_unique<NesHdRuntime>();
        runtime->impl.pack_root = pack_root;
        return runtime.release();
    } catch (const std::bad_alloc &) {
        set_error(error, error_size, "Out of memory while creating the HD runtime");
    }
    return nullptr;
}

extern "C" void nes_hd_runtime_destroy(NesHdRuntime *runtime) {
    if (!runtime) return;
    (void)nes_hd_runtime_enable(runtime, false, nullptr, 0);
    delete runtime;
}

extern "C" bool nes_hd_runtime_set_game(NesHdRuntime *handle, const NesHdGameInfo *game,
                                         char *error, size_t error_size) {
    clear_error(error, error_size);
    if (!handle || !game || !game->rom_path || !*game->rom_path
        || (game->chr_size && !game->chr_rom)) {
        set_error(error, error_size, "HD game information is incomplete");
        return false;
    }
    std::string sha1;
    if (!valid_sha1(game->rom_sha1, sha1)) {
        set_error(error, error_size, "HD game SHA-1 must contain exactly 40 hexadecimal digits");
        return false;
    }
    if (game->chr_is_rom && game->chr_size > 268435456u) {
        set_error(error, error_size, "CHR-ROM exceeds the HD fallback scan limit");
        return false;
    }
    try {
        auto &runtime = handle->impl;
        (void)nes_hd_runtime_enable(handle, false, nullptr, 0);
        runtime.pack.reset();
        runtime.renderer.reset();
        runtime.active_path.clear();
        runtime.candidates.clear();
        runtime.rom_path = game->rom_path;
        runtime.rom_sha1 = std::move(sha1);
        runtime.game_name = game_stem(runtime.rom_path);
        runtime.chr_copy.clear();
        if (game->chr_is_rom && game->chr_size)
            runtime.chr_copy.assign(game->chr_rom, game->chr_rom + game->chr_size);
        runtime.chr_is_rom = game->chr_is_rom;
        runtime.status = runtime.rom_sha1.empty()
            ? "Game attached; SHA-1 unavailable, using name-based HD discovery"
            : "Game attached";
        return true;
    } catch (const std::bad_alloc &) {
        set_error(error, error_size, "Out of memory while attaching the HD game");
        return false;
    }
}

extern "C" void nes_hd_runtime_clear_game(NesHdRuntime *handle) {
    if (!handle) return;
    (void)nes_hd_runtime_enable(handle, false, nullptr, 0);
    auto &runtime = handle->impl;
    runtime.pack.reset(); runtime.renderer.reset(); runtime.candidates.clear(); runtime.output.clear();
    runtime.rom_path.clear(); runtime.rom_sha1.clear(); runtime.game_name.clear(); runtime.chr_copy.clear();
    runtime.active_path.clear(); runtime.status = "No game is attached"; runtime.trace_ready = false;
}

extern "C" void nes_hd_runtime_set_memory_reader(NesHdRuntime *handle,
                                                  NesHdMemoryReader reader, void *context) {
    if (!handle) return;
    handle->impl.memory_reader = reader;
    handle->impl.memory_context = context;
}

extern "C" bool nes_hd_runtime_set_protected_paths(NesHdRuntime *handle,
                                                    const char *const *paths, size_t count,
                                                    char *error, size_t error_size) {
    clear_error(error, error_size);
    if (!handle || (count && !paths) || count > 64) {
        set_error(error, error_size, "HD protected output path list is invalid");
        return false;
    }
    try {
        std::vector<std::string> copy;
        copy.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            if (!paths[i] || !paths[i][0]) continue;
            if (std::strlen(paths[i]) >= NES_FILE_PATH_LIMIT) {
                set_error(error, error_size, "HD protected output path is too long");
                return false;
            }
            copy.emplace_back(paths[i]);
        }
        handle->impl.protected_paths = std::move(copy);
        return true;
    } catch (const std::bad_alloc &) {
        set_error(error, error_size, "Out of memory while setting HD protected output paths");
        return false;
    }
}

extern "C" bool nes_hd_runtime_discover(NesHdRuntime *handle,
                                         char *error, size_t error_size) {
    clear_error(error, error_size);
    if (!handle || handle->impl.game_name.empty()) {
        set_error(error, error_size, "No game is attached for HD pack discovery");
        return false;
    }
    auto &runtime = handle->impl;
    runtime.candidates.clear();
    try {
        std::unordered_set<std::string> seen;
        std::error_code ec;
        fs::path root = utf8_path(runtime.pack_root);
        if (!fs::exists(root, ec)) {
            runtime.status = "No HD pack directory exists yet";
            return true;
        }
        fs::path game_dir = root / utf8_path(runtime.game_name);
        fs::path hires = game_dir / "hires.txt";
        if (fs::is_regular_file(hires, ec) && !ec) {
            std::string problem;
            if (!add_candidate(runtime, path_utf8(game_dir), false, true, seen, problem)) {
                set_error(error, error_size, problem); return false;
            }
        }
        ec.clear();
        fs::path direct_zip = root / utf8_path(runtime.game_name + ".zip");
        if (fs::is_regular_file(direct_zip, ec) && !ec) {
            std::string problem;
            if (!add_candidate(runtime, path_utf8(direct_zip), false, true, seen, problem)) {
                set_error(error, error_size, problem); return false;
            }
        }
        ec.clear();
        if (fs::is_directory(game_dir, ec) && !ec) {
            for (fs::directory_iterator it(game_dir, ec), end; !ec && it != end; it.increment(ec)) {
                if (!it->is_regular_file(ec) || ec) continue;
                if (lowercase_ascii(path_utf8(it->path().extension())) != ".zip") continue;
                std::string problem;
                if (!add_candidate(runtime, path_utf8(it->path()), true, true, seen, problem)) {
                    set_error(error, error_size, problem); return false;
                }
            }
        }
        ec.clear();
        size_t inspected = 0;
        for (fs::directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec)) {
            if (++inspected > 1024) {
                set_error(error, error_size, "HD pack discovery exceeded the directory entry limit");
                return false;
            }
            if (runtime.candidates.size() >= NES_HD_MAX_CANDIDATES) break;
            fs::path candidate;
            bool candidate_ok = false;
            if (it->is_directory(ec) && !ec && fs::is_regular_file(it->path() / "hires.txt", ec) && !ec) {
                candidate = it->path(); candidate_ok = true;
            } else if (!ec && it->is_regular_file(ec) && !ec
                       && lowercase_ascii(path_utf8(it->path().extension())) == ".zip") {
                candidate = it->path(); candidate_ok = true;
            }
            if (!candidate_ok) { ec.clear(); continue; }
            std::string problem;
            if (!add_candidate(runtime, path_utf8(candidate), false, false, seen, problem)) {
                set_error(error, error_size, problem); return false;
            }
        }
        runtime.status = runtime.candidates.empty() ? "No compatible HD packs found" : "HD packs discovered";
        return true;
    } catch (const std::bad_alloc &) {
        set_error(error, error_size, "Out of memory during HD pack discovery");
    } catch (const fs::filesystem_error &exception) {
        set_error(error, error_size, std::string("HD pack discovery failed: ") + exception.what());
    }
    return false;
}

extern "C" size_t nes_hd_runtime_candidate_count(const NesHdRuntime *handle) {
    return handle ? handle->impl.candidates.size() : 0;
}

extern "C" bool nes_hd_runtime_candidate_at(const NesHdRuntime *handle, size_t index,
                                             NesHdPackCandidateInfo *info) {
    if (!handle || !info || index >= handle->impl.candidates.size()) return false;
    const auto &candidate = handle->impl.candidates[index];
    *info = {candidate.path.c_str(), candidate.name.c_str(), candidate.archive,
             candidate.installed, candidate.compatible,
             candidate.path == handle->impl.active_path};
    return true;
}

extern "C" bool nes_hd_runtime_load(NesHdRuntime *handle, const char *path,
                                     char *error, size_t error_size) {
    clear_error(error, error_size);
    if (!handle || !path || !*path || handle->impl.game_name.empty()) {
        set_error(error, error_size, "HD pack load requires an attached game and a pack path");
        return false;
    }
    try {
        PreparedPack prepared;
        std::string problem;
        if (!prepare_pack(handle->impl, path, prepared, problem)
            || !commit_pack(handle->impl, std::move(prepared), path, problem)) {
            set_error(error, error_size, problem); return false;
        }
        return true;
    } catch (const std::bad_alloc &) {
        set_error(error, error_size, "Out of memory while switching HD packs");
        return false;
    }
}

extern "C" bool nes_hd_runtime_switch(NesHdRuntime *handle, size_t candidate_index,
                                       char *error, size_t error_size) {
    if (!handle || candidate_index >= handle->impl.candidates.size()) {
        set_error(error, error_size, "HD pack selection is unavailable");
        return false;
    }
    return nes_hd_runtime_load(handle, handle->impl.candidates[candidate_index].path.c_str(), error, error_size);
}

extern "C" void nes_hd_runtime_reset_audio(NesHdRuntime *handle) {
    if (handle) stop_audio(handle->impl);
}

extern "C" bool nes_hd_runtime_enable(NesHdRuntime *handle, bool enabled,
                                       char *error, size_t error_size) {
    clear_error(error, error_size);
    if (!handle) { set_error(error, error_size, "HD runtime is unavailable"); return false; }
    auto &runtime = handle->impl;
    if (enabled == runtime.enabled) return true;
    if (enabled) {
        if (!runtime.pack || !runtime.renderer) {
            set_error(error, error_size, "No validated HD pack is loaded"); return false;
        }
        std::string problem;
        if (!attach_host_hooks(runtime, problem)) { set_error(error, error_size, problem); return false; }
        if (!nes_video_trace_use(NES_VIDEO_TRACE_HD, true)) {
            detach_host_hooks(runtime);
            set_error(error, error_size, "Video trace buffers could not be allocated for HD rendering");
            return false;
        }
        runtime.enabled = true;
        runtime.status = "HD pack enabled";
    } else {
        if (runtime.enabled) (void)nes_video_trace_use(NES_VIDEO_TRACE_HD, false);
        runtime.enabled = false;
        runtime.trace_ready = false;
        detach_host_hooks(runtime);
        runtime.status = runtime.pack ? "HD pack disabled" : "No HD pack loaded";
    }
    return true;
}

extern "C" bool nes_hd_runtime_install(NesHdRuntime *handle, const char *source_path,
                                        const char *install_name,
                                        char *installed_path, size_t installed_path_size,
                                        char *error, size_t error_size) {
    clear_error(error, error_size);
    if (installed_path && installed_path_size) installed_path[0] = '\0';
    if (!handle || !source_path || !*source_path || handle->impl.game_name.empty()) {
        set_error(error, error_size, "HD pack install requires an attached game and source path");
        return false;
    }
    std::string filename;
    std::string default_name = install_name && *install_name ? std::string() : candidate_name(source_path);
    if (!safe_install_name(install_name && *install_name ? install_name : default_name.c_str(), filename)) {
        set_error(error, error_size, "HD pack install name is unsafe"); return false;
    }
    try {
        PreparedPack prepared;
        std::string problem;
        if (!prepare_pack(handle->impl, source_path, prepared, problem)) {
            set_error(error, error_size, problem); return false;
        }
        std::vector<std::uint8_t> archive;
        if (!cupid::hd::create_archive(*prepared.pack, archive, problem)) {
            set_error(error, error_size, problem); return false;
        }
        fs::path directory = utf8_path(handle->impl.pack_root) / utf8_path(handle->impl.game_name);
        std::error_code ec;
        fs::create_directories(directory, ec);
        if (ec) { set_error(error, error_size, "Could not create the managed HD pack directory"); return false; }
        std::string destination = path_utf8(directory / utf8_path(filename));
        if (installed_path && installed_path_size && destination.size() >= installed_path_size) {
            set_error(error, error_size, "Installed HD pack path does not fit the caller buffer");
            return false;
        }
        NesFileResult written = nes_file_write_atomic(destination.c_str(), archive.data(), archive.size());
        if (written != NES_FILE_OK) {
            set_error(error, error_size, std::string("Could not install HD pack: ") + nes_file_result_message(written));
            return false;
        }
        if (!commit_pack(handle->impl, std::move(prepared), destination, problem)) {
            set_error(error, error_size, problem); return false;
        }
        if (installed_path && installed_path_size) {
            std::memcpy(installed_path, destination.c_str(), destination.size() + 1);
        }
        (void)nes_hd_runtime_discover(handle, nullptr, 0);
        return true;
    } catch (const std::bad_alloc &) {
        set_error(error, error_size, "Out of memory while installing the HD pack");
    } catch (const fs::filesystem_error &exception) {
        set_error(error, error_size, std::string("HD pack install failed: ") + exception.what());
    }
    return false;
}

extern "C" bool nes_hd_runtime_export(const NesHdRuntime *handle, const char *zip_path,
                                       char *error, size_t error_size) {
    clear_error(error, error_size);
    if (!handle || !handle->impl.pack || !zip_path || !*zip_path) {
        set_error(error, error_size, "HD pack export requires a loaded pack and output path"); return false;
    }
    try {
        std::vector<std::uint8_t> archive;
        std::string problem;
        if (!output_path_allowed(handle->impl, zip_path, problem)) {
            set_error(error, error_size, problem);
            return false;
        }
        if (!cupid::hd::create_archive(*handle->impl.pack, archive, problem)) {
            set_error(error, error_size, problem); return false;
        }
        NesFileResult result = nes_file_write_atomic(zip_path, archive.data(), archive.size());
        if (result != NES_FILE_OK) {
            set_error(error, error_size, std::string("Could not export HD pack: ") + nes_file_result_message(result));
            return false;
        }
        return true;
    } catch (const std::bad_alloc &) {
        set_error(error, error_size, "Out of memory while exporting the HD pack");
        return false;
    }
}

extern "C" bool nes_hd_runtime_capture(const NesHdRuntime *handle,
                                        const NesHdFrameSource *source,
                                        const char *zip_path,
                                        char *error, size_t error_size) {
    clear_error(error, error_size);
    if (!handle || !source || handle->impl.game_name.empty()) {
        set_error(error, error_size, "HD pack capture requires an attached game and frame source");
        return false;
    }
    try {
        std::string problem;
        if (!output_path_allowed(handle->impl, zip_path, problem)) {
            set_error(error, error_size, problem);
            return false;
        }
        std::vector<std::uint8_t> archive;
        if (!build_capture_archive(handle->impl, *source, archive, problem)) {
            set_error(error, error_size, problem);
            return false;
        }
        NesFileResult result = nes_file_write_atomic(zip_path, archive.data(), archive.size());
        if (result != NES_FILE_OK) {
            set_error(error, error_size,
                      std::string("Could not write captured HD pack: ") + nes_file_result_message(result));
            return false;
        }
        return true;
    } catch (const std::bad_alloc &) {
        set_error(error, error_size, "Out of memory while capturing the HD pack");
    } catch (const fs::filesystem_error &exception) {
        set_error(error, error_size, std::string("HD pack capture failed: ") + exception.what());
    }
    return false;
}

extern "C" bool nes_hd_runtime_render(NesHdRuntime *handle, const NesHdFrameSource *source,
                                       NesHdFrame *frame, char *error, size_t error_size) {
    clear_error(error, error_size);
    if (!handle || !source || !frame || !source->fallback_pixels || !source->fallback_width
        || !source->fallback_height || source->screens < 1 || source->screens > 2) {
        set_error(error, error_size, "HD frame source is incomplete"); return false;
    }
    auto &runtime = handle->impl;
    *frame = {source->fallback_pixels, source->fallback_width, source->fallback_height,
              source->screens, 1, source->overscan, false, false};
    runtime.trace_ready = false;
    if (!runtime.enabled || !runtime.renderer) return true;
    bool complete = true;
    for (unsigned side = 0; side < source->screens; ++side)
        complete = complete && source->trace[side] && source->trace[side]->complete && source->trace[side]->pixels;
    frame->trace_ready = runtime.trace_ready = complete;
    if (!complete) {
        runtime.status = "HD trace is warming up; showing raw presentation";
        return true;
    }
    try {
        std::array<RenderedFrame, 2> rendered;
        RenderSettings settings;
        settings.background = source->show_background;
        settings.sprites = source->show_sprites;
        settings.use_pack_overscan = source->allow_pack_overscan;
        settings.overscan = {source->overscan.top, source->overscan.right,
                             source->overscan.bottom, source->overscan.left};
        std::string problem;
        for (unsigned side = 0; side < source->screens; ++side) {
            ReaderContext reader{&runtime, side};
            auto memory = runtime.memory_reader ? read_memory : nullptr;
            if (!runtime.renderer->render(*source->trace[side], settings, memory, &reader,
                                          rendered[side], problem)) {
                set_error(error, error_size, problem); runtime.status = problem; return false;
            }
        }
        for (unsigned side = 1; side < source->screens; ++side) {
            if (rendered[side].width != rendered[0].width || rendered[side].height != rendered[0].height
                || rendered[side].scale != rendered[0].scale) {
                set_error(error, error_size, "HD dual-screen sides produced incompatible dimensions"); return false;
            }
        }
        const unsigned side_width = rendered[0].width;
        const unsigned height = rendered[0].height;
        const unsigned width = side_width * source->screens;
        runtime.output.resize(static_cast<std::size_t>(width) * height);
        for (unsigned y = 0; y < height; ++y) {
            for (unsigned side = 0; side < source->screens; ++side) {
                std::copy_n(rendered[side].pixels.data() + static_cast<std::size_t>(y) * side_width,
                            side_width, runtime.output.data() + static_cast<std::size_t>(y) * width + side * side_width);
            }
        }
        const auto &crop = rendered[0].overscan;
        *frame = {runtime.output.data(), width, height, source->screens, rendered[0].scale,
                  {crop.left, crop.right, crop.top, crop.bottom}, true, true};
        runtime.status = "HD pack rendered";
        return true;
    } catch (const std::bad_alloc &) {
        set_error(error, error_size, "Out of memory while rendering the HD frame");
        return false;
    }
}

extern "C" bool nes_hd_runtime_info(const NesHdRuntime *handle, NesHdRuntimeInfo *info) {
    if (!handle || !info) return false;
    const auto &runtime = handle->impl;
    bool audio = runtime.bgm.track != nullptr
        || std::any_of(runtime.sfx.begin(), runtime.sfx.end(), [](const Voice &voice) { return voice.track != nullptr; });
    *info = {!runtime.game_name.empty(), runtime.pack != nullptr, runtime.enabled,
             runtime.trace_ready, audio, alternate_registers(runtime), runtime.candidates.size(),
             runtime.active_path.empty() ? nullptr : runtime.active_path.c_str(), runtime.status.c_str()};
    return true;
}
