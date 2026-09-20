/*
 * hd_pack_model.hpp - renderer-facing HD pack data model
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_NES_HD_PACK_MODEL_HPP
#define CUPID_NES_HD_PACK_MODEL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace cupid::hd {

inline constexpr std::uint32_t kSupportedPackVersion = 109;
inline constexpr std::uint32_t kBackgroundPriorityCount = 40;

enum class TileSource : std::uint8_t {
    ChrRom,
    ChrRam,
};

struct TileKey {
    TileSource source = TileSource::ChrRom;
    std::uint32_t palette = 0;
    std::uint32_t chr_rom_index = 0;
    std::array<std::uint8_t, 16> chr_ram{};
};

enum class ConditionType : std::uint8_t {
    HorizontalMirror,
    VerticalMirror,
    BackgroundPriority,
    FrameRange,
    MemoryCheck,
    MemoryCheckConstant,
    TileNearby,
    TileAtPosition,
    SpriteAtPosition,
    SpriteNearby,
    SpritePalette,
    PositionCheckX,
    PositionCheckY,
    OriginPositionCheckX,
    OriginPositionCheckY,
};

enum class CompareOperator : std::uint8_t {
    Equal,
    NotEqual,
    GreaterThan,
    LessThan,
    LessThanOrEqual,
    GreaterThanOrEqual,
};

enum class MemorySpace : std::uint8_t {
    Cpu,
    Ppu,
};

struct Condition {
    std::string name;
    ConditionType type = ConditionType::HorizontalMirror;
    bool inverted = false;

    // Tile/sprite position and nearby conditions.
    std::int32_t x = 0;
    std::int32_t y = 0;
    TileKey tile{};
    bool ignore_palette = false;

    // Memory, frame-range, and position conditions.
    MemorySpace memory_space = MemorySpace::Cpu;
    CompareOperator comparison = CompareOperator::Equal;
    std::uint32_t operand_a = 0;
    std::uint32_t operand_b = 0;
    std::uint8_t mask = 0xff;

    // Built-in sprite-palette conditions use values 0..3.
    std::uint8_t sprite_palette = 0;
};

using ConditionRef = std::size_t;

struct Image {
    std::string path;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    // 0xAARRGGBB, straight alpha.
    std::vector<std::uint32_t> argb32;
};

struct TileRule {
    TileKey key{};
    std::size_t bitmap_index = 0;
    std::uint32_t x = 0;
    std::uint32_t y = 0;
    std::int32_t brightness = 255;
    bool default_tile = false;
    std::vector<ConditionRef> conditions;

    // Version 109 keeps these mapping hints for CHR-RAM definitions. They are
    // metadata only; matching is still performed with the 16-byte CHR-RAM key.
    std::uint32_t chr_bank_id = 0;
    std::int32_t runtime_tile_index = -1;
};

enum class BlendMode : std::uint8_t {
    Alpha,
    Add,
    Subtract,
};

struct Background {
    std::size_t image_index = 0;
    std::int32_t brightness = 255;
    float horizontal_scroll_ratio = 0.0f;
    float vertical_scroll_ratio = 0.0f;
    std::uint8_t priority = 10;
    std::uint32_t left = 0;
    std::uint32_t top = 0;
    BlendMode blend_mode = BlendMode::Alpha;
    std::vector<ConditionRef> conditions;
};

struct Addition {
    TileKey original{};
    std::int32_t offset_x = 0;
    std::int32_t offset_y = 0;
    TileKey additional{};
    bool ignore_palette = false;
};

struct Fallback {
    std::uint32_t tile_index = 0;
    std::uint32_t fallback_tile_index = 0;
};

struct Overscan {
    std::uint32_t top = 0;
    std::uint32_t right = 0;
    std::uint32_t bottom = 0;
    std::uint32_t left = 0;
};

enum class RenderOption : std::uint32_t {
    None = 0,
    DisableCache = 1u << 0,
    DisableOriginalTiles = 1u << 1,
    AutomaticFallbackTiles = 1u << 2,
};

enum class HardwareOption : std::uint32_t {
    None = 0,
    DisableSpriteLimit = 1u << 0,
    AlternateRegisterRange = 1u << 1,
};

constexpr RenderOption operator|(RenderOption left, RenderOption right) {
    return static_cast<RenderOption>(static_cast<std::uint32_t>(left)
                                   | static_cast<std::uint32_t>(right));
}

constexpr RenderOption &operator|=(RenderOption &left, RenderOption right) {
    left = left | right;
    return left;
}

constexpr HardwareOption operator|(HardwareOption left, HardwareOption right) {
    return static_cast<HardwareOption>(static_cast<std::uint32_t>(left)
                                     | static_cast<std::uint32_t>(right));
}

constexpr HardwareOption &operator|=(HardwareOption &left, HardwareOption right) {
    left = left | right;
    return left;
}

enum class AudioRole : std::uint8_t {
    BackgroundMusic,
    SoundEffect,
};

struct AudioTrack {
    AudioRole role = AudioRole::BackgroundMusic;
    std::uint8_t album = 0;
    std::uint8_t track = 0;
    std::string path;
    std::uint32_t sample_rate = 0;
    std::uint8_t channels = 0;
    // Interleaved signed 16-bit PCM in source channel order.
    std::vector<std::int16_t> pcm;
    // Sample-frame offset used when background music reaches end-of-stream.
    std::uint32_t loop_position = 0;
};

enum class AssetKind : std::uint8_t {
    Definition,
    TileImage,
    BackgroundImage,
    Audio,
    Palette,
};

struct PackAsset {
    std::string path;
    AssetKind kind = AssetKind::Definition;
    std::vector<std::uint8_t> bytes;
};

struct Pack {
    std::uint32_t version = kSupportedPackVersion;
    std::uint32_t scale = 1;
    std::vector<std::string> supported_rom_sha1;

    // <img> numbering indexes this vector only.
    std::vector<Image> tile_images;
    // Backgrounds deduplicate their files independently of <img> numbering.
    std::vector<Image> background_images;
    std::vector<TileRule> tiles;
    std::vector<Condition> conditions;
    std::vector<Background> backgrounds;
    std::vector<Addition> additions;
    std::vector<Fallback> fallbacks;
    std::vector<AudioTrack> audio_tracks;

    std::optional<Overscan> overscan;
    RenderOption render_options = RenderOption::None;
    // These are reported to the caller for compatibility information. Loading
    // a pack never applies them to the emulated hardware.
    HardwareOption hardware_options = HardwareOption::None;
    std::optional<std::array<std::uint32_t, 64>> palette;

    // Exact validated source bytes used by create_archive(). Paths are
    // normalized relative UTF-8 pack member names and include hires.txt.
    std::vector<PackAsset> assets;
};

}  // namespace cupid::hd

#endif
