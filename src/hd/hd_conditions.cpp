/*
 * hd_conditions.cpp - Version 109 tile predicates and frame-local memory reads
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "hd_render_internal.hpp"
#include <algorithm>
#include <cstring>

namespace cupid::hd {

TileIdentity::TileIdentity(const TileKey &key) : ram(key.source == TileSource::ChrRam) {
    if (ram) bytes = key.chr_ram;
    else index = key.chr_rom_index;
}

TileIdentity::TileIdentity(const NesVideoTile &tile) : ram(tile.chr.ram) {
    if (ram) std::copy_n(tile.chr.bytes, bytes.size(), bytes.begin());
    else index = tile.chr.index;
}

bool TileIdentity::operator==(const TileIdentity &other) const {
    return ram == other.ram && (ram ? bytes == other.bytes : index == other.index);
}

std::size_t TileHash::operator()(const TileIdentity &key) const {
    std::uint64_t hash = 1469598103934665603ull;
    if (key.ram) {
        for (auto byte : key.bytes) hash = (hash ^ byte) * 1099511628211ull;
    } else {
        hash ^= key.index;
        hash *= 1099511628211ull;
    }
    return static_cast<std::size_t>(hash ^ static_cast<unsigned>(key.ram));
}

FrameRules::FrameRules(const Pack &pack, const NesVideoTraceFrame &frame,
                       const std::unordered_map<std::uint32_t, std::uint32_t> &fallback,
                       MemoryReader reader, void *context)
    : pack_(pack), frame_(frame), pixels_(frame.pixels), fallback_(fallback),
      reader_(reader), context_(context), conditions_cache_(pack.conditions.size(), -1) {}

void FrameRules::set_pixels(const NesVideoPixel *pixels) { pixels_ = pixels; }

bool FrameRules::ok() const { return error_ == nullptr; }

const char *FrameRules::error() const { return error_ ? error_ : ""; }

void FrameRules::fail(const char *message) {
    if (!error_) error_ = message;
}

bool FrameRules::consume(std::uint64_t work) {
    if (work > remaining_work_) {
        fail("HD pack exceeded the per-frame rendering work limit");
        return false;
    }
    remaining_work_ -= work;
    return true;
}

static bool compare(std::uint32_t a, std::uint32_t b, CompareOperator operation) {
    switch (operation) {
        case CompareOperator::Equal: return a == b;
        case CompareOperator::NotEqual: return a != b;
        case CompareOperator::GreaterThan: return a > b;
        case CompareOperator::LessThan: return a < b;
        case CompareOperator::LessThanOrEqual: return a <= b;
        case CompareOperator::GreaterThanOrEqual: return a >= b;
    }
    return false;
}

std::uint8_t FrameRules::memory(MemorySpace space, std::uint32_t address) {
    if (address > (space == MemorySpace::Cpu ? 0xFFFFu : 0x3FFFu) || !reader_) {
        fail("HD memory condition requires a valid side-effect-free memory reader");
        return 0;
    }
    std::uint32_t key = address | (space == MemorySpace::Ppu ? 0x10000u : 0);
    auto found = memory_cache_.find(key);
    if (found != memory_cache_.end()) return found->second;
    std::uint8_t value = reader_(context_, space, static_cast<std::uint16_t>(address));
    memory_cache_.emplace(key, value);
    return value;
}

bool FrameRules::matches(const TileKey &key, const NesVideoTile &tile, bool ignore_palette) const {
    if (!tile.chr.valid || (!ignore_palette && key.palette != tile.palette)) return false;
    if ((key.source == TileSource::ChrRam) != tile.chr.ram) return false;
    if (tile.chr.ram) return std::equal(key.chr_ram.begin(), key.chr_ram.end(), tile.chr.bytes);
    if (key.chr_rom_index == tile.chr.index) return true;
    auto fallback = fallback_.find(tile.chr.index);
    return fallback != fallback_.end() && fallback->second == key.chr_rom_index;
}

bool FrameRules::tile_at(std::int64_t index, const Condition &condition, bool sprite) const {
    if (index < 0 || index >= NES_VIDEO_TRACE_WIDTH * NES_VIDEO_TRACE_HEIGHT) return false;
    const NesVideoPixel &pixel = pixels_[index];
    if (!sprite) return matches(condition.tile, pixel.background, condition.ignore_palette);
    for (unsigned i = 0; i < std::min<unsigned>(pixel.sprite_count, NES_VIDEO_TRACE_SPRITES); ++i)
        if (matches(condition.tile, pixel.sprites[i], condition.ignore_palette)) return true;
    return false;
}

bool FrameRules::condition(ConditionRef reference, int x, int y, const NesVideoTile *tile) {
    if (!consume()) return false;
    if (reference >= pack_.conditions.size()) {
        fail("HD rule refers to an undefined condition");
        return false;
    }
    if (conditions_cache_[reference] >= 0) return conditions_cache_[reference] != 0;
    const Condition &condition = pack_.conditions[reference];
    bool result = false, cache = false;
    switch (condition.type) {
        case ConditionType::HorizontalMirror: result = tile && (tile->attributes & 0x40); break;
        case ConditionType::VerticalMirror: result = tile && (tile->attributes & 0x80); break;
        case ConditionType::BackgroundPriority: result = tile && (tile->attributes & 0x20); break;
        case ConditionType::SpritePalette:
            result = tile && tile->palette_offset == 0x10u + condition.sprite_palette * 4u;
            break;
        case ConditionType::FrameRange:
            if (!condition.operand_a) {
                fail("HD frame range has a zero period");
                return false;
            }
            result = frame_.frame_number % condition.operand_a >= condition.operand_b;
            cache = true;
            break;
        case ConditionType::MemoryCheck:
        case ConditionType::MemoryCheckConstant: {
            auto a = memory(condition.memory_space, condition.operand_a) & condition.mask;
            if (!ok()) return false;
            std::uint32_t b = condition.type == ConditionType::MemoryCheck
                ? memory(condition.memory_space, condition.operand_b) & condition.mask : condition.operand_b;
            if (!ok()) return false;
            result = compare(a, b, condition.comparison);
            cache = true;
            break;
        }
        case ConditionType::PositionCheckX:
            result = compare(static_cast<std::uint32_t>(x), condition.operand_b, condition.comparison);
            break;
        case ConditionType::PositionCheckY:
            result = compare(static_cast<std::uint32_t>(y), condition.operand_b, condition.comparison);
            break;
        case ConditionType::OriginPositionCheckX:
            if (tile) result = compare(static_cast<std::uint32_t>(x - tile->x), condition.operand_b, condition.comparison);
            break;
        case ConditionType::OriginPositionCheckY:
            if (tile) result = compare(static_cast<std::uint32_t>(y - ((tile->attributes & 0x80) ? 7 - tile->y : tile->y)),
                                       condition.operand_b, condition.comparison);
            break;
        case ConditionType::TileAtPosition:
        case ConditionType::SpriteAtPosition:
            result = tile_at(static_cast<std::int64_t>(condition.y) * 256 + condition.x,
                             condition, condition.type == ConditionType::SpriteAtPosition);
            break;
        case ConditionType::TileNearby:
            result = tile_at((static_cast<std::int64_t>(y) + condition.y) * 256 + x + condition.x, condition, false);
            break;
        case ConditionType::SpriteNearby: {
            std::int64_t dx = tile && (tile->attributes & 0x40) ? -static_cast<std::int64_t>(condition.x) : condition.x;
            std::int64_t dy = tile && (tile->attributes & 0x80) ? -static_cast<std::int64_t>(condition.y) : condition.y;
            result = tile_at((y + dy) * 256 + x + dx, condition, true);
            break;
        }
    }
    result = result != condition.inverted;
    if (cache) conditions_cache_[reference] = result ? 1 : 0;
    return result;
}

bool FrameRules::conditions(const std::vector<ConditionRef> &references, int x, int y, const NesVideoTile *tile) {
    for (auto reference : references) if (!condition(reference, x, y, tile)) return false;
    return true;
}

std::uint32_t adjust_asset(std::uint32_t pixel, std::int32_t brightness, std::uint8_t mask) {
    unsigned alpha = pixel >> 24;
    unsigned components[3] = {(pixel >> 16) & 255u, (pixel >> 8) & 255u, pixel & 255u};
    for (auto &component : components) {
        if (alpha != 255) component = ((alpha + 1u) * component) >> 8;
        if (brightness != 255) component = static_cast<unsigned>(std::min<std::uint64_t>(255,
            (static_cast<std::uint64_t>(brightness) * (component + 1)) >> 8));
    }
    if (mask & 1) {
        unsigned average = (components[0] + components[1] + components[2]) / 3;
        components[0] = components[1] = components[2] = average;
    }
    for (unsigned channel = 0; channel < 3; ++channel) {
        unsigned numerator = 1000;
        for (unsigned emphasis = 0; emphasis < 3; ++emphasis)
            if (mask & (0x20u << emphasis)) numerator = numerator * (channel == emphasis ? 11u : 9u) / 10u;
        components[channel] = std::min(255u, components[channel] * numerator / 1000u);
    }
    return (alpha << 24) | (components[0] << 16) | (components[1] << 8) | components[2];
}

std::uint32_t blend_asset(std::uint32_t destination, std::uint32_t source, BlendMode mode) {
    unsigned alpha = source >> 24;
    if (!alpha) return destination;
    if (alpha == 255 && mode == BlendMode::Alpha) return source;
    std::uint32_t output = 0xFF000000;
    for (unsigned shift = 0; shift <= 16; shift += 8) {
        unsigned incoming = (source >> shift) & 255u;
        unsigned original = (destination >> shift) & 255u;
        unsigned result;
        if (mode == BlendMode::Add) result = std::min(255u, original + incoming);
        else if (mode == BlendMode::Subtract) result = original > incoming ? original - incoming : 0;
        else result = incoming + (((256u - alpha) * original) >> 8);
        output |= std::min(255u, result) << shift;
    }
    return output;
}

} // namespace cupid::hd
