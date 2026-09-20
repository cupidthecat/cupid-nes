/*
 * hd_render_internal.hpp - Frame-local rule evaluation and indexed tile rules
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_HD_RENDER_INTERNAL_HPP
#define CUPID_HD_RENDER_INTERNAL_HPP

#include "hd_renderer.hpp"
#include <array>
#include <unordered_map>

namespace cupid::hd {

struct TileIdentity {
    bool ram = false;
    std::uint32_t index = 0;
    std::array<std::uint8_t, 16> bytes{};
    explicit TileIdentity(const TileKey &key);
    explicit TileIdentity(const NesVideoTile &tile);
    bool operator==(const TileIdentity &other) const;
};

struct TileHash {
    std::size_t operator()(const TileIdentity &key) const;
};

struct RuleBucket {
    std::unordered_map<std::uint32_t, std::vector<std::size_t>> palettes;
    std::vector<std::size_t> defaults;
};

struct Renderer::Impl {
    std::shared_ptr<const Pack> pack;
    std::unordered_map<TileIdentity, RuleBucket, TileHash> tiles;
    std::unordered_map<TileIdentity, std::vector<std::size_t>, TileHash> additions;
    std::unordered_map<std::uint32_t, std::uint32_t> fallback;
    std::vector<NesVideoPixel> added_pixels;
};

class FrameRules {
public:
    FrameRules(const Pack &pack, const NesVideoTraceFrame &frame,
               const std::unordered_map<std::uint32_t, std::uint32_t> &fallback,
               MemoryReader reader, void *context);
    bool consume(std::uint64_t work = 1);
    bool conditions(const std::vector<ConditionRef> &references, int x, int y, const NesVideoTile *tile);
    bool matches(const TileKey &key, const NesVideoTile &tile, bool ignore_palette) const;
    void set_pixels(const NesVideoPixel *pixels);
    bool ok() const;
    const char *error() const;
    void fail(const char *message);

private:
    bool condition(ConditionRef reference, int x, int y, const NesVideoTile *tile);
    std::uint8_t memory(MemorySpace space, std::uint32_t address);
    bool tile_at(std::int64_t index, const Condition &condition, bool sprite) const;
    const Pack &pack_;
    const NesVideoTraceFrame &frame_;
    const NesVideoPixel *pixels_;
    const std::unordered_map<std::uint32_t, std::uint32_t> &fallback_;
    MemoryReader reader_;
    void *context_;
    std::unordered_map<std::uint32_t, std::uint8_t> memory_cache_;
    std::vector<std::int8_t> conditions_cache_;
    std::uint64_t remaining_work_ = 32000000;
    const char *error_ = nullptr;
};

/* Asset adjustment includes the format's integer alpha premultiplication.
 * blend_asset accepts the resulting premultiplied source value. */
std::uint32_t adjust_asset(std::uint32_t pixel, std::int32_t brightness, std::uint8_t mask);
std::uint32_t blend_asset(std::uint32_t destination, std::uint32_t source, BlendMode mode);

} // namespace cupid::hd
#endif
