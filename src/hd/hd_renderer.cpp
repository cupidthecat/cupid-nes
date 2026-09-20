/*
 * hd_renderer.cpp - Tile replacement, conditions and ordered background layers
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "hd_render_internal.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace cupid::hd {

static bool option(RenderOption options, RenderOption flag) {
    return (static_cast<std::uint32_t>(options) & static_cast<std::uint32_t>(flag)) != 0;
}

static bool check(bool valid, std::string &error, const char *message) {
    if (valid) return true;
    error = message;
    return false;
}

static bool validate_image(const Image &image, std::string &error) {
    return check(image.width && image.height && image.width <= 8192 && image.height <= 8192,
                 error, "HD image dimensions exceed the supported limit")
        && check(static_cast<std::uint64_t>(image.width) * image.height == image.argb32.size(),
                 error, "HD decoded image does not match its dimensions");
}

static bool validate_pack(const Pack &pack, std::string &error) {
    if (!check(pack.version == kSupportedPackVersion, error, "Unsupported HD pack format version")
        || !check(pack.scale >= 1 && pack.scale <= 10, error, "HD scale must be from 1 through 10")
        || !check(pack.tiles.size() <= 131072 && pack.conditions.size() <= 8192
                  && pack.backgrounds.size() <= 1024 && pack.additions.size() <= 4096
                  && pack.fallbacks.size() <= 131072, error,
                  "HD definition exceeds the rendering rule limit")) return false;
    std::uint64_t image_pixels = 0;
    for (const auto &image : pack.tile_images) {
        if (!validate_image(image, error)) return false;
        image_pixels += image.argb32.size();
    }
    for (const auto &image : pack.background_images) {
        if (!validate_image(image, error)) return false;
        image_pixels += image.argb32.size();
    }
    if (!check(image_pixels <= 67108864, error, "HD decoded images exceed the rendering memory limit"))
        return false;
    auto conditions = [&](const std::vector<ConditionRef> &references) -> bool {
        if (!check(references.size() <= 64, error, "HD rule has too many conditions")) return false;
        for (auto ref : references)
            if (!check(ref < pack.conditions.size(), error, "HD rule has an undefined condition")) return false;
        return true;
    };
    for (const auto &tile : pack.tiles) {
        if (!check(tile.bitmap_index < pack.tile_images.size(), error,
                   "HD tile has an invalid bitmap index")) return false;
        const auto &image = pack.tile_images[tile.bitmap_index];
        if (!check(tile.x <= image.width && tile.y <= image.height
                   && 8 * pack.scale <= image.width - tile.x && 8 * pack.scale <= image.height - tile.y,
                   error, "HD replacement tile extends beyond its image")
            || !check(tile.brightness >= 0 && tile.brightness <= 25500, error,
                      "HD brightness is outside the supported range")
            || !conditions(tile.conditions)) return false;
    }
    for (const auto &background : pack.backgrounds) {
        if (!check(background.image_index < pack.background_images.size(), error,
                   "HD background has an invalid image index")
            || !check(background.priority < kBackgroundPriorityCount && background.brightness >= 0
                      && background.brightness <= 25500, error,
                      "Invalid HD background priority or brightness")
            || !check(std::isfinite(background.horizontal_scroll_ratio)
                      && std::isfinite(background.vertical_scroll_ratio)
                      && std::abs(background.horizontal_scroll_ratio) <= 1000
                      && std::abs(background.vertical_scroll_ratio) <= 1000, error,
                      "Invalid HD background scroll ratio")
            || !conditions(background.conditions)) return false;
    }
    return true;
}

static bool build_fallbacks(Renderer::Impl &impl, const std::uint8_t *chr_rom,
                            std::size_t chr_size, std::string &error) {
    if (chr_rom && chr_size && option(impl.pack->render_options, RenderOption::AutomaticFallbackTiles)) {
        if (!check(chr_size <= 268435456, error, "CHR data exceeds the automatic fallback scan limit"))
            return false;
        std::unordered_map<TileIdentity, std::uint32_t, TileHash> used;
        for (const auto &tile : impl.pack->tiles) {
            if (tile.key.source != TileSource::ChrRom) continue;
            std::uint64_t offset = static_cast<std::uint64_t>(tile.key.chr_rom_index) * 16;
            if (offset > chr_size || chr_size - offset < 16) continue;
            TileKey key;
            key.source = TileSource::ChrRam;
            std::copy_n(chr_rom + offset, 16, key.chr_ram.begin());
            used.emplace(TileIdentity(key), tile.key.chr_rom_index);
        }
        for (std::size_t offset = 0; offset + 16 <= chr_size; offset += 16) {
            TileKey key;
            key.source = TileSource::ChrRam;
            std::copy_n(chr_rom + offset, 16, key.chr_ram.begin());
            if (std::all_of(key.chr_ram.begin(), key.chr_ram.end(), [](std::uint8_t value) { return value == 0; })) continue;
            auto found = used.find(TileIdentity(key));
            if (found != used.end() && found->second != offset / 16)
                impl.fallback[static_cast<std::uint32_t>(offset / 16)] = found->second;
        }
    }
    for (const auto &entry : impl.pack->fallbacks) impl.fallback[entry.tile_index] = entry.fallback_tile_index;
    return true;
}

std::unique_ptr<Renderer> Renderer::create(std::shared_ptr<const Pack> pack,
                                          const std::uint8_t *chr_rom, std::size_t chr_size,
                                          std::string &error) {
    error.clear();
    if (!check(pack != nullptr, error, "No HD pack was supplied")) return nullptr;
    if (!validate_pack(*pack, error)) return nullptr;
    try {
        auto impl = std::make_unique<Impl>();
        impl->pack = std::move(pack);
        for (std::size_t i = 0; i < impl->pack->tiles.size(); ++i) {
            const auto &rule = impl->pack->tiles[i];
            auto &bucket = impl->tiles[TileIdentity(rule.key)];
            bucket.palettes[rule.key.palette].push_back(i);
            if (rule.default_tile) bucket.defaults.push_back(i);
        }
        for (std::size_t i = 0; i < impl->pack->additions.size(); ++i) {
            const auto &addition = impl->pack->additions[i];
            impl->additions[TileIdentity(addition.original)].push_back(i);
        }
        if (!build_fallbacks(*impl, chr_rom, chr_size, error)) return nullptr;
        return std::unique_ptr<Renderer>(new Renderer(std::move(impl)));
    } catch (const std::bad_alloc &) {
        error = "Not enough memory to prepare the HD renderer";
    }
    return nullptr;
}

Renderer::Renderer(std::unique_ptr<Impl> implementation) : impl_(std::move(implementation)) {}
Renderer::~Renderer() = default;
const Pack &Renderer::pack() const { return *impl_->pack; }

struct Match {
    const TileRule *rule = nullptr;
    NesVideoTile tile{};
};

static Match matching_rule(Renderer::Impl &impl, FrameRules &rules, int x, int y, const NesVideoTile &tile) {
    Match match{nullptr, tile};
    if (!tile.chr.valid) return match;
    const std::vector<std::size_t> *candidates = nullptr;
    auto direct = impl.tiles.find(TileIdentity(tile));
    if (direct != impl.tiles.end()) {
        auto palette = direct->second.palettes.find(tile.palette);
        if (palette != direct->second.palettes.end()) candidates = &palette->second;
    }
    if (!candidates && !tile.chr.ram) {
        auto fallback = impl.fallback.find(tile.chr.index);
        if (fallback != impl.fallback.end()) {
            auto key = TileIdentity(tile);
            key.index = fallback->second;
            auto aliased = impl.tiles.find(key);
            if (aliased != impl.tiles.end()) {
                auto palette = aliased->second.palettes.find(tile.palette);
                if (palette != aliased->second.palettes.end()) candidates = &palette->second;
                else if (!aliased->second.defaults.empty()) candidates = &aliased->second.defaults;
                if (candidates) match.tile.chr.index = fallback->second;
            }
        }
    }
    if (!candidates && direct != impl.tiles.end() && !direct->second.defaults.empty()) candidates = &direct->second.defaults;
    if (candidates) {
        for (std::size_t index : *candidates) {
            if (index >= impl.pack->tiles.size()) {
                rules.fail("HD renderer tile cache is invalid");
                break;
            }
            const auto *rule = &impl.pack->tiles[index];
            if (!rules.consume()) break;
            if (rules.conditions(rule->conditions, x, y, &match.tile)) {
                match.rule = rule;
                break;
            }
            if (!rules.ok()) break;
        }
    }
    return match;
}

static void insert_additions(Renderer::Impl &impl, FrameRules &rules, int x, int y,
                              const NesVideoTile &source) {
    if (!source.chr.valid) return;
    auto apply = [&](const TileIdentity &identity) {
        auto found = impl.additions.find(identity);
        if (found == impl.additions.end()) return;
        for (std::size_t index : found->second) {
            if (index >= impl.pack->additions.size()) {
                rules.fail("HD renderer addition cache is invalid");
                return;
            }
            const auto *addition = &impl.pack->additions[index];
            if (!rules.consume()) return;
            if (!rules.matches(addition->original, source, addition->ignore_palette)) continue;
            std::int64_t dx = addition->offset_x, dy = addition->offset_y;
            if (source.attributes & 0x40) dx = -dx;
            if (source.attributes & 0x80) dy = -dy;
            std::int64_t target_x = x + dx, target_y = y + dy;
            if (target_x < 0 || target_y < 0 || target_x >= 256 || target_y >= 240) continue;
            auto &target = impl.added_pixels[static_cast<std::size_t>(target_y * 256 + target_x)];
            if (target.sprite_count >= NES_VIDEO_TRACE_SPRITES) continue;
            NesVideoTile tile = source;
            tile.color_index = 0;
            tile.palette = addition->additional.palette;
            tile.chr.ram = addition->additional.source == TileSource::ChrRam;
            tile.chr.index = addition->additional.chr_rom_index;
            std::copy(addition->additional.chr_ram.begin(), addition->additional.chr_ram.end(), tile.chr.bytes);
            target.sprites[target.sprite_count++] = tile;
        }
    };
    TileIdentity identity(source);
    apply(identity);
    if (!source.chr.ram) {
        auto fallback = impl.fallback.find(source.chr.index);
        if (fallback != impl.fallback.end() && fallback->second != source.chr.index) {
            identity.index = fallback->second;
            apply(identity);
        }
    }
}

static const NesVideoPixel *prepare_additions(Renderer::Impl &impl, FrameRules &rules,
                                              const NesVideoTraceFrame &frame, bool sprites) {
    if (!sprites || impl.additions.empty()) return frame.pixels;
    impl.added_pixels.assign(frame.pixels, frame.pixels + 256u * 240u);
    for (int y = 0; y < 240; ++y) {
        for (int x = 0; x < 256; ++x) {
            const auto &pixel = frame.pixels[static_cast<std::size_t>(y * 256 + x)];
            for (unsigned i = 0; i < std::min<unsigned>(pixel.sprite_count, NES_VIDEO_TRACE_SPRITES); ++i) {
                NesVideoTile source = pixel.sprites[i];
                insert_additions(impl, rules, x, y, source);
            }
            NesVideoTile background = pixel.background;
            insert_additions(impl, rules, x, y, background);
            if (!rules.ok()) return impl.added_pixels.data();
        }
    }
    return impl.added_pixels.data();
}

static std::uint32_t original_color(const Pack &pack, std::uint32_t rgb, std::uint8_t color, std::uint8_t mask) {
    return pack.palette ? adjust_asset((*pack.palette)[color & 0x3Fu], 255, mask) : rgb;
}

static std::uint32_t tile_color(const Pack &pack, const Match &match, unsigned sub_x, unsigned sub_y,
                                 std::uint32_t underneath, std::uint8_t mask) {
    const auto &tile = match.tile;
    if (!match.rule) return tile.color_index ? original_color(pack, tile.rgb, tile.color, mask) : underneath;
    const auto &rule = *match.rule;
    const auto &image = pack.tile_images[rule.bitmap_index];
    unsigned sx = tile.x * pack.scale + sub_x;
    unsigned sy = tile.y * pack.scale + sub_y;
    if (tile.attributes & 0x40) sx = 8 * pack.scale - 1 - sx;
    if (tile.attributes & 0x80) sy = tile.y * pack.scale + pack.scale - 1 - sub_y;
    if (sx >= 8 * pack.scale || sy >= 8 * pack.scale) return underneath;
    std::size_t offset = static_cast<std::size_t>(rule.y + sy) * image.width + rule.x + sx;
    auto incoming = adjust_asset(image.argb32[offset], rule.brightness, mask);
    return blend_asset(underneath, incoming, BlendMode::Alpha);
}

using BackgroundLayers = std::array<std::vector<const Background *>, 4>;

static BackgroundLayers select_backgrounds(const Pack &pack, FrameRules &rules, bool visible) {
    BackgroundLayers selected;
    if (!visible) return selected;
    std::array<bool, kBackgroundPriorityCount> occupied{};
    for (const auto &background : pack.backgrounds) {
        if (!occupied[background.priority] && rules.conditions(background.conditions, 0, 0, nullptr)) {
            occupied[background.priority] = true;
            selected[background.priority / 10].push_back(&background);
        }
        if (!rules.ok()) break;
    }
    for (auto &layer : selected) std::sort(layer.begin(), layer.end(), [](auto a, auto b) { return a->priority < b->priority; });
    return selected;
}

static std::uint32_t draw_backgrounds(const Pack &pack, FrameRules &rules,
                                       const std::vector<const Background *> &backgrounds,
                                       const NesVideoPixel &line, int x, int y,
                                       unsigned sub_x, unsigned sub_y, std::uint32_t color) {
    unsigned scroll_x = ((line.scroll_address & 0x1Fu) << 3) | line.fine_x
                        | ((line.scroll_address & 0x400) ? 256u : 0);
    unsigned scroll_y = ((line.scroll_address & 0x3E0u) >> 2) | ((line.scroll_address & 0x7000u) >> 12);
    if (line.scroll_address & 0x800) scroll_y += 240;
    for (const auto *background : backgrounds) {
        if (!rules.consume()) return color;
        const auto &image = pack.background_images[background->image_index];
        auto dx = static_cast<std::int64_t>(scroll_x * static_cast<double>(background->horizontal_scroll_ratio));
        auto dy = static_cast<std::int64_t>(scroll_y * static_cast<double>(background->vertical_scroll_ratio));
        std::int64_t native_x = x + dx, native_y = y + dy;
        if (native_x < 0 || native_y < 0) continue;
        std::uint64_t sx = (static_cast<std::uint64_t>(native_x) + background->left) * pack.scale + sub_x;
        std::uint64_t sy = (static_cast<std::uint64_t>(native_y) + background->top) * pack.scale + sub_y;
        if (sx >= image.width || sy >= image.height) continue;
        auto incoming = adjust_asset(image.argb32[static_cast<std::size_t>(sy * image.width + sx)],
                                     background->brightness, line.mask);
        color = blend_asset(color, incoming, background->blend_mode);
    }
    return color;
}

static bool validate_frame_conditions(const Pack &pack, MemoryReader reader, std::string &error) {
    for (const auto &condition : pack.conditions) {
        if (condition.type != ConditionType::MemoryCheck
            && condition.type != ConditionType::MemoryCheckConstant) continue;
        if (!reader) {
            error = "HD memory condition requires a valid side-effect-free memory reader";
            return false;
        }
        const std::uint32_t limit = condition.memory_space == MemorySpace::Cpu ? 0xFFFFu : 0x3FFFu;
        if (condition.operand_a > limit
            || (condition.type == ConditionType::MemoryCheck && condition.operand_b > limit)) {
            error = "HD memory condition address is outside its address space";
            return false;
        }
    }
    return true;
}

bool Renderer::render(const NesVideoTraceFrame &frame, const RenderSettings &settings,
                       MemoryReader reader, void *context, RenderedFrame &output, std::string &error) {
    error.clear();
    if (!validate_frame_conditions(*impl_->pack, reader, error)) return false;
    if (!check(frame.complete && frame.pixels, error,
               "A complete PPU presentation frame is not available yet")) return false;
    const Pack &pack = *impl_->pack;
    Overscan crop = settings.use_pack_overscan && pack.overscan ? *pack.overscan : settings.overscan;
    if (!check(crop.top < 240 && crop.bottom < 240 && crop.top + crop.bottom < 240
               && crop.left < 256 && crop.right < 256 && crop.left + crop.right < 256,
               error, "HD overscan removes the entire image")) return false;
    try {
        FrameRules rules(pack, frame, impl_->fallback, reader, context);
        auto backgrounds = select_backgrounds(pack, rules, settings.background);
        if (!rules.ok()) { error = rules.error(); return false; }
        bool custom_background = std::any_of(backgrounds.begin(), backgrounds.end(), [](const auto &layer) { return !layer.empty(); });
        const auto *pixels = prepare_additions(*impl_, rules, frame, settings.sprites);
        if (!rules.ok()) { error = rules.error(); return false; }
        rules.set_pixels(pixels);
        RenderedFrame candidate;
        candidate.width = (256 - crop.left - crop.right) * pack.scale;
        candidate.height = (240 - crop.top - crop.bottom) * pack.scale;
        candidate.scale = pack.scale;
        candidate.overscan = crop;
        candidate.pixels.resize(static_cast<std::size_t>(candidate.width) * candidate.height);
        bool original_background = !option(pack.render_options, RenderOption::DisableOriginalTiles);
        for (unsigned y = crop.top; y < 240 - crop.bottom; ++y) {
            const auto &line = pixels[y * 256u];
            for (unsigned x = crop.left; x < 256 - crop.right; ++x) {
                const auto &pixel = pixels[y * 256u + x];
                Match background = settings.background ? matching_rule(*impl_, rules, x, y, pixel.background) : Match{};
                if (!rules.ok()) { error = rules.error(); return false; }
                std::array<Match, NES_VIDEO_TRACE_SPRITES> sprites{};
                unsigned sprite_count = settings.sprites ? std::min<unsigned>(pixel.sprite_count, NES_VIDEO_TRACE_SPRITES) : 0;
                bool replacement = background.rule != nullptr;
                int lowest_background_sprite = static_cast<int>(NES_VIDEO_TRACE_SPRITES);
                for (unsigned i = 0; i < sprite_count; ++i) {
                    sprites[i] = matching_rule(*impl_, rules, x, y, pixel.sprites[i]);
                    if (!rules.ok()) { error = rules.error(); return false; }
                    replacement |= sprites[i].rule != nullptr;
                    if ((sprites[i].tile.attributes & 0x20) && sprites[i].tile.color_index)
                        lowest_background_sprite = std::min(lowest_background_sprite, static_cast<int>(i));
                }
                bool use_raw = !replacement && !custom_background && !pack.palette && original_background;
                auto raw = nes_video_trace_layer_rgb(&pixel, settings.background, settings.sprites);
                for (unsigned sy = 0; sy < pack.scale; ++sy) {
                    std::size_t row = static_cast<std::size_t>((y - crop.top) * pack.scale + sy) * candidate.width;
                    for (unsigned sx = 0; sx < pack.scale; ++sx) {
                        if (!rules.consume()) { error = rules.error(); return false; }
                        std::uint32_t color = raw;
                        if (!use_raw) {
                            color = original_color(pack, pixel.backdrop_rgb, pixel.backdrop, pixel.mask);
                            color = draw_backgrounds(pack, rules, backgrounds[0], line, x, y, sx, sy, color);
                            for (int i = static_cast<int>(sprite_count) - 1; i >= 0; --i)
                                if (sprites[i].tile.attributes & 0x20) color = tile_color(pack, sprites[i], sx, sy, color, pixel.mask);
                            color = draw_backgrounds(pack, rules, backgrounds[1], line, x, y, sx, sy, color);
                            if (settings.background && (background.rule || original_background))
                                color = tile_color(pack, background, sx, sy, color, pixel.mask);
                            color = draw_backgrounds(pack, rules, backgrounds[2], line, x, y, sx, sy, color);
                            for (int i = static_cast<int>(sprite_count) - 1; i >= 0; --i)
                                if (!(sprites[i].tile.attributes & 0x20) && i < lowest_background_sprite)
                                    color = tile_color(pack, sprites[i], sx, sy, color, pixel.mask);
                            color = draw_backgrounds(pack, rules, backgrounds[3], line, x, y, sx, sy, color);
                            if (!rules.ok()) { error = rules.error(); return false; }
                        }
                        candidate.pixels[row + (x - crop.left) * pack.scale + sx] = color;
                    }
                }
            }
        }
        output = std::move(candidate);
        return true;
    } catch (const std::bad_alloc &) {
        error = "Not enough memory to render the HD frame";
    }
    return false;
}

} // namespace cupid::hd
