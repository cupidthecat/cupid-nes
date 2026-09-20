/*
 * hd_renderer_accuracy.cpp - Replacement pixels, conditions and presentation limits
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "../hd/hd_renderer.hpp"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <limits>

using namespace cupid::hd;

#define CHECK(condition) do { \
    if (!(condition)) { \
        std::fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

namespace {
struct FrameFixture {
    std::vector<NesVideoPixel> pixels;
    NesVideoTraceFrame frame{};
    FrameFixture() : pixels(256 * 240) {
        for (unsigned y = 0; y < 240; ++y) {
            for (unsigned x = 0; x < 256; ++x) {
                auto &pixel = pixels[y * 256 + x];
                auto &tile = pixel.background;
                tile.chr.valid = true;
                tile.chr.index = 1;
                std::fill_n(tile.chr.bytes, 8, 0xFF);
                tile.x = static_cast<std::uint8_t>(x & 7u);
                tile.y = static_cast<std::uint8_t>(y & 7u);
                tile.palette = 0x0F213141;
                tile.rgb = 0xFF0000FF;
                tile.color = 0x21;
                tile.color_index = 1;
                pixel.backdrop = 0x0F;
                pixel.backdrop_rgb = 0xFF101010;
                pixel.original_rgb = tile.rgb;
                pixel.original_signal = tile.color;
                pixel.mask = 0x1E;
            }
        }
        frame = {pixels.data(), 7, 1, true};
    }
};

std::shared_ptr<Pack> pack_with_tile(unsigned scale, std::uint32_t color) {
    auto pack = std::make_shared<Pack>();
    pack->scale = scale;
    Image image;
    image.width = image.height = scale * 8;
    image.argb32.assign(static_cast<std::size_t>(image.width) * image.height, color);
    pack->tile_images.push_back(std::move(image));
    TileRule rule;
    rule.key.chr_rom_index = 1;
    rule.key.palette = 0x0F213141;
    pack->tiles.push_back(rule);
    return pack;
}

int pixels_scaling_and_flips() {
    FrameFixture fixture;
    std::string error;
    auto pack = pack_with_tile(2, 0xFFFF0000);
    auto renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && error.empty());
    RenderedFrame result;
    CHECK(renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.width == 512 && result.height == 480 && result.scale == 2);
    CHECK(std::all_of(result.pixels.begin(), result.pixels.end(), [](auto color) { return color == 0xFFFF0000; }));
    pack = pack_with_tile(2, 0);
    for (unsigned y = 0; y < 16; ++y) {
        for (unsigned x = 0; x < 16; ++x)
            pack->tile_images[0].argb32[y * 16 + x] = 0xFF000000u | (x << 16) | (y << 8);
    }
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer);
    CHECK(renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[11 * result.width + 7] == 0xFF070B00);
    auto &pixel = fixture.pixels[5 * 256 + 3];
    pixel.background.attributes = 0xC0;
    pixel.background.y = 2; // The PPU already fetched the vertically mirrored row.
    CHECK(renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[10 * result.width + 6] == 0xFF090500);
    CHECK(result.pixels[11 * result.width + 7] == 0xFF080400);

    pack = pack_with_tile(1, 0x80FF0000);
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    // The replacement is blended over the backdrop, which is drawn first.
    CHECK(result.pixels[0] == 0xFF880808);
    pack->tiles[0].brightness = 510;
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFFFF0909);
    return 0;
}

int identity_fallback_and_raw_output() {
    FrameFixture fixture;
    std::string error;
    RenderedFrame result;
    auto pack = pack_with_tile(1, 0xFF00FF00);
    pack->tiles[0].key.palette = 0x11223344;
    auto renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == fixture.pixels[0].original_rgb);
    pack->tiles[0].default_tile = true;
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFF00FF00);
    pack->tiles[0].default_tile = false;
    pack->tiles[0].key.chr_rom_index = 2;
    pack->tiles[0].key.palette = 0x0F213141;
    pack->fallbacks.push_back({1, 2});
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFF00FF00);
    std::uint8_t chr[48] = {0};
    std::fill_n(chr + 16, 8, 0xFF);
    std::fill_n(chr + 32, 8, 0xFF);
    pack->fallbacks.clear();
    pack->render_options = RenderOption::AutomaticFallbackTiles;
    renderer = Renderer::create(pack, chr, sizeof(chr), error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFF00FF00);
    pack->tiles[0].key.source = TileSource::ChrRam;
    std::fill_n(pack->tiles[0].key.chr_ram.begin(), 8, 0xFF);
    for (auto &pixel : fixture.pixels) pixel.background.chr.ram = true;
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFF00FF00);
    fixture.pixels[0].background.chr.bytes[0] = 0;
    CHECK(renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == fixture.pixels[0].original_rgb && result.pixels[1] == 0xFF00FF00);
    fixture.pixels[1].background.chr.valid = false;
    CHECK(renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[1] == fixture.pixels[1].original_rgb);
    return 0;
}

struct MemoryFixture { unsigned cpu_calls = 0, ppu_calls = 0; };
std::uint8_t read_memory(void *context, MemorySpace space, std::uint16_t address) {
    auto &memory = *static_cast<MemoryFixture *>(context);
    if (space == MemorySpace::Cpu) ++memory.cpu_calls;
    else ++memory.ppu_calls;
    return static_cast<std::uint8_t>(address == 0x44 ? 0xA5 : 0xA1);
}

int conditions_and_failure_retention() {
    FrameFixture fixture;
    std::string error;
    RenderedFrame result;
    auto pack = pack_with_tile(1, 0xFFFFFF00);
    Condition condition;
    condition.type = ConditionType::PositionCheckX;
    condition.comparison = CompareOperator::GreaterThanOrEqual;
    condition.operand_b = 100;
    pack->conditions.push_back(condition);
    pack->tiles[0].conditions.push_back(0);
    auto renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[99] == 0xFF0000FF && result.pixels[100] == 0xFFFFFF00);
    pack->conditions[0].inverted = true;
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[99] == 0xFFFFFF00 && result.pixels[100] == 0xFF0000FF);
    condition = {};
    condition.type = ConditionType::FrameRange;
    condition.operand_a = 4; condition.operand_b = 2;
    pack->conditions[0] = condition;
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFFFFFF00);
    fixture.frame.frame_number = 4;
    CHECK(renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFF0000FF);

    condition = {};
    condition.type = ConditionType::MemoryCheckConstant;
    condition.operand_a = 0x44; condition.operand_b = 0xA0; condition.mask = 0xF0;
    pack->conditions[0] = condition;
    Condition second = condition;
    second.memory_space = MemorySpace::Ppu;
    second.type = ConditionType::MemoryCheck;
    second.operand_b = 0x45;
    pack->conditions.push_back(second);
    pack->tiles[0].conditions.push_back(1);
    renderer = Renderer::create(pack, nullptr, 0, error);
    MemoryFixture memory;
    CHECK(renderer && renderer->render(fixture.frame, {}, read_memory, &memory, result, error));
    CHECK(result.pixels[0] == 0xFFFFFF00 && memory.cpu_calls == 1 && memory.ppu_calls == 2);
    RenderedFrame previous = result;
    CHECK(!renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(!error.empty() && result.pixels == previous.pixels && result.width == previous.width);
    pack->conditions[0].operand_a = 0x10000;
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && !renderer->render(fixture.frame, {}, read_memory, &memory, result, error));
    CHECK(result.pixels == previous.pixels);
    pack->tiles[0].conditions = {99};
    CHECK(!Renderer::create(pack, nullptr, 0, error));
    pack->tiles[0].conditions.clear();
    pack->scale = 11;
    CHECK(!Renderer::create(pack, nullptr, 0, error));
    return 0;
}

int nearby_origin_and_sprite_conditions() {
    FrameFixture fixture;
    std::string error;
    RenderedFrame result;
    auto pack = pack_with_tile(1, 0xFF00FFFF);
    Condition condition;
    condition.type = ConditionType::TileNearby;
    condition.x = 1;
    condition.tile = pack->tiles[0].key;
    pack->conditions.push_back(condition);
    pack->tiles[0].conditions = {0};
    fixture.pixels[1].background.chr.index = 2;
    auto renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFF0000FF && result.pixels[2] == 0xFF00FFFF);
    CHECK(result.pixels.back() == 0xFF0000FF); // One-past-the-frame lookup is rejected.
    condition.type = ConditionType::TileAtPosition;
    condition.x = 0; condition.y = 0;
    pack->conditions[0] = condition;
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFF00FFFF);
    condition.type = ConditionType::OriginPositionCheckX;
    condition.operand_b = 8;
    pack->conditions[0] = condition;
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[7] == 0xFF0000FF && result.pixels[8] == 0xFF00FFFF && result.pixels[15] == 0xFF00FFFF);
    condition.type = ConditionType::HorizontalMirror;
    pack->conditions[0] = condition;
    fixture.pixels[0].background.attributes = 0x40;
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFF00FFFF && result.pixels[2] == 0xFF0000FF);
    return 0;
}

int layers_priority_additions_and_crop() {
    FrameFixture fixture;
    std::string error;
    RenderedFrame result;
    auto pack = pack_with_tile(1, 0xFFFF0000);
    Image sprite;
    sprite.width = sprite.height = 8;
    sprite.argb32.assign(64, 0xFF00FF00);
    pack->tile_images.push_back(sprite);
    TileRule sprite_rule;
    sprite_rule.bitmap_index = 1;
    sprite_rule.key.chr_rom_index = 2;
    sprite_rule.key.palette = 0xFF112233;
    pack->tiles.push_back(sprite_rule);
    auto &pixel = fixture.pixels[0];
    pixel.sprite_count = 1;
    pixel.sprites[0] = pixel.background;
    pixel.sprites[0].chr.index = 2;
    pixel.sprites[0].palette = 0xFF112233;
    pixel.sprites[0].attributes = 0x20;
    pixel.sprites[0].rgb = 0xFFFFFFFF;
    auto renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFFFF0000);
    pixel.sprites[0].attributes = 0;
    CHECK(renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFF00FF00);
    RenderSettings settings;
    settings.sprites = false;
    CHECK(renderer->render(fixture.frame, settings, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFFFF0000);
    settings.sprites = true; settings.background = false;
    CHECK(renderer->render(fixture.frame, settings, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFF00FF00);

    Image background;
    background.width = 256; background.height = 240;
    background.argb32.assign(256 * 240, 0xFF202020);
    pack->background_images.push_back(background);
    Background layer;
    layer.priority = 30;
    pack->backgrounds.push_back(layer);
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFF202020);
    pack->backgrounds[0].blend_mode = BlendMode::Add;
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFF20FF20);
    pack->backgrounds[0].blend_mode = BlendMode::Subtract;
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFF00DF00);
    pack->backgrounds[0].priority = 10;
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[0] == 0xFF00FF00 && result.pixels[1] == 0xFFFF0000);

    pack->backgrounds.clear();
    Addition addition;
    addition.original = pack->tiles[0].key;
    addition.additional = sprite_rule.key;
    addition.offset_x = 8;
    pack->additions.push_back(addition);
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.pixels[7] == 0xFFFF0000 && result.pixels[8] == 0xFF00FF00);
    CHECK(fixture.pixels[8].sprite_count == 0); // Additions never alter the observed frame.
    renderer.reset();
    pack->overscan = Overscan{8, 4, 8, 4};
    renderer = Renderer::create(pack, nullptr, 0, error);
    CHECK(renderer && renderer->render(fixture.frame, {}, nullptr, nullptr, result, error));
    CHECK(result.width == 248 && result.height == 224 && result.overscan.left == 4);
    settings = {};
    settings.use_pack_overscan = false;
    settings.overscan = {120, 0, 120, 0};
    RenderedFrame previous = result;
    CHECK(!renderer->render(fixture.frame, settings, nullptr, nullptr, result, error));
    CHECK(result.pixels == previous.pixels);
    return 0;
}
} // namespace

extern "C" int test_hd_renderer_accuracy(void) {
    int failures = pixels_scaling_and_flips() + identity_fallback_and_raw_output()
        + conditions_and_failure_retention() + nearby_origin_and_sprite_conditions()
        + layers_priority_additions_and_crop();
    std::printf("HD renderer: 5 groups, %d failures\n", failures);
    return failures;
}
