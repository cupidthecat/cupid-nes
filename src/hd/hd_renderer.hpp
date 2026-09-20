/*
 * hd_renderer.hpp - Replacement graphics over observed PPU output
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_HD_RENDERER_HPP
#define CUPID_HD_RENDERER_HPP

#include "hd_pack_model.hpp"
#include "../video/video_trace.h"
#include <memory>

namespace cupid::hd {

struct RenderSettings {
    bool background = true;
    bool sprites = true;
    bool use_pack_overscan = true;
    Overscan overscan{};
};

struct RenderedFrame {
    std::vector<std::uint32_t> pixels;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t scale = 1;
    Overscan overscan{};
};

/* Called at most once per watched address per rendered frame. The embedding
 * supplies side-effect-free CPU/PPU peeks for the selected console. */
using MemoryReader = std::uint8_t (*)(void *, MemorySpace, std::uint16_t);

class Renderer {
public:
    struct Impl;
    static std::unique_ptr<Renderer> create(std::shared_ptr<const Pack> pack,
                                           const std::uint8_t *chr_rom, std::size_t chr_size,
                                           std::string &error);
    ~Renderer();
    Renderer(const Renderer &) = delete;
    Renderer &operator=(const Renderer &) = delete;

    bool render(const NesVideoTraceFrame &frame, const RenderSettings &settings,
                MemoryReader reader, void *context, RenderedFrame &output, std::string &error);
    const Pack &pack() const;

private:
    explicit Renderer(std::unique_ptr<Impl> implementation);
    std::unique_ptr<Impl> impl_;
};

} // namespace cupid::hd
#endif
