/*
 * hd_assets.hpp - bounded HD image and audio decoding
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_NES_HD_ASSETS_HPP
#define CUPID_NES_HD_ASSETS_HPP

#include "hd_pack_model.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace cupid::hd {

struct AssetLimits {
    std::size_t max_definition_bytes = 4u * 1024u * 1024u;
    std::size_t max_member_bytes = 64u * 1024u * 1024u;
    std::size_t max_total_asset_bytes = 256u * 1024u * 1024u;
    std::size_t max_decoded_bytes = 256u * 1024u * 1024u;
    std::uint32_t max_image_width = 16384;
    std::uint32_t max_image_height = 16384;
    std::size_t max_image_pixels = 64u * 1024u * 1024u;
    std::size_t max_audio_frames = 192000u * 60u * 30u;
    std::size_t max_decoder_workspace_bytes = 8u * 1024u * 1024u;
};

bool decode_png(const std::vector<std::uint8_t> &encoded,
                const std::string &path,
                const AssetLimits &limits,
                Image &out,
                std::string &error);

bool decode_audio(const std::vector<std::uint8_t> &encoded,
                  const std::string &path,
                  const AssetLimits &limits,
                  AudioTrack &out,
                  std::string &error);

}  // namespace cupid::hd

#endif
