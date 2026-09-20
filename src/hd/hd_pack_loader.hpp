/*
 * hd_pack_loader.hpp - bounded HD pack definition and archive loading
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_NES_HD_PACK_LOADER_HPP
#define CUPID_NES_HD_PACK_LOADER_HPP

#include "hd_assets.hpp"
#include "hd_pack_model.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace cupid::hd {

struct LoadLimits : AssetLimits {
    std::size_t max_archive_bytes = 256u * 1024u * 1024u;
    std::size_t max_entries = 4096;
    std::size_t max_lines = 1000000;
    std::size_t max_images = 8192;
    std::size_t max_tiles = 131072;
    std::size_t max_conditions = 8192;
    std::size_t max_backgrounds = 1024;
    std::size_t max_additions = 4096;
    std::size_t max_fallbacks = 131072;
    std::size_t max_audio_tracks = 1024;
};

struct LoadResult {
    std::unique_ptr<Pack> candidate;
    std::string error;
    std::vector<std::string> warnings;

    explicit operator bool() const { return candidate != nullptr; }
};

LoadResult load_pack_directory(const std::string &directory,
                               const LoadLimits &limits = {});
LoadResult load_pack_zip(const std::string &zip_path,
                         const LoadLimits &limits = {});
LoadResult load_pack_zip_bytes(const std::uint8_t *data,
                               std::size_t size,
                               const LoadLimits &limits = {});

bool create_archive(const Pack &pack,
                    std::vector<std::uint8_t> &zip_bytes,
                    std::string &error);

}  // namespace cupid::hd

#endif
