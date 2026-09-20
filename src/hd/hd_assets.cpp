/*
 * hd_assets.cpp - bounded HD image and audio decoding
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "hd_assets.hpp"

#include "../third_party/spng/spng.h"
#include "../third_party/stb/stb_vorbis.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <memory>
#include <sstream>

namespace cupid::hd {
namespace {

template <typename T>
bool checked_multiply(T left, T right, T &result) {
    if (left != 0 && right > std::numeric_limits<T>::max() / left) return false;
    result = left * right;
    return true;
}

std::string png_error(const std::string &path, int code) {
    std::ostringstream stream;
    stream << "PNG asset '" << path << "' is invalid: " << spng_strerror(code);
    return stream.str();
}

std::uint16_t read_le16(const std::uint8_t *data) {
    return static_cast<std::uint16_t>(data[0])
         | static_cast<std::uint16_t>(data[1] << 8);
}

std::uint32_t read_le32(const std::uint8_t *data) {
    return static_cast<std::uint32_t>(data[0])
         | (static_cast<std::uint32_t>(data[1]) << 8)
         | (static_cast<std::uint32_t>(data[2]) << 16)
         | (static_cast<std::uint32_t>(data[3]) << 24);
}

bool decode_wav(const std::vector<std::uint8_t> &encoded,
                const std::string &path,
                const AssetLimits &limits,
                AudioTrack &out,
                std::string &error) {
    if (encoded.size() < 12 || std::memcmp(encoded.data(), "RIFF", 4) != 0
        || std::memcmp(encoded.data() + 8, "WAVE", 4) != 0) {
        error = "Audio asset '" + path + "' is neither Ogg Vorbis nor PCM WAV";
        return false;
    }

    bool have_format = false;
    const std::uint8_t *pcm = nullptr;
    std::size_t pcm_bytes = 0;
    std::uint16_t format = 0;
    std::uint16_t channels = 0;
    std::uint32_t sample_rate = 0;
    std::uint16_t block_align = 0;
    std::uint16_t bits = 0;

    std::size_t offset = 12;
    while (offset + 8 <= encoded.size()) {
        const std::uint8_t *chunk = encoded.data() + offset;
        const std::uint32_t chunk_size = read_le32(chunk + 4);
        offset += 8;
        if (chunk_size > encoded.size() - offset) {
            error = "PCM WAV asset '" + path + "' has a truncated chunk";
            return false;
        }
        if (std::memcmp(chunk, "fmt ", 4) == 0) {
            if (chunk_size < 16) {
                error = "PCM WAV asset '" + path + "' has a short format chunk";
                return false;
            }
            const std::uint8_t *fmt = encoded.data() + offset;
            format = read_le16(fmt);
            channels = read_le16(fmt + 2);
            sample_rate = read_le32(fmt + 4);
            block_align = read_le16(fmt + 12);
            bits = read_le16(fmt + 14);
            have_format = true;
        } else if (std::memcmp(chunk, "data", 4) == 0) {
            pcm = encoded.data() + offset;
            pcm_bytes = chunk_size;
        }
        const std::size_t advance = static_cast<std::size_t>(chunk_size) + (chunk_size & 1u);
        if (advance > encoded.size() - offset) {
            error = "PCM WAV asset '" + path + "' has invalid chunk padding";
            return false;
        }
        offset += advance;
    }

    if (!have_format || !pcm) {
        error = "PCM WAV asset '" + path + "' is missing format or sample data";
        return false;
    }
    if (format != 1 || bits != 16 || (channels != 1 && channels != 2) || sample_rate == 0) {
        error = "PCM WAV asset '" + path + "' must be 16-bit mono or stereo PCM";
        return false;
    }
    if (block_align != channels * sizeof(std::int16_t) || pcm_bytes % block_align != 0) {
        error = "PCM WAV asset '" + path + "' has inconsistent sample alignment";
        return false;
    }
    const std::size_t frames = pcm_bytes / block_align;
    if (frames > limits.max_audio_frames || pcm_bytes > limits.max_decoded_bytes) {
        error = "PCM WAV asset '" + path + "' exceeds the decoded audio limit";
        return false;
    }
    out.sample_rate = sample_rate;
    out.channels = static_cast<std::uint8_t>(channels);
    out.pcm.resize(pcm_bytes / sizeof(std::int16_t));
    for (std::size_t i = 0; i < out.pcm.size(); ++i) {
        out.pcm[i] = static_cast<std::int16_t>(read_le16(pcm + i * 2));
    }
    return true;
}

bool looks_like_wav(const std::vector<std::uint8_t> &encoded) {
    return encoded.size() >= 12 && std::memcmp(encoded.data(), "RIFF", 4) == 0
        && std::memcmp(encoded.data() + 8, "WAVE", 4) == 0;
}

}  // namespace

bool decode_png(const std::vector<std::uint8_t> &encoded,
                const std::string &path,
                const AssetLimits &limits,
                Image &out,
                std::string &error) {
    out = {};
    if (encoded.empty()) {
        error = "PNG asset '" + path + "' is empty";
        return false;
    }
    spng_ctx *raw = spng_ctx_new(0);
    if (!raw) {
        error = "PNG decoder could not allocate a context for '" + path + "'";
        return false;
    }
    std::unique_ptr<spng_ctx, decltype(&spng_ctx_free)> ctx(raw, spng_ctx_free);

    int status = spng_set_image_limits(ctx.get(), limits.max_image_width, limits.max_image_height);
    if (status == SPNG_OK) {
        const std::size_t chunk_limit = std::min(limits.max_member_bytes, limits.max_decoded_bytes);
        status = spng_set_chunk_limits(ctx.get(), chunk_limit, chunk_limit);
    }
    if (status == SPNG_OK) status = spng_set_crc_action(ctx.get(), SPNG_CRC_ERROR, SPNG_CRC_ERROR);
    if (status == SPNG_OK) status = spng_set_png_buffer(ctx.get(), encoded.data(), encoded.size());
    if (status != SPNG_OK) {
        error = png_error(path, status);
        return false;
    }

    spng_ihdr ihdr{};
    status = spng_get_ihdr(ctx.get(), &ihdr);
    if (status != SPNG_OK) {
        error = png_error(path, status);
        return false;
    }
    std::size_t pixels = 0;
    if (!checked_multiply<std::size_t>(ihdr.width, ihdr.height, pixels)
        || pixels > limits.max_image_pixels) {
        error = "PNG asset '" + path + "' exceeds the image pixel limit";
        return false;
    }
    std::size_t decoded_size = 0;
    status = spng_decoded_image_size(ctx.get(), SPNG_FMT_RGBA8, &decoded_size);
    if (status != SPNG_OK || decoded_size > limits.max_decoded_bytes
        || decoded_size != pixels * 4u) {
        error = status == SPNG_OK ? "PNG asset '" + path + "' exceeds the decoded image limit"
                                  : png_error(path, status);
        return false;
    }
    std::vector<std::uint8_t> rgba(decoded_size);
    status = spng_decode_image(ctx.get(), rgba.data(), rgba.size(), SPNG_FMT_RGBA8, SPNG_DECODE_TRNS);
    if (status != SPNG_OK) {
        error = png_error(path, status);
        return false;
    }

    out.path = path;
    out.width = ihdr.width;
    out.height = ihdr.height;
    out.argb32.resize(pixels);
    for (std::size_t i = 0; i < pixels; ++i) {
        const std::uint8_t *pixel = rgba.data() + i * 4;
        out.argb32[i] = (static_cast<std::uint32_t>(pixel[3]) << 24)
                      | (static_cast<std::uint32_t>(pixel[0]) << 16)
                      | (static_cast<std::uint32_t>(pixel[1]) << 8)
                      | static_cast<std::uint32_t>(pixel[2]);
    }
    return true;
}

bool decode_audio(const std::vector<std::uint8_t> &encoded,
                  const std::string &path,
                  const AssetLimits &limits,
                  AudioTrack &out,
                  std::string &error) {
    out.pcm.clear();
    out.sample_rate = 0;
    out.channels = 0;
    if (looks_like_wav(encoded)) return decode_wav(encoded, path, limits, out, error);
    if (encoded.empty() || encoded.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        error = "Ogg Vorbis asset '" + path + "' has an invalid encoded size";
        return false;
    }

    if (limits.max_decoder_workspace_bytes == 0
        || limits.max_decoder_workspace_bytes > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        error = "Ogg Vorbis decoder workspace limit is invalid";
        return false;
    }
    std::vector<char> workspace(limits.max_decoder_workspace_bytes);
    stb_vorbis_alloc allocation{workspace.data(), static_cast<int>(workspace.size())};
    int open_error = 0;
    stb_vorbis *raw = stb_vorbis_open_memory(encoded.data(), static_cast<int>(encoded.size()),
                                             &open_error, &allocation);
    if (!raw) {
        error = "Ogg Vorbis asset '" + path + "' could not be opened (decoder error "
              + std::to_string(open_error) + ")";
        return false;
    }
    std::unique_ptr<stb_vorbis, decltype(&stb_vorbis_close)> decoder(raw, stb_vorbis_close);
    const stb_vorbis_info info = stb_vorbis_get_info(decoder.get());
    if ((info.channels != 1 && info.channels != 2) || info.sample_rate == 0) {
        error = "Ogg Vorbis asset '" + path + "' must be mono or stereo";
        return false;
    }
    const std::size_t frames = stb_vorbis_stream_length_in_samples(decoder.get());
    if (frames > limits.max_audio_frames) {
        error = "Ogg Vorbis asset '" + path + "' exceeds the decoded audio frame limit";
        return false;
    }
    std::size_t sample_count = 0;
    std::size_t decoded_bytes = 0;
    if (!checked_multiply<std::size_t>(frames, static_cast<std::size_t>(info.channels), sample_count)
        || !checked_multiply<std::size_t>(sample_count, sizeof(std::int16_t), decoded_bytes)
        || decoded_bytes > limits.max_decoded_bytes
        || sample_count > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        error = "Ogg Vorbis asset '" + path + "' exceeds the decoded audio memory limit";
        return false;
    }
    out.pcm.resize(sample_count);
    std::size_t decoded_frames = 0;
    while (decoded_frames < frames) {
        const std::size_t remaining_frames = frames - decoded_frames;
        const std::size_t batch_frames = std::min<std::size_t>(remaining_frames, 4096);
        const int shorts = static_cast<int>(batch_frames * static_cast<std::size_t>(info.channels));
        const int got = stb_vorbis_get_samples_short_interleaved(
            decoder.get(), info.channels,
            reinterpret_cast<short *>(out.pcm.data() + decoded_frames * info.channels), shorts);
        if (got <= 0) break;
        decoded_frames += static_cast<std::size_t>(got);
    }
    if (decoded_frames != frames) {
        error = "Ogg Vorbis asset '" + path + "' ended before its declared sample count";
        return false;
    }
    out.sample_rate = info.sample_rate;
    out.channels = static_cast<std::uint8_t>(info.channels);
    return true;
}

}  // namespace cupid::hd
