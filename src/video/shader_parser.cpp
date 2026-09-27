/*
 * shader_parser.cpp - Bounded RetroArch-style GLSL preset parsing
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "shader_internal.hpp"
#include "shader_preset.h"
#include "../hd/hd_assets.hpp"
#include "../util/file_io.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>

namespace cupid::shader {
namespace {
namespace fs = std::filesystem;

std::string trim(std::string text) {
    auto start = text.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return {};
    }
    return text.substr(start, text.find_last_not_of(" \t\r\n") - start + 1);
}

std::vector<uint8_t> read(const fs::path &path, size_t limit) {
    uint8_t *data = nullptr;
    size_t size = 0;
    NesFileResult result = nes_file_read_all(path.u8string().c_str(), limit, &data, &size);
    std::unique_ptr<uint8_t, decltype(&std::free)> memory(data, &std::free);
    if (result != NES_FILE_OK) {
        throw std::runtime_error(path.u8string() + ": " + nes_file_result_message(result));
    }
    return std::vector<uint8_t>(data, data + size);
}

fs::path resource(const fs::path &root, const fs::path &base, const std::string &name) {
    if (name.empty() || name.find('\0') != std::string::npos) {
        throw std::runtime_error("Empty or invalid shader resource path");
    }
    fs::path requested = fs::u8path(name);
    if (requested.is_absolute() || requested.has_root_name()) {
        throw std::runtime_error("Shader resource paths must be relative");
    }
    fs::path resolved = fs::canonical(base / requested);
    auto relative = resolved.lexically_relative(root);
    if (relative.empty() || *relative.begin() == ".." || !fs::is_regular_file(resolved)) {
        throw std::runtime_error("Shader resource escapes the preset directory");
    }
    return resolved;
}

bool identifier(const std::string &name) {
    if (name.empty() || name.size() > 63 || !(std::isalpha(static_cast<unsigned char>(name[0])) || name[0] == '_')) {
        return false;
    }
    return std::all_of(name.begin(), name.end(), [](unsigned char c) { return std::isalnum(c) || c == '_'; });
}

float number(const std::string &text) {
    char *end = nullptr;
    float value = std::strtof(text.c_str(), &end);
    if (text.empty() || *end || !std::isfinite(value)) {
        throw std::runtime_error("Invalid finite shader value: " + text);
    }
    return value;
}

std::vector<std::string> list(const std::string &text) {
    std::vector<std::string> names;
    std::istringstream stream(text);
    std::string name;
    while (std::getline(stream, name, ';')) {
        name = trim(name);
        if (!identifier(name) || std::find(names.begin(), names.end(), name) != names.end()) {
            throw std::runtime_error("Invalid or duplicate shader identifier");
        }
        names.push_back(name);
    }
    return names;
}

std::string source(const fs::path &path, const fs::path &root, Preset &preset, size_t &budget, unsigned depth) {
    if (depth > 16) {
        throw std::runtime_error("Shader include nesting exceeds 16 levels");
    }
    auto bytes = read(path, 1024u * 1024u);
    if (bytes.size() > budget) {
        throw std::runtime_error("Shader sources exceed 4 MiB");
    }
    budget -= bytes.size();
    if (std::find(bytes.begin(), bytes.end(), uint8_t{0}) != bytes.end()) {
        throw std::runtime_error("Shader source contains a NUL byte");
    }
    std::istringstream stream(std::string(bytes.begin(), bytes.end()));
    std::string line, output;
    while (std::getline(stream, line)) {
        std::string cleaned = trim(line);
        if (cleaned.rfind("#include", 0) == 0) {
            std::istringstream include(cleaned.substr(8));
            std::string name, trailing;
            if (!(include >> std::quoted(name)) || (include >> trailing)) {
                throw std::runtime_error("Invalid shader include");
            }
            output += source(resource(root, path.parent_path(), name), root, preset, budget, depth + 1);
        } else if (cleaned.rfind("#pragma parameter ", 0) == 0) {
            std::istringstream declaration(cleaned.substr(18));
            Parameter p;
            if (!(declaration >> p.name >> std::quoted(p.label) >> p.value >> p.minimum >> p.maximum >> p.step) ||
                !identifier(p.name) || !std::isfinite(p.value) || !std::isfinite(p.minimum) ||
                !std::isfinite(p.maximum) || !std::isfinite(p.step) || p.minimum > p.maximum || p.value < p.minimum ||
                p.value > p.maximum || p.step <= 0) {
                throw std::runtime_error("Invalid shader parameter declaration");
            }
            auto found = std::find_if(preset.parameters.begin(), preset.parameters.end(),
                                      [&p](const Parameter &v) { return v.name == p.name; });
            if (found == preset.parameters.end()) {
                if (preset.parameters.size() >= 64) {
                    throw std::runtime_error("Preset has more than 64 parameters");
                }
                preset.parameters.push_back(p);
            } else if (found->minimum != p.minimum || found->maximum != p.maximum || found->step != p.step) {
                throw std::runtime_error("Conflicting parameter declarations across shader passes");
            }
            output += '\n';
        } else {
            output += line;
            output += '\n';
        }
    }
    return output;
}
} // namespace

bool read_preset(const std::string &path, Preset &out, std::string &error) {
    try {
        fs::path file = fs::canonical(fs::u8path(path));
        if (file.extension() != ".glslp") {
            throw std::runtime_error(
                "Only GLSL .glslp presets are supported; Slang and Cg require another GPU backend");
        }
        auto bytes = read(file, 256u * 1024u);
        if (std::find(bytes.begin(), bytes.end(), uint8_t{0}) != bytes.end()) {
            throw std::runtime_error("Preset contains a NUL byte");
        }
        std::map<std::string, std::string> keys;
        std::istringstream stream(std::string(bytes.begin(), bytes.end()));
        std::string line;
        while (std::getline(stream, line)) {
            line = trim(line);
            if (line.empty() || line[0] == '#') {
                continue;
            }
            auto equal = line.find('=');
            if (equal == std::string::npos) {
                throw std::runtime_error("Preset line lacks '='");
            }
            std::string key = trim(line.substr(0, equal)), value = trim(line.substr(equal + 1));
            if (!value.empty() && value[0] == '"') {
                auto close = value.find('"', 1);
                if (close == std::string::npos) {
                    throw std::runtime_error("Unterminated preset value");
                }
                auto rest = trim(value.substr(close + 1));
                if (!rest.empty() && rest[0] != '#') {
                    throw std::runtime_error("Unexpected text after preset value");
                }
                value = value.substr(1, close - 1);
            } else {
                value = trim(value.substr(0, value.find('#')));
            }
            if (!identifier(key) || !keys.emplace(key, value).second) {
                throw std::runtime_error("Invalid or duplicate preset key: " + key);
            }
        }
        auto take = [&keys](const std::string &key, const std::string &fallback = "") {
            auto it = keys.find(key);
            if (it == keys.end()) {
                return fallback;
            }
            std::string value = it->second;
            keys.erase(it);
            return value;
        };
        auto flag = [&take](const std::string &key) {
            std::string value = take(key, "false");
            if (value != "true" && value != "false") {
                throw std::runtime_error("Boolean preset key must be true or false: " + key);
            }
            return value == "true";
        };
        float count = number(take("shaders"));
        if (count < 1 || count > 8 || std::floor(count) != count) {
            throw std::runtime_error("Preset must contain 1 to 8 passes");
        }
        Preset next;
        size_t budget = 4u * 1024u * 1024u;
        for (unsigned i = 0; i < static_cast<unsigned>(count); ++i) {
            std::string n = std::to_string(i);
            Pass pass;
            pass.source = source(resource(file.parent_path(), file.parent_path(), take("shader" + n)),
                                 file.parent_path(), next, budget, 0);
            pass.linear = flag("filter_linear" + n);
            if (flag("mipmap_input" + n) || flag("float_framebuffer" + n) || flag("srgb_framebuffer" + n)) {
                throw std::runtime_error("Mipmapped, floating-point, and sRGB shader targets are not supported");
            }
            if (take("wrap_mode" + n, "clamp_to_edge") != "clamp_to_edge") {
                throw std::runtime_error("Only clamp_to_edge wrapping is supported");
            }
            pass.alias = take("alias" + n);
            if (!pass.alias.empty() && !identifier(pass.alias)) {
                throw std::runtime_error("Invalid pass alias");
            }
            std::string type = take("scale_type" + n, "source"), scale = take("scale" + n, "1");
            pass.scale_type_x = take("scale_type_x" + n, type);
            pass.scale_type_y = take("scale_type_y" + n, type);
            pass.scale_x = number(take("scale_x" + n, scale));
            pass.scale_y = number(take("scale_y" + n, scale));
            for (const auto &axis :
                 {std::make_pair(pass.scale_type_x, pass.scale_x), std::make_pair(pass.scale_type_y, pass.scale_y)}) {
                if ((axis.first != "source" && axis.first != "viewport" && axis.first != "absolute") ||
                    axis.second <= 0 || axis.second > (axis.first == "absolute" ? 8192 : 8)) {
                    throw std::runtime_error("Unsupported shader pass scale");
                }
            }
            float mod = number(take("frame_count_mod" + n, "0"));
            if (mod < 0 || mod > 1000000 || std::floor(mod) != mod) {
                throw std::runtime_error("Invalid frame_count_mod");
            }
            pass.frame_mod = static_cast<unsigned>(mod);
            next.passes.push_back(std::move(pass));
        }
        auto textures = list(take("textures"));
        if (textures.size() > 8) {
            throw std::runtime_error("Preset has more than eight lookup textures");
        }
        size_t decoded = 0;
        for (const auto &name : textures) {
            auto encoded = read(resource(file.parent_path(), file.parent_path(), take(name)), 32u * 1024u * 1024u);
            hd::AssetLimits limits;
            limits.max_image_pixels = 16u * 1024u * 1024u;
            hd::Image image;
            std::string why;
            if (!hd::decode_png(encoded, name, limits, image, why)) {
                throw std::runtime_error(why);
            }
            decoded += image.argb32.size() * sizeof(uint32_t);
            if (decoded > 64u * 1024u * 1024u) {
                throw std::runtime_error("Lookup textures exceed 64 MiB decoded");
            }
            bool linear = flag(name + "_linear");
            if (flag(name + "_mipmap") || take(name + "_wrap_mode", "clamp_to_edge") != "clamp_to_edge") {
                throw std::runtime_error("Lookup textures require clamp_to_edge without mipmaps");
            }
            next.textures.push_back({name, image.width, image.height, std::move(image.argb32), linear});
        }
        auto overrides = list(take("parameters"));
        for (const auto &name : overrides) {
            auto found = std::find_if(next.parameters.begin(), next.parameters.end(),
                                      [&name](const Parameter &p) { return p.name == name; });
            if (found == next.parameters.end()) {
                throw std::runtime_error("Preset overrides an undeclared parameter: " + name);
            }
            std::string value = take(name);
            if (!value.empty()) {
                found->value = number(value);
                if (found->value < found->minimum || found->value > found->maximum) {
                    throw std::runtime_error("Parameter override is out of range");
                }
            }
        }
        if (!keys.empty()) {
            throw std::runtime_error("Unsupported preset key: " + keys.begin()->first);
        }
        out = std::move(next);
        error.clear();
        return true;
    } catch (const std::exception &e) {
        error = e.what();
        return false;
    }
}
} // namespace cupid::shader

bool nes_shader_discover(const char *directory, char (*paths)[1024], size_t capacity, size_t *count, char *error,
                         size_t size) {
    if (count) {
        *count = 0;
    }
    try {
        if (!directory || !paths || !count || !capacity || capacity > 256) {
            throw std::runtime_error("Shader discovery needs space for 1 to 256 paths");
        }
        std::vector<std::string> found;
        size_t examined = 0;
        for (const auto &entry : std::filesystem::directory_iterator(std::filesystem::u8path(directory))) {
            if (++examined > 4096) {
                throw std::runtime_error("Shader directory exceeds 4096 entries");
            }
            if (entry.is_symlink() || !entry.is_regular_file() || entry.path().extension() != ".glslp") {
                continue;
            }
            auto name = std::filesystem::absolute(entry.path()).u8string();
            if (name.size() >= 1024 || found.size() >= capacity) {
                throw std::runtime_error("Shader discovery output exceeds capacity");
            }
            found.push_back(std::move(name));
        }
        std::sort(found.begin(), found.end());
        for (size_t i = 0; i < found.size(); ++i) {
            std::memcpy(paths[i], found[i].c_str(), found[i].size() + 1);
        }
        *count = found.size();
        if (error && size) {
            error[0] = 0;
        }
        return true;
    } catch (const std::exception &e) {
        if (error && size) {
            std::snprintf(error, size, "%s", e.what());
        }
        return false;
    }
}
