/*
 * shader_internal.hpp - Bounded GLSL preset resource model
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_SHADER_INTERNAL_HPP
#define CUPID_SHADER_INTERNAL_HPP
#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace cupid::shader {
struct Parameter {
    std::string name, label;
    float value, minimum, maximum, step;
};

struct Pass {
    std::string source, alias;
    std::string scale_type_x = "source", scale_type_y = "source";
    float scale_x = 1, scale_y = 1;
    unsigned frame_mod = 0;
    bool linear = false;
};

struct Texture {
    std::string name;
    unsigned width, height;
    std::vector<uint32_t> pixels;
    bool linear;
};

struct Preset {
    std::vector<Pass> passes;
    std::vector<Texture> textures;
    std::vector<Parameter> parameters;
};

bool read_preset(const std::string &path, Preset &out, std::string &error);
} // namespace cupid::shader
#endif
