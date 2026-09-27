/*
 * shader_preset_accuracy.cpp - Preset parser and real GPU output regressions
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#if defined(_MSC_VER) && defined(__clang__)
#include <x86intrin.h>
#endif
#include <SDL2/SDL.h>
#include "../video/shader_preset.h"
#include "../video/shader_internal.hpp"
#include "../hd/hd_assets.hpp"
#include "../util/file_io.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        if (!(x)) {                                                                                                    \
            std::fprintf(stderr, "Shader %d: %s (%s)\n", __LINE__, #x, error);                                         \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

namespace {
const char *source = R"GLSL(#version 130
#pragma parameter Gain "Brightness" 1.0 0.0 2.0 0.1
#ifdef VERTEX
in vec4 VertexCoord;
in vec2 TexCoord;
out vec2 uv;
uniform mat4 MVPMatrix;
void main() { gl_Position=MVPMatrix*VertexCoord; uv=TexCoord; }
#elif defined(FRAGMENT)
in vec2 uv;
out vec4 FragColor;
uniform sampler2D Texture;
uniform float Gain;
void main() { FragColor=vec4(texture(Texture,uv).rgb*Gain,1.0); }
#endif
)GLSL";

bool write(const char *name, const std::string &text) {
    std::ofstream stream(std::string("build/shader-regression/") + name, std::ios::binary | std::ios::trunc);
    stream << text;
    return stream.good();
}

bool fixture() {
    std::error_code ec;
    std::filesystem::create_directories("build/shader-regression", ec);
    return !ec && write("pass.glsl", source) &&
           write("preset.glslp", "shaders = 2\nshader0 = \"pass.glsl\"\nshader1 = \"pass.glsl\"\nscale0 = "
                                 "2\nparameters = \"Gain\"\nGain = 0.5\n");
}
} // namespace

extern "C" int run_shader_preset_accuracy_tests(void) {
    char error[512] = {0};
    CHECK(fixture());
    cupid::shader::Preset preset;
    std::string why;
    CHECK(cupid::shader::read_preset("build/shader-regression/preset.glslp", preset, why));
    CHECK(preset.passes.size() == 2 && preset.parameters.size() == 1 && preset.parameters[0].value == 0.5f);
    CHECK(write("invalid.glslp", "shaders=1\nshader0=\"../outside.glsl\"\n"));
    CHECK(!cupid::shader::read_preset("build/shader-regression/invalid.glslp", preset, why));
    CHECK(write("invalid.glslp", "shaders=1\nshader0=pass.glsl\nparameters=Gain\nGain=nan\n"));
    CHECK(!cupid::shader::read_preset("build/shader-regression/invalid.glslp", preset, why));
    CHECK(write("invalid.glslp", "shaders=1\nshader0=pass.glsl\nfloat_framebuffer0=true\n"));
    CHECK(!cupid::shader::read_preset("build/shader-regression/invalid.glslp", preset, why));
    CHECK(write("invalid.glslp", "shaders=1\nshader0=pass.glsl\nunknown=1\n"));
    CHECK(!cupid::shader::read_preset("build/shader-regression/invalid.glslp", preset, why));
    CHECK(write("cycle.glsl", "#include \"cycle.glsl\"\n") &&
          write("invalid.glslp", "shaders=1\nshader0=cycle.glsl\n"));
    CHECK(!cupid::shader::read_preset("build/shader-regression/invalid.glslp", preset, why));
    char paths[8][1024];
    size_t count = 0;
    CHECK(nes_shader_discover("build/shader-regression", paths, 8, &count, error, sizeof(error)) && count >= 2);
    CHECK(!nes_shader_discover("build/shader-regression", paths, 1, &count, error, sizeof(error)));
    std::puts("Shader preset bounds, includes, parameters, discovery and unsupported-resource checks passed");
    return 0;
}

extern "C" int run_shader_gpu_accuracy_tests(void) {
    char error[512] = {0};
    CHECK(fixture());
    CHECK(SDL_InitSubSystem(SDL_INIT_VIDEO) == 0);
    SDL_GLContext previous_context = SDL_GL_GetCurrentContext();
    NesShaderPreset *shader = nes_shader_create();
    CHECK(shader);
    CHECK(nes_shader_load(shader, "build/shader-regression/preset.glslp", error, sizeof(error)));
    CHECK(SDL_GL_GetCurrentContext() == previous_context);
    CHECK(nes_shader_parameter_count(shader) == 1 && nes_shader_enabled(shader));
    uint32_t pixels[] = {0xffffffff, 0xffff0000, 0xff00ff00, 0xff0000ff};
    NesVideoPresentationFrame input = {};
    input.pixels = pixels;
    input.width = input.height = 2;
    input.screens = 1;
    NesVideoPresentationFrame output = {};
    CHECK(nes_shader_render(shader, &input, 4, 4, &output, error, sizeof(error)));
    CHECK(SDL_GL_GetCurrentContext() == previous_context);
    CHECK(output.width == 4 && output.height == 4 && pixels[0] == 0xffffffff);
    unsigned red = (output.pixels[0] >> 16) & 255;
    CHECK(red >= 63 && red <= 65);
    CHECK((output.pixels[3] & 0xffff) == 0 && (output.pixels[15] & 0xffff00) == 0);
    CHECK(nes_shader_set_parameter(shader, 0, 1, error, sizeof(error)));
    CHECK(!nes_shader_set_parameter(shader, 0, 3, error, sizeof(error)));
    CHECK(nes_shader_render(shader, &input, 4, 4, &output, error, sizeof(error)) && output.pixels[0] == 0xffffffff);
    CHECK(write("broken.glslp", "shaders=1\nshader0=broken.glsl\n") && write("broken.glsl", "this is not GLSL\n"));
    CHECK(!nes_shader_load(shader, "build/shader-regression/broken.glslp", error, sizeof(error)) &&
          nes_shader_enabled(shader));
    CHECK(nes_shader_render(shader, &input, 4, 4, &output, error, sizeof(error)) && output.pixels[0] == 0xffffffff);
    CHECK(nes_shader_reload(shader, error, sizeof(error)));
    NesShaderParameter parameter;
    CHECK(nes_shader_parameter(shader, 0, &parameter) && parameter.value == 0.5f);
    NesShaderParameterValue named[2] = {{"Gain", 1.25f}, {"Gain", 1.0f}};
    CHECK(!nes_shader_apply_parameters(shader, named, 2, error, sizeof(error)));
    CHECK(nes_shader_parameter(shader, 0, &parameter) && parameter.value == 0.5f);
    CHECK(nes_shader_apply_parameters(shader, named, 1, error, sizeof(error)));
    size_t named_count = 0;
    CHECK(nes_shader_capture_parameters(shader, named, 2, &named_count) && named_count == 1 &&
          !std::strcmp(named[0].name, "Gain") && named[0].value == 1.25f);
    nes_shader_enable(shader, false);
    CHECK(nes_shader_render(shader, &input, 4, 4, &output, error, sizeof(error)) && output.pixels == pixels);
    std::error_code copy_error;
    std::filesystem::copy_file("src/tests/fixtures/hd_pack_v109/tiles.png", "build/shader-regression/lut.png",
                               std::filesystem::copy_options::overwrite_existing, copy_error);
    CHECK(!copy_error);
    std::string lookup_source(source);
    size_t uniform = lookup_source.find("uniform sampler2D Texture;");
    lookup_source.insert(uniform, "uniform sampler2D LUT;\n");
    size_t expression = lookup_source.find("texture(Texture,uv).rgb*Gain");
    lookup_source.replace(expression, std::strlen("texture(Texture,uv).rgb*Gain"), "texture(LUT,vec2(0.0)).rgb");
    CHECK(write("lookup.glsl", lookup_source));
    CHECK(write("lookup.glslp", "shaders=1\nshader0=lookup.glsl\ntextures=LUT\nLUT=lut.png\n"));
    CHECK(nes_shader_load(shader, "build/shader-regression/lookup.glslp", error, sizeof(error)));
    uint8_t *encoded = nullptr;
    size_t encoded_size = 0;
    CHECK(nes_file_read_all("build/shader-regression/lut.png", 1024u * 1024u, &encoded, &encoded_size) == NES_FILE_OK);
    std::vector<uint8_t> bytes(encoded, encoded + encoded_size);
    std::free(encoded);
    cupid::hd::Image lookup;
    std::string why;
    CHECK(cupid::hd::decode_png(bytes, "lut.png", {}, lookup, why));
    CHECK(nes_shader_render(shader, &input, 4, 4, &output, error, sizeof(error)));
    CHECK((output.pixels[0] & 0xffffffu) == (lookup.argb32[0] & 0xffffffu));
    nes_shader_destroy(shader);
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
    std::puts("Actual OpenGL multi-pass output, orientation, parameters, failed-reload retention and disable passed");
    return 0;
}
