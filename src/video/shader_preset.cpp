/*
 * shader_preset.cpp - Native OpenGL multi-pass presentation
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "shader_preset.h"
#if defined(_MSC_VER) && defined(__clang__)
#include <x86intrin.h>
#endif
#include <SDL2/SDL.h>
#define GL_GLEXT_PROTOTYPES
#include <SDL2/SDL_opengl.h>
#include "shader_internal.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>
#include <stdexcept>

namespace {
#define GL_FUNCTIONS(X)                                                                                                \
    X(glGetError)                                                                                                      \
    X(glGetIntegerv) X(glGenTextures) X(glDeleteTextures) X(glBindTexture) X(glTexImage2D) X(glTexParameteri)          \
        X(glPixelStorei) X(glViewport) X(glReadPixels) X(glDrawArrays) X(glDisable) X(glCreateShader)                  \
            X(glShaderSource) X(glCompileShader) X(glGetShaderiv) X(glGetShaderInfoLog) X(glDeleteShader)              \
                X(glCreateProgram) X(glAttachShader) X(glLinkProgram) X(glGetProgramiv) X(glGetProgramInfoLog)         \
                    X(glDeleteProgram) X(glUseProgram) X(glGetUniformLocation) X(glUniform1i) X(glUniform1f)           \
                        X(glUniform2f) X(glUniformMatrix4fv) X(glGenFramebuffers) X(glDeleteFramebuffers)              \
                            X(glBindFramebuffer) X(glFramebufferTexture2D) X(glCheckFramebufferStatus)                 \
                                X(glActiveTexture) X(glGenBuffers) X(glDeleteBuffers) X(glBindBuffer) X(glBufferData)  \
                                    X(glGetAttribLocation) X(glEnableVertexAttribArray) X(glDisableVertexAttribArray)  \
                                        X(glVertexAttribPointer) X(glGetActiveUniform) X(glGetActiveAttrib)

struct GL {
#define DECLARE(name) decltype(&::name) name = nullptr;
    GL_FUNCTIONS(DECLARE)
#undef DECLARE

    void load() {
#define LOAD(name)                                                                                                     \
    name = reinterpret_cast<decltype(name)>(SDL_GL_GetProcAddress(#name));                                             \
    if (!name) {                                                                                                       \
        throw std::runtime_error("GPU lacks " #name);                                                                  \
    }
        GL_FUNCTIONS(LOAD)
#undef LOAD
    }
};

struct Current {
    SDL_Window *window = SDL_GL_GetCurrentWindow();
    SDL_GLContext context = SDL_GL_GetCurrentContext();

    ~Current() {
        SDL_GL_MakeCurrent(window, context);
    }
};

struct Gpu {
    SDL_Window *window = nullptr;
    SDL_GLContext context = nullptr;
    GL gl;
    GLuint vbo = 0, fbo = 0, input = 0;
    std::vector<GLuint> programs, targets, textures;
    GLint max_size = 0, max_units = 0;

    ~Gpu() {
        Current previous;
        if (context && SDL_GL_MakeCurrent(window, context) == 0) {
            for (GLuint program : programs) {
                gl.glDeleteProgram(program);
            }
            if (!targets.empty()) {
                gl.glDeleteTextures(static_cast<GLsizei>(targets.size()), targets.data());
            }
            if (!textures.empty()) {
                gl.glDeleteTextures(static_cast<GLsizei>(textures.size()), textures.data());
            }
            if (input) {
                gl.glDeleteTextures(1, &input);
            }
            if (vbo) {
                gl.glDeleteBuffers(1, &vbo);
            }
            if (fbo) {
                gl.glDeleteFramebuffers(1, &fbo);
            }
        }
        if (previous.context == context) {
            previous.context = nullptr;
            previous.window = nullptr;
        }
        SDL_GL_DeleteContext(context);
        SDL_DestroyWindow(window);
    }

    void texture(GLuint name, unsigned width, unsigned height, const uint32_t *pixels, bool linear) {
        if (width > static_cast<unsigned>(max_size) || height > static_cast<unsigned>(max_size)) {
            throw std::runtime_error("Shader texture exceeds GPU dimension limit");
        }
        gl.glBindTexture(GL_TEXTURE_2D, name);
        gl.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, linear ? GL_LINEAR : GL_NEAREST);
        gl.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, linear ? GL_LINEAR : GL_NEAREST);
        gl.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        gl.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        gl.glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, static_cast<GLsizei>(width), static_cast<GLsizei>(height), 0,
                        GL_BGRA, GL_UNSIGNED_BYTE, pixels);
    }
};

bool fail(char *error, size_t size, const std::string &message) {
    if (error && size) {
        std::snprintf(error, size, "%s", message.c_str());
    }
    return false;
}

std::string stage_source(const std::string &source, bool vertex) {
    std::string text = source;
    std::string defines =
        vertex ? "\n#define VERTEX\n#define PARAMETER_UNIFORM\n" : "\n#define FRAGMENT\n#define PARAMETER_UNIFORM\n";
    auto version = text.find("#version");
    if (version == std::string::npos) {
        return "#version 130" + defines + text;
    }
    auto end = text.find('\n', version);
    if (end == std::string::npos) {
        end = text.size();
    }
    text.insert(end, defines);
    return text;
}

GLuint compile(Gpu &gpu, const std::string &source, GLenum stage) {
    GLuint shader = gpu.gl.glCreateShader(stage);
    if (!shader) {
        throw std::runtime_error("Could not allocate GPU shader");
    }
    const char *text = source.c_str();
    gpu.gl.glShaderSource(shader, 1, &text, nullptr);
    gpu.gl.glCompileShader(shader);
    GLint success = 0;
    gpu.gl.glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[2048] = {};
        gpu.gl.glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        gpu.gl.glDeleteShader(shader);
        throw std::runtime_error(std::string("GLSL compile: ") + log);
    }
    return shader;
}

bool supported_uniform(const std::string &name, const cupid::shader::Preset &preset, size_t index) {
    static const char *const standard[] = {"Texture",       "InputSize",      "TextureSize", "OutputSize",
                                           "FrameCount",    "FrameDirection", "MVPMatrix",   "OrigTexture",
                                           "OrigInputSize", "OrigTextureSize"};
    for (const char *value : standard) {
        if (name == value) {
            return true;
        }
    }
    for (const auto &parameter : preset.parameters) {
        if (parameter.name == name) {
            return true;
        }
    }
    for (const auto &texture : preset.textures) {
        if (texture.name == name) {
            return true;
        }
    }
    for (size_t i = 0; i < index; ++i) {
        for (const auto &prefix :
             {"Pass" + std::to_string(i + 1), "PassPrev" + std::to_string(index - i), preset.passes[i].alias}) {
            if (!prefix.empty() &&
                (name == prefix + "Texture" || name == prefix + "InputSize" || name == prefix + "TextureSize")) {
                return true;
            }
        }
    }
    return false;
}

std::unique_ptr<Gpu> build(const cupid::shader::Preset &preset) {
    Current previous;
    auto gpu = std::make_unique<Gpu>();
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);
    SDL_GL_SetAttribute(SDL_GL_SHARE_WITH_CURRENT_CONTEXT, 0);
    gpu->window = SDL_CreateWindow("Cupid shader processing", 0, 0, 16, 16, SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
    if (gpu->window) {
        gpu->context = SDL_GL_CreateContext(gpu->window);
    }
    if (!gpu->context) {
        throw std::runtime_error(std::string("OpenGL 3.3 unavailable: ") + SDL_GetError());
    }
    gpu->gl.load();
    auto &g = gpu->gl;
    g.glGetIntegerv(GL_MAX_TEXTURE_SIZE, &gpu->max_size);
    g.glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &gpu->max_units);
    g.glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    g.glPixelStorei(GL_PACK_ALIGNMENT, 4);
    g.glDisable(GL_BLEND);
    g.glDisable(GL_DEPTH_TEST);
    g.glDisable(GL_DITHER);
    g.glGenFramebuffers(1, &gpu->fbo);
    g.glGenBuffers(1, &gpu->vbo);
    g.glGenTextures(1, &gpu->input);
    const GLfloat vertices[] = {0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1};
    g.glBindBuffer(GL_ARRAY_BUFFER, gpu->vbo);
    g.glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    gpu->targets.resize(preset.passes.size());
    g.glGenTextures(static_cast<GLsizei>(gpu->targets.size()), gpu->targets.data());
    gpu->textures.resize(preset.textures.size());
    if (!gpu->textures.empty()) {
        g.glGenTextures(static_cast<GLsizei>(gpu->textures.size()), gpu->textures.data());
    }
    for (size_t i = 0; i < preset.textures.size(); ++i) {
        const auto &t = preset.textures[i];
        gpu->texture(gpu->textures[i], t.width, t.height, t.pixels.data(), t.linear);
    }
    for (size_t i = 0; i < preset.passes.size(); ++i) {
        GLuint vertex = compile(*gpu, stage_source(preset.passes[i].source, true), GL_VERTEX_SHADER), fragment = 0;
        try {
            fragment = compile(*gpu, stage_source(preset.passes[i].source, false), GL_FRAGMENT_SHADER);
        } catch (...) {
            g.glDeleteShader(vertex);
            throw;
        }
        GLuint program = g.glCreateProgram();
        g.glAttachShader(program, vertex);
        g.glAttachShader(program, fragment);
        g.glLinkProgram(program);
        g.glDeleteShader(vertex);
        g.glDeleteShader(fragment);
        gpu->programs.push_back(program);
        GLint success = 0;
        g.glGetProgramiv(program, GL_LINK_STATUS, &success);
        if (!success) {
            char log[2048] = {};
            g.glGetProgramInfoLog(program, sizeof(log), nullptr, log);
            throw std::runtime_error(std::string("GLSL link: ") + log);
        }
        GLint count = 0;
        g.glGetProgramiv(program, GL_ACTIVE_UNIFORMS, &count);
        for (GLint u = 0; u < count; ++u) {
            char name[256] = {};
            GLint array_size = 0;
            GLenum type = 0;
            g.glGetActiveUniform(program, static_cast<GLuint>(u), sizeof(name), nullptr, &array_size, &type, name);
            if (array_size != 1 || !supported_uniform(name, preset, i)) {
                throw std::runtime_error(
                    std::string("Unsupported shader uniform (temporal feedback is unavailable): ") + name);
            }
        }
        g.glGetProgramiv(program, GL_ACTIVE_ATTRIBUTES, &count);
        for (GLint a = 0; a < count; ++a) {
            char name[256] = {};
            GLint array_size = 0;
            GLenum type = 0;
            g.glGetActiveAttrib(program, static_cast<GLuint>(a), sizeof(name), nullptr, &array_size, &type, name);
            if (std::strcmp(name, "VertexCoord") && std::strcmp(name, "TexCoord") &&
                std::strcmp(name, "OrigTexCoord")) {
                throw std::runtime_error(std::string("Unsupported shader vertex attribute: ") + name);
            }
        }
    }
    if (g.glGetError() != GL_NO_ERROR) {
        throw std::runtime_error("GPU resource allocation failed");
    }
    return gpu;
}

unsigned dimension(const std::string &type, float scale, unsigned source, unsigned viewport) {
    double value = scale * static_cast<double>(type == "absolute" ? 1 : type == "viewport" ? viewport : source);
    if (!std::isfinite(value) || value < 1 || value > 8192) {
        throw std::runtime_error("Shader output dimensions must be 1 to 8192 pixels");
    }
    return static_cast<unsigned>(std::round(value));
}
} // namespace

struct NesShaderPreset {
    cupid::shader::Preset preset;
    std::unique_ptr<Gpu> gpu;
    std::string path;
    std::vector<uint32_t> pixels;
    uint64_t frame = 0;
    bool enabled = false;
};

NesShaderPreset *nes_shader_create(void) {
    return new (std::nothrow) NesShaderPreset;
}

void nes_shader_destroy(NesShaderPreset *shader) {
    delete shader;
}

bool nes_shader_load(NesShaderPreset *shader, const char *path, char *error, size_t size) {
    if (!shader || !path || !*path) {
        return fail(error, size, "Select a GLSL preset");
    }
    try {
        cupid::shader::Preset candidate;
        std::string why;
        if (!cupid::shader::read_preset(path, candidate, why)) {
            return fail(error, size, why);
        }
        std::string next_path(path);
        auto gpu = build(candidate);
        shader->gpu = std::move(gpu);
        shader->preset = std::move(candidate);
        shader->path = std::move(next_path);
        shader->enabled = true;
        shader->frame = 0;
        if (error && size) {
            error[0] = 0;
        }
        return true;
    } catch (const std::exception &e) {
        return fail(error, size, e.what());
    }
}

bool nes_shader_reload(NesShaderPreset *shader, char *error, size_t size) {
    return shader ? nes_shader_load(shader, shader->path.c_str(), error, size) : fail(error, size, "No shader runtime");
}

void nes_shader_enable(NesShaderPreset *shader, bool enabled) {
    if (shader) {
        shader->enabled = enabled && shader->gpu != nullptr;
    }
}

bool nes_shader_enabled(const NesShaderPreset *shader) {
    return shader && shader->enabled;
}

const char *nes_shader_path(const NesShaderPreset *shader) {
    return shader ? shader->path.c_str() : "";
}

size_t nes_shader_parameter_count(const NesShaderPreset *shader) {
    return shader ? shader->preset.parameters.size() : 0;
}

bool nes_shader_parameter(const NesShaderPreset *shader, size_t index, NesShaderParameter *parameter) {
    if (!shader || !parameter || index >= shader->preset.parameters.size()) {
        return false;
    }
    const auto &p = shader->preset.parameters[index];
    *parameter = {p.name.c_str(), p.label.c_str(), p.value, p.minimum, p.maximum, p.step};
    return true;
}

bool nes_shader_set_parameter(NesShaderPreset *shader, size_t index, float value, char *error, size_t size) {
    if (!shader || index >= shader->preset.parameters.size() || !std::isfinite(value)) {
        return fail(error, size, "Invalid shader parameter");
    }
    auto &p = shader->preset.parameters[index];
    if (value < p.minimum || value > p.maximum) {
        return fail(error, size, "Shader parameter is outside its declared bounds");
    }
    p.value = value;
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

bool nes_shader_apply_parameters(NesShaderPreset *shader, const NesShaderParameterValue *values, size_t count,
                                 char *error, size_t size) {
    if (!shader || (count && !values) || count > NES_SHADER_MAX_PARAMETERS) {
        return fail(error, size, "Invalid shader parameter settings");
    }
    size_t indices[NES_SHADER_MAX_PARAMETERS];
    for (size_t i = 0; i < count; ++i) {
        if (!std::memchr(values[i].name, 0, sizeof(values[i].name)) || !std::isfinite(values[i].value)) {
            return fail(error, size, "Invalid shader parameter name or value");
        }
        auto found = std::find_if(shader->preset.parameters.begin(), shader->preset.parameters.end(),
                                  [&](const cupid::shader::Parameter &p) { return p.name == values[i].name; });
        if (found == shader->preset.parameters.end() || values[i].value < found->minimum ||
            values[i].value > found->maximum) {
            return fail(error, size, "Saved shader parameter is unknown or outside its declared bounds");
        }
        indices[i] = static_cast<size_t>(found - shader->preset.parameters.begin());
        for (size_t j = 0; j < i; ++j) {
            if (indices[j] == indices[i]) {
                return fail(error, size, "Duplicate saved shader parameter");
            }
        }
    }
    for (size_t i = 0; i < count; ++i) {
        shader->preset.parameters[indices[i]].value = values[i].value;
    }
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

bool nes_shader_capture_parameters(const NesShaderPreset *shader, NesShaderParameterValue *values, size_t capacity,
                                   size_t *count) {
    if (!shader || !count || capacity < shader->preset.parameters.size() ||
        (!values && !shader->preset.parameters.empty())) {
        return false;
    }
    *count = shader->preset.parameters.size();
    for (size_t i = 0; i < *count; ++i) {
        std::snprintf(values[i].name, sizeof(values[i].name), "%s", shader->preset.parameters[i].name.c_str());
        values[i].value = shader->preset.parameters[i].value;
    }
    return true;
}

bool nes_shader_render(NesShaderPreset *shader, const NesVideoPresentationFrame *input, unsigned viewport_width,
                       unsigned viewport_height, NesVideoPresentationFrame *output, char *error, size_t size) {
    if (!shader || !input || !output || !input->pixels || !input->width || !input->height) {
        return fail(error, size, "Shader needs a complete presentation frame");
    }
    if (!shader->enabled) {
        *output = *input;
        return true;
    }
    try {
        Current previous;
        auto &gpu = *shader->gpu;
        auto &g = gpu.gl;
        if (SDL_GL_MakeCurrent(gpu.window, gpu.context) != 0) {
            throw std::runtime_error(SDL_GetError());
        }
        while (g.glGetError() != GL_NO_ERROR) {
        }
        g.glActiveTexture(GL_TEXTURE0);
        gpu.texture(gpu.input, input->width, input->height, input->pixels, false);
        std::vector<std::pair<unsigned, unsigned>> dimensions;
        unsigned width = input->width, height = input->height;
        size_t allocated = 0;
        for (size_t i = 0; i < shader->preset.passes.size(); ++i) {
            const auto &pass = shader->preset.passes[i];
            unsigned w = dimension(pass.scale_type_x, pass.scale_x, width, viewport_width);
            unsigned h = dimension(pass.scale_type_y, pass.scale_y, height, viewport_height);
            allocated += static_cast<size_t>(w) * h;
            if (allocated > 32u * 1024u * 1024u) {
                throw std::runtime_error("Shader pass targets exceed 128 MiB");
            }
            dimensions.emplace_back(w, h);
            width = w;
            height = h;
        }
        width = input->width;
        height = input->height;
        g.glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo);
        g.glBindFramebuffer(GL_FRAMEBUFFER, gpu.fbo);
        for (size_t i = 0; i < shader->preset.passes.size(); ++i) {
            const auto &pass = shader->preset.passes[i];
            GLuint program = gpu.programs[i];
            unsigned w = dimensions[i].first, h = dimensions[i].second;
            g.glActiveTexture(GL_TEXTURE0);
            gpu.texture(gpu.targets[i], w, h, nullptr, false);
            g.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, gpu.targets[i], 0);
            if (g.glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
                throw std::runtime_error("GPU framebuffer is incomplete");
            }
            g.glViewport(0, 0, static_cast<GLsizei>(w), static_cast<GLsizei>(h));
            g.glUseProgram(program);
            auto location = [&](const std::string &name) { return g.glGetUniformLocation(program, name.c_str()); };
            auto vector = [&](const std::string &name, unsigned x, unsigned y) {
                g.glUniform2f(location(name), static_cast<float>(x), static_cast<float>(y));
            };
            GLint unit = 0;
            auto sampler = [&](const std::string &name, GLuint texture) {
                GLint loc = location(name);
                if (loc < 0) {
                    return;
                }
                if (unit >= gpu.max_units) {
                    throw std::runtime_error("Shader uses more samplers than this GPU supports");
                }
                g.glActiveTexture(GL_TEXTURE0 + unit);
                g.glBindTexture(GL_TEXTURE_2D, texture);
                g.glUniform1i(loc, unit++);
            };
            GLuint source = i ? gpu.targets[i - 1] : gpu.input;
            g.glBindTexture(GL_TEXTURE_2D, source);
            g.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, pass.linear ? GL_LINEAR : GL_NEAREST);
            g.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, pass.linear ? GL_LINEAR : GL_NEAREST);
            sampler("Texture", source);
            sampler("OrigTexture", gpu.input);
            vector("InputSize", width, height);
            vector("TextureSize", width, height);
            vector("OutputSize", w, h);
            vector("OrigInputSize", input->width, input->height);
            vector("OrigTextureSize", input->width, input->height);
            g.glUniform1i(location("FrameCount"),
                          static_cast<GLint>(shader->frame % (pass.frame_mod ? pass.frame_mod : 0x7fffffffu)));
            g.glUniform1i(location("FrameDirection"), 1);
            const GLfloat matrix[] = {2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 1, 0, -1, -1, 0, 1};
            g.glUniformMatrix4fv(location("MVPMatrix"), 1, GL_FALSE, matrix);
            for (size_t t = 0; t < shader->preset.textures.size(); ++t) {
                sampler(shader->preset.textures[t].name, gpu.textures[t]);
            }
            for (size_t p = 0; p < i; ++p) {
                for (const auto &prefix : {"Pass" + std::to_string(p + 1), "PassPrev" + std::to_string(i - p),
                                           shader->preset.passes[p].alias}) {
                    if (prefix.empty()) {
                        continue;
                    }
                    sampler(prefix + "Texture", gpu.targets[p]);
                    vector(prefix + "InputSize", dimensions[p].first, dimensions[p].second);
                    vector(prefix + "TextureSize", dimensions[p].first, dimensions[p].second);
                }
            }
            for (const auto &parameter : shader->preset.parameters) {
                g.glUniform1f(location(parameter.name), parameter.value);
            }
            std::vector<GLuint> attributes;
            for (const char *name : {"VertexCoord", "TexCoord", "OrigTexCoord"}) {
                GLint attribute = g.glGetAttribLocation(program, name);
                if (attribute < 0) {
                    continue;
                }
                bool position = !std::strcmp(name, "VertexCoord");
                g.glEnableVertexAttribArray(static_cast<GLuint>(attribute));
                attributes.push_back(static_cast<GLuint>(attribute));
                g.glVertexAttribPointer(static_cast<GLuint>(attribute), position ? 4 : 2, GL_FLOAT, GL_FALSE,
                                        6 * sizeof(GLfloat),
                                        reinterpret_cast<const void *>(position ? 0 : 4 * sizeof(GLfloat)));
            }
            g.glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            for (GLuint attribute : attributes) {
                g.glDisableVertexAttribArray(attribute);
            }
            width = w;
            height = h;
        }
        shader->pixels.resize(static_cast<size_t>(width) * height);
        g.glReadPixels(0, 0, static_cast<GLsizei>(width), static_cast<GLsizei>(height), GL_BGRA, GL_UNSIGNED_BYTE,
                       shader->pixels.data());
        if (g.glGetError() != GL_NO_ERROR) {
            throw std::runtime_error("GPU shader rendering failed; ordinary output remains available");
        }
        *output = *input;
        output->pixels = shader->pixels.data();
        output->width = width;
        output->height = height;
        ++shader->frame;
        if (error && size) {
            error[0] = 0;
        }
        return true;
    } catch (const std::exception &e) {
        return fail(error, size, e.what());
    }
}
