/**
 * @file occupancy_grid.cpp
 *
 * Storage encoding (R8 single-channel texture):
 *   byte 0          → unknown sentinel
 *   byte 1..255     → occupancy 0..100% (linear; byte = round(1 + occ/100*254))
 *
 * The shader checks for the `0` sentinel and otherwise lerps free→occupied.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include "scene/renderable/occupancy_grid.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "glad/glad.h"
#include <glm/gtc/type_ptr.hpp>

#include "scene/shader.hpp"

namespace quickviz {
namespace {

constexpr uint8_t kUnknownByte = 0;

// Two-triangle quad covering the unit square in XY (Z=0). Layout:
//   x  y  u  v
// Vertex order: top-left, bottom-left, bottom-right, top-left, bottom-right, top-right.
// Texture coords:
//   u: 0..1 along X axis; v: 0..1 along Y axis.
const float kQuadVertices[] = {
    0.0f, 1.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 1.0f,
    1.0f, 0.0f, 1.0f, 0.0f,
    1.0f, 1.0f, 1.0f, 1.0f,
};

const std::string kVertexShader = R"(
#version 330 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aTex;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform mat4 coordSystemTransform;

uniform vec3  uOrigin;
uniform vec3  uExtent;       // xy = (width*resolution, height*resolution); z unused

out vec2 vTex;

void main() {
    // Position the unit quad at uOrigin with extent uExtent.xy.
    vec3 world = uOrigin + vec3(aPos * uExtent.xy, 0.0);
    gl_Position = projection * view * coordSystemTransform * model * vec4(world, 1.0);
    vTex = aTex;
}
)";

const std::string kFragmentShader = R"(
#version 330 core
in vec2 vTex;
out vec4 FragColor;

uniform sampler2D uTex;
uniform vec4 uFree;
uniform vec4 uOccupied;
uniform vec4 uUnknown;

void main() {
    float r = texture(uTex, vTex).r;       // 0..1
    // Sentinel: byte 0 == 0.0 means "unknown".
    // Sample of byte 0 is exactly 0.0; bytes 1..255 are >= 1/255.
    if (r < 0.5/255.0) {
        FragColor = uUnknown;
    } else {
        // Reverse the encoding: occupancy = (byte - 1) / 254.
        float occupancy = (r * 255.0 - 1.0) / 254.0;
        FragColor = mix(uFree, uOccupied, clamp(occupancy, 0.0, 1.0));
    }
}
)";

uint8_t EncodeOccupancy(int8_t v) {
  if (v < 0) return kUnknownByte;
  // Clamp into [0, 100], then map linearly to [1, 255].
  const int clamped = std::clamp(static_cast<int>(v), 0, 100);
  // 1 + round(clamped * 254 / 100).
  return static_cast<uint8_t>(1 + (clamped * 254 + 50) / 100);
}

}  // namespace

OccupancyGrid::OccupancyGrid() { AllocateGpuResources(); }

OccupancyGrid::~OccupancyGrid() { ReleaseGpuResources(); }

void OccupancyGrid::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;

  Shader vs(kVertexShader.c_str(), Shader::Type::kVertex);
  Shader fs(kFragmentShader.c_str(), Shader::Type::kFragment);
  if (!vs.Compile()) {
    throw std::runtime_error("OccupancyGrid: vertex shader compile failed");
  }
  if (!fs.Compile()) {
    throw std::runtime_error("OccupancyGrid: fragment shader compile failed");
  }
  shader_.AttachShader(vs);
  shader_.AttachShader(fs);
  if (!shader_.LinkProgram()) {
    throw std::runtime_error("OccupancyGrid: shader program link failed");
  }

  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(kQuadVertices), kQuadVertices,
               GL_STATIC_DRAW);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                        (void*)0);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                        (void*)(2 * sizeof(float)));
  glEnableVertexAttribArray(1);
  glBindVertexArray(0);

  // Texture is created lazily on the first SetGrid; AllocateGpuResources
  // only sets up shader + VAO/VBO so the renderable is safe to draw
  // before the first SetGrid (it'll just render nothing).
}

void OccupancyGrid::ReleaseGpuResources() noexcept {
  if (texture_id_) {
    glDeleteTextures(1, &texture_id_);
    texture_id_ = 0;
  }
  if (vbo_) {
    glDeleteBuffers(1, &vbo_);
    vbo_ = 0;
  }
  if (vao_) {
    glDeleteVertexArrays(1, &vao_);
    vao_ = 0;
  }
}

void OccupancyGrid::SetGrid(uint32_t width, uint32_t height, float resolution,
                            const glm::vec3& origin,
                            const std::vector<int8_t>& values) {
  if (width == 0 || height == 0) {
    width_ = height_ = 0;
    origin_ = origin;
    resolution_ = resolution;
    return;
  }
  if (resolution <= 0.0f) {
    throw std::invalid_argument(
        "OccupancyGrid::SetGrid: resolution must be positive");
  }
  if (values.size() != static_cast<std::size_t>(width) * height) {
    throw std::invalid_argument(
        "OccupancyGrid::SetGrid: values.size() must equal width * height");
  }

  EnsureTextureSize(width, height);
  resolution_ = resolution;
  origin_ = origin;
  UploadTexel(values);
}

void OccupancyGrid::EnsureTextureSize(uint32_t width, uint32_t height) {
  if (texture_id_ == 0) {
    glGenTextures(1, &texture_id_);
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  }

  if (width_ != width || height_ != height) {
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0, GL_RED,
                 GL_UNSIGNED_BYTE, nullptr);
    width_ = width;
    height_ = height;
  }
}

void OccupancyGrid::UploadTexel(const std::vector<int8_t>& values) {
  std::vector<uint8_t> packed;
  packed.reserve(values.size());
  for (auto v : values) packed.push_back(EncodeOccupancy(v));

  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_, GL_RED,
                  GL_UNSIGNED_BYTE, packed.data());
}

void OccupancyGrid::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                           const glm::mat4& coord_transform) {
  if (width_ == 0 || height_ == 0 || texture_id_ == 0) return;

  shader_.Use();
  shader_.SetUniform("projection", projection);
  shader_.SetUniform("view", view);
  shader_.SetUniform("model", GetTransform());
  shader_.SetUniform("coordSystemTransform", coord_transform);
  shader_.SetUniform("uOrigin", origin_);
  shader_.SetUniform(
      "uExtent",
      glm::vec3(static_cast<float>(width_) * resolution_,
                static_cast<float>(height_) * resolution_, 0.0f));
  shader_.SetUniform("uFree", free_color_);
  shader_.SetUniform("uOccupied", occupied_color_);
  shader_.SetUniform("uUnknown", unknown_color_);

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture_id_);
  shader_.SetUniform("uTex", 0);

  glBindVertexArray(vao_);
  glDrawArrays(GL_TRIANGLES, 0, 6);
  glBindVertexArray(0);
}

}  // namespace quickviz
