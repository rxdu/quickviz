/**
 * @file tf_frame_tree.cpp
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include "scene/renderable/tf_frame_tree.hpp"

#include <stdexcept>
#include <unordered_set>

#include "glad/glad.h"
#include <glm/gtc/type_ptr.hpp>

#include "scene/shader.hpp"

namespace quickviz {
namespace {

const std::string kVertexShader = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform mat4 coordSystemTransform;

out vec3 vColor;

void main() {
    gl_Position = projection * view * coordSystemTransform * model * vec4(aPos, 1.0);
    vColor = aColor;
}
)";

const std::string kFragmentShader = R"(
#version 330 core
in vec3 vColor;
out vec4 FragColor;
void main() { FragColor = vec4(vColor, 1.0); }
)";

}  // namespace

TfFrameTree::TfFrameTree(float axis_length) : axis_length_(axis_length) {
  AllocateGpuResources();
}

TfFrameTree::~TfFrameTree() { ReleaseGpuResources(); }

void TfFrameTree::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;

  Shader vs(kVertexShader.c_str(), Shader::Type::kVertex);
  Shader fs(kFragmentShader.c_str(), Shader::Type::kFragment);
  if (!vs.Compile()) {
    throw std::runtime_error("TfFrameTree: vertex shader compile failed");
  }
  if (!fs.Compile()) {
    throw std::runtime_error("TfFrameTree: fragment shader compile failed");
  }
  shader_.AttachShader(vs);
  shader_.AttachShader(fs);
  if (!shader_.LinkProgram()) {
    throw std::runtime_error("TfFrameTree: shader program link failed");
  }

  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        (void*)offsetof(Vertex, position));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        (void*)offsetof(Vertex, color));
  glEnableVertexAttribArray(1);
  glBindVertexArray(0);
}

void TfFrameTree::ReleaseGpuResources() noexcept {
  if (vbo_) {
    glDeleteBuffers(1, &vbo_);
    vbo_ = 0;
  }
  if (vao_) {
    glDeleteVertexArrays(1, &vao_);
    vao_ = 0;
  }
}

void TfFrameTree::SetFrame(const std::string& name, const std::string& parent,
                           const glm::mat4& transform_in_parent) {
  if (name.empty()) {
    throw std::invalid_argument("TfFrameTree::SetFrame: name cannot be empty");
  }
  if (name == parent) {
    throw std::invalid_argument(
        "TfFrameTree::SetFrame: a frame cannot be its own parent");
  }
  Frame& f = frames_[name];
  f.parent = parent;
  f.local = transform_in_parent;
  dirty_ = true;
}

std::size_t TfFrameTree::RemoveFrame(const std::string& name) {
  if (frames_.find(name) == frames_.end()) return 0;

  // Collect descendants by name (children-of relation).
  std::unordered_set<std::string> to_remove;
  to_remove.insert(name);

  bool changed = true;
  while (changed) {
    changed = false;
    for (const auto& [n, f] : frames_) {
      if (to_remove.count(n)) continue;
      if (to_remove.count(f.parent)) {
        to_remove.insert(n);
        changed = true;
      }
    }
  }

  for (const auto& n : to_remove) frames_.erase(n);
  dirty_ = true;
  return to_remove.size();
}

void TfFrameTree::Clear() {
  frames_.clear();
  dirty_ = true;
}

bool TfFrameTree::HasFrame(const std::string& name) const {
  return frames_.find(name) != frames_.end();
}

glm::mat4 TfFrameTree::GetWorldTransform(const std::string& name) const {
  auto it = frames_.find(name);
  if (it == frames_.end()) return glm::mat4(1.0f);

  // Walk parents, accumulating transforms. Detect cycles by visited set.
  glm::mat4 accumulated = it->second.local;
  std::string current = it->second.parent;
  std::unordered_set<std::string> visited{name};
  while (!current.empty()) {
    if (visited.count(current)) {
      // Cycle: bail out, return identity for safety.
      return glm::mat4(1.0f);
    }
    visited.insert(current);
    auto parent_it = frames_.find(current);
    if (parent_it == frames_.end()) {
      // Broken chain: parent referenced but not registered. Treat as
      // root for visualization purposes (the missing parent is
      // implicitly identity).
      break;
    }
    accumulated = parent_it->second.local * accumulated;
    current = parent_it->second.parent;
  }
  return accumulated;
}

void TfFrameTree::SetAxisLength(float length) {
  if (length <= 0.0f) {
    throw std::invalid_argument(
        "TfFrameTree::SetAxisLength: length must be positive");
  }
  axis_length_ = length;
  dirty_ = true;
}

void TfFrameTree::RebuildVertices() {
  vertices_.clear();
  if (frames_.empty()) {
    dirty_ = false;
    return;
  }

  // Pre-compute world transforms once.
  std::unordered_map<std::string, glm::mat4> world_xforms;
  world_xforms.reserve(frames_.size());
  for (const auto& [name, _] : frames_) {
    world_xforms.emplace(name, GetWorldTransform(name));
  }

  vertices_.reserve(frames_.size() * 6 +
                    (show_connections_ ? frames_.size() * 2 : 0));

  const glm::vec3 red(1.0f, 0.0f, 0.0f);
  const glm::vec3 green(0.0f, 1.0f, 0.0f);
  const glm::vec3 blue(0.0f, 0.0f, 1.0f);

  for (const auto& [name, frame] : frames_) {
    const glm::mat4& W = world_xforms.at(name);
    const glm::vec3 origin(W[3]);
    const glm::vec3 ex = glm::vec3(W * glm::vec4(axis_length_, 0, 0, 1));
    const glm::vec3 ey = glm::vec3(W * glm::vec4(0, axis_length_, 0, 1));
    const glm::vec3 ez = glm::vec3(W * glm::vec4(0, 0, axis_length_, 1));

    vertices_.push_back({origin, red});
    vertices_.push_back({ex, red});
    vertices_.push_back({origin, green});
    vertices_.push_back({ey, green});
    vertices_.push_back({origin, blue});
    vertices_.push_back({ez, blue});

    if (show_connections_ && !frame.parent.empty()) {
      auto parent_it = world_xforms.find(frame.parent);
      if (parent_it != world_xforms.end()) {
        const glm::vec3 parent_origin(parent_it->second[3]);
        vertices_.push_back({parent_origin, connection_color_});
        vertices_.push_back({origin, connection_color_});
      }
    }
  }

  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  const std::size_t bytes = vertices_.size() * sizeof(Vertex);
  if (vertices_.size() > buffer_capacity_) {
    glBufferData(GL_ARRAY_BUFFER, bytes, vertices_.data(), GL_DYNAMIC_DRAW);
    buffer_capacity_ = vertices_.size();
  } else {
    glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, vertices_.data());
  }

  dirty_ = false;
}

void TfFrameTree::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                         const glm::mat4& coord_transform) {
  if (dirty_) RebuildVertices();
  if (vertices_.empty()) return;

  shader_.Use();
  shader_.SetUniform("projection", projection);
  shader_.SetUniform("view", view);
  shader_.SetUniform("model", GetTransform());
  shader_.SetUniform("coordSystemTransform", coord_transform);

  glBindVertexArray(vao_);
  glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertices_.size()));
  glBindVertexArray(0);
}

}  // namespace quickviz
