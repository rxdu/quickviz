/**
 * @file arrow.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Implementation of arrow renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/arrow.hpp"

#include <iostream>
#include <cmath>

#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/vector_angle.hpp>

#include "gldraw/shader.hpp"

namespace quickviz {

namespace {
const char* kArrowVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

out vec3 FragPos;
out vec3 Normal;

uniform mat4 mvp;
uniform mat4 model;

void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
    gl_Position = mvp * vec4(aPos, 1.0);
}
)";

const char* kArrowFragmentShader = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;

out vec4 FragColor;

uniform vec3 color;
uniform vec3 lightPos;
uniform vec3 viewPos;

void main() {
    // Ambient
    float ambientStrength = 0.3;
    vec3 ambient = ambientStrength * color;
    
    // Diffuse
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * color;
    
    // Specular
    float specularStrength = 0.5;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularStrength * spec * vec3(1.0, 1.0, 1.0);
    
    vec3 result = ambient + diffuse + specular;
    FragColor = vec4(result, 1.0);
}
)";

const char* kSimpleArrowVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 mvp;

void main() {
    gl_Position = mvp * vec4(aPos, 1.0);
}
)";

const char* kSimpleArrowFragmentShader = R"(
#version 330 core
out vec4 FragColor;
uniform vec3 color;

void main() {
    FragColor = vec4(color, 1.0);
}
)";

}  // namespace

Arrow::Arrow() {
  GenerateArrowGeometry();
}

Arrow::Arrow(const glm::vec3& start, const glm::vec3& end) 
    : start_point_(start), end_point_(end) {
  GenerateArrowGeometry();
}

Arrow::~Arrow() {
  if (IsGpuResourcesAllocated()) {
    ReleaseGpuResources();
  }
}

void Arrow::SetStartPoint(const glm::vec3& start) {
  start_point_ = start;
  needs_update_ = true;
}

void Arrow::SetEndPoint(const glm::vec3& end) {
  end_point_ = end;
  needs_update_ = true;
}

void Arrow::SetDirection(const glm::vec3& origin, const glm::vec3& direction, float length) {
  start_point_ = origin;
  end_point_ = origin + glm::normalize(direction) * length;
  needs_update_ = true;
}

void Arrow::SetColor(const glm::vec3& color) {
  color_ = color;
}

void Arrow::SetShaftRadius(float radius) {
  shaft_radius_ = radius;
  needs_update_ = true;
}

void Arrow::SetHeadRadius(float radius) {
  head_radius_ = radius;
  needs_update_ = true;
}

void Arrow::SetHeadLengthRatio(float ratio) {
  head_length_ratio_ = ratio;
  needs_update_ = true;
}

void Arrow::SetResolution(int segments) {
  segments_ = segments;
  needs_update_ = true;
}

void Arrow::SetShowAsLine(bool as_line) {
  show_as_line_ = as_line;
  needs_update_ = true;
}

float Arrow::GetLength() const {
  return glm::length(end_point_ - start_point_);
}

glm::vec3 Arrow::GetDirection() const {
  glm::vec3 dir = end_point_ - start_point_;
  if (glm::length(dir) > 0) {
    return glm::normalize(dir);
  }
  return glm::vec3(0, 0, 1);
}

void Arrow::GenerateCylinder(std::vector<glm::vec3>& vertices,
                             std::vector<uint32_t>& indices,
                             const glm::vec3& base, const glm::vec3& top,
                             float radius, int segments) {
  glm::vec3 axis = top - base;
  float height = glm::length(axis);
  if (height == 0) return;
  
  axis = glm::normalize(axis);
  
  // Find perpendicular vectors
  glm::vec3 perp1, perp2;
  if (std::abs(axis.y) < 0.9f) {
    perp1 = glm::normalize(glm::cross(axis, glm::vec3(0, 1, 0)));
  } else {
    perp1 = glm::normalize(glm::cross(axis, glm::vec3(1, 0, 0)));
  }
  perp2 = glm::cross(axis, perp1);
  
  size_t base_idx = vertices.size();
  
  // Generate vertices
  for (int i = 0; i <= segments; ++i) {
    float angle = 2.0f * M_PI * i / segments;
    float cos_a = cos(angle);
    float sin_a = sin(angle);
    
    glm::vec3 offset = radius * (cos_a * perp1 + sin_a * perp2);
    
    // Bottom vertex
    vertices.push_back(base + offset);
    // Top vertex
    vertices.push_back(top + offset);
  }
  
  // Generate indices for cylinder sides
  for (int i = 0; i < segments; ++i) {
    size_t idx = base_idx + i * 2;
    
    // Triangle 1
    indices.push_back(idx);
    indices.push_back(idx + 2);
    indices.push_back(idx + 1);
    
    // Triangle 2
    indices.push_back(idx + 1);
    indices.push_back(idx + 2);
    indices.push_back(idx + 3);
  }
}

void Arrow::GenerateCone(std::vector<glm::vec3>& vertices,
                        std::vector<uint32_t>& indices,
                        const glm::vec3& base, const glm::vec3& tip,
                        float radius, int segments) {
  glm::vec3 axis = tip - base;
  float height = glm::length(axis);
  if (height == 0) return;
  
  axis = glm::normalize(axis);
  
  // Find perpendicular vectors
  glm::vec3 perp1, perp2;
  if (std::abs(axis.y) < 0.9f) {
    perp1 = glm::normalize(glm::cross(axis, glm::vec3(0, 1, 0)));
  } else {
    perp1 = glm::normalize(glm::cross(axis, glm::vec3(1, 0, 0)));
  }
  perp2 = glm::cross(axis, perp1);
  
  size_t base_idx = vertices.size();
  
  // Add tip vertex
  vertices.push_back(tip);
  
  // Add base vertices
  for (int i = 0; i <= segments; ++i) {
    float angle = 2.0f * M_PI * i / segments;
    float cos_a = cos(angle);
    float sin_a = sin(angle);
    
    glm::vec3 offset = radius * (cos_a * perp1 + sin_a * perp2);
    vertices.push_back(base + offset);
  }
  
  // Generate indices for cone sides
  for (int i = 0; i < segments; ++i) {
    indices.push_back(base_idx);  // Tip
    indices.push_back(base_idx + i + 1);
    indices.push_back(base_idx + i + 2);
  }
  
  // Add base cap
  size_t center_idx = vertices.size();
  vertices.push_back(base);  // Center of base
  
  for (int i = 0; i < segments; ++i) {
    indices.push_back(center_idx);
    indices.push_back(base_idx + i + 2);
    indices.push_back(base_idx + i + 1);
  }
}

void Arrow::GenerateArrowGeometry() {
  shaft_vertices_.clear();
  shaft_indices_.clear();
  head_vertices_.clear();
  head_indices_.clear();
  
  float total_length = GetLength();
  if (total_length == 0) return;
  
  glm::vec3 direction = GetDirection();
  float head_length = total_length * head_length_ratio_;
  float shaft_length = total_length - head_length;
  
  glm::vec3 shaft_end = start_point_ + direction * shaft_length;
  
  if (show_as_line_) {
    // Simple line for shaft
    shaft_vertices_.push_back(start_point_);
    shaft_vertices_.push_back(shaft_end);
    shaft_indices_.push_back(0);
    shaft_indices_.push_back(1);
  } else {
    // Generate cylinder for shaft
    GenerateCylinder(shaft_vertices_, shaft_indices_,
                    start_point_, shaft_end,
                    shaft_radius_, segments_);
  }
  
  // Generate cone for head
  GenerateCone(head_vertices_, head_indices_,
              shaft_end, end_point_,
              head_radius_, segments_);
}

void Arrow::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;
  
  try {
    // Compile appropriate shader based on rendering mode
    if (show_as_line_) {
      Shader vs(kSimpleArrowVertexShader, Shader::Type::kVertex);
      Shader fs(kSimpleArrowFragmentShader, Shader::Type::kFragment);
      if (!vs.Compile() || !fs.Compile()) {
        throw std::runtime_error("Simple arrow shader compilation failed");
      }
      shader_.AttachShader(vs);
      shader_.AttachShader(fs);
    } else {
      Shader vs(kArrowVertexShader, Shader::Type::kVertex);
      Shader fs(kArrowFragmentShader, Shader::Type::kFragment);
      if (!vs.Compile() || !fs.Compile()) {
        throw std::runtime_error("Arrow shader compilation failed");
      }
      shader_.AttachShader(vs);
      shader_.AttachShader(fs);
    }
    
    if (!shader_.LinkProgram()) {
      throw std::runtime_error("Arrow shader linking failed");
    }
    
    // Create VAOs and VBOs
    glGenVertexArrays(1, &vao_shaft_);
    glGenBuffers(1, &vbo_shaft_vertices_);
    glGenBuffers(1, &vbo_shaft_indices_);
    
    glGenVertexArrays(1, &vao_head_);
    glGenBuffers(1, &vbo_head_vertices_);
    glGenBuffers(1, &vbo_head_indices_);
    
    UpdateGpuBuffers();
    
  } catch (const std::exception& e) {
    std::cerr << "Arrow::AllocateGpuResources: " << e.what() << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void Arrow::ReleaseGpuResources() noexcept {
  if (vao_shaft_ != 0) {
    glDeleteVertexArrays(1, &vao_shaft_);
    vao_shaft_ = 0;
  }
  if (vbo_shaft_vertices_ != 0) {
    glDeleteBuffers(1, &vbo_shaft_vertices_);
    vbo_shaft_vertices_ = 0;
  }
  if (vbo_shaft_indices_ != 0) {
    glDeleteBuffers(1, &vbo_shaft_indices_);
    vbo_shaft_indices_ = 0;
  }
  if (vao_head_ != 0) {
    glDeleteVertexArrays(1, &vao_head_);
    vao_head_ = 0;
  }
  if (vbo_head_vertices_ != 0) {
    glDeleteBuffers(1, &vbo_head_vertices_);
    vbo_head_vertices_ = 0;
  }
  if (vbo_head_indices_ != 0) {
    glDeleteBuffers(1, &vbo_head_indices_);
    vbo_head_indices_ = 0;
  }
}

void Arrow::UpdateGpuBuffers() {
  if (!IsGpuResourcesAllocated()) return;
  
  // Update shaft VAO
  if (!shaft_vertices_.empty()) {
    glBindVertexArray(vao_shaft_);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo_shaft_vertices_);
    glBufferData(GL_ARRAY_BUFFER, shaft_vertices_.size() * sizeof(glm::vec3),
                 shaft_vertices_.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(0);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_shaft_indices_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, shaft_indices_.size() * sizeof(uint32_t),
                 shaft_indices_.data(), GL_DYNAMIC_DRAW);
    
    glBindVertexArray(0);
  }
  
  // Update head VAO
  if (!head_vertices_.empty()) {
    glBindVertexArray(vao_head_);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo_head_vertices_);
    glBufferData(GL_ARRAY_BUFFER, head_vertices_.size() * sizeof(glm::vec3),
                 head_vertices_.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(0);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_head_indices_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, head_indices_.size() * sizeof(uint32_t),
                 head_indices_.data(), GL_DYNAMIC_DRAW);
    
    glBindVertexArray(0);
  }
  
  needs_update_ = false;
}

void Arrow::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                   const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }
  
  if (needs_update_) {
    GenerateArrowGeometry();
    UpdateGpuBuffers();
  }
  
  if (shaft_vertices_.empty() || head_vertices_.empty()) return;
  
  glm::mat4 mvp = projection * view * coord_transform;
  
  shader_.Use();
  shader_.SetUniform("mvp", mvp);
  shader_.SetUniform("color", color_);
  
  if (!show_as_line_) {
    shader_.TrySetUniform("model", coord_transform);
    shader_.TrySetUniform("lightPos", glm::vec3(10, 10, 10));
    shader_.TrySetUniform("viewPos", glm::vec3(0, 0, 5));
  }
  
  // Draw shaft
  glBindVertexArray(vao_shaft_);
  if (show_as_line_) {
    glLineWidth(2.0f);
    glDrawElements(GL_LINES, shaft_indices_.size(), GL_UNSIGNED_INT, nullptr);
  } else {
    glDrawElements(GL_TRIANGLES, shaft_indices_.size(), GL_UNSIGNED_INT, nullptr);
  }
  glBindVertexArray(0);
  
  // Draw head
  glBindVertexArray(vao_head_);
  glDrawElements(GL_TRIANGLES, head_indices_.size(), GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}

}  // namespace quickviz