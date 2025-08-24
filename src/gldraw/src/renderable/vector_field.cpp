/**
 * @file vector_field.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-24
 * @brief Implementation of vector field renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/vector_field.hpp"

#include <iostream>
#include <algorithm>
#include <cmath>

#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/vector_angle.hpp>

#include "gldraw/shader.hpp"

namespace quickviz {

namespace {

// Instanced arrow shader with per-instance transform and color
const char* kInstancedArrowVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in mat4 aInstanceTransform;  // Per-instance transform (takes locations 2-5)
layout (location = 6) in vec3 aInstanceColor;       // Per-instance color

out vec3 FragPos;
out vec3 Normal;
out vec3 Color;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uCoordTransform;

void main() {
    mat4 model = uCoordTransform * aInstanceTransform;
    vec4 worldPos = model * vec4(aPos, 1.0);
    FragPos = vec3(worldPos);
    Normal = mat3(transpose(inverse(model))) * aNormal;
    Color = aInstanceColor;
    gl_Position = uProjection * uView * worldPos;
}
)";

const char* kInstancedArrowFragmentShader = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;
in vec3 Color;

out vec4 FragColor;

uniform vec3 uLightPos;
uniform vec3 uViewPos;
uniform float uOpacity;

void main() {
    // Simple Phong lighting
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(uLightPos - FragPos);
    
    // Ambient
    float ambientStrength = 0.3;
    vec3 ambient = ambientStrength * Color;
    
    // Diffuse
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * Color;
    
    // Specular
    float specularStrength = 0.5;
    vec3 viewDir = normalize(uViewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularStrength * spec * vec3(1.0);
    
    vec3 result = ambient + diffuse + specular;
    FragColor = vec4(result, uOpacity);
}
)";

// Simple line shader
const char* kLineVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 Color;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uCoordTransform;

void main() {
    gl_Position = uProjection * uView * uCoordTransform * vec4(aPos, 1.0);
    Color = aColor;
}
)";

const char* kLineFragmentShader = R"(
#version 330 core
in vec3 Color;
out vec4 FragColor;

uniform float uOpacity;

void main() {
    FragColor = vec4(Color, uOpacity);
}
)";

}  // namespace

VectorField::VectorField() {
  // Initialize with default arrow geometry
  CreateArrowGeometry();
}

VectorField::~VectorField() {
  if (IsGpuResourcesAllocated()) {
    ReleaseGpuResources();
  }
}

void VectorField::SetVectors(const std::vector<glm::vec3>& origins,
                             const std::vector<glm::vec3>& vectors) {
  if (origins.size() != vectors.size()) {
    std::cerr << "VectorField: Origins and vectors must have the same size" << std::endl;
    return;
  }
  
  origins_ = origins;
  vectors_ = vectors;
  
  needs_transform_update_ = true;
  needs_color_update_ = true;
  needs_visibility_update_ = true;
}

void VectorField::AddVector(const glm::vec3& origin, const glm::vec3& vector) {
  origins_.push_back(origin);
  vectors_.push_back(vector);
  
  needs_transform_update_ = true;
  needs_color_update_ = true;
  needs_visibility_update_ = true;
}

void VectorField::ClearVectors() {
  origins_.clear();
  vectors_.clear();
  colors_.clear();
  transforms_.clear();
  instance_colors_.clear();
  visible_indices_.clear();
  
  needs_transform_update_ = true;
  needs_color_update_ = true;
  needs_visibility_update_ = true;
}

void VectorField::GenerateGridField(const glm::vec3& min_corner,
                                    const glm::vec3& max_corner,
                                    const glm::ivec3& resolution,
                                    std::function<glm::vec3(const glm::vec3&)> field_function) {
  ClearVectors();
  
  glm::vec3 step = (max_corner - min_corner) / glm::vec3(resolution);
  
  for (int i = 0; i <= resolution.x; ++i) {
    for (int j = 0; j <= resolution.y; ++j) {
      for (int k = 0; k <= resolution.z; ++k) {
        glm::vec3 pos = min_corner + glm::vec3(i, j, k) * step;
        glm::vec3 vec = field_function(pos);
        
        if (glm::length(vec) > magnitude_threshold_) {
          origins_.push_back(pos);
          vectors_.push_back(vec);
        }
      }
    }
  }
  
  needs_transform_update_ = true;
  needs_color_update_ = true;
  needs_visibility_update_ = true;
}

void VectorField::SetColorMode(ColorMode mode) {
  color_mode_ = mode;
  needs_color_update_ = true;
}

void VectorField::SetUniformColor(const glm::vec3& color) {
  uniform_color_ = color;
  if (color_mode_ == ColorMode::kUniform) {
    needs_color_update_ = true;
  }
}

void VectorField::SetCustomColors(const std::vector<glm::vec3>& colors) {
  colors_ = colors;
  if (color_mode_ == ColorMode::kCustom) {
    needs_color_update_ = true;
  }
}

void VectorField::SetColorRange(float min_magnitude, float max_magnitude) {
  min_magnitude_ = min_magnitude;
  max_magnitude_ = max_magnitude;
  if (color_mode_ == ColorMode::kMagnitude) {
    needs_color_update_ = true;
  }
}

void VectorField::SetRenderStyle(RenderStyle style) {
  render_style_ = style;
  needs_transform_update_ = true;
}

void VectorField::SetArrowScale(float scale) {
  arrow_scale_ = scale;
  needs_transform_update_ = true;
}

void VectorField::SetLineWidth(float width) {
  line_width_ = width;
}

void VectorField::SetOpacity(float opacity) {
  opacity_ = glm::clamp(opacity, 0.0f, 1.0f);
}

void VectorField::SetSubsampling(float ratio) {
  subsampling_ratio_ = glm::clamp(ratio, 0.0f, 1.0f);
  needs_visibility_update_ = true;
}

void VectorField::SetMagnitudeThreshold(float min_magnitude) {
  magnitude_threshold_ = min_magnitude;
  needs_visibility_update_ = true;
}

void VectorField::SetLevelOfDetail(bool enabled, float distance_threshold) {
  use_lod_ = enabled;
  lod_distance_ = distance_threshold;
  needs_visibility_update_ = true;
}

glm::vec3 VectorField::GetVector(size_t index) const {
  return index < vectors_.size() ? vectors_[index] : glm::vec3(0.0f);
}

glm::vec3 VectorField::GetOrigin(size_t index) const {
  return index < origins_.size() ? origins_[index] : glm::vec3(0.0f);
}

float VectorField::GetMaxMagnitude() const {
  float max_mag = 0.0f;
  for (const auto& vec : vectors_) {
    max_mag = std::max(max_mag, glm::length(vec));
  }
  return max_mag;
}

void VectorField::GetBoundingBox(glm::vec3& min_corner, glm::vec3& max_corner) const {
  if (origins_.empty()) {
    min_corner = max_corner = glm::vec3(0.0f);
    return;
  }
  
  min_corner = max_corner = origins_[0];
  for (size_t i = 0; i < origins_.size(); ++i) {
    min_corner = glm::min(min_corner, origins_[i]);
    max_corner = glm::max(max_corner, origins_[i]);
    
    glm::vec3 tip = origins_[i] + vectors_[i];
    min_corner = glm::min(min_corner, tip);
    max_corner = glm::max(max_corner, tip);
  }
}

void VectorField::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;
  
  // Compile shaders
  Shader arrow_vs(kInstancedArrowVertexShader, Shader::Type::kVertex);
  Shader arrow_fs(kInstancedArrowFragmentShader, Shader::Type::kFragment);
  
  arrow_shader_.AttachShader(arrow_vs);
  arrow_shader_.AttachShader(arrow_fs);
  if (!arrow_shader_.LinkProgram()) {
    std::cerr << "VectorField: Failed to link arrow shader" << std::endl;
    return;
  }
  
  Shader line_vs(kLineVertexShader, Shader::Type::kVertex);
  Shader line_fs(kLineFragmentShader, Shader::Type::kFragment);
  
  line_shader_.AttachShader(line_vs);
  line_shader_.AttachShader(line_fs);
  if (!line_shader_.LinkProgram()) {
    std::cerr << "VectorField: Failed to link line shader" << std::endl;
    return;
  }
  
  // Create arrow VAO and buffers
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_vertices_);
  glGenBuffers(1, &vbo_normals_);
  glGenBuffers(1, &vbo_transforms_);
  glGenBuffers(1, &vbo_colors_);
  glGenBuffers(1, &ebo_);
  
  glBindVertexArray(vao_);
  
  // Upload arrow geometry (shared for all instances)
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glBufferData(GL_ARRAY_BUFFER, arrow_vertices_.size() * sizeof(glm::vec3),
               arrow_vertices_.data(), GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(0);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_normals_);
  glBufferData(GL_ARRAY_BUFFER, arrow_normals_.size() * sizeof(glm::vec3),
               arrow_normals_.data(), GL_STATIC_DRAW);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(1);
  
  // Instance transform buffer (mat4 takes 4 attribute locations)
  glBindBuffer(GL_ARRAY_BUFFER, vbo_transforms_);
  for (int i = 0; i < 4; ++i) {
    glVertexAttribPointer(2 + i, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4),
                         reinterpret_cast<void*>(i * sizeof(glm::vec4)));
    glEnableVertexAttribArray(2 + i);
    glVertexAttribDivisor(2 + i, 1);  // Instance attribute
  }
  
  // Instance color buffer
  glBindBuffer(GL_ARRAY_BUFFER, vbo_colors_);
  glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(6);
  glVertexAttribDivisor(6, 1);  // Instance attribute
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, arrow_indices_.size() * sizeof(uint32_t),
               arrow_indices_.data(), GL_STATIC_DRAW);
  
  glBindVertexArray(0);
  
  // Create line VAO and buffers
  glGenVertexArrays(1, &vao_lines_);
  glGenBuffers(1, &vbo_line_vertices_);
  glGenBuffers(1, &vbo_line_colors_);
  
  glBindVertexArray(vao_lines_);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_line_vertices_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(0);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_line_colors_);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(1);
  
  glBindVertexArray(0);
}

void VectorField::ReleaseGpuResources() noexcept {
  if (vao_) {
    glDeleteVertexArrays(1, &vao_);
    vao_ = 0;
  }
  if (vbo_vertices_) {
    glDeleteBuffers(1, &vbo_vertices_);
    vbo_vertices_ = 0;
  }
  if (vbo_normals_) {
    glDeleteBuffers(1, &vbo_normals_);
    vbo_normals_ = 0;
  }
  if (vbo_transforms_) {
    glDeleteBuffers(1, &vbo_transforms_);
    vbo_transforms_ = 0;
  }
  if (vbo_colors_) {
    glDeleteBuffers(1, &vbo_colors_);
    vbo_colors_ = 0;
  }
  if (ebo_) {
    glDeleteBuffers(1, &ebo_);
    ebo_ = 0;
  }
  
  if (vao_lines_) {
    glDeleteVertexArrays(1, &vao_lines_);
    vao_lines_ = 0;
  }
  if (vbo_line_vertices_) {
    glDeleteBuffers(1, &vbo_line_vertices_);
    vbo_line_vertices_ = 0;
  }
  if (vbo_line_colors_) {
    glDeleteBuffers(1, &vbo_line_colors_);
    vbo_line_colors_ = 0;
  }
}

void VectorField::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                        const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) AllocateGpuResources();
  if (origins_.empty()) return;
  
  // Update data if needed
  if (needs_visibility_update_) {
    UpdateVisibility();
    needs_visibility_update_ = false;
  }
  
  if (needs_transform_update_) {
    UpdateTransforms();
    needs_transform_update_ = false;
  }
  
  if (needs_color_update_) {
    UpdateColors();
    needs_color_update_ = false;
  }
  
  if (visible_indices_.empty()) return;
  
  // Enable blending if needed
  if (opacity_ < 1.0f) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }
  
  if (render_style_ == RenderStyle::kLines) {
    // Draw as lines
    UpdateLineGeometry();
    
    glLineWidth(line_width_);
    
    line_shader_.Use();
    line_shader_.SetUniform("uProjection", projection);
    line_shader_.SetUniform("uView", view);
    line_shader_.SetUniform("uCoordTransform", coord_transform);
    line_shader_.SetUniform("uOpacity", opacity_);
    
    glBindVertexArray(vao_lines_);
    
    // Update line vertex data
    glBindBuffer(GL_ARRAY_BUFFER, vbo_line_vertices_);
    glBufferData(GL_ARRAY_BUFFER, line_vertices_.size() * sizeof(glm::vec3),
                 line_vertices_.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo_line_colors_);
    glBufferData(GL_ARRAY_BUFFER, line_colors_.size() * sizeof(glm::vec3),
                 line_colors_.data(), GL_DYNAMIC_DRAW);
    
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(line_vertices_.size()));
    glBindVertexArray(0);
    
    glLineWidth(1.0f);
  } else {
    // Draw as 3D arrows using instancing
    arrow_shader_.Use();
    arrow_shader_.SetUniform("uProjection", projection);
    arrow_shader_.SetUniform("uView", view);
    arrow_shader_.SetUniform("uCoordTransform", coord_transform);
    arrow_shader_.SetUniform("uOpacity", opacity_);
    
    // Set lighting
    glm::vec3 view_pos = glm::vec3(glm::inverse(view)[3]);
    glm::vec3 light_pos = view_pos + glm::vec3(5.0f, 5.0f, 5.0f);
    arrow_shader_.SetUniform("uLightPos", light_pos);
    arrow_shader_.SetUniform("uViewPos", view_pos);
    
    glBindVertexArray(vao_);
    
    // Update instance data for visible vectors
    std::vector<glm::mat4> visible_transforms;
    std::vector<glm::vec3> visible_colors;
    
    for (uint32_t idx : visible_indices_) {
      visible_transforms.push_back(transforms_[idx]);
      visible_colors.push_back(instance_colors_[idx]);
    }
    
    // Upload instance transforms
    glBindBuffer(GL_ARRAY_BUFFER, vbo_transforms_);
    glBufferData(GL_ARRAY_BUFFER, visible_transforms.size() * sizeof(glm::mat4),
                 visible_transforms.data(), GL_DYNAMIC_DRAW);
    
    // Upload instance colors
    glBindBuffer(GL_ARRAY_BUFFER, vbo_colors_);
    glBufferData(GL_ARRAY_BUFFER, visible_colors.size() * sizeof(glm::vec3),
                 visible_colors.data(), GL_DYNAMIC_DRAW);
    
    // Draw all instances
    glDrawElementsInstanced(GL_TRIANGLES, 
                           static_cast<GLsizei>(arrow_indices_.size()),
                           GL_UNSIGNED_INT, 
                           nullptr,
                           static_cast<GLsizei>(visible_indices_.size()));
    
    glBindVertexArray(0);
  }
  
  if (opacity_ < 1.0f) {
    glDisable(GL_BLEND);
  }
}

void VectorField::CreateArrowGeometry() {
  // Create a single arrow mesh pointing in +Z direction, unit length
  // This will be transformed per-instance to match each vector
  
  arrow_vertices_.clear();
  arrow_normals_.clear();
  arrow_indices_.clear();
  
  const float shaft_length = 0.7f;
  const float head_length = 0.3f;
  const float shaft_radius = 0.03f;
  const float head_radius = 0.08f;
  const int segments = 8;
  
  // Create shaft as octagonal prism
  for (int i = 0; i < segments; ++i) {
    float angle = 2.0f * M_PI * i / segments;
    float x = shaft_radius * cos(angle);
    float y = shaft_radius * sin(angle);
    
    // Bottom circle
    arrow_vertices_.push_back(glm::vec3(x, y, 0.0f));
    arrow_normals_.push_back(glm::vec3(x/shaft_radius, y/shaft_radius, 0.0f));
    
    // Top circle
    arrow_vertices_.push_back(glm::vec3(x, y, shaft_length));
    arrow_normals_.push_back(glm::vec3(x/shaft_radius, y/shaft_radius, 0.0f));
  }
  
  // Shaft triangles
  for (int i = 0; i < segments; ++i) {
    int next = (i + 1) % segments;
    int base = i * 2;
    int base_next = next * 2;
    
    // Two triangles per side
    arrow_indices_.push_back(base);
    arrow_indices_.push_back(base + 1);
    arrow_indices_.push_back(base_next);
    
    arrow_indices_.push_back(base_next);
    arrow_indices_.push_back(base + 1);
    arrow_indices_.push_back(base_next + 1);
  }
  
  // Create head as cone
  int head_base_start = arrow_vertices_.size();
  
  // Head base circle
  for (int i = 0; i < segments; ++i) {
    float angle = 2.0f * M_PI * i / segments;
    float x = head_radius * cos(angle);
    float y = head_radius * sin(angle);
    
    arrow_vertices_.push_back(glm::vec3(x, y, shaft_length));
    
    // Cone side normal
    glm::vec3 radial = glm::vec3(x/head_radius, y/head_radius, 0.0f);
    glm::vec3 normal = glm::normalize(radial + glm::vec3(0, 0, head_radius/head_length));
    arrow_normals_.push_back(normal);
  }
  
  // Cone tip
  int tip_index = arrow_vertices_.size();
  arrow_vertices_.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
  arrow_normals_.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
  
  // Cone triangles
  for (int i = 0; i < segments; ++i) {
    int next = (i + 1) % segments;
    
    arrow_indices_.push_back(head_base_start + i);
    arrow_indices_.push_back(head_base_start + next);
    arrow_indices_.push_back(tip_index);
  }
}

void VectorField::UpdateTransforms() {
  transforms_.clear();
  transforms_.reserve(vectors_.size());
  
  for (size_t i = 0; i < vectors_.size(); ++i) {
    transforms_.push_back(ComputeTransformForVector(origins_[i], vectors_[i]));
  }
}

void VectorField::UpdateColors() {
  instance_colors_.clear();
  instance_colors_.reserve(vectors_.size());
  
  for (size_t i = 0; i < vectors_.size(); ++i) {
    instance_colors_.push_back(ComputeColorForVector(vectors_[i], i));
  }
}

void VectorField::UpdateVisibility() {
  visible_indices_.clear();
  
  size_t step = static_cast<size_t>(1.0f / subsampling_ratio_);
  if (step == 0) step = 1;
  
  for (size_t i = 0; i < vectors_.size(); i += step) {
    if (ShouldRenderVector(vectors_[i], i)) {
      visible_indices_.push_back(static_cast<uint32_t>(i));
    }
  }
}

void VectorField::UpdateLineGeometry() {
  line_vertices_.clear();
  line_colors_.clear();
  
  for (uint32_t idx : visible_indices_) {
    glm::vec3 origin = origins_[idx];
    glm::vec3 vector = vectors_[idx] * arrow_scale_;
    glm::vec3 color = instance_colors_[idx];
    
    line_vertices_.push_back(origin);
    line_vertices_.push_back(origin + vector);
    line_colors_.push_back(color);
    line_colors_.push_back(color);
  }
}

glm::vec3 VectorField::ComputeColorForVector(const glm::vec3& vector, size_t index) const {
  switch (color_mode_) {
    case ColorMode::kUniform:
      return uniform_color_;
      
    case ColorMode::kMagnitude: {
      float mag = glm::length(vector);
      float t = glm::clamp((mag - min_magnitude_) / (max_magnitude_ - min_magnitude_), 0.0f, 1.0f);
      // Blue to red gradient
      return glm::vec3(t, 0.0f, 1.0f - t);
    }
    
    case ColorMode::kDirection: {
      glm::vec3 dir = glm::normalize(vector);
      // Map direction to RGB
      return glm::vec3(
        (dir.x + 1.0f) * 0.5f,
        (dir.y + 1.0f) * 0.5f,
        (dir.z + 1.0f) * 0.5f
      );
    }
    
    case ColorMode::kCustom:
      return index < colors_.size() ? colors_[index] : uniform_color_;
      
    default:
      return uniform_color_;
  }
}

glm::mat4 VectorField::ComputeTransformForVector(const glm::vec3& origin, 
                                                 const glm::vec3& vector) const {
  float length = glm::length(vector);
  if (length < 0.001f) return glm::mat4(1.0f);
  
  glm::vec3 direction = vector / length;
  float scaled_length = length * arrow_scale_;
  
  // Build rotation to align +Z with vector direction
  glm::vec3 z_axis(0.0f, 0.0f, 1.0f);
  glm::mat4 transform(1.0f);
  
  if (glm::length(direction - z_axis) > 0.001f && glm::length(direction + z_axis) > 0.001f) {
    glm::vec3 axis = glm::cross(z_axis, direction);
    if (glm::length(axis) > 0.001f) {
      axis = glm::normalize(axis);
      float angle = acos(glm::clamp(glm::dot(z_axis, direction), -1.0f, 1.0f));
      transform = glm::rotate(glm::mat4(1.0f), angle, axis);
    }
  } else if (glm::dot(direction, z_axis) < 0) {
    // Pointing in -Z direction, rotate 180 degrees around X
    transform = glm::rotate(glm::mat4(1.0f), float(M_PI), glm::vec3(1.0f, 0.0f, 0.0f));
  }
  
  // Apply scale and translation
  transform = glm::translate(glm::mat4(1.0f), origin) * transform;
  transform = glm::scale(transform, glm::vec3(1.0f, 1.0f, scaled_length));
  
  return transform;
}

bool VectorField::ShouldRenderVector(const glm::vec3& vector, size_t index) const {
  float mag = glm::length(vector);
  return mag >= magnitude_threshold_;
}

}  // namespace quickviz