/**
 * @file cylinder.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Implementation of cylinder renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/cylinder.hpp"

#include <iostream>
#include <cmath>

#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "gldraw/shader.hpp"

namespace quickviz {

namespace {
const char* kSolidVertexShader = R"(
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
    gl_Position = mvp * model * vec4(aPos, 1.0);
}
)";

const char* kSolidFragmentShader = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;

out vec4 FragColor;

uniform vec3 color;
uniform float opacity;
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
    FragColor = vec4(result, opacity);
}
)";

const char* kWireframeVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 mvp;
uniform mat4 model;

void main() {
    gl_Position = mvp * model * vec4(aPos, 1.0);
}
)";

const char* kWireframeFragmentShader = R"(
#version 330 core
out vec4 FragColor;
uniform vec3 color;

void main() {
    FragColor = vec4(color, 1.0);
}
)";

}  // namespace

Cylinder::Cylinder() {
  GenerateCylinderGeometry();
}

Cylinder::Cylinder(const glm::vec3& base_center, const glm::vec3& top_center, float radius)
    : base_center_(base_center), top_center_(top_center), radius_(radius) {
  GenerateCylinderGeometry();
}

Cylinder::Cylinder(const glm::vec3& center, float height, float radius)
    : radius_(radius) {
  base_center_ = center - glm::vec3(0, height * 0.5f, 0);
  top_center_ = center + glm::vec3(0, height * 0.5f, 0);
  GenerateCylinderGeometry();
}

Cylinder::~Cylinder() {
  if (IsGpuResourcesAllocated()) {
    ReleaseGpuResources();
  }
}

void Cylinder::SetBaseCenter(const glm::vec3& center) {
  base_center_ = center;
  needs_update_ = true;
}

void Cylinder::SetTopCenter(const glm::vec3& center) {
  top_center_ = center;
  needs_update_ = true;
}

void Cylinder::SetCenterAndHeight(const glm::vec3& center, float height) {
  base_center_ = center - glm::vec3(0, height * 0.5f, 0);
  top_center_ = center + glm::vec3(0, height * 0.5f, 0);
  needs_update_ = true;
}

void Cylinder::SetRadius(float radius) {
  radius_ = radius;
  needs_update_ = true;
}

void Cylinder::SetTransform(const glm::mat4& transform) {
  transform_ = transform;
}

void Cylinder::SetColor(const glm::vec3& color) {
  color_ = color;
}

void Cylinder::SetWireframeColor(const glm::vec3& color) {
  wireframe_color_ = color;
}

void Cylinder::SetOpacity(float opacity) {
  opacity_ = opacity;
}

void Cylinder::SetRenderMode(RenderMode mode) {
  render_mode_ = mode;
}

void Cylinder::SetResolution(int radial_segments) {
  radial_segments_ = radial_segments;
  needs_update_ = true;
}

void Cylinder::SetWireframeWidth(float width) {
  wireframe_width_ = width;
}

void Cylinder::SetShowTopCap(bool show) {
  show_top_cap_ = show;
}

void Cylinder::SetShowBottomCap(bool show) {
  show_bottom_cap_ = show;
}

float Cylinder::GetHeight() const {
  return glm::length(top_center_ - base_center_);
}

glm::vec3 Cylinder::GetAxis() const {
  glm::vec3 axis = top_center_ - base_center_;
  if (glm::length(axis) > 0) {
    return glm::normalize(axis);
  }
  return glm::vec3(0, 1, 0);
}

void Cylinder::GenerateCylinderGeometry() {
  vertices_.clear();
  normals_.clear();
  side_indices_.clear();
  top_cap_indices_.clear();
  bottom_cap_indices_.clear();
  wireframe_indices_.clear();
  
  glm::vec3 axis = top_center_ - base_center_;
  float height = glm::length(axis);
  if (height == 0) {
    axis = glm::vec3(0, 1, 0);
    height = 1.0f;
  } else {
    axis = glm::normalize(axis);
  }
  
  // Find perpendicular vectors for the circular cross-section
  glm::vec3 perp1, perp2;
  if (std::abs(axis.y) < 0.9f) {
    perp1 = glm::normalize(glm::cross(axis, glm::vec3(0, 1, 0)));
  } else {
    perp1 = glm::normalize(glm::cross(axis, glm::vec3(1, 0, 0)));
  }
  perp2 = glm::cross(axis, perp1);
  
  // Generate vertices for cylinder sides
  for (int i = 0; i <= radial_segments_; ++i) {
    float angle = 2.0f * M_PI * i / radial_segments_;
    float cos_a = cos(angle);
    float sin_a = sin(angle);
    
    glm::vec3 radial_dir = cos_a * perp1 + sin_a * perp2;
    glm::vec3 offset = radius_ * radial_dir;
    
    // Bottom vertex
    vertices_.push_back(base_center_ + offset);
    normals_.push_back(radial_dir);  // Outward normal
    
    // Top vertex
    vertices_.push_back(top_center_ + offset);
    normals_.push_back(radial_dir);  // Outward normal
  }
  
  // Generate side indices
  for (int i = 0; i < radial_segments_; ++i) {
    int base_idx = i * 2;
    
    // Triangle 1 (bottom-left, top-left, bottom-right)
    side_indices_.push_back(base_idx);
    side_indices_.push_back(base_idx + 2);
    side_indices_.push_back(base_idx + 1);
    
    // Triangle 2 (bottom-right, top-left, top-right)
    side_indices_.push_back(base_idx + 2);
    side_indices_.push_back(base_idx + 3);
    side_indices_.push_back(base_idx + 1);
  }
  
  // Generate cap vertices and indices if needed
  if (show_bottom_cap_ || show_top_cap_) {
    size_t cap_start_idx = vertices_.size();
    
    // Bottom cap center
    vertices_.push_back(base_center_);
    normals_.push_back(-axis);  // Downward normal
    
    // Top cap center  
    vertices_.push_back(top_center_);
    normals_.push_back(axis);   // Upward normal
    
    // Bottom cap vertices
    for (int i = 0; i < radial_segments_; ++i) {
      float angle = 2.0f * M_PI * i / radial_segments_;
      float cos_a = cos(angle);
      float sin_a = sin(angle);
      
      glm::vec3 radial_dir = cos_a * perp1 + sin_a * perp2;
      vertices_.push_back(base_center_ + radius_ * radial_dir);
      normals_.push_back(-axis);
    }
    
    // Top cap vertices
    for (int i = 0; i < radial_segments_; ++i) {
      float angle = 2.0f * M_PI * i / radial_segments_;
      float cos_a = cos(angle);
      float sin_a = sin(angle);
      
      glm::vec3 radial_dir = cos_a * perp1 + sin_a * perp2;
      vertices_.push_back(top_center_ + radius_ * radial_dir);
      normals_.push_back(axis);
    }
    
    // Generate bottom cap indices
    size_t bottom_center_idx = cap_start_idx;
    size_t bottom_rim_start = cap_start_idx + 2;
    for (int i = 0; i < radial_segments_; ++i) {
      int next_i = (i + 1) % radial_segments_;
      bottom_cap_indices_.push_back(bottom_center_idx);
      bottom_cap_indices_.push_back(bottom_rim_start + next_i);
      bottom_cap_indices_.push_back(bottom_rim_start + i);
    }
    
    // Generate top cap indices
    size_t top_center_idx = cap_start_idx + 1;
    size_t top_rim_start = cap_start_idx + 2 + radial_segments_;
    for (int i = 0; i < radial_segments_; ++i) {
      int next_i = (i + 1) % radial_segments_;
      top_cap_indices_.push_back(top_center_idx);
      top_cap_indices_.push_back(top_rim_start + i);
      top_cap_indices_.push_back(top_rim_start + next_i);
    }
  }
  
  // Generate wireframe indices (circles and vertical lines)
  // Bottom circle
  for (int i = 0; i < radial_segments_; ++i) {
    wireframe_indices_.push_back(i * 2);
    wireframe_indices_.push_back((i + 1) % radial_segments_ * 2);
  }
  
  // Top circle
  for (int i = 0; i < radial_segments_; ++i) {
    wireframe_indices_.push_back(i * 2 + 1);
    wireframe_indices_.push_back((i + 1) % radial_segments_ * 2 + 1);
  }
  
  // Vertical lines
  for (int i = 0; i < radial_segments_; i += radial_segments_ / 8) {  // Show 8 vertical lines
    wireframe_indices_.push_back(i * 2);
    wireframe_indices_.push_back(i * 2 + 1);
  }
  
  needs_update_ = true;
}

void Cylinder::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;
  
  try {
    // Compile solid shader
    Shader solid_vs(kSolidVertexShader, Shader::Type::kVertex);
    Shader solid_fs(kSolidFragmentShader, Shader::Type::kFragment);
    if (!solid_vs.Compile() || !solid_fs.Compile()) {
      throw std::runtime_error("Solid cylinder shader compilation failed");
    }
    solid_shader_.AttachShader(solid_vs);
    solid_shader_.AttachShader(solid_fs);
    if (!solid_shader_.LinkProgram()) {
      throw std::runtime_error("Solid cylinder shader linking failed");
    }
    
    // Compile wireframe shader
    Shader wireframe_vs(kWireframeVertexShader, Shader::Type::kVertex);
    Shader wireframe_fs(kWireframeFragmentShader, Shader::Type::kFragment);
    if (!wireframe_vs.Compile() || !wireframe_fs.Compile()) {
      throw std::runtime_error("Wireframe cylinder shader compilation failed");
    }
    wireframe_shader_.AttachShader(wireframe_vs);
    wireframe_shader_.AttachShader(wireframe_fs);
    if (!wireframe_shader_.LinkProgram()) {
      throw std::runtime_error("Wireframe cylinder shader linking failed");
    }
    
    // Create VAOs and VBOs for sides
    glGenVertexArrays(1, &vao_sides_);
    glGenBuffers(1, &vbo_vertices_);
    glGenBuffers(1, &vbo_normals_);
    glGenBuffers(1, &ebo_sides_);
    
    // Create VAO and EBOs for caps
    glGenVertexArrays(1, &vao_caps_);
    glGenBuffers(1, &ebo_top_cap_);
    glGenBuffers(1, &ebo_bottom_cap_);
    
    // Create VAO and EBO for wireframe
    glGenVertexArrays(1, &vao_wireframe_);
    glGenBuffers(1, &ebo_wireframe_);
    
    UpdateGpuBuffers();
    
  } catch (const std::exception& e) {
    std::cerr << "Cylinder::AllocateGpuResources: " << e.what() << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void Cylinder::ReleaseGpuResources() noexcept {
  if (vao_sides_ != 0) {
    glDeleteVertexArrays(1, &vao_sides_);
    vao_sides_ = 0;
  }
  if (vao_caps_ != 0) {
    glDeleteVertexArrays(1, &vao_caps_);
    vao_caps_ = 0;
  }
  if (vao_wireframe_ != 0) {
    glDeleteVertexArrays(1, &vao_wireframe_);
    vao_wireframe_ = 0;
  }
  if (vbo_vertices_ != 0) {
    glDeleteBuffers(1, &vbo_vertices_);
    vbo_vertices_ = 0;
  }
  if (vbo_normals_ != 0) {
    glDeleteBuffers(1, &vbo_normals_);
    vbo_normals_ = 0;
  }
  if (ebo_sides_ != 0) {
    glDeleteBuffers(1, &ebo_sides_);
    ebo_sides_ = 0;
  }
  if (ebo_top_cap_ != 0) {
    glDeleteBuffers(1, &ebo_top_cap_);
    ebo_top_cap_ = 0;
  }
  if (ebo_bottom_cap_ != 0) {
    glDeleteBuffers(1, &ebo_bottom_cap_);
    ebo_bottom_cap_ = 0;
  }
  if (ebo_wireframe_ != 0) {
    glDeleteBuffers(1, &ebo_wireframe_);
    ebo_wireframe_ = 0;
  }
}

void Cylinder::UpdateGpuBuffers() {
  if (!IsGpuResourcesAllocated()) return;
  
  // Update vertex and normal buffers
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3),
               vertices_.data(), GL_DYNAMIC_DRAW);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_normals_);
  glBufferData(GL_ARRAY_BUFFER, normals_.size() * sizeof(glm::vec3),
               normals_.data(), GL_DYNAMIC_DRAW);
  
  // Setup sides VAO
  glBindVertexArray(vao_sides_);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(0);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_normals_);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(1);
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_sides_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, side_indices_.size() * sizeof(uint32_t),
               side_indices_.data(), GL_DYNAMIC_DRAW);
  
  // Setup caps VAO
  glBindVertexArray(vao_caps_);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(0);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_normals_);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(1);
  
  // Update cap index buffers
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_bottom_cap_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, bottom_cap_indices_.size() * sizeof(uint32_t),
               bottom_cap_indices_.data(), GL_DYNAMIC_DRAW);
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_top_cap_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, top_cap_indices_.size() * sizeof(uint32_t),
               top_cap_indices_.data(), GL_DYNAMIC_DRAW);
  
  // Setup wireframe VAO
  glBindVertexArray(vao_wireframe_);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(0);
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_wireframe_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, wireframe_indices_.size() * sizeof(uint32_t),
               wireframe_indices_.data(), GL_DYNAMIC_DRAW);
  
  glBindVertexArray(0);
  needs_update_ = false;
}

void Cylinder::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                      const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }
  
  if (needs_update_) {
    GenerateCylinderGeometry();
    UpdateGpuBuffers();
  }
  
  glm::mat4 mvp = projection * view * coord_transform;
  glm::mat4 final_transform = coord_transform * transform_;
  
  // Draw solid or transparent cylinder
  if (render_mode_ == RenderMode::kSolid || render_mode_ == RenderMode::kTransparent) {
    solid_shader_.Use();
    solid_shader_.SetUniform("mvp", mvp);
    solid_shader_.SetUniform("model", transform_);
    solid_shader_.SetUniform("color", color_);
    solid_shader_.SetUniform("opacity", opacity_);
    solid_shader_.TrySetUniform("lightPos", glm::vec3(10, 10, 10));
    solid_shader_.TrySetUniform("viewPos", glm::vec3(0, 0, 5));
    
    if (render_mode_ == RenderMode::kTransparent || opacity_ < 1.0f) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    // Draw sides
    glBindVertexArray(vao_sides_);
    glDrawElements(GL_TRIANGLES, side_indices_.size(), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
    
    // Draw caps
    if (show_bottom_cap_ && !bottom_cap_indices_.empty()) {
      glBindVertexArray(vao_caps_);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_bottom_cap_);
      glDrawElements(GL_TRIANGLES, bottom_cap_indices_.size(), GL_UNSIGNED_INT, nullptr);
      glBindVertexArray(0);
    }
    
    if (show_top_cap_ && !top_cap_indices_.empty()) {
      glBindVertexArray(vao_caps_);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_top_cap_);
      glDrawElements(GL_TRIANGLES, top_cap_indices_.size(), GL_UNSIGNED_INT, nullptr);
      glBindVertexArray(0);
    }
    
    if (render_mode_ == RenderMode::kTransparent || opacity_ < 1.0f) {
      glDisable(GL_BLEND);
    }
  }
  
  // Draw wireframe
  if (render_mode_ == RenderMode::kWireframe || render_mode_ == RenderMode::kOutline) {
    wireframe_shader_.Use();
    wireframe_shader_.SetUniform("mvp", mvp);
    wireframe_shader_.SetUniform("model", transform_);
    wireframe_shader_.SetUniform("color", wireframe_color_);
    
    glLineWidth(wireframe_width_);
    glBindVertexArray(vao_wireframe_);
    glDrawElements(GL_LINES, wireframe_indices_.size(), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
  }
}

}  // namespace quickviz