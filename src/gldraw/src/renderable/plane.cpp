/**
 * @file plane.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Implementation of plane renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/plane.hpp"

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
layout (location = 2) in vec2 aTexCoord;

out vec3 FragPos;
out vec3 Normal;
out vec2 TexCoord;

uniform mat4 mvp;
uniform mat4 model;

void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
    TexCoord = aTexCoord;
    gl_Position = mvp * model * vec4(aPos, 1.0);
}
)";

const char* kSolidFragmentShader = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoord;

out vec4 FragColor;

uniform vec3 color;
uniform float opacity;
uniform vec3 lightPos;
uniform vec3 viewPos;
uniform bool useTexture;

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
    float specularStrength = 0.3;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 16);
    vec3 specular = specularStrength * spec * vec3(1.0, 1.0, 1.0);
    
    vec3 result = ambient + diffuse + specular;
    
    // Simple procedural texture based on texture coordinates
    if (useTexture) {
        float checker = sin(TexCoord.x * 20.0) * sin(TexCoord.y * 20.0);
        result *= (0.8 + 0.2 * checker);
    }
    
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

Plane::Plane() {
  GeneratePlaneGeometry();
}

Plane::Plane(const glm::vec3& center, const glm::vec3& normal, const glm::vec2& size)
    : center_(center), normal_(glm::normalize(normal)), size_(size) {
  GeneratePlaneGeometry();
}

Plane::Plane(const glm::vec3& point1, const glm::vec3& point2, const glm::vec3& point3, const glm::vec3& point4) {
  SetFromCorners(point1, point2, point3, point4);
}

Plane::~Plane() {
  if (IsGpuResourcesAllocated()) {
    ReleaseGpuResources();
  }
}

void Plane::SetCenter(const glm::vec3& center) {
  center_ = center;
  needs_update_ = true;
}

void Plane::SetNormal(const glm::vec3& normal) {
  normal_ = glm::normalize(normal);
  needs_update_ = true;
}

void Plane::SetSize(const glm::vec2& size) {
  size_ = size;
  needs_update_ = true;
}

void Plane::SetFromCorners(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, const glm::vec3& p4) {
  // Calculate center as average of corners
  center_ = (p1 + p2 + p3 + p4) * 0.25f;
  
  // Calculate normal from cross product
  glm::vec3 v1 = p2 - p1;
  glm::vec3 v2 = p4 - p1;
  normal_ = glm::normalize(glm::cross(v1, v2));
  
  // Calculate size from distances
  float width = glm::length(p2 - p1);
  float height = glm::length(p4 - p1);
  size_ = glm::vec2(width, height);
  
  needs_update_ = true;
}

void Plane::SetFromPointAndNormal(const glm::vec3& point, const glm::vec3& normal, const glm::vec2& size) {
  center_ = point;
  normal_ = glm::normalize(normal);
  size_ = size;
  needs_update_ = true;
}

void Plane::SetTransform(const glm::mat4& transform) {
  transform_ = transform;
}

void Plane::SetColor(const glm::vec3& color) {
  color_ = color;
}

void Plane::SetWireframeColor(const glm::vec3& color) {
  wireframe_color_ = color;
}

void Plane::SetOpacity(float opacity) {
  opacity_ = opacity;
}

void Plane::SetRenderMode(RenderMode mode) {
  render_mode_ = mode;
}

void Plane::SetGridResolution(int width_segments, int height_segments) {
  width_segments_ = width_segments;
  height_segments_ = height_segments;
  needs_update_ = true;
}

void Plane::SetShowGrid(bool show) {
  show_grid_ = show;
}

void Plane::SetGridColor(const glm::vec3& color) {
  grid_color_ = color;
}

void Plane::SetWireframeWidth(float width) {
  wireframe_width_ = width;
}

void Plane::SetShowNormal(bool show, float length) {
  show_normal_ = show;
  normal_length_ = length;
  needs_update_ = true;
}

void Plane::SetNormalColor(const glm::vec3& color) {
  normal_color_ = color;
}

void Plane::SetTextureCoordinates(bool enable) {
  use_texture_coords_ = enable;
  needs_update_ = true;
}

glm::vec4 Plane::GetPlaneEquation() const {
  // Plane equation: ax + by + cz + d = 0
  // where (a,b,c) is the normal and d = -dot(normal, point_on_plane)
  float d = -glm::dot(normal_, center_);
  return glm::vec4(normal_.x, normal_.y, normal_.z, d);
}

void Plane::GeneratePlaneGeometry() {
  vertices_.clear();
  normals_.clear();
  tex_coords_.clear();
  solid_indices_.clear();
  wireframe_indices_.clear();
  normal_lines_.clear();
  
  // Create coordinate system for the plane
  glm::vec3 u, v;
  if (std::abs(normal_.y) < 0.9f) {
    u = glm::normalize(glm::cross(normal_, glm::vec3(0, 1, 0)));
  } else {
    u = glm::normalize(glm::cross(normal_, glm::vec3(1, 0, 0)));
  }
  v = glm::cross(normal_, u);
  
  // Generate grid vertices
  for (int j = 0; j <= height_segments_; ++j) {
    for (int i = 0; i <= width_segments_; ++i) {
      float s = (float)i / width_segments_ - 0.5f;  // [-0.5, 0.5]
      float t = (float)j / height_segments_ - 0.5f; // [-0.5, 0.5]
      
      glm::vec3 vertex = center_ + (s * size_.x) * u + (t * size_.y) * v;
      vertices_.push_back(vertex);
      normals_.push_back(normal_);
      
      if (use_texture_coords_) {
        tex_coords_.push_back(glm::vec2(s + 0.5f, t + 0.5f));
      } else {
        tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
      }
    }
  }
  
  // Generate solid indices (triangles)
  for (int j = 0; j < height_segments_; ++j) {
    for (int i = 0; i < width_segments_; ++i) {
      int idx = j * (width_segments_ + 1) + i;
      
      // First triangle
      solid_indices_.push_back(idx);
      solid_indices_.push_back(idx + width_segments_ + 1);
      solid_indices_.push_back(idx + 1);
      
      // Second triangle
      solid_indices_.push_back(idx + 1);
      solid_indices_.push_back(idx + width_segments_ + 1);
      solid_indices_.push_back(idx + width_segments_ + 2);
    }
  }
  
  // Generate wireframe indices
  if (show_grid_) {
    // Horizontal lines
    for (int j = 0; j <= height_segments_; ++j) {
      for (int i = 0; i < width_segments_; ++i) {
        int idx = j * (width_segments_ + 1) + i;
        wireframe_indices_.push_back(idx);
        wireframe_indices_.push_back(idx + 1);
      }
    }
    
    // Vertical lines
    for (int i = 0; i <= width_segments_; ++i) {
      for (int j = 0; j < height_segments_; ++j) {
        int idx = j * (width_segments_ + 1) + i;
        wireframe_indices_.push_back(idx);
        wireframe_indices_.push_back(idx + width_segments_ + 1);
      }
    }
  } else {
    // Just the outline
    // Top edge
    for (int i = 0; i < width_segments_; ++i) {
      wireframe_indices_.push_back(i);
      wireframe_indices_.push_back(i + 1);
    }
    
    // Bottom edge
    int bottom_start = height_segments_ * (width_segments_ + 1);
    for (int i = 0; i < width_segments_; ++i) {
      wireframe_indices_.push_back(bottom_start + i);
      wireframe_indices_.push_back(bottom_start + i + 1);
    }
    
    // Left edge
    for (int j = 0; j < height_segments_; ++j) {
      int idx = j * (width_segments_ + 1);
      wireframe_indices_.push_back(idx);
      wireframe_indices_.push_back(idx + width_segments_ + 1);
    }
    
    // Right edge
    for (int j = 0; j < height_segments_; ++j) {
      int idx = j * (width_segments_ + 1) + width_segments_;
      wireframe_indices_.push_back(idx);
      wireframe_indices_.push_back(idx + width_segments_ + 1);
    }
  }
  
  // Generate normal visualization
  if (show_normal_) {
    normal_lines_.push_back(center_);
    normal_lines_.push_back(center_ + normal_ * normal_length_);
  }
  
  needs_update_ = true;
}

void Plane::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;
  
  try {
    // Compile solid shader
    Shader solid_vs(kSolidVertexShader, Shader::Type::kVertex);
    Shader solid_fs(kSolidFragmentShader, Shader::Type::kFragment);
    if (!solid_vs.Compile() || !solid_fs.Compile()) {
      throw std::runtime_error("Solid plane shader compilation failed");
    }
    solid_shader_.AttachShader(solid_vs);
    solid_shader_.AttachShader(solid_fs);
    if (!solid_shader_.LinkProgram()) {
      throw std::runtime_error("Solid plane shader linking failed");
    }
    
    // Compile wireframe shader
    Shader wireframe_vs(kWireframeVertexShader, Shader::Type::kVertex);
    Shader wireframe_fs(kWireframeFragmentShader, Shader::Type::kFragment);
    if (!wireframe_vs.Compile() || !wireframe_fs.Compile()) {
      throw std::runtime_error("Wireframe plane shader compilation failed");
    }
    wireframe_shader_.AttachShader(wireframe_vs);
    wireframe_shader_.AttachShader(wireframe_fs);
    if (!wireframe_shader_.LinkProgram()) {
      throw std::runtime_error("Wireframe plane shader linking failed");
    }
    
    // Create VAOs and VBOs
    glGenVertexArrays(1, &vao_plane_);
    glGenBuffers(1, &vbo_vertices_);
    glGenBuffers(1, &vbo_normals_);
    glGenBuffers(1, &vbo_tex_coords_);
    glGenBuffers(1, &ebo_solid_);
    
    glGenVertexArrays(1, &vao_wireframe_);
    glGenBuffers(1, &ebo_wireframe_);
    
    glGenVertexArrays(1, &vao_normals_);
    glGenBuffers(1, &vbo_normal_lines_);
    
    UpdateGpuBuffers();
    
  } catch (const std::exception& e) {
    std::cerr << "Plane::AllocateGpuResources: " << e.what() << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void Plane::ReleaseGpuResources() noexcept {
  if (vao_plane_ != 0) {
    glDeleteVertexArrays(1, &vao_plane_);
    vao_plane_ = 0;
  }
  if (vao_wireframe_ != 0) {
    glDeleteVertexArrays(1, &vao_wireframe_);
    vao_wireframe_ = 0;
  }
  if (vao_normals_ != 0) {
    glDeleteVertexArrays(1, &vao_normals_);
    vao_normals_ = 0;
  }
  if (vbo_vertices_ != 0) {
    glDeleteBuffers(1, &vbo_vertices_);
    vbo_vertices_ = 0;
  }
  if (vbo_normals_ != 0) {
    glDeleteBuffers(1, &vbo_normals_);
    vbo_normals_ = 0;
  }
  if (vbo_tex_coords_ != 0) {
    glDeleteBuffers(1, &vbo_tex_coords_);
    vbo_tex_coords_ = 0;
  }
  if (vbo_normal_lines_ != 0) {
    glDeleteBuffers(1, &vbo_normal_lines_);
    vbo_normal_lines_ = 0;
  }
  if (ebo_solid_ != 0) {
    glDeleteBuffers(1, &ebo_solid_);
    ebo_solid_ = 0;
  }
  if (ebo_wireframe_ != 0) {
    glDeleteBuffers(1, &ebo_wireframe_);
    ebo_wireframe_ = 0;
  }
}

void Plane::UpdateGpuBuffers() {
  if (!IsGpuResourcesAllocated()) return;
  
  // Update vertex buffers
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3),
               vertices_.data(), GL_DYNAMIC_DRAW);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_normals_);
  glBufferData(GL_ARRAY_BUFFER, normals_.size() * sizeof(glm::vec3),
               normals_.data(), GL_DYNAMIC_DRAW);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_tex_coords_);
  glBufferData(GL_ARRAY_BUFFER, tex_coords_.size() * sizeof(glm::vec2),
               tex_coords_.data(), GL_DYNAMIC_DRAW);
  
  // Setup plane VAO
  glBindVertexArray(vao_plane_);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(0);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_normals_);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(1);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_tex_coords_);
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), nullptr);
  glEnableVertexAttribArray(2);
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_solid_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, solid_indices_.size() * sizeof(uint32_t),
               solid_indices_.data(), GL_DYNAMIC_DRAW);
  
  // Setup wireframe VAO
  glBindVertexArray(vao_wireframe_);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(0);
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_wireframe_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, wireframe_indices_.size() * sizeof(uint32_t),
               wireframe_indices_.data(), GL_DYNAMIC_DRAW);
  
  // Setup normal lines VAO
  if (show_normal_ && !normal_lines_.empty()) {
    glBindVertexArray(vao_normals_);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo_normal_lines_);
    glBufferData(GL_ARRAY_BUFFER, normal_lines_.size() * sizeof(glm::vec3),
                 normal_lines_.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(0);
  }
  
  glBindVertexArray(0);
  needs_update_ = false;
}

void Plane::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                   const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }
  
  if (needs_update_) {
    GeneratePlaneGeometry();
    UpdateGpuBuffers();
  }
  
  glm::mat4 mvp = projection * view * coord_transform;
  glm::mat4 final_transform = coord_transform * transform_;
  
  // Draw solid or transparent plane
  if (render_mode_ == RenderMode::kSolid || render_mode_ == RenderMode::kTransparent) {
    solid_shader_.Use();
    solid_shader_.SetUniform("mvp", mvp);
    solid_shader_.SetUniform("model", transform_);
    solid_shader_.SetUniform("color", color_);
    solid_shader_.SetUniform("opacity", opacity_);
    solid_shader_.SetUniform("useTexture", use_texture_coords_);
    solid_shader_.TrySetUniform("lightPos", glm::vec3(10, 10, 10));
    solid_shader_.TrySetUniform("viewPos", glm::vec3(0, 0, 5));
    
    if (render_mode_ == RenderMode::kTransparent || opacity_ < 1.0f) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    glBindVertexArray(vao_plane_);
    glDrawElements(GL_TRIANGLES, solid_indices_.size(), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
    
    if (render_mode_ == RenderMode::kTransparent || opacity_ < 1.0f) {
      glDisable(GL_BLEND);
    }
  }
  
  // Draw wireframe/grid
  if (render_mode_ == RenderMode::kWireframe || show_grid_) {
    wireframe_shader_.Use();
    wireframe_shader_.SetUniform("mvp", mvp);
    wireframe_shader_.SetUniform("model", transform_);
    wireframe_shader_.SetUniform("color", show_grid_ ? grid_color_ : wireframe_color_);
    
    glLineWidth(wireframe_width_);
    glBindVertexArray(vao_wireframe_);
    glDrawElements(GL_LINES, wireframe_indices_.size(), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
  }
  
  // Draw points
  if (render_mode_ == RenderMode::kPoints) {
    wireframe_shader_.Use();
    wireframe_shader_.SetUniform("mvp", mvp);
    wireframe_shader_.SetUniform("model", transform_);
    wireframe_shader_.SetUniform("color", color_);
    
    glEnable(GL_PROGRAM_POINT_SIZE);
    glBindVertexArray(vao_wireframe_);
    glDrawArrays(GL_POINTS, 0, vertices_.size());
    glBindVertexArray(0);
    glDisable(GL_PROGRAM_POINT_SIZE);
  }
  
  // Draw normal visualization
  if (show_normal_ && !normal_lines_.empty()) {
    wireframe_shader_.Use();
    wireframe_shader_.SetUniform("mvp", mvp);
    wireframe_shader_.SetUniform("model", transform_);
    wireframe_shader_.SetUniform("color", normal_color_);
    
    glLineWidth(wireframe_width_ * 2.0f);  // Make normal thicker
    glBindVertexArray(vao_normals_);
    glDrawArrays(GL_LINES, 0, normal_lines_.size());
    glBindVertexArray(0);
  }
}

}  // namespace quickviz