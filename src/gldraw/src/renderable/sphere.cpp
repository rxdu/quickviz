/**
 * @file sphere.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Implementation of sphere renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/sphere.hpp"

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
uniform vec3 center;
uniform float radius;

void main() {
    vec3 worldPos = center + aPos * radius;
    FragPos = vec3(model * vec4(worldPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
    gl_Position = mvp * model * vec4(worldPos, 1.0);
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
uniform vec3 center;
uniform float radius;

void main() {
    vec3 worldPos = center + aPos * radius;
    gl_Position = mvp * model * vec4(worldPos, 1.0);
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

Sphere::Sphere() {
  GenerateSphereGeometry();
}

Sphere::Sphere(const glm::vec3& center, float radius)
    : center_(center), radius_(radius) {
  GenerateSphereGeometry();
}

Sphere::~Sphere() {
  if (IsGpuResourcesAllocated()) {
    ReleaseGpuResources();
  }
}

void Sphere::SetCenter(const glm::vec3& center) {
  center_ = center;
}

void Sphere::SetRadius(float radius) {
  radius_ = radius;
}

void Sphere::SetTransform(const glm::mat4& transform) {
  transform_ = transform;
}

void Sphere::SetColor(const glm::vec3& color) {
  color_ = color;
}

void Sphere::SetWireframeColor(const glm::vec3& color) {
  wireframe_color_ = color;
}

void Sphere::SetOpacity(float opacity) {
  opacity_ = opacity;
}

void Sphere::SetRenderMode(RenderMode mode) {
  render_mode_ = mode;
}

void Sphere::SetResolution(int latitude_segments, int longitude_segments) {
  latitude_segments_ = latitude_segments;
  longitude_segments_ = longitude_segments;
  needs_update_ = true;
}

void Sphere::SetWireframeWidth(float width) {
  wireframe_width_ = width;
}

void Sphere::SetShowPoles(bool show, float pole_size) {
  show_poles_ = show;
  pole_size_ = pole_size;
}

void Sphere::SetShowEquator(bool show, const glm::vec3& color) {
  show_equator_ = show;
  equator_color_ = color;
  if (show) {
    needs_update_ = true;  // Need to regenerate equator geometry
  }
}

float Sphere::GetSurfaceArea() const {
  return 4.0f * M_PI * radius_ * radius_;
}

float Sphere::GetVolume() const {
  return (4.0f / 3.0f) * M_PI * radius_ * radius_ * radius_;
}

void Sphere::GenerateSphereGeometry() {
  vertices_.clear();
  normals_.clear();
  solid_indices_.clear();
  wireframe_indices_.clear();
  equator_vertices_.clear();
  
  // Generate vertices and normals
  for (int lat = 0; lat <= latitude_segments_; ++lat) {
    float theta = lat * M_PI / latitude_segments_;
    float sin_theta = sin(theta);
    float cos_theta = cos(theta);
    
    for (int lon = 0; lon <= longitude_segments_; ++lon) {
      float phi = lon * 2.0f * M_PI / longitude_segments_;
      float sin_phi = sin(phi);
      float cos_phi = cos(phi);
      
      // Unit sphere vertex (will be scaled by radius in shader)
      glm::vec3 vertex(cos_phi * sin_theta, cos_theta, sin_phi * sin_theta);
      vertices_.push_back(vertex);
      normals_.push_back(vertex);  // For unit sphere, normal = position
    }
  }
  
  // Generate solid indices
  for (int lat = 0; lat < latitude_segments_; ++lat) {
    for (int lon = 0; lon < longitude_segments_; ++lon) {
      int first = lat * (longitude_segments_ + 1) + lon;
      int second = first + longitude_segments_ + 1;
      
      // First triangle
      solid_indices_.push_back(first);
      solid_indices_.push_back(second);
      solid_indices_.push_back(first + 1);
      
      // Second triangle
      solid_indices_.push_back(second);
      solid_indices_.push_back(second + 1);
      solid_indices_.push_back(first + 1);
    }
  }
  
  // Generate wireframe indices (latitude and longitude lines)
  // Latitude lines
  for (int lat = 0; lat <= latitude_segments_; ++lat) {
    for (int lon = 0; lon < longitude_segments_; ++lon) {
      int current = lat * (longitude_segments_ + 1) + lon;
      int next = lat * (longitude_segments_ + 1) + lon + 1;
      wireframe_indices_.push_back(current);
      wireframe_indices_.push_back(next);
    }
  }
  
  // Longitude lines
  for (int lon = 0; lon <= longitude_segments_; ++lon) {
    for (int lat = 0; lat < latitude_segments_; ++lat) {
      int current = lat * (longitude_segments_ + 1) + lon;
      int next = (lat + 1) * (longitude_segments_ + 1) + lon;
      wireframe_indices_.push_back(current);
      wireframe_indices_.push_back(next);
    }
  }
  
  // Generate equator geometry if needed
  if (show_equator_) {
    int equator_segments = longitude_segments_ * 2;  // Higher resolution for equator
    for (int i = 0; i <= equator_segments; ++i) {
      float phi = i * 2.0f * M_PI / equator_segments;
      glm::vec3 equator_point(cos(phi), 0.0f, sin(phi));
      equator_vertices_.push_back(equator_point);
    }
  }
  
  needs_update_ = true;
}

void Sphere::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;
  
  try {
    // Compile solid shader
    Shader solid_vs(kSolidVertexShader, Shader::Type::kVertex);
    Shader solid_fs(kSolidFragmentShader, Shader::Type::kFragment);
    if (!solid_vs.Compile() || !solid_fs.Compile()) {
      throw std::runtime_error("Solid sphere shader compilation failed");
    }
    solid_shader_.AttachShader(solid_vs);
    solid_shader_.AttachShader(solid_fs);
    if (!solid_shader_.LinkProgram()) {
      throw std::runtime_error("Solid sphere shader linking failed");
    }
    
    // Compile wireframe shader
    Shader wireframe_vs(kWireframeVertexShader, Shader::Type::kVertex);
    Shader wireframe_fs(kWireframeFragmentShader, Shader::Type::kFragment);
    if (!wireframe_vs.Compile() || !wireframe_fs.Compile()) {
      throw std::runtime_error("Wireframe sphere shader compilation failed");
    }
    wireframe_shader_.AttachShader(wireframe_vs);
    wireframe_shader_.AttachShader(wireframe_fs);
    if (!wireframe_shader_.LinkProgram()) {
      throw std::runtime_error("Wireframe sphere shader linking failed");
    }
    
    // Create VAOs and VBOs for solid rendering
    glGenVertexArrays(1, &vao_solid_);
    glGenBuffers(1, &vbo_vertices_);
    glGenBuffers(1, &vbo_normals_);
    glGenBuffers(1, &ebo_solid_);
    
    // Create VAO and EBO for wireframe rendering
    glGenVertexArrays(1, &vao_wireframe_);
    glGenBuffers(1, &ebo_wireframe_);
    
    // Create VAO and VBO for equator
    glGenVertexArrays(1, &vao_equator_);
    glGenBuffers(1, &vbo_equator_);
    
    UpdateGpuBuffers();
    
  } catch (const std::exception& e) {
    std::cerr << "Sphere::AllocateGpuResources: " << e.what() << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void Sphere::ReleaseGpuResources() noexcept {
  if (vao_solid_ != 0) {
    glDeleteVertexArrays(1, &vao_solid_);
    vao_solid_ = 0;
  }
  if (vao_wireframe_ != 0) {
    glDeleteVertexArrays(1, &vao_wireframe_);
    vao_wireframe_ = 0;
  }
  if (vao_equator_ != 0) {
    glDeleteVertexArrays(1, &vao_equator_);
    vao_equator_ = 0;
  }
  if (vbo_vertices_ != 0) {
    glDeleteBuffers(1, &vbo_vertices_);
    vbo_vertices_ = 0;
  }
  if (vbo_normals_ != 0) {
    glDeleteBuffers(1, &vbo_normals_);
    vbo_normals_ = 0;
  }
  if (vbo_equator_ != 0) {
    glDeleteBuffers(1, &vbo_equator_);
    vbo_equator_ = 0;
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

void Sphere::UpdateGpuBuffers() {
  if (!IsGpuResourcesAllocated()) return;
  
  // Update vertex and normal buffers
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3),
               vertices_.data(), GL_DYNAMIC_DRAW);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_normals_);
  glBufferData(GL_ARRAY_BUFFER, normals_.size() * sizeof(glm::vec3),
               normals_.data(), GL_DYNAMIC_DRAW);
  
  // Setup solid VAO
  glBindVertexArray(vao_solid_);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(0);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_normals_);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(1);
  
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
  
  // Setup equator VAO
  if (show_equator_ && !equator_vertices_.empty()) {
    glBindVertexArray(vao_equator_);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo_equator_);
    glBufferData(GL_ARRAY_BUFFER, equator_vertices_.size() * sizeof(glm::vec3),
                 equator_vertices_.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(0);
  }
  
  glBindVertexArray(0);
  needs_update_ = false;
}

void Sphere::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                    const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }
  
  if (needs_update_) {
    GenerateSphereGeometry();
    UpdateGpuBuffers();
  }
  
  glm::mat4 mvp = projection * view * coord_transform;
  glm::mat4 final_transform = coord_transform * transform_;
  
  // Draw solid sphere or transparent sphere
  if (render_mode_ == RenderMode::kSolid || render_mode_ == RenderMode::kTransparent) {
    solid_shader_.Use();
    solid_shader_.SetUniform("mvp", mvp);
    solid_shader_.SetUniform("model", transform_);
    solid_shader_.SetUniform("center", center_);
    solid_shader_.SetUniform("radius", radius_);
    solid_shader_.SetUniform("color", color_);
    solid_shader_.SetUniform("opacity", opacity_);
    solid_shader_.TrySetUniform("lightPos", glm::vec3(10, 10, 10));
    solid_shader_.TrySetUniform("viewPos", glm::vec3(0, 0, 5));
    
    if (render_mode_ == RenderMode::kTransparent || opacity_ < 1.0f) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    glBindVertexArray(vao_solid_);
    glDrawElements(GL_TRIANGLES, solid_indices_.size(), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
    
    if (render_mode_ == RenderMode::kTransparent || opacity_ < 1.0f) {
      glDisable(GL_BLEND);
    }
  }
  
  // Draw wireframe
  if (render_mode_ == RenderMode::kWireframe) {
    wireframe_shader_.Use();
    wireframe_shader_.SetUniform("mvp", mvp);
    wireframe_shader_.SetUniform("model", transform_);
    wireframe_shader_.SetUniform("center", center_);
    wireframe_shader_.SetUniform("radius", radius_);
    wireframe_shader_.SetUniform("color", wireframe_color_);
    
    glLineWidth(wireframe_width_);
    glBindVertexArray(vao_wireframe_);
    glDrawElements(GL_LINES, wireframe_indices_.size(), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
  }
  
  // Draw points
  if (render_mode_ == RenderMode::kPoints) {
    wireframe_shader_.Use();  // Reuse wireframe shader for points
    wireframe_shader_.SetUniform("mvp", mvp);
    wireframe_shader_.SetUniform("model", transform_);
    wireframe_shader_.SetUniform("center", center_);
    wireframe_shader_.SetUniform("radius", radius_);
    wireframe_shader_.SetUniform("color", color_);
    
    glEnable(GL_PROGRAM_POINT_SIZE);
    glBindVertexArray(vao_wireframe_);
    glDrawArrays(GL_POINTS, 0, vertices_.size());
    glBindVertexArray(0);
    glDisable(GL_PROGRAM_POINT_SIZE);
  }
  
  // Draw poles
  if (show_poles_) {
    wireframe_shader_.Use();
    wireframe_shader_.SetUniform("mvp", mvp);
    wireframe_shader_.SetUniform("model", transform_);
    wireframe_shader_.SetUniform("center", center_);
    wireframe_shader_.SetUniform("radius", radius_);
    wireframe_shader_.SetUniform("color", wireframe_color_);
    
    glEnable(GL_PROGRAM_POINT_SIZE);
    glBindVertexArray(vao_wireframe_);
    // Draw only the pole vertices (first and last row)
    glDrawArrays(GL_POINTS, 0, longitude_segments_ + 1);  // North pole row
    glDrawArrays(GL_POINTS, latitude_segments_ * (longitude_segments_ + 1), 
                 longitude_segments_ + 1);  // South pole row
    glBindVertexArray(0);
    glDisable(GL_PROGRAM_POINT_SIZE);
  }
  
  // Draw equator
  if (show_equator_ && !equator_vertices_.empty()) {
    wireframe_shader_.Use();
    wireframe_shader_.SetUniform("mvp", mvp);
    wireframe_shader_.SetUniform("model", transform_);
    wireframe_shader_.SetUniform("center", center_);
    wireframe_shader_.SetUniform("radius", radius_);
    wireframe_shader_.SetUniform("color", equator_color_);
    
    glLineWidth(wireframe_width_ * 2.0f);  // Make equator thicker
    glBindVertexArray(vao_equator_);
    glDrawArrays(GL_LINE_STRIP, 0, equator_vertices_.size());
    glBindVertexArray(0);
  }
}

}  // namespace quickviz