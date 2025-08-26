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
uniform vec3 center;
uniform float radius;

void main() {
    vec3 worldPos = center + aPos * radius;
    FragPos = worldPos;
    Normal = aNormal;
    gl_Position = mvp * vec4(worldPos, 1.0);
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
    float ambientStrength = 0.5;
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
uniform vec3 center;
uniform float radius;

void main() {
    vec3 worldPos = center + aPos * radius;
    gl_Position = mvp * vec4(worldPos, 1.0);
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

Sphere::Sphere() : GeometricPrimitive() {
  // Initialize material with legacy colors
  material_.diffuse_color = legacy_color_;
  material_.wireframe_color = legacy_wireframe_color_;
  material_.opacity = legacy_opacity_;
  original_material_ = material_;
  GenerateSphereGeometry();
}

Sphere::Sphere(const glm::vec3& center, float radius)
    : GeometricPrimitive(), center_(center), radius_(radius) {
  // Initialize material with legacy colors
  material_.diffuse_color = legacy_color_;
  material_.wireframe_color = legacy_wireframe_color_;
  material_.opacity = legacy_opacity_;
  original_material_ = material_;
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
  MarkForUpdate();
}

glm::mat4 Sphere::GetTransform() const {
  // Create transform matrix from center and radius
  glm::mat4 transform = glm::mat4(1.0f);
  transform = glm::translate(transform, center_);
  transform = glm::scale(transform, glm::vec3(radius_));
  return transform_ * transform;
}

float Sphere::GetVolume() const {
  return (4.0f / 3.0f) * M_PI * radius_ * radius_ * radius_;
}

float Sphere::GetSurfaceArea() const {
  return 4.0f * M_PI * radius_ * radius_;
}

glm::vec3 Sphere::GetCentroid() const {
  return center_;
}

std::pair<glm::vec3, glm::vec3> Sphere::GetBoundingBox() const {
  glm::vec3 half_extents(radius_, radius_, radius_);
  return {center_ - half_extents, center_ + half_extents};
}

// Legacy color methods now update both legacy values and base class material
// These are kept for backward compatibility but deprecated

void Sphere::SetRenderMode(RenderMode mode) {
  // Convert legacy enum to base class enum
  GeometricPrimitive::RenderMode base_mode;
  switch (mode) {
    case RenderMode::kWireframe:
      base_mode = GeometricPrimitive::RenderMode::kWireframe;
      break;
    case RenderMode::kSolid:
      base_mode = GeometricPrimitive::RenderMode::kSolid;
      break;
    case RenderMode::kTransparent:
      base_mode = GeometricPrimitive::RenderMode::kTransparent;
      break;
    case RenderMode::kPoints:
      base_mode = GeometricPrimitive::RenderMode::kPoints;
      break;
    default:
      base_mode = GeometricPrimitive::RenderMode::kSolid;
      break;
  }
  GeometricPrimitive::SetRenderMode(base_mode);
}

void Sphere::SetResolution(int latitude_segments, int longitude_segments) {
  latitude_segments_ = latitude_segments;
  longitude_segments_ = longitude_segments;
  MarkForUpdate();
}

// SetWireframeWidth now handled by base class

void Sphere::SetShowPoles(bool show, float pole_size) {
  show_poles_ = show;
  pole_size_ = pole_size;
}

void Sphere::SetShowEquator(bool show, const glm::vec3& color) {
  show_equator_ = show;
  equator_color_ = color;
  if (show) {
    MarkForUpdate();  // Need to regenerate equator geometry
  }
}

// Volume and surface area methods now implemented above as virtual overrides

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
  
  MarkForUpdate();
}

void Sphere::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;
  
  try {
    // Initialize specialized shaders for parametric sphere rendering
    // Main rendering uses these specialized shaders for optimal performance
    Shader solid_vs(kSolidVertexShader, Shader::Type::kVertex);
    Shader solid_fs(kSolidFragmentShader, Shader::Type::kFragment);
    if (!solid_vs.Compile() || !solid_fs.Compile()) {
      throw std::runtime_error("Sphere shader compilation failed");
    }
    solid_shader_.AttachShader(solid_vs);
    solid_shader_.AttachShader(solid_fs);
    if (!solid_shader_.LinkProgram()) {
      throw std::runtime_error("Sphere shader linking failed");
    }
    
    Shader wireframe_vs(kWireframeVertexShader, Shader::Type::kVertex);
    Shader wireframe_fs(kWireframeFragmentShader, Shader::Type::kFragment);
    if (!wireframe_vs.Compile() || !wireframe_fs.Compile()) {
      throw std::runtime_error("Sphere wireframe shader compilation failed");
    }
    wireframe_shader_.AttachShader(wireframe_vs);
    wireframe_shader_.AttachShader(wireframe_fs);
    if (!wireframe_shader_.LinkProgram()) {
      throw std::runtime_error("Sphere wireframe shader linking failed");
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
  ClearUpdateFlag();
}

// Template Method Implementation
void Sphere::PrepareShaders(const glm::mat4& mvp_matrix, const glm::mat4& model_matrix) {
  // Make sure GPU resources are allocated and updated
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }
  
  if (NeedsUpdate()) {
    GenerateSphereGeometry();
    UpdateGpuBuffers();
  }
  
  // Store matrices for rendering methods (using specialized shaders)
  stored_mvp_matrix_ = mvp_matrix;
  stored_model_matrix_ = model_matrix;
}

void Sphere::RenderSolid() {
  if (vao_solid_ == 0) return;
  
  // Use specialized solid shader for sphere rendering
  solid_shader_.Use();
  solid_shader_.SetUniform("mvp", stored_mvp_matrix_);
  solid_shader_.SetUniform("center", center_);
  solid_shader_.SetUniform("radius", radius_);
  solid_shader_.SetUniform("color", material_.diffuse_color);
  solid_shader_.SetUniform("opacity", material_.opacity);
  solid_shader_.TrySetUniform("lightPos", glm::vec3(10, 10, 10));
  solid_shader_.TrySetUniform("viewPos", glm::vec3(0, 0, 5));
  
  glBindVertexArray(vao_solid_);
  glDrawElements(GL_TRIANGLES, solid_indices_.size(), GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
  
  // Render sphere-specific features
  RenderSpecialFeatures(stored_mvp_matrix_, stored_model_matrix_);
}

void Sphere::RenderWireframe() {
  if (vao_wireframe_ == 0) return;
  
  // Use specialized wireframe shader for sphere rendering
  wireframe_shader_.Use();
  wireframe_shader_.SetUniform("mvp", stored_mvp_matrix_);
  wireframe_shader_.SetUniform("center", center_);
  wireframe_shader_.SetUniform("radius", radius_);
  wireframe_shader_.SetUniform("color", material_.wireframe_color);
  
  glBindVertexArray(vao_wireframe_);
  glDrawElements(GL_LINES, wireframe_indices_.size(), GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
  
  // Render sphere-specific features
  RenderSpecialFeatures(stored_mvp_matrix_, stored_model_matrix_);
}

void Sphere::RenderPoints() {
  if (vao_wireframe_ == 0) return;
  
  // Use specialized wireframe shader for point rendering
  wireframe_shader_.Use();
  wireframe_shader_.SetUniform("mvp", stored_mvp_matrix_);
  wireframe_shader_.SetUniform("center", center_);
  wireframe_shader_.SetUniform("radius", radius_);
  wireframe_shader_.SetUniform("color", material_.diffuse_color);
  
  glBindVertexArray(vao_wireframe_);
  glDrawArrays(GL_POINTS, 0, vertices_.size());
  glBindVertexArray(0);
  
  // Render sphere-specific features
  RenderSpecialFeatures(stored_mvp_matrix_, stored_model_matrix_);
}

void Sphere::RenderSpecialFeatures(const glm::mat4& mvp_matrix, const glm::mat4& model_matrix) {
  // Draw poles using specialized shader
  if (show_poles_) {
    wireframe_shader_.Use();
    wireframe_shader_.SetUniform("mvp", mvp_matrix);
    wireframe_shader_.SetUniform("model", model_matrix);
    wireframe_shader_.SetUniform("center", center_);
    wireframe_shader_.SetUniform("radius", radius_);
    wireframe_shader_.SetUniform("color", material_.wireframe_color);
    
    glEnable(GL_PROGRAM_POINT_SIZE);
    glBindVertexArray(vao_wireframe_);
    // Draw only the pole vertices (first and last row)
    glDrawArrays(GL_POINTS, 0, longitude_segments_ + 1);  // North pole row
    glDrawArrays(GL_POINTS, latitude_segments_ * (longitude_segments_ + 1), 
                 longitude_segments_ + 1);  // South pole row
    glBindVertexArray(0);
    glDisable(GL_PROGRAM_POINT_SIZE);
  }
  
  // Draw equator using specialized shader
  if (show_equator_ && !equator_vertices_.empty()) {
    wireframe_shader_.Use();
    wireframe_shader_.SetUniform("mvp", mvp_matrix);
    wireframe_shader_.SetUniform("model", model_matrix);
    wireframe_shader_.SetUniform("center", center_);
    wireframe_shader_.SetUniform("radius", radius_);
    wireframe_shader_.SetUniform("color", equator_color_);
    
    glLineWidth(wireframe_width_ * 2.0f);  // Make equator thicker
    glBindVertexArray(vao_equator_);
    glDrawArrays(GL_LINE_STRIP, 0, equator_vertices_.size());
    glBindVertexArray(0);
  }
}

// Highlighting now handled by base class GeometricPrimitive::SetHighlighted

void Sphere::UpdateTransformFromCenterRadius() {
  // Helper method to update transform matrix from center and radius
  glm::mat4 transform = glm::mat4(1.0f);
  transform = glm::translate(transform, center_);
  transform = glm::scale(transform, glm::vec3(radius_));
  transform_ = transform;
}

}  // namespace quickviz