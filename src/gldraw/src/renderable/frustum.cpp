/**
 * @file frustum.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Implementation of frustum renderer for sensor FOV visualization
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/frustum.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "gldraw/shader.hpp"

namespace quickviz {

namespace {

const char* kFrustumVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uModel;
uniform mat4 uCoordTransform;

out vec3 FragPos;
out vec3 Normal;

void main() {
    mat4 mvp = uProjection * uView * uCoordTransform * uModel;
    vec4 worldPos = uCoordTransform * uModel * vec4(aPos, 1.0);
    gl_Position = mvp * vec4(aPos, 1.0);
    
    FragPos = vec3(worldPos);
    Normal = mat3(transpose(inverse(uCoordTransform * uModel))) * aNormal;
}
)";

const char* kFrustumFragmentShader = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;

uniform vec3 uColor;
uniform float uAlpha;
uniform vec3 uLightPos;
uniform vec3 uViewPos;

out vec4 FragColor;

void main() {
    // Simple Phong lighting
    vec3 ambient = 0.3 * uColor;
    
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(uLightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * uColor;
    
    vec3 viewDir = normalize(uViewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = 0.2 * spec * vec3(1.0);
    
    vec3 result = ambient + diffuse + specular;
    FragColor = vec4(result, uAlpha);
}
)";

const char* kLineVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uModel;
uniform mat4 uCoordTransform;

void main() {
    gl_Position = uProjection * uView * uCoordTransform * uModel * vec4(aPos, 1.0);
}
)";

const char* kLineFragmentShader = R"(
#version 330 core
uniform vec3 uColor;

out vec4 FragColor;

void main() {
    FragColor = vec4(uColor, 1.0);
}
)";

} // anonymous namespace

Frustum::Frustum()
    : origin_(0.0f), direction_(0.0f, 0.0f, -1.0f), 
      near_distance_(0.1f), far_distance_(10.0f),
      color_(0.2f, 0.8f, 0.2f), alpha_(0.3f), render_mode_(RenderMode::kTransparent),
      wireframe_color_(0.0f, 1.0f, 0.0f), wireframe_width_(2.0f),
      show_near_face_(false), show_far_face_(true), show_side_faces_(true),
      show_center_line_(true), center_line_color_(1.0f, 1.0f, 0.0f),
      show_corner_markers_(false), corner_marker_size_(0.1f),
      vao_(0), vbo_vertices_(0), vbo_normals_(0), ebo_(0),
      vao_wireframe_(0), vbo_wireframe_(0), ebo_wireframe_(0),
      vao_lines_(0), vbo_lines_(0), needs_update_(true) {
      
  // Initialize with a default perspective frustum
  SetFromPerspective(origin_, direction_, 45.0f, 1.0f, near_distance_, far_distance_);
}

Frustum::~Frustum() {
  ReleaseGpuResources();
}

void Frustum::SetFromPerspective(const glm::vec3& origin, const glm::vec3& direction,
                                float fov_degrees, float aspect_ratio,
                                float near_distance, float far_distance) {
  origin_ = origin;
  direction_ = glm::normalize(direction);
  near_distance_ = near_distance;
  far_distance_ = far_distance;
  
  float fov_rad = glm::radians(fov_degrees);
  float half_height_near = near_distance * tan(fov_rad * 0.5f);
  float half_width_near = half_height_near * aspect_ratio;
  float half_height_far = far_distance * tan(fov_rad * 0.5f);
  float half_width_far = half_height_far * aspect_ratio;
  
  // Create coordinate system
  glm::vec3 up = glm::vec3(0, 1, 0);
  if (abs(glm::dot(direction_, up)) > 0.9f) {
    up = glm::vec3(1, 0, 0);
  }
  
  glm::vec3 right = glm::normalize(glm::cross(direction_, up));
  up = glm::normalize(glm::cross(right, direction_));
  
  glm::vec3 near_center = origin_ + direction_ * near_distance;
  glm::vec3 far_center = origin_ + direction_ * far_distance;
  
  // Near plane corners (top-left, top-right, bottom-right, bottom-left)
  near_corners_[0] = near_center + up * half_height_near - right * half_width_near;
  near_corners_[1] = near_center + up * half_height_near + right * half_width_near;
  near_corners_[2] = near_center - up * half_height_near + right * half_width_near;
  near_corners_[3] = near_center - up * half_height_near - right * half_width_near;
  
  // Far plane corners
  far_corners_[0] = far_center + up * half_height_far - right * half_width_far;
  far_corners_[1] = far_center + up * half_height_far + right * half_width_far;
  far_corners_[2] = far_center - up * half_height_far + right * half_width_far;
  far_corners_[3] = far_center - up * half_height_far - right * half_width_far;
  
  needs_update_ = true;
}

void Frustum::SetFromOrthographic(const glm::vec3& origin, const glm::vec3& direction,
                                 float width, float height,
                                 float near_distance, float far_distance) {
  origin_ = origin;
  direction_ = glm::normalize(direction);
  near_distance_ = near_distance;
  far_distance_ = far_distance;
  
  float half_width = width * 0.5f;
  float half_height = height * 0.5f;
  
  // Create coordinate system
  glm::vec3 up = glm::vec3(0, 1, 0);
  if (abs(glm::dot(direction_, up)) > 0.9f) {
    up = glm::vec3(1, 0, 0);
  }
  
  glm::vec3 right = glm::normalize(glm::cross(direction_, up));
  up = glm::normalize(glm::cross(right, direction_));
  
  glm::vec3 near_center = origin_ + direction_ * near_distance;
  glm::vec3 far_center = origin_ + direction_ * far_distance;
  
  // All corners have same relative positions for orthographic projection
  near_corners_[0] = near_center + up * half_height - right * half_width;
  near_corners_[1] = near_center + up * half_height + right * half_width;
  near_corners_[2] = near_center - up * half_height + right * half_width;
  near_corners_[3] = near_center - up * half_height - right * half_width;
  
  far_corners_[0] = far_center + up * half_height - right * half_width;
  far_corners_[1] = far_center + up * half_height + right * half_width;
  far_corners_[2] = far_center - up * half_height + right * half_width;
  far_corners_[3] = far_center - up * half_height - right * half_width;
  
  needs_update_ = true;
}

void Frustum::SetFromCorners(const glm::vec3& origin,
                            const glm::vec3* near_corners,
                            const glm::vec3* far_corners) {
  origin_ = origin;
  
  for (int i = 0; i < 4; ++i) {
    near_corners_[i] = near_corners[i];
    far_corners_[i] = far_corners[i];
  }
  
  // Calculate direction from origin to center of near plane
  glm::vec3 near_center = (near_corners[0] + near_corners[1] + near_corners[2] + near_corners[3]) * 0.25f;
  direction_ = glm::normalize(near_center - origin);
  
  near_distance_ = glm::length(near_center - origin);
  
  glm::vec3 far_center = (far_corners[0] + far_corners[1] + far_corners[2] + far_corners[3]) * 0.25f;
  far_distance_ = glm::length(far_center - origin);
  
  needs_update_ = true;
}

void Frustum::SetFromLidarSector(const glm::vec3& origin, const glm::vec3& direction,
                                float horizontal_fov, float vertical_fov,
                                float min_range, float max_range) {
  // LiDAR sector is like a perspective frustum but with specific angular ranges
  float aspect_ratio = horizontal_fov / vertical_fov;
  SetFromPerspective(origin, direction, vertical_fov, aspect_ratio, min_range, max_range);
}

void Frustum::SetColor(const glm::vec3& color) {
  color_ = color;
}

void Frustum::SetTransparency(float alpha) {
  alpha_ = std::max(0.0f, std::min(alpha, 1.0f));
}

void Frustum::SetRenderMode(RenderMode mode) {
  render_mode_ = mode;
}

void Frustum::SetWireframeColor(const glm::vec3& color) {
  wireframe_color_ = color;
}

void Frustum::SetWireframeWidth(float width) {
  wireframe_width_ = std::max(1.0f, width);
}

void Frustum::SetShowNearFace(bool show) {
  show_near_face_ = show;
  needs_update_ = true;
}

void Frustum::SetShowFarFace(bool show) {
  show_far_face_ = show;
  needs_update_ = true;
}

void Frustum::SetShowSideFaces(bool show) {
  show_side_faces_ = show;
  needs_update_ = true;
}

void Frustum::SetShowCenterLine(bool show) {
  show_center_line_ = show;
  needs_update_ = true;
}

void Frustum::SetCenterLineColor(const glm::vec3& color) {
  center_line_color_ = color;
}

void Frustum::SetShowCornerMarkers(bool show) {
  show_corner_markers_ = show;
  needs_update_ = true;
}

void Frustum::SetCornerMarkerSize(float size) {
  corner_marker_size_ = std::max(0.01f, size);
  needs_update_ = true;
}

void Frustum::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;
  
  try {
    // Setup main shader
    Shader vs(kFrustumVertexShader, Shader::Type::kVertex);
    Shader fs(kFrustumFragmentShader, Shader::Type::kFragment);
    if (!vs.Compile() || !fs.Compile()) {
      throw std::runtime_error("Frustum main shader compilation failed");
    }
    shader_.AttachShader(vs);
    shader_.AttachShader(fs);
    if (!shader_.LinkProgram()) {
      throw std::runtime_error("Frustum main shader linking failed");
    }
    
    // Setup line shader
    Shader line_vs(kLineVertexShader, Shader::Type::kVertex);
    Shader line_fs(kLineFragmentShader, Shader::Type::kFragment);
    if (!line_vs.Compile() || !line_fs.Compile()) {
      throw std::runtime_error("Frustum line shader compilation failed");
    }
    line_shader_.AttachShader(line_vs);
    line_shader_.AttachShader(line_fs);
    if (!line_shader_.LinkProgram()) {
      throw std::runtime_error("Frustum line shader linking failed");
    }
    
    // Wireframe shader needs separate shader objects (can't reuse line_vs/line_fs)
    Shader wireframe_vs(kLineVertexShader, Shader::Type::kVertex);
    Shader wireframe_fs(kLineFragmentShader, Shader::Type::kFragment);
    if (!wireframe_vs.Compile() || !wireframe_fs.Compile()) {
      throw std::runtime_error("Frustum wireframe shader compilation failed");
    }
    wireframe_shader_.AttachShader(wireframe_vs);
    wireframe_shader_.AttachShader(wireframe_fs);
    if (!wireframe_shader_.LinkProgram()) {
      throw std::runtime_error("Frustum wireframe shader linking failed");
    }
    
    GenerateFrustumGeometry();
    
    // Create VAOs and VBOs
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_vertices_);
    glGenBuffers(1, &vbo_normals_);
    glGenBuffers(1, &ebo_);
    
    glGenVertexArrays(1, &vao_wireframe_);
    glGenBuffers(1, &vbo_wireframe_);
    glGenBuffers(1, &ebo_wireframe_);
    
    glGenVertexArrays(1, &vao_lines_);
    glGenBuffers(1, &vbo_lines_);
    
    UpdateGpuBuffers();
    
  } catch (const std::exception& e) {
    std::cerr << "Frustum::AllocateGpuResources: " << e.what() << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void Frustum::ReleaseGpuResources() noexcept {
  if (vao_ != 0) {
    glDeleteVertexArrays(1, &vao_);
    vao_ = 0;
  }
  if (vbo_vertices_ != 0) {
    glDeleteBuffers(1, &vbo_vertices_);
    vbo_vertices_ = 0;
  }
  if (vbo_normals_ != 0) {
    glDeleteBuffers(1, &vbo_normals_);
    vbo_normals_ = 0;
  }
  if (ebo_ != 0) {
    glDeleteBuffers(1, &ebo_);
    ebo_ = 0;
  }
  if (vao_wireframe_ != 0) {
    glDeleteVertexArrays(1, &vao_wireframe_);
    vao_wireframe_ = 0;
  }
  if (vbo_wireframe_ != 0) {
    glDeleteBuffers(1, &vbo_wireframe_);
    vbo_wireframe_ = 0;
  }
  if (ebo_wireframe_ != 0) {
    glDeleteBuffers(1, &ebo_wireframe_);
    ebo_wireframe_ = 0;
  }
  if (vao_lines_ != 0) {
    glDeleteVertexArrays(1, &vao_lines_);
    vao_lines_ = 0;
  }
  if (vbo_lines_ != 0) {
    glDeleteBuffers(1, &vbo_lines_);
    vbo_lines_ = 0;
  }
}

void Frustum::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                     const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }
  
  if (needs_update_) {
    GenerateFrustumGeometry();
    UpdateGpuBuffers();
    needs_update_ = false;
  }
  
  glm::mat4 model = glm::mat4(1.0f);
  
  // Draw faces based on render mode
  if (render_mode_ == RenderMode::kSolid || render_mode_ == RenderMode::kTransparent) {
    if (render_mode_ == RenderMode::kTransparent) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    shader_.Use();
    shader_.SetUniform("uProjection", projection);
    shader_.SetUniform("uView", view);
    shader_.SetUniform("uModel", model);
    shader_.SetUniform("uCoordTransform", coord_transform);
    shader_.SetUniform("uColor", color_);
    shader_.SetUniform("uAlpha", alpha_);
    shader_.TrySetUniform("uLightPos", glm::vec3(10, 10, 10));
    shader_.TrySetUniform("uViewPos", glm::vec3(0, 0, 5));
    
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()), GL_UNSIGNED_INT, 0);
    
    if (render_mode_ == RenderMode::kTransparent) {
      glDisable(GL_BLEND);
    }
  }
  
  // Draw wireframe
  if (render_mode_ == RenderMode::kWireframe || render_mode_ == RenderMode::kOutline) {
    wireframe_shader_.Use();
    wireframe_shader_.SetUniform("uProjection", projection);
    wireframe_shader_.SetUniform("uView", view);
    wireframe_shader_.SetUniform("uModel", model);
    wireframe_shader_.SetUniform("uCoordTransform", coord_transform);
    wireframe_shader_.SetUniform("uColor", wireframe_color_);
    
    if (render_mode_ == RenderMode::kOutline) {
      glLineWidth(wireframe_width_);
    }
    
    glBindVertexArray(vao_wireframe_);
    glDrawElements(GL_LINES, static_cast<GLsizei>(wireframe_indices_.size()), GL_UNSIGNED_INT, 0);
    
    if (render_mode_ == RenderMode::kOutline) {
      glLineWidth(1.0f);
    }
  }
  
  // Draw points
  if (render_mode_ == RenderMode::kPoints) {
    wireframe_shader_.Use();
    wireframe_shader_.SetUniform("uProjection", projection);
    wireframe_shader_.SetUniform("uView", view);
    wireframe_shader_.SetUniform("uModel", model);
    wireframe_shader_.SetUniform("uCoordTransform", coord_transform);
    wireframe_shader_.SetUniform("uColor", wireframe_color_);
    
    glPointSize(corner_marker_size_ * 10.0f);
    
    glBindVertexArray(vao_wireframe_);
    glDrawArrays(GL_POINTS, 0, 8); // 8 corner points
    
    glPointSize(1.0f);
  }
  
  // Draw center line and corner markers
  if ((show_center_line_ || show_corner_markers_) && !line_vertices_.empty()) {
    line_shader_.Use();
    line_shader_.SetUniform("uProjection", projection);
    line_shader_.SetUniform("uView", view);
    line_shader_.SetUniform("uModel", model);
    line_shader_.SetUniform("uCoordTransform", coord_transform);
    line_shader_.SetUniform("uColor", center_line_color_);
    
    glBindVertexArray(vao_lines_);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(line_vertices_.size()));
  }
  
  glBindVertexArray(0);
}

bool Frustum::ContainsPoint(const glm::vec3& point) const {
  // Simple frustum containment test using plane equations
  // This is a simplified version - more sophisticated tests could be implemented
  
  // Check if point is within distance bounds
  glm::vec3 to_point = point - origin_;
  float distance = glm::dot(to_point, direction_);
  
  if (distance < near_distance_ || distance > far_distance_) {
    return false;
  }
  
  // For simplicity, assume it's inside if within distance bounds
  // A full implementation would test against all frustum planes
  return true;
}

std::vector<glm::vec3> Frustum::GetCornerPoints() const {
  std::vector<glm::vec3> corners;
  corners.reserve(8);
  
  for (int i = 0; i < 4; ++i) {
    corners.push_back(near_corners_[i]);
    corners.push_back(far_corners_[i]);
  }
  
  return corners;
}

void Frustum::GenerateFrustumGeometry() {
  vertices_.clear();
  normals_.clear();
  indices_.clear();
  wireframe_vertices_.clear();
  wireframe_indices_.clear();
  line_vertices_.clear();
  
  // Store vertices in clear layout: near corners first, then far corners
  // Near corners: 0,1,2,3 (top-left, top-right, bottom-right, bottom-left)
  // Far corners: 4,5,6,7 (same order)
  for (int i = 0; i < 4; ++i) {
    vertices_.push_back(near_corners_[i]);
  }
  for (int i = 0; i < 4; ++i) {
    vertices_.push_back(far_corners_[i]);
  }
  
  // Initialize normals array to match vertices size
  normals_.resize(8, glm::vec3(0, 1, 0));  // Default upward normal
  
  // Generate faces based on visibility settings
  if (show_near_face_) {
    // Near face triangle 1: 0,1,2 (CCW when viewed from inside frustum)
    indices_.push_back(0); indices_.push_back(1); indices_.push_back(2);
    // Near face triangle 2: 0,2,3
    indices_.push_back(0); indices_.push_back(2); indices_.push_back(3);
    
    // Set normals for near face (pointing toward origin)
    glm::vec3 normal = -direction_;
    for (int i = 0; i < 4; ++i) {
      normals_[i] = normal;
    }
  }
  
  if (show_far_face_) {
    // Far face triangle 1: 4,6,5 (CCW when viewed from outside frustum)
    indices_.push_back(4); indices_.push_back(6); indices_.push_back(5);
    // Far face triangle 2: 4,7,6
    indices_.push_back(4); indices_.push_back(7); indices_.push_back(6);
    
    // Set normals for far face (pointing away from origin)
    glm::vec3 normal = direction_;
    for (int i = 4; i < 8; ++i) {
      normals_[i] = normal;
    }
  }
  
  if (show_side_faces_) {
    // Generate 4 side faces
    for (int i = 0; i < 4; ++i) {
      int next = (i + 1) % 4;
      
      // Side face: two triangles connecting near and far corners
      // Triangle 1: near[i], far[i], near[next]
      indices_.push_back(i); indices_.push_back(i + 4); indices_.push_back(next);
      // Triangle 2: near[next], far[i], far[next]  
      indices_.push_back(next); indices_.push_back(i + 4); indices_.push_back(next + 4);
      
      // Calculate side face normal
      glm::vec3 v1 = near_corners_[next] - near_corners_[i];
      glm::vec3 v2 = far_corners_[i] - near_corners_[i];
      glm::vec3 normal = glm::normalize(glm::cross(v1, v2));
      
      // Apply normal to near and far vertices of this face
      if (normals_[i] == glm::vec3(0, 1, 0)) normals_[i] = normal;
      if (normals_[i + 4] == glm::vec3(0, 1, 0)) normals_[i + 4] = normal;
      if (normals_[next] == glm::vec3(0, 1, 0)) normals_[next] = normal;
      if (normals_[next + 4] == glm::vec3(0, 1, 0)) normals_[next + 4] = normal;
    }
  }
  
  // Copy vertices for wireframe (same layout)
  wireframe_vertices_ = vertices_;
  
  // Generate wireframe indices - edges of frustum
  // Near face edges (0->1->2->3->0)
  wireframe_indices_.insert(wireframe_indices_.end(), {0,1, 1,2, 2,3, 3,0});
  // Far face edges (4->5->6->7->4)  
  wireframe_indices_.insert(wireframe_indices_.end(), {4,5, 5,6, 6,7, 7,4});
  // Connecting edges (near to far)
  wireframe_indices_.insert(wireframe_indices_.end(), {0,4, 1,5, 2,6, 3,7});
  
  // Generate line geometry for center line and corner markers
  if (show_center_line_) {
    line_vertices_.push_back(origin_);
    glm::vec3 far_center = (far_corners_[0] + far_corners_[1] + far_corners_[2] + far_corners_[3]) * 0.25f;
    line_vertices_.push_back(far_center);
  }
  
  if (show_corner_markers_) {
    // Add small crosses at each corner (8 corners total)
    for (int i = 0; i < 4; ++i) {
      // Near corners
      glm::vec3 corner = near_corners_[i];
      glm::vec3 offset_x(corner_marker_size_, 0, 0);
      glm::vec3 offset_y(0, corner_marker_size_, 0);
      glm::vec3 offset_z(0, 0, corner_marker_size_);
      
      line_vertices_.insert(line_vertices_.end(), {
        corner - offset_x, corner + offset_x,
        corner - offset_y, corner + offset_y,  
        corner - offset_z, corner + offset_z
      });
      
      // Far corners
      corner = far_corners_[i];
      line_vertices_.insert(line_vertices_.end(), {
        corner - offset_x, corner + offset_x,
        corner - offset_y, corner + offset_y,
        corner - offset_z, corner + offset_z  
      });
    }
  }
}

void Frustum::UpdateGpuBuffers() {
  if (!IsGpuResourcesAllocated()) return;
  
  // Update main VAO
  glBindVertexArray(vao_);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3), 
               vertices_.data(), GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(0);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_normals_);
  glBufferData(GL_ARRAY_BUFFER, normals_.size() * sizeof(glm::vec3), 
               normals_.data(), GL_DYNAMIC_DRAW);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(1);
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(uint32_t), 
               indices_.data(), GL_DYNAMIC_DRAW);
  
  // Update wireframe VAO
  glBindVertexArray(vao_wireframe_);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_wireframe_);
  glBufferData(GL_ARRAY_BUFFER, wireframe_vertices_.size() * sizeof(glm::vec3), 
               wireframe_vertices_.data(), GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(0);
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_wireframe_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, wireframe_indices_.size() * sizeof(uint32_t), 
               wireframe_indices_.data(), GL_DYNAMIC_DRAW);
  
  // Update lines VAO
  if (!line_vertices_.empty()) {
    glBindVertexArray(vao_lines_);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo_lines_);
    glBufferData(GL_ARRAY_BUFFER, line_vertices_.size() * sizeof(glm::vec3), 
                 line_vertices_.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);
  }
  
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

} // namespace quickviz