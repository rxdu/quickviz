/**
 * @file bounding_box.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Implementation of bounding box renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/bounding_box.hpp"

#include <iostream>

#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "gldraw/shader.hpp"

namespace quickviz {

namespace {
const char* kEdgeVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 mvp;
uniform mat4 transform;

void main() {
    gl_Position = mvp * transform * vec4(aPos, 1.0);
}
)";

const char* kEdgeFragmentShader = R"(
#version 330 core
out vec4 FragColor;
uniform vec3 color;

void main() {
    FragColor = vec4(color, 1.0);
}
)";

const char* kFaceVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

out vec3 FragPos;
out vec3 Normal;

uniform mat4 mvp;
uniform mat4 transform;

void main() {
    FragPos = vec3(transform * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(transform))) * aNormal;
    gl_Position = mvp * transform * vec4(aPos, 1.0);
}
)";

const char* kFaceFragmentShader = R"(
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
    
    vec3 result = ambient + diffuse;
    FragColor = vec4(result, opacity);
}
)";

}  // namespace

BoundingBox::BoundingBox() {
  GenerateBoxGeometry();
}

BoundingBox::BoundingBox(const glm::vec3& min_point, const glm::vec3& max_point)
    : min_point_(min_point), max_point_(max_point) {
  GenerateBoxGeometry();
}

BoundingBox::~BoundingBox() {
  if (IsGpuResourcesAllocated()) {
    ReleaseGpuResources();
  }
}

void BoundingBox::SetBounds(const glm::vec3& min_point, const glm::vec3& max_point) {
  min_point_ = min_point;
  max_point_ = max_point;
  needs_update_ = true;
}

void BoundingBox::SetCenter(const glm::vec3& center, const glm::vec3& size) {
  glm::vec3 half_size = size * 0.5f;
  min_point_ = center - half_size;
  max_point_ = center + half_size;
  needs_update_ = true;
}

void BoundingBox::SetTransform(const glm::mat4& transform) {
  transform_ = transform;
}

void BoundingBox::SetColor(const glm::vec3& color) {
  face_color_ = color;
}

void BoundingBox::SetEdgeColor(const glm::vec3& color) {
  edge_color_ = color;
}

void BoundingBox::SetOpacity(float opacity) {
  opacity_ = opacity;
}

void BoundingBox::SetEdgeWidth(float width) {
  edge_width_ = width;
}

void BoundingBox::SetRenderMode(RenderMode mode) {
  render_mode_ = mode;
  
  // Auto-configure visibility based on mode
  switch (mode) {
    case RenderMode::kWireframe:
      show_edges_ = true;
      show_faces_ = false;
      break;
    case RenderMode::kSolid:
      show_edges_ = false;
      show_faces_ = true;
      opacity_ = 1.0f;
      break;
    case RenderMode::kTransparent:
      show_edges_ = true;
      show_faces_ = true;
      if (opacity_ > 0.8f) opacity_ = 0.3f;  // Ensure transparency
      break;
  }
}

void BoundingBox::SetShowEdges(bool show) {
  show_edges_ = show;
}

void BoundingBox::SetShowFaces(bool show) {
  show_faces_ = show;
}

void BoundingBox::SetShowCornerPoints(bool show, float point_size) {
  show_corner_points_ = show;
  corner_point_size_ = point_size;
}

glm::vec3 BoundingBox::GetCenter() const {
  return (min_point_ + max_point_) * 0.5f;
}

glm::vec3 BoundingBox::GetSize() const {
  return max_point_ - min_point_;
}

void BoundingBox::GenerateBoxGeometry() {
  vertices_.clear();
  edge_indices_.clear();
  face_indices_.clear();
  
  // Generate 8 vertices of the box
  vertices_ = {
    // Bottom face (z = min)
    {min_point_.x, min_point_.y, min_point_.z},  // 0: min, min, min
    {max_point_.x, min_point_.y, min_point_.z},  // 1: max, min, min
    {max_point_.x, max_point_.y, min_point_.z},  // 2: max, max, min
    {min_point_.x, max_point_.y, min_point_.z},  // 3: min, max, min
    
    // Top face (z = max)
    {min_point_.x, min_point_.y, max_point_.z},  // 4: min, min, max
    {max_point_.x, min_point_.y, max_point_.z},  // 5: max, min, max
    {max_point_.x, max_point_.y, max_point_.z},  // 6: max, max, max
    {min_point_.x, max_point_.y, max_point_.z}   // 7: min, max, max
  };
  
  // Generate edge indices (12 edges of a cube)
  edge_indices_ = {
    // Bottom face edges
    0, 1,  1, 2,  2, 3,  3, 0,
    // Top face edges
    4, 5,  5, 6,  6, 7,  7, 4,
    // Vertical edges
    0, 4,  1, 5,  2, 6,  3, 7
  };
  
  // Generate face indices (12 triangles for 6 faces)
  face_indices_ = {
    // Bottom face (z = min)
    0, 1, 2,  2, 3, 0,
    // Top face (z = max)
    4, 7, 6,  6, 5, 4,
    // Front face (y = min)
    0, 4, 5,  5, 1, 0,
    // Back face (y = max)
    2, 6, 7,  7, 3, 2,
    // Left face (x = min)
    0, 3, 7,  7, 4, 0,
    // Right face (x = max)
    1, 5, 6,  6, 2, 1
  };
  
  needs_update_ = true;
}

void BoundingBox::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;
  
  try {
    // Compile edge shader
    Shader edge_vs(kEdgeVertexShader, Shader::Type::kVertex);
    Shader edge_fs(kEdgeFragmentShader, Shader::Type::kFragment);
    if (!edge_vs.Compile() || !edge_fs.Compile()) {
      throw std::runtime_error("Edge shader compilation failed");
    }
    edge_shader_.AttachShader(edge_vs);
    edge_shader_.AttachShader(edge_fs);
    if (!edge_shader_.LinkProgram()) {
      throw std::runtime_error("Edge shader linking failed");
    }
    
    // Compile face shader
    Shader face_vs(kFaceVertexShader, Shader::Type::kVertex);
    Shader face_fs(kFaceFragmentShader, Shader::Type::kFragment);
    if (!face_vs.Compile() || !face_fs.Compile()) {
      throw std::runtime_error("Face shader compilation failed");
    }
    face_shader_.AttachShader(face_vs);
    face_shader_.AttachShader(face_fs);
    if (!face_shader_.LinkProgram()) {
      throw std::runtime_error("Face shader linking failed");
    }
    
    // Create VAOs and VBOs for edges
    glGenVertexArrays(1, &vao_edges_);
    glGenBuffers(1, &vbo_vertices_);
    glGenBuffers(1, &ebo_edges_);
    
    // Create VAO and EBO for faces (shares vertex buffer)
    glGenVertexArrays(1, &vao_faces_);
    glGenBuffers(1, &ebo_faces_);
    
    UpdateGpuBuffers();
    
  } catch (const std::exception& e) {
    std::cerr << "BoundingBox::AllocateGpuResources: " << e.what() << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void BoundingBox::ReleaseGpuResources() noexcept {
  if (vao_edges_ != 0) {
    glDeleteVertexArrays(1, &vao_edges_);
    vao_edges_ = 0;
  }
  if (vao_faces_ != 0) {
    glDeleteVertexArrays(1, &vao_faces_);
    vao_faces_ = 0;
  }
  if (vbo_vertices_ != 0) {
    glDeleteBuffers(1, &vbo_vertices_);
    vbo_vertices_ = 0;
  }
  if (ebo_edges_ != 0) {
    glDeleteBuffers(1, &ebo_edges_);
    ebo_edges_ = 0;
  }
  if (ebo_faces_ != 0) {
    glDeleteBuffers(1, &ebo_faces_);
    ebo_faces_ = 0;
  }
}

void BoundingBox::UpdateGpuBuffers() {
  if (!IsGpuResourcesAllocated()) return;
  
  // Update vertex buffer (shared between edges and faces)
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3),
               vertices_.data(), GL_DYNAMIC_DRAW);
  
  // Setup edge VAO
  glBindVertexArray(vao_edges_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(0);
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_edges_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, edge_indices_.size() * sizeof(uint32_t),
               edge_indices_.data(), GL_DYNAMIC_DRAW);
  
  // Setup face VAO
  glBindVertexArray(vao_faces_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(0);
  
  // For faces, we need normals - generate them per-face
  // For simplicity, we'll calculate normals in the fragment shader
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_faces_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, face_indices_.size() * sizeof(uint32_t),
               face_indices_.data(), GL_DYNAMIC_DRAW);
  
  glBindVertexArray(0);
  needs_update_ = false;
}

void BoundingBox::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                         const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }
  
  if (needs_update_) {
    GenerateBoxGeometry();
    UpdateGpuBuffers();
  }
  
  glm::mat4 mvp = projection * view * coord_transform;
  glm::mat4 final_transform = coord_transform * transform_;
  
  // Draw faces first (if transparent, they need to be drawn before edges)
  if (show_faces_) {
    face_shader_.Use();
    face_shader_.SetUniform("mvp", mvp);
    face_shader_.SetUniform("transform", transform_);
    face_shader_.SetUniform("color", face_color_);
    face_shader_.SetUniform("opacity", opacity_);
    face_shader_.TrySetUniform("lightPos", glm::vec3(10, 10, 10));
    face_shader_.TrySetUniform("viewPos", glm::vec3(0, 0, 5));
    
    if (opacity_ < 1.0f) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    glBindVertexArray(vao_faces_);
    glDrawElements(GL_TRIANGLES, face_indices_.size(), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
    
    if (opacity_ < 1.0f) {
      glDisable(GL_BLEND);
    }
  }
  
  // Draw edges
  if (show_edges_) {
    edge_shader_.Use();
    edge_shader_.SetUniform("mvp", mvp);
    edge_shader_.SetUniform("transform", transform_);
    edge_shader_.SetUniform("color", edge_color_);
    
    glLineWidth(edge_width_);
    glBindVertexArray(vao_edges_);
    glDrawElements(GL_LINES, edge_indices_.size(), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
  }
  
  // Draw corner points
  if (show_corner_points_) {
    edge_shader_.Use();  // Reuse edge shader for points
    edge_shader_.SetUniform("mvp", mvp);
    edge_shader_.SetUniform("transform", transform_);
    edge_shader_.SetUniform("color", edge_color_);
    
    glEnable(GL_PROGRAM_POINT_SIZE);
    glBindVertexArray(vao_edges_);
    glDrawArrays(GL_POINTS, 0, 8);  // Draw all 8 vertices as points
    glBindVertexArray(0);
    glDisable(GL_PROGRAM_POINT_SIZE);
  }
}

}  // namespace quickviz