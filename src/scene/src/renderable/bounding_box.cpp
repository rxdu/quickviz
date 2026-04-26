/**
 * @file bounding_box.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Implementation of bounding box renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "scene/renderable/bounding_box.hpp"

#include <iostream>

#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "scene/shader.hpp"

namespace quickviz {

namespace {
const char* kEdgeVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 mvp;

void main() {
    // CRITICAL FIX: MVP already includes full transformation
    gl_Position = mvp * vec4(aPos, 1.0);
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
uniform mat4 model;

void main() {
    // CRITICAL FIX: MVP already includes full transformation
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
    gl_Position = mvp * vec4(aPos, 1.0);
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

BoundingBox::BoundingBox() : GeometricPrimitive() {
  // Initialize material with legacy colors
  material_.diffuse_color = legacy_face_color_;
  material_.wireframe_color = edge_color_;
  material_.opacity = legacy_opacity_;
  original_material_ = material_;
  GenerateBoxGeometry();
}

BoundingBox::BoundingBox(const glm::vec3& min_point, const glm::vec3& max_point)
    : GeometricPrimitive(), min_point_(min_point), max_point_(max_point) {
  // Initialize material with legacy colors
  material_.diffuse_color = legacy_face_color_;
  material_.wireframe_color = edge_color_;
  material_.opacity = legacy_opacity_;
  original_material_ = material_;
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
  MarkForUpdate();
}

void BoundingBox::SetCenter(const glm::vec3& center, const glm::vec3& size) {
  glm::vec3 half_size = size * 0.5f;
  min_point_ = center - half_size;
  max_point_ = center + half_size;
  MarkForUpdate();
}

void BoundingBox::SetTransform(const glm::mat4& transform) {
  transform_ = transform;
  MarkForUpdate();
}

glm::mat4 BoundingBox::GetTransform() const {
  // Create transform matrix from bounding box center
  glm::vec3 center = GetCenter();
  glm::mat4 transform = glm::mat4(1.0f);
  transform = glm::translate(transform, center);
  return transform_ * transform;
}

float BoundingBox::GetVolume() const {
  glm::vec3 size = GetSize();
  return size.x * size.y * size.z;
}

float BoundingBox::GetSurfaceArea() const {
  glm::vec3 size = GetSize();
  return 2.0f * (size.x * size.y + size.y * size.z + size.z * size.x);
}

glm::vec3 BoundingBox::GetCentroid() const {
  return GetCenter();
}

std::pair<glm::vec3, glm::vec3> BoundingBox::GetBoundingBox() const {
  // CRITICAL FIX: Return bounding box in local coordinates
  // Transform will be applied by the selection system
  glm::vec3 center = (min_point_ + max_point_) * 0.5f;
  glm::vec3 local_min = min_point_ - center;
  glm::vec3 local_max = max_point_ - center;
  return {local_min, local_max};
}

// Legacy SetColor handled by base class

void BoundingBox::SetEdgeColor(const glm::vec3& color) {
  edge_color_ = color;
  // Also update the material wireframe color
  material_.wireframe_color = color;
  if (!is_highlighted_) {
    original_material_.wireframe_color = color;
  }
}

// Legacy SetOpacity handled by base class

void BoundingBox::SetEdgeWidth(float width) {
  edge_width_ = width;
}

void BoundingBox::SetRenderMode(RenderMode mode) {
  // Convert legacy enum to base class enum
  GeometricPrimitive::RenderMode base_mode;
  switch (mode) {
    case RenderMode::kWireframe:
      base_mode = GeometricPrimitive::RenderMode::kWireframe;
      show_edges_ = true;
      show_faces_ = false;
      break;
    case RenderMode::kSolid:
      base_mode = GeometricPrimitive::RenderMode::kSolid;
      show_edges_ = false;
      show_faces_ = true;
      break;
    case RenderMode::kTransparent:
      base_mode = GeometricPrimitive::RenderMode::kTransparent;
      show_edges_ = true;
      show_faces_ = true;
      break;
    default:
      base_mode = GeometricPrimitive::RenderMode::kWireframe;
      break;
  }
  GeometricPrimitive::SetRenderMode(base_mode);
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
  
  // CRITICAL FIX: Generate vertices in local coordinates to eliminate double transformation bug
  // Transform will be applied by the MVP matrix, not by vertex generation
  glm::vec3 center = (min_point_ + max_point_) * 0.5f;
  glm::vec3 local_min = min_point_ - center;
  glm::vec3 local_max = max_point_ - center;
  
  vertices_ = {
    // Bottom face (z = min) - in local coordinates
    {local_min.x, local_min.y, local_min.z},  // 0: min, min, min
    {local_max.x, local_min.y, local_min.z},  // 1: max, min, min
    {local_max.x, local_max.y, local_min.z},  // 2: max, max, min
    {local_min.x, local_max.y, local_min.z},  // 3: min, max, min
    
    // Top face (z = max) - in local coordinates
    {local_min.x, local_min.y, local_max.z},  // 4: min, min, max
    {local_max.x, local_min.y, local_max.z},  // 5: max, min, max
    {local_max.x, local_max.y, local_max.z},  // 6: max, max, max
    {local_min.x, local_max.y, local_max.z}   // 7: min, max, max
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
  
  MarkForUpdate();
}

void BoundingBox::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;
  
  try {
    // Initialize specialized shaders optimized for bounding box rendering
    Shader edge_vs(kEdgeVertexShader, Shader::Type::kVertex);
    Shader edge_fs(kEdgeFragmentShader, Shader::Type::kFragment);
    if (!edge_vs.Compile() || !edge_fs.Compile()) {
      throw std::runtime_error("BoundingBox edge shader compilation failed");
    }
    edge_shader_.AttachShader(edge_vs);
    edge_shader_.AttachShader(edge_fs);
    if (!edge_shader_.LinkProgram()) {
      throw std::runtime_error("BoundingBox edge shader linking failed");
    }
    
    Shader face_vs(kFaceVertexShader, Shader::Type::kVertex);
    Shader face_fs(kFaceFragmentShader, Shader::Type::kFragment);
    if (!face_vs.Compile() || !face_fs.Compile()) {
      throw std::runtime_error("BoundingBox face shader compilation failed");
    }
    face_shader_.AttachShader(face_vs);
    face_shader_.AttachShader(face_fs);
    if (!face_shader_.LinkProgram()) {
      throw std::runtime_error("BoundingBox face shader linking failed");
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
  ClearUpdateFlag();
}

// Template Method Implementation
void BoundingBox::PrepareShaders(const glm::mat4& mvp_matrix, const glm::mat4& model_matrix) {
  // Make sure GPU resources are allocated and updated
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }
  
  if (NeedsUpdate()) {
    GenerateBoxGeometry();
    UpdateGpuBuffers();
  }
  
  // BoundingBox uses its own legacy shaders for all modes since it has a different
  // vertex structure than the shared shaders expect
  // Store matrices for rendering methods
  stored_mvp_matrix_ = mvp_matrix;
  stored_model_matrix_ = model_matrix;
}

void BoundingBox::RenderSolid() {
  if (vao_faces_ == 0) return;
  
  // Use specialized face shader for solid rendering
  face_shader_.Use();
  face_shader_.SetUniform("mvp", stored_mvp_matrix_);
  face_shader_.SetUniform("model", stored_model_matrix_);
  face_shader_.SetUniform("color", material_.diffuse_color);
  face_shader_.SetUniform("opacity", material_.opacity);
  face_shader_.TrySetUniform("lightPos", glm::vec3(10, 10, 10));
  face_shader_.TrySetUniform("viewPos", glm::vec3(0, 0, 5));
  
  // Draw faces
  glBindVertexArray(vao_faces_);
  glDrawElements(GL_TRIANGLES, face_indices_.size(), GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
  
  // Render bounding box-specific features
  RenderSpecialFeatures(stored_mvp_matrix_, stored_model_matrix_);
}

void BoundingBox::RenderWireframe() {
  if (vao_edges_ == 0) return;
  
  // Use specialized edge shader for wireframe rendering
  edge_shader_.Use();
  edge_shader_.SetUniform("mvp", stored_mvp_matrix_);
  edge_shader_.SetUniform("color", material_.wireframe_color);
  
  // Draw wireframe edges
  glBindVertexArray(vao_edges_);
  glDrawElements(GL_LINES, edge_indices_.size(), GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
  
  // Render bounding box-specific features
  RenderSpecialFeatures(stored_mvp_matrix_, stored_model_matrix_);
}

void BoundingBox::RenderPoints() {
  if (vao_edges_ == 0) return;
  
  // Use specialized edge shader for point rendering
  edge_shader_.Use();
  edge_shader_.SetUniform("mvp", stored_mvp_matrix_);
  edge_shader_.SetUniform("color", material_.diffuse_color);
  
  // Draw all 8 corner points
  glBindVertexArray(vao_edges_);
  glDrawArrays(GL_POINTS, 0, 8);
  glBindVertexArray(0);
  
  // Render bounding box-specific features
  RenderSpecialFeatures(stored_mvp_matrix_, stored_model_matrix_);
}

void BoundingBox::RenderSpecialFeatures(const glm::mat4& mvp_matrix, const glm::mat4& model_matrix) {
  // Handle special edge rendering for transparent mode or when explicitly requested
  if ((render_mode_ == GeometricPrimitive::RenderMode::kTransparent && show_edges_) ||
      (render_mode_ != GeometricPrimitive::RenderMode::kWireframe && show_edges_)) {
    
    // For edges on transparent objects, temporarily enable depth writing
    // so edges can properly occlude objects behind them
    bool was_transparent = (render_mode_ == GeometricPrimitive::RenderMode::kTransparent);
    if (was_transparent) {
      glDepthMask(GL_TRUE);  // Enable depth writing for edges
    }
    
    edge_shader_.Use();
    edge_shader_.SetUniform("mvp", mvp_matrix);
    edge_shader_.SetUniform("color", edge_color_);
    
    glLineWidth(edge_width_);
    glBindVertexArray(vao_edges_);
    glDrawElements(GL_LINES, edge_indices_.size(), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
    
    // Restore depth writing state for transparent objects
    if (was_transparent) {
      glDepthMask(GL_FALSE);  // Restore no depth writing for subsequent transparent rendering
    }
  }
  
  // Draw corner points if requested
  if (show_corner_points_) {
    edge_shader_.Use();
    edge_shader_.SetUniform("mvp", mvp_matrix);
    edge_shader_.SetUniform("color", edge_color_);
    
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(corner_point_size_);
    glBindVertexArray(vao_edges_);
    glDrawArrays(GL_POINTS, 0, 8);  // Draw all 8 vertices as points
    glBindVertexArray(0);
    glDisable(GL_PROGRAM_POINT_SIZE);
  }
}

void BoundingBox::RenderIdBuffer(const glm::mat4& mvp_matrix) {
  if (vao_faces_ == 0) return;
  
  // Use the shared GeometricPrimitive ID shader but render with bounding box's geometry directly
  // This ensures correct ID color rendering for GPU selection
  
  // Get the static ID shader from GeometricPrimitive
  static std::unique_ptr<ShaderProgram> id_shader = nullptr;
  static bool shader_initialized = false;
  
  if (!shader_initialized) {
    try {
      const char* id_vertex_shader = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        
        uniform mat4 uMVP;
        
        void main() {
          gl_Position = uMVP * vec4(aPos, 1.0);
        }
      )";
      
      const char* id_fragment_shader = R"(
        #version 330 core
        out vec4 FragColor;
        
        uniform vec3 uIdColor;
        
        void main() {
          FragColor = vec4(uIdColor, 1.0);
        }
      )";
      
      id_shader = std::make_unique<ShaderProgram>();
      Shader vs(id_vertex_shader, Shader::Type::kVertex);
      Shader fs(id_fragment_shader, Shader::Type::kFragment);
      
      if (vs.Compile() && fs.Compile()) {
        id_shader->AttachShader(vs);
        id_shader->AttachShader(fs);
        if (!id_shader->LinkProgram()) {
          id_shader = nullptr;
        }
      } else {
        id_shader = nullptr;
      }
      shader_initialized = true;
    } catch (const std::exception&) {
      id_shader = nullptr;
      shader_initialized = true;
    }
  }
  
  if (!id_shader) return;
  
  // Use the ID shader with bounding box geometry
  id_shader->Use();
  id_shader->SetUniform("uMVP", mvp_matrix);
  id_shader->SetUniform("uIdColor", id_color_);
  
  // Render bounding box faces with ID color
  glBindVertexArray(vao_faces_);
  glDrawElements(GL_TRIANGLES, face_indices_.size(), GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}

void BoundingBox::UpdateTransformFromBounds() {
  // Helper method to update transform matrix from bounding box
  glm::vec3 center = GetCenter();
  glm::mat4 transform = glm::mat4(1.0f);
  transform = glm::translate(transform, center);
  transform_ = transform;
}

}  // namespace quickviz