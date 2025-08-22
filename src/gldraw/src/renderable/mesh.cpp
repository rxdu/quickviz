/**
 * @file mesh.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-01-22
 * @brief Implementation of triangle mesh renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/mesh.hpp"

#ifdef IMVIEW_WITH_GLAD
#include <glad/glad.h>
#else
#include <GL/gl.h>
#include <GL/glext.h>
#endif

#include <iostream>
#include <stdexcept>

#include "gldraw/shader.hpp"

namespace quickviz {

namespace {
// Simple mesh vertex shader
const char* mesh_vertex_shader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uMVP;
uniform mat4 uModel;

out vec3 Normal;
out vec3 FragPos;

void main() {
    FragPos = vec3(uModel * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(uModel))) * aNormal;
    gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

// Simple mesh fragment shader
const char* mesh_fragment_shader = R"(
#version 330 core
in vec3 Normal;
in vec3 FragPos;

out vec4 FragColor;

uniform vec3 uColor;
uniform float uAlpha;
uniform vec3 uLightPos;

void main() {
    vec3 lightColor = vec3(1.0, 1.0, 1.0);
    vec3 lightPos = uLightPos;
    
    // Ambient
    float ambientStrength = 0.3;
    vec3 ambient = ambientStrength * lightColor;
    
    // Diffuse
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;
    
    vec3 result = (ambient + diffuse) * uColor;
    FragColor = vec4(result, uAlpha);
}
)";

// Simple wireframe shader
const char* wireframe_vertex_shader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 uMVP;

void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

const char* wireframe_fragment_shader = R"(
#version 330 core
out vec4 FragColor;

uniform vec3 uColor;

void main() {
    FragColor = vec4(uColor, 1.0);
}
)";

// Normal visualization shader
const char* normal_vertex_shader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uMVP;
uniform float uNormalScale;

void main() {
    vec3 pos = aPos;
    if (gl_VertexID % 2 == 1) {
        pos += aNormal * uNormalScale;
    }
    gl_Position = uMVP * vec4(pos, 1.0);
}
)";

const char* normal_fragment_shader = R"(
#version 330 core
out vec4 FragColor;

uniform vec3 uColor;

void main() {
    FragColor = vec4(uColor, 1.0);
}
)";

} // anonymous namespace

Mesh::Mesh() {
  // Shaders will be compiled when GPU resources are allocated
}

Mesh::~Mesh() {
  ReleaseGpuResources();
}

void Mesh::SetVertices(const std::vector<glm::vec3>& vertices) {
  vertices_ = vertices;
  needs_update_ = true;
}

void Mesh::SetIndices(const std::vector<uint32_t>& indices) {
  indices_ = indices;
  needs_update_ = true;
}

void Mesh::SetNormals(const std::vector<glm::vec3>& normals) {
  normals_ = normals;
  needs_update_ = true;
}

void Mesh::SetColor(const glm::vec3& color) {
  color_ = color;
}

void Mesh::SetTransparency(float alpha) {
  alpha_ = alpha;
}

void Mesh::SetWireframeMode(bool wireframe) {
  wireframe_mode_ = wireframe;
}

void Mesh::SetWireframeColor(const glm::vec3& color) {
  wireframe_color_ = color;
}

void Mesh::SetShowNormals(bool show, float scale) {
  show_normals_ = show;
  normal_scale_ = scale;
}

void Mesh::SetNormalColor(const glm::vec3& color) {
  normal_color_ = color;
}

void Mesh::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) {
    return;
  }

  try {
    // Compile and link mesh shader
    Shader mesh_vs(mesh_vertex_shader, Shader::Type::kVertex);
    Shader mesh_fs(mesh_fragment_shader, Shader::Type::kFragment);
    if (!mesh_vs.Compile() || !mesh_fs.Compile()) {
      throw std::runtime_error("Mesh shader compilation failed");
    }
    mesh_shader_.AttachShader(mesh_vs);
    mesh_shader_.AttachShader(mesh_fs);
    if (!mesh_shader_.LinkProgram()) {
      throw std::runtime_error("Mesh shader linking failed");
    }
    
    // Compile and link wireframe shader
    Shader wire_vs(wireframe_vertex_shader, Shader::Type::kVertex);
    Shader wire_fs(wireframe_fragment_shader, Shader::Type::kFragment);
    if (!wire_vs.Compile() || !wire_fs.Compile()) {
      throw std::runtime_error("Wireframe shader compilation failed");
    }
    wireframe_shader_.AttachShader(wire_vs);
    wireframe_shader_.AttachShader(wire_fs);
    if (!wireframe_shader_.LinkProgram()) {
      throw std::runtime_error("Wireframe shader linking failed");
    }
    
    // Compile and link normal shader
    Shader norm_vs(normal_vertex_shader, Shader::Type::kVertex);
    Shader norm_fs(normal_fragment_shader, Shader::Type::kFragment);
    if (!norm_vs.Compile() || !norm_fs.Compile()) {
      throw std::runtime_error("Normal shader compilation failed");
    }
    normal_shader_.AttachShader(norm_vs);
    normal_shader_.AttachShader(norm_fs);
    if (!normal_shader_.LinkProgram()) {
      throw std::runtime_error("Normal shader linking failed");
    }

    // Generate OpenGL objects
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vertex_vbo_);
    glGenBuffers(1, &index_vbo_);
    glGenBuffers(1, &normal_vbo_);

    // Update GPU buffers with current data
    UpdateGpuBuffers();

  } catch (const std::exception& e) {
    std::cerr << "Mesh::AllocateGpuResources: " << e.what() << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void Mesh::ReleaseGpuResources() noexcept {
  if (vao_ != 0) {
    glDeleteVertexArrays(1, &vao_);
    vao_ = 0;
  }
  if (vertex_vbo_ != 0) {
    glDeleteBuffers(1, &vertex_vbo_);
    vertex_vbo_ = 0;
  }
  if (index_vbo_ != 0) {
    glDeleteBuffers(1, &index_vbo_);
    index_vbo_ = 0;
  }
  if (normal_vbo_ != 0) {
    glDeleteBuffers(1, &normal_vbo_);
    normal_vbo_ = 0;
  }
}

void Mesh::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                  const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }

  if (vertices_.empty() || indices_.empty()) {
    return;
  }

  if (needs_update_) {
    UpdateGpuBuffers();
    needs_update_ = false;
  }

  glm::mat4 model = glm::mat4(1.0f);
  glm::mat4 mvp = projection * view * coord_transform * model;

  // Draw solid mesh
  if (!wireframe_mode_) {
    DrawMesh(mvp);
  }
  
  // Draw wireframe if enabled
  if (wireframe_mode_) {
    DrawWireframe(mvp);
  }
  
  // Draw normals if enabled
  if (show_normals_ && HasNormals()) {
    DrawNormals(mvp);
  }
}

void Mesh::GenerateNormals() {
  if (vertices_.empty() || indices_.empty()) {
    return;
  }

  normals_.clear();
  normals_.resize(vertices_.size(), glm::vec3(0.0f));

  // Calculate face normals and accumulate to vertex normals
  for (size_t i = 0; i < indices_.size(); i += 3) {
    uint32_t i0 = indices_[i];
    uint32_t i1 = indices_[i + 1];
    uint32_t i2 = indices_[i + 2];

    if (i0 >= vertices_.size() || i1 >= vertices_.size() || i2 >= vertices_.size()) {
      continue;
    }

    glm::vec3 v0 = vertices_[i0];
    glm::vec3 v1 = vertices_[i1];
    glm::vec3 v2 = vertices_[i2];

    glm::vec3 edge1 = v1 - v0;
    glm::vec3 edge2 = v2 - v0;
    glm::vec3 face_normal = glm::normalize(glm::cross(edge1, edge2));

    normals_[i0] += face_normal;
    normals_[i1] += face_normal;
    normals_[i2] += face_normal;
  }

  // Normalize vertex normals
  for (auto& normal : normals_) {
    normal = glm::normalize(normal);
  }
}

void Mesh::UpdateGpuBuffers() {
  if (!IsGpuResourcesAllocated()) {
    return;
  }

  // Generate normals if not provided
  if (normals_.empty() && !vertices_.empty()) {
    GenerateNormals();
  }

  glBindVertexArray(vao_);

  // Upload vertex data
  glBindBuffer(GL_ARRAY_BUFFER, vertex_vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3),
               vertices_.data(), GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(0);

  // Upload normal data
  if (!normals_.empty()) {
    glBindBuffer(GL_ARRAY_BUFFER, normal_vbo_);
    glBufferData(GL_ARRAY_BUFFER, normals_.size() * sizeof(glm::vec3),
                 normals_.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(1);
  }

  // Upload index data
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_vbo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(uint32_t),
               indices_.data(), GL_STATIC_DRAW);

  glBindVertexArray(0);
}

void Mesh::DrawMesh(const glm::mat4& mvp) {
  glBindVertexArray(vao_);
  
  mesh_shader_.Use();
  mesh_shader_.SetUniform("uMVP", mvp);
  mesh_shader_.SetUniform("uModel", glm::mat4(1.0f));
  mesh_shader_.SetUniform("uColor", color_);
  mesh_shader_.SetUniform("uAlpha", alpha_);
  mesh_shader_.SetUniform("uLightPos", glm::vec3(10.0f, 10.0f, 10.0f));

  if (alpha_ < 1.0f) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }

  glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, 0);

  if (alpha_ < 1.0f) {
    glDisable(GL_BLEND);
  }
  
  glBindVertexArray(0);
}

void Mesh::DrawWireframe(const glm::mat4& mvp) {
  glBindVertexArray(vao_);
  
  wireframe_shader_.Use();
  wireframe_shader_.SetUniform("uMVP", mvp);
  wireframe_shader_.SetUniform("uColor", wireframe_color_);

  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, 0);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  
  glBindVertexArray(0);
}

void Mesh::DrawNormals(const glm::mat4& mvp) {
  // TODO: Implement normal visualization
  // This requires a geometry shader or separate line rendering
  // For now, skip implementation
}

}  // namespace quickviz