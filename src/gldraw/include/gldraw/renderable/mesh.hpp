/**
 * @file mesh.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-01-22
 * @brief Triangle mesh renderer for arbitrary surface visualization
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_MESH_HPP
#define QUICKVIZ_MESH_HPP

#include <vector>
#include <glm/glm.hpp>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief Renderable triangle mesh for arbitrary surface data
 */
class Mesh : public OpenGlObject {
 public:
  Mesh();
  ~Mesh();

  // Mesh data setup
  void SetVertices(const std::vector<glm::vec3>& vertices);
  void SetIndices(const std::vector<uint32_t>& indices);
  void SetNormals(const std::vector<glm::vec3>& normals);
  void SetColor(const glm::vec3& color);
  void SetTransparency(float alpha);
  
  // Rendering options
  void SetWireframeMode(bool wireframe);
  void SetWireframeColor(const glm::vec3& color);
  void SetShowNormals(bool show, float scale = 0.1f);
  void SetNormalColor(const glm::vec3& color);

  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_ != 0; }

  // Utility methods
  size_t GetVertexCount() const { return vertices_.size(); }
  size_t GetTriangleCount() const { return indices_.size() / 3; }
  bool HasNormals() const { return normals_.size() == vertices_.size(); }
  
 private:
  void GenerateNormals();
  void UpdateGpuBuffers();
  void DrawMesh(const glm::mat4& mvp);
  void DrawWireframe(const glm::mat4& mvp);
  void DrawNormals(const glm::mat4& mvp);

  // Mesh data
  std::vector<glm::vec3> vertices_;
  std::vector<uint32_t> indices_;
  std::vector<glm::vec3> normals_;
  
  // Rendering properties
  glm::vec3 color_ = glm::vec3(0.7f, 0.7f, 0.9f);
  float alpha_ = 1.0f;
  bool wireframe_mode_ = false;
  glm::vec3 wireframe_color_ = glm::vec3(0.0f, 0.0f, 0.0f);
  bool show_normals_ = false;
  float normal_scale_ = 0.1f;
  glm::vec3 normal_color_ = glm::vec3(0.0f, 1.0f, 0.0f);
  
  // OpenGL resources
  uint32_t vao_ = 0;
  uint32_t vertex_vbo_ = 0;
  uint32_t index_vbo_ = 0;
  uint32_t normal_vbo_ = 0;
  
  ShaderProgram mesh_shader_;
  ShaderProgram wireframe_shader_;
  ShaderProgram normal_shader_;
  
  bool needs_update_ = true;
};

}  // namespace quickviz

#endif /* QUICKVIZ_MESH_HPP */