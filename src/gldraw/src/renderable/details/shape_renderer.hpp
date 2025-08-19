/**
 * @file shape_renderer.hpp
 * @author Claude Code Assistant
 * @date 2025-01-04
 * @brief Unified shape renderer to eliminate code duplication
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_SHAPE_RENDERER_HPP
#define OPENGL_RENDERER_SHAPE_RENDERER_HPP

#include <vector>
#include <functional>
#include <glm/glm.hpp>
#include "glad/glad.h"

namespace quickviz {

// Forward declarations
class ShaderProgram;

/**
 * @brief OpenGL resource manager for temporary rendering resources
 * 
 * Manages VAO/VBO creation, binding, and cleanup to eliminate duplication
 * across different shape rendering methods.
 */
class TempGLResources {
public:
  TempGLResources();
  ~TempGLResources();
  
  // Non-copyable, movable
  TempGLResources(const TempGLResources&) = delete;
  TempGLResources& operator=(const TempGLResources&) = delete;
  TempGLResources(TempGLResources&& other) noexcept;
  TempGLResources& operator=(TempGLResources&& other) noexcept;
  
  void Bind();
  void UploadVertices(const std::vector<float>& vertices);
  void SetupVertexAttributes();
  void SetupDefaultAttributes(const glm::vec4& color);
  
  GLuint GetVAO() const { return vao_; }
  GLuint GetVBO() const { return vbo_; }
  
private:
  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  
  void Cleanup();
};

/**
 * @brief Vertex layout specification for different shape types
 */
struct VertexLayout {
  bool has_position = true;
  bool has_color = false;
  bool has_size = false;
  
  static VertexLayout PositionOnly() {
    return {true, false, false};
  }
  
  static VertexLayout PositionColor() {
    return {true, true, false};
  }
  
  static VertexLayout PositionColorSize() {
    return {true, true, true};
  }
};

/**
 * @brief Rendering parameters for shape drawing
 */
struct RenderParams {
  glm::vec4 color;
  float thickness = 1.0f;
  bool filled = true;
  int line_type = 0; // LineType as int to avoid forward declaration
  GLenum primitive_type = GL_TRIANGLES;
  int vertex_count = 0;
  
  // Shader uniforms
  int render_mode = 0;
};

/**
 * @brief Unified shape renderer that eliminates code duplication
 * 
 * This class encapsulates the common OpenGL rendering patterns used
 * across all shape types, reducing code duplication and improving
 * maintainability.
 */
class ShapeRenderer {
public:
  explicit ShapeRenderer(ShaderProgram* shader);
  ~ShapeRenderer() = default;
  
  /**
   * @brief Render a shape with given vertices and parameters
   * @param vertices Flat array of vertex data  
   * @param layout Vertex attribute layout
   * @param params Rendering parameters
   */
  void RenderShape(const std::vector<float>& vertices,
                   const VertexLayout& layout,
                   const RenderParams& params);
  
  /**
   * @brief Render a shape using a vertex generator function
   * @param vertex_generator Function that generates vertices
   * @param layout Vertex attribute layout
   * @param params Rendering parameters
   */
  void RenderShape(const std::function<std::vector<float>()>& vertex_generator,
                   const VertexLayout& layout,
                   const RenderParams& params);
                   
  /**
   * @brief Set common shader uniforms for shape rendering
   * @param params Rendering parameters containing uniform values
   */
  void SetShaderUniforms(const RenderParams& params);
  
private:
  ShaderProgram* shader_;
  
  void SetupVertexAttributes(const VertexLayout& layout);
  void RenderPrimitive(const RenderParams& params);
};

/**
 * @brief Factory functions for creating common render parameters
 */
namespace RenderParamFactory {
  RenderParams FilledShape(const glm::vec4& color, float thickness = 1.0f);
  RenderParams OutlineShape(const glm::vec4& color, float thickness = 1.0f, int line_type = 0);
  RenderParams Points(const glm::vec4& color, float size = 1.0f);
  RenderParams Lines(const glm::vec4& color, float thickness = 1.0f, int line_type = 0);
}

} // namespace quickviz

#endif /* OPENGL_RENDERER_SHAPE_RENDERER_HPP */