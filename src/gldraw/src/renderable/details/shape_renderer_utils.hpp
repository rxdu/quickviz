/**
 * @file shape_renderer_utils.hpp
 * @author Canvas Refactoring Phase 1.3
 * @date 2025-01-11
 * @brief Consolidated shape rendering utilities for Canvas
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_SHAPE_RENDERER_UTILS_HPP
#define OPENGL_RENDERER_SHAPE_RENDERER_UTILS_HPP

#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include "gldraw/renderable/types.hpp"
#include "opengl_resource_pool.hpp"

namespace quickviz {
namespace internal {

/**
 * @brief Parameters for shape rendering operations
 */
struct ShapeRenderParams {
  glm::vec4 color{1.0f, 1.0f, 1.0f, 1.0f};
  float thickness = 1.0f;
  LineType line_type = LineType::kSolid;
  bool filled = true;
  unsigned int primitive_type = 0; // GL_TRIANGLES, GL_LINES, etc.
};

/**
 * @brief Consolidated shape vertex generators
 * 
 * These functions generate vertex data for common shapes used in Canvas rendering.
 * All functions follow the same pattern: return vertices as flat array of floats (x,y,z).
 */
class ShapeVertexGenerator {
public:
  /**
   * @brief Generate vertices for a circle
   * @param center Center point of the circle
   * @param radius Circle radius
   * @param segments Number of segments for circle approximation
   * @param filled Whether to include center vertex for filled rendering
   * @return Vertex data as flat array [x,y,z, x,y,z, ...]
   */
  static std::vector<float> GenerateCircle(const glm::vec3& center, float radius, 
                                          int segments = 32, bool filled = true);

  /**
   * @brief Generate vertices for an ellipse
   * @param center Center point of the ellipse
   * @param rx Radius in x direction
   * @param ry Radius in y direction
   * @param angle Rotation angle in radians
   * @param start_angle Start angle for arc (radians)
   * @param end_angle End angle for arc (radians)
   * @param segments Number of segments for ellipse approximation
   * @param filled Whether to include center vertex for filled rendering
   * @return Vertex data as flat array
   */
  static std::vector<float> GenerateEllipse(const glm::vec3& center, float rx, float ry,
                                           float angle, float start_angle, float end_angle,
                                           int segments = 32, bool filled = true);

  /**
   * @brief Generate vertices for a rectangle
   * @param position Bottom-left corner position
   * @param width Rectangle width
   * @param height Rectangle height
   * @return Vertex data as flat array (4 vertices for rectangle corners)
   */
  static std::vector<float> GenerateRectangle(const glm::vec3& position, 
                                             float width, float height);

  /**
   * @brief Generate vertices for a line
   * @param start Start point of the line
   * @param end End point of the line
   * @return Vertex data as flat array (2 vertices)
   */
  static std::vector<float> GenerateLine(const glm::vec3& start, const glm::vec3& end);

  /**
   * @brief Generate vertices for a polygon
   * @param vertices Polygon vertex positions
   * @return Vertex data as flat array
   */
  static std::vector<float> GeneratePolygon(const std::vector<glm::vec3>& vertices);
};

/**
 * @brief Efficient shape renderer using resource pool
 * 
 * This class consolidates all the common OpenGL rendering patterns used throughout
 * Canvas shape rendering, reducing code duplication and improving maintainability.
 */
class EfficientShapeRenderer {
public:
  /**
   * @brief Constructor
   * @param resource_pool Shared resource pool for VAO/VBO management
   * @param shader_program Shader program for rendering (non-owning pointer)
   */
  EfficientShapeRenderer(std::shared_ptr<OpenGLResourcePool> resource_pool,
                        void* shader_program); // Using void* to avoid circular includes

  /**
   * @brief Render a shape with given vertices and parameters
   * @param vertices Vertex data as flat array [x,y,z, x,y,z, ...]
   * @param params Rendering parameters (color, thickness, etc.)
   */
  void RenderShape(const std::vector<float>& vertices, const ShapeRenderParams& params);

  /**
   * @brief Set up common rendering state for shape rendering
   * @param projection Projection matrix
   * @param view View matrix
   * @param coord_transform Coordinate transformation matrix
   */
  void SetupRenderingState(const glm::mat4& projection, const glm::mat4& view,
                          const glm::mat4& coord_transform);

  /**
   * @brief Clean up rendering state
   */
  void CleanupRenderingState();

private:
  std::shared_ptr<OpenGLResourcePool> resource_pool_;
  void* shader_program_; // ShaderProgram* - using void* to avoid circular includes
  
  // Internal rendering helpers
  void SetupShaderUniforms(const ShapeRenderParams& params);
  void ConfigureVertexAttributes(const OpenGLResourcePool::TempResources& resources,
                                const std::vector<float>& vertices);
};

} // namespace internal
} // namespace quickviz

#endif /* OPENGL_RENDERER_SHAPE_RENDERER_UTILS_HPP */