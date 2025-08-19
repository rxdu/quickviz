/**
 * @file render_strategy.hpp
 * @author Claude Code Assistant
 * @date 2025-01-04
 * @brief Render strategy interface for Canvas implementation
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_RENDER_STRATEGY_HPP
#define OPENGL_RENDERER_RENDER_STRATEGY_HPP

#include <glm/glm.hpp>
#include "renderable/details/canvas_data.hpp"

namespace quickviz {

// Forward declarations
class ShaderProgram;

/**
 * @brief Context passed to render strategies containing shared rendering state
 */
struct RenderContext {
  glm::mat4 projection;
  glm::mat4 view;
  glm::mat4 coord_transform;
  
  // OpenGL resources
  ShaderProgram* primitive_shader;
  uint32_t primitive_vao;
  uint32_t primitive_vbo;
  
  // Performance tracking (using void* to avoid forward declaration issues)
  void* render_stats;
  const void* perf_config;
  
  // Constructor
  RenderContext(const glm::mat4& proj, const glm::mat4& v, const glm::mat4& coord,
                ShaderProgram* shader, uint32_t vao, uint32_t vbo,
                void* stats, const void* config)
    : projection(proj), view(v), coord_transform(coord),
      primitive_shader(shader), primitive_vao(vao), primitive_vbo(vbo),
      render_stats(stats), perf_config(config) {}
};

/**
 * @brief Abstract base class for Canvas rendering strategies
 */
class RenderStrategy {
public:
  virtual ~RenderStrategy() = default;
  
  /**
   * @brief Render the canvas data using this strategy
   * @param data Canvas data to render
   * @param context Rendering context with matrices and OpenGL resources
   */
  virtual void Render(const CanvasData& data, const RenderContext& context) = 0;
  
  /**
   * @brief Check if this strategy can handle the given data efficiently
   * @param data Canvas data to check
   * @return true if this strategy is suitable for the data
   */
  virtual bool CanHandle(const CanvasData& data) const = 0;
};

} // namespace quickviz

#endif /* OPENGL_RENDERER_RENDER_STRATEGY_HPP */