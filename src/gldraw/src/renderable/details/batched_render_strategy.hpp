/**
 * @file batched_render_strategy.hpp
 * @author Claude Code Assistant  
 * @date 2025-01-04
 * @brief Batched rendering strategy for Canvas
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_BATCHED_RENDER_STRATEGY_HPP
#define OPENGL_RENDERER_BATCHED_RENDER_STRATEGY_HPP

#include <unordered_map>
#include "render_strategy.hpp"
#include "shape_renderer.hpp"
#include "gldraw/renderable/types.hpp"

namespace quickviz {

// Forward declarations
struct LineBatch;
struct ShapeBatch;

/**
 * @brief Render strategy that uses batching for improved performance
 * 
 * This strategy batches similar primitives together to reduce draw calls
 * and state changes, providing better performance for scenes with many objects.
 */
class BatchedRenderStrategy : public RenderStrategy {
public:
  /**
   * @brief Constructor
   * @param line_batch Reference to line batch data
   * @param filled_batch Reference to filled shape batch data  
   * @param outline_batch Reference to outline shape batch data
   * @param shape_renderer Unified shape renderer for individual shapes
   */
  BatchedRenderStrategy(std::unordered_map<LineType, LineBatch>& line_batches, ShapeBatch& filled_batch, std::unordered_map<LineType, ShapeBatch>& outline_batches, ShapeRenderer* shape_renderer = nullptr);
  
  ~BatchedRenderStrategy() override = default;
  
  // RenderStrategy interface
  void Render(const CanvasData& data, const RenderContext& context) override;
  bool CanHandle(const CanvasData& data) const override;
  
private:
  // Batch rendering methods
  void RenderBatches(const RenderContext& context);
  void RenderPoints(const CanvasData& data, const RenderContext& context);
  void RenderIndividualShapes(const CanvasData& data, const RenderContext& context);
  
  // References to batch data (owned by Canvas)
  std::unordered_map<LineType, LineBatch>& line_batches_;
  ShapeBatch& filled_shape_batch_;
  std::unordered_map<LineType, ShapeBatch>& outline_shape_batches_;
  
  // Unified shape renderer for individual shapes
  ShapeRenderer* shape_renderer_;
};

} // namespace quickviz

#endif /* OPENGL_RENDERER_BATCHED_RENDER_STRATEGY_HPP */