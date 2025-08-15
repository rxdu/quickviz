/**
 * @file individual_render_strategy.hpp
 * @author Claude Code Assistant
 * @date 2025-01-04  
 * @brief Individual shape rendering strategy for Canvas
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_INDIVIDUAL_RENDER_STRATEGY_HPP
#define OPENGL_RENDERER_INDIVIDUAL_RENDER_STRATEGY_HPP

#include "render_strategy.hpp"
#include "shape_renderer.hpp"

namespace quickviz {

/**
 * @brief Render strategy that renders each shape individually
 * 
 * This strategy renders shapes one by one without batching. It's used
 * as a fallback when batching is disabled or for shapes that don't
 * benefit from batching (e.g., complex polygons, ellipses).
 */
class IndividualRenderStrategy : public RenderStrategy {
public:
  IndividualRenderStrategy(ShapeRenderer* shape_renderer = nullptr);
  ~IndividualRenderStrategy() override = default;
  
  // RenderStrategy interface
  void Render(const CanvasData& data, const RenderContext& context) override;
  bool CanHandle(const CanvasData& data) const override;
  
  // Public single shape rendering methods for Canvas access
  void RenderSingleEllipse(const Ellipse& ellipse, const RenderContext& context);
  void RenderSinglePolygon(const Polygon& polygon, const RenderContext& context);
  
private:
  // Individual shape rendering methods
  void RenderPoints(const CanvasData& data, const RenderContext& context);
  void RenderLines(const CanvasData& data, const RenderContext& context);
  void RenderRectangles(const CanvasData& data, const RenderContext& context);
  void RenderCircles(const CanvasData& data, const RenderContext& context);
  void RenderEllipses(const CanvasData& data, const RenderContext& context);
  void RenderPolygons(const CanvasData& data, const RenderContext& context);
  
  // Single primitive rendering methods for sequence-ordered rendering
  void RenderSinglePoint(const Point& point, const RenderContext& context);
  void RenderSingleLine(const Line& line, const RenderContext& context);
  void RenderSingleRectangle(const Rectangle& rect, const RenderContext& context);
  void RenderSingleCircle(const Circle& circle, const RenderContext& context);
  
  // Helper methods
  void SetupCommonRenderState(const RenderContext& context);
  void CleanupRenderState();
  
  // Unified shape renderer for individual shapes
  ShapeRenderer* shape_renderer_;
};

} // namespace quickviz

#endif /* OPENGL_RENDERER_INDIVIDUAL_RENDER_STRATEGY_HPP */