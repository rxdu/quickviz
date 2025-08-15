/**
 * @file data_aware_render_strategy.hpp
 * @author Canvas Refactoring Phase 2.2
 * @date 2025-01-11
 * @brief Data manager aware render strategies for Canvas
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_DATA_AWARE_RENDER_STRATEGY_HPP
#define OPENGL_RENDERER_DATA_AWARE_RENDER_STRATEGY_HPP

#include "render_strategy.hpp"
#include "canvas_data_manager.hpp"
#include "shape_renderer_utils.hpp"

namespace quickviz {
namespace internal {

/**
 * @brief Enhanced render context for data manager integration
 */
struct EnhancedRenderContext : public RenderContext {
  const CanvasDataManager* data_manager;
  EfficientShapeRenderer* efficient_renderer;
  
  EnhancedRenderContext(const RenderContext& base, 
                       const CanvasDataManager* dm,
                       EfficientShapeRenderer* renderer)
    : RenderContext(base), data_manager(dm), efficient_renderer(renderer) {}
};

/**
 * @brief Batched rendering strategy that works with CanvasDataManager
 */
class DataAwareBatchedStrategy : public RenderStrategy {
public:
  DataAwareBatchedStrategy() = default;
  ~DataAwareBatchedStrategy() override = default;
  
  // RenderStrategy interface
  void Render(const CanvasData& data, const RenderContext& context) override;
  bool CanHandle(const CanvasData& data) const override;
  
private:
  void RenderWithDataManager(const EnhancedRenderContext& context);
  void RenderBatches(const EnhancedRenderContext& context);
  void RenderIndividualShapes(const CanvasData& data, const EnhancedRenderContext& context);
  void RenderPoints(const CanvasData& data, const EnhancedRenderContext& context);
};

/**
 * @brief Individual rendering strategy that works with CanvasDataManager
 */
class DataAwareIndividualStrategy : public RenderStrategy {
public:
  DataAwareIndividualStrategy() = default;
  ~DataAwareIndividualStrategy() override = default;
  
  // RenderStrategy interface
  void Render(const CanvasData& data, const RenderContext& context) override;
  bool CanHandle(const CanvasData& data) const override;
  
private:
  void RenderAllShapesIndividually(const CanvasData& data, const EnhancedRenderContext& context);
  void RenderPoints(const CanvasData& data, const EnhancedRenderContext& context);
  void RenderLines(const CanvasData& data, const EnhancedRenderContext& context);
  void RenderRectangles(const CanvasData& data, const EnhancedRenderContext& context);
  void RenderCircles(const CanvasData& data, const EnhancedRenderContext& context);
  void RenderEllipses(const CanvasData& data, const EnhancedRenderContext& context);
  void RenderPolygons(const CanvasData& data, const EnhancedRenderContext& context);
};

/**
 * @brief Adaptive strategy selector that chooses the best strategy dynamically
 */
class AdaptiveStrategySelector {
public:
  AdaptiveStrategySelector();
  ~AdaptiveStrategySelector() = default;
  
  /**
   * @brief Select the best rendering strategy for the given data
   * @param data Canvas data to analyze
   * @param data_manager Data manager for accessing batches
   * @return Best strategy for rendering the data
   */
  RenderStrategy* SelectStrategy(const CanvasData& data, const CanvasDataManager* data_manager);
  
private:
  std::unique_ptr<DataAwareBatchedStrategy> batched_strategy_;
  std::unique_ptr<DataAwareIndividualStrategy> individual_strategy_;
  
  // Heuristics for strategy selection
  bool ShouldUseBatching(const CanvasData& data, const CanvasDataManager* data_manager) const;
  size_t CalculateTotalShapes(const CanvasData& data) const;
  bool HasComplexShapes(const CanvasData& data) const;
};

} // namespace internal
} // namespace quickviz

#endif /* OPENGL_RENDERER_DATA_AWARE_RENDER_STRATEGY_HPP */