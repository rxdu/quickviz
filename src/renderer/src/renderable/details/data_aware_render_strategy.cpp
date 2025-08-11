/**
 * @file data_aware_render_strategy.cpp
 * @author Canvas Refactoring Phase 2.2
 * @date 2025-01-11
 * @brief Implementation of data manager aware render strategies
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "data_aware_render_strategy.hpp"
#include "glad/glad.h"
#include "renderer/shader_program.hpp"
#include "renderer/renderable/details/canvas_performance.hpp"
#include <algorithm>

namespace quickviz {
namespace internal {

//==============================================================================
// DataAwareBatchedStrategy Implementation
//==============================================================================

bool DataAwareBatchedStrategy::CanHandle(const CanvasData& data) const {
  // Batched strategy can handle any data, but works best with many shapes
  return true;
}

void DataAwareBatchedStrategy::Render(const CanvasData& data, const RenderContext& context) {
  // Enhanced context needs to be created by Canvas
  // This is a compatibility layer for the existing interface
  RenderIndividualShapes(data, EnhancedRenderContext(context, nullptr, nullptr));
}

void DataAwareBatchedStrategy::RenderWithDataManager(const EnhancedRenderContext& context) {
  if (!context.data_manager) return;
  
  const auto& data = context.data_manager->GetShapeData();
  
  // Skip if there's no data to render
  if (data.points.empty() && data.lines.empty() && 
      data.rectangles.empty() && data.circles.empty() &&
      data.ellipses.empty() && data.polygons.empty()) {
    return;
  }

  // Setup common rendering state
  if (context.primitive_shader) {
    context.primitive_shader->Use();
    context.primitive_shader->TrySetUniform("projection", context.projection);
    context.primitive_shader->TrySetUniform("view", context.view);
    context.primitive_shader->TrySetUniform("model", glm::mat4(1.0f));
    context.primitive_shader->TrySetUniform("coordSystemTransform", context.coord_transform);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }

  // Use efficient batched rendering
  RenderBatches(context);
  
  // Render points individually (they're already efficient)
  RenderPoints(data, context);
  
  // Handle shapes not in batches
  RenderIndividualShapes(data, context);
  
  // Cleanup OpenGL state
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glBindVertexArray(0);
  glUseProgram(0);
}

void DataAwareBatchedStrategy::RenderBatches(const EnhancedRenderContext& context) {
  if (!context.data_manager) return;
  
  const auto& batch_tracker = context.data_manager->GetBatchOrderTracker();
  const auto& line_batches = context.data_manager->GetLineBatches();
  const auto& filled_batch = context.data_manager->GetFilledShapeBatch();
  const auto& outline_batches = context.data_manager->GetOutlineShapeBatches();
  
  // Render batches in order to preserve sequence
  auto ordered_primitives = batch_tracker.render_order;
  std::sort(ordered_primitives.begin(), ordered_primitives.end(),
            [](const BatchOrderTracker::OrderedPrimitive& a, const BatchOrderTracker::OrderedPrimitive& b) {
              return a.sequence_number < b.sequence_number;
            });
  
  for (const auto& primitive : ordered_primitives) {
    switch (primitive.type) {
      case BatchOrderTracker::OrderedPrimitive::Type::kLine: {
        auto it = line_batches.find(primitive.line_type);
        if (it != line_batches.end() && !it->second.vertices.empty()) {
          // Render line batch (simplified implementation)
          // Full implementation would involve proper VAO/VBO setup and rendering
        }
        break;
      }
      case BatchOrderTracker::OrderedPrimitive::Type::kFilledShape: {
        if (!filled_batch.vertices.empty()) {
          // Render filled shape batch (simplified implementation)
        }
        break;
      }
      case BatchOrderTracker::OrderedPrimitive::Type::kOutlineShape: {
        auto it = outline_batches.find(primitive.line_type);
        if (it != outline_batches.end() && !it->second.vertices.empty()) {
          // Render outline shape batch (simplified implementation)
        }
        break;
      }
    }
  }
}

void DataAwareBatchedStrategy::RenderIndividualShapes(const CanvasData& data, const EnhancedRenderContext& context) {
  if (!context.efficient_renderer) return;
  
  // Setup rendering state
  context.efficient_renderer->SetupRenderingState(context.projection, context.view, context.coord_transform);
  
  // Render ellipses (not typically batched due to complexity)
  for (const auto& ellipse : data.ellipses) {
    auto vertices = ShapeVertexGenerator::GenerateEllipse(
        ellipse.center, ellipse.rx, ellipse.ry, ellipse.angle,
        ellipse.start_angle, ellipse.end_angle, ellipse.num_segments, ellipse.filled);
    
    ShapeRenderParams params;
    params.color = ellipse.color;
    params.thickness = ellipse.thickness;
    params.line_type = ellipse.line_type;
    params.filled = ellipse.filled;
    params.primitive_type = ellipse.filled ? GL_TRIANGLE_FAN : GL_LINE_STRIP;
    
    context.efficient_renderer->RenderShape(vertices, params);
  }
  
  // Render polygons
  for (const auto& polygon : data.polygons) {
    auto vertices = ShapeVertexGenerator::GeneratePolygon(polygon.vertices);
    
    ShapeRenderParams params;
    params.color = polygon.color;
    params.thickness = polygon.thickness;
    params.line_type = polygon.line_type;
    params.filled = polygon.filled;
    params.primitive_type = polygon.filled ? GL_TRIANGLE_FAN : GL_LINE_LOOP;
    
    context.efficient_renderer->RenderShape(vertices, params);
  }
  
  // Cleanup
  context.efficient_renderer->CleanupRenderingState();
}

void DataAwareBatchedStrategy::RenderPoints(const CanvasData& data, const EnhancedRenderContext& context) {
  if (data.points.empty() || !context.primitive_shader) return;
  
  // Points are rendered individually as they're already efficient
  context.primitive_shader->TrySetUniform("renderMode", 0); // Point rendering mode
  
  glBindVertexArray(context.primitive_vao);
  glBindBuffer(GL_ARRAY_BUFFER, context.primitive_vbo);
  
  for (const auto& point : data.points) {
    // Upload point data
    float vertices[] = {point.position.x, point.position.y, point.position.z};
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
    
    // Set point size and color
    context.primitive_shader->TrySetUniform("uniformColor", point.color);
    glPointSize(point.size);
    
    // Render point
    glDrawArrays(GL_POINTS, 0, 1);
  }
  
  glPointSize(1.0f); // Reset to default
}

//==============================================================================
// DataAwareIndividualStrategy Implementation
//==============================================================================

bool DataAwareIndividualStrategy::CanHandle(const CanvasData& data) const {
  // Individual strategy can handle any data
  return true;
}

void DataAwareIndividualStrategy::Render(const CanvasData& data, const RenderContext& context) {
  // Enhanced context needs to be created by Canvas
  RenderAllShapesIndividually(data, EnhancedRenderContext(context, nullptr, nullptr));
}

void DataAwareIndividualStrategy::RenderAllShapesIndividually(const CanvasData& data, const EnhancedRenderContext& context) {
  // Setup common rendering state
  if (context.primitive_shader) {
    context.primitive_shader->Use();
    context.primitive_shader->TrySetUniform("projection", context.projection);
    context.primitive_shader->TrySetUniform("view", context.view);
    context.primitive_shader->TrySetUniform("model", glm::mat4(1.0f));
    context.primitive_shader->TrySetUniform("coordSystemTransform", context.coord_transform);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }
  
  // Render all shape types individually using efficient renderer
  if (context.efficient_renderer) {
    context.efficient_renderer->SetupRenderingState(context.projection, context.view, context.coord_transform);
    
    RenderLines(data, context);
    RenderRectangles(data, context);
    RenderCircles(data, context);
    RenderEllipses(data, context);
    RenderPolygons(data, context);
    
    context.efficient_renderer->CleanupRenderingState();
  }
  
  // Points are always rendered individually
  RenderPoints(data, context);
  
  // Cleanup
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glBindVertexArray(0);
  glUseProgram(0);
}

void DataAwareIndividualStrategy::RenderPoints(const CanvasData& data, const EnhancedRenderContext& context) {
  if (data.points.empty() || !context.primitive_shader) return;
  
  context.primitive_shader->TrySetUniform("renderMode", 0); // Point rendering mode
  
  for (const auto& point : data.points) {
    // Implementation similar to batched strategy
    // Simplified for brevity
  }
}

void DataAwareIndividualStrategy::RenderLines(const CanvasData& data, const EnhancedRenderContext& context) {
  if (!context.efficient_renderer) return;
  
  for (const auto& line : data.lines) {
    auto vertices = ShapeVertexGenerator::GenerateLine(line.start, line.end);
    
    ShapeRenderParams params;
    params.color = line.color;
    params.thickness = line.thickness;
    params.line_type = line.line_type;
    params.primitive_type = GL_LINES;
    
    context.efficient_renderer->RenderShape(vertices, params);
  }
}

void DataAwareIndividualStrategy::RenderRectangles(const CanvasData& data, const EnhancedRenderContext& context) {
  if (!context.efficient_renderer) return;
  
  for (const auto& rect : data.rectangles) {
    auto vertices = ShapeVertexGenerator::GenerateRectangle(rect.position, rect.width, rect.height);
    
    ShapeRenderParams params;
    params.color = rect.color;
    params.thickness = rect.thickness;
    params.line_type = rect.line_type;
    params.filled = rect.filled;
    params.primitive_type = rect.filled ? GL_TRIANGLE_FAN : GL_LINE_LOOP;
    
    context.efficient_renderer->RenderShape(vertices, params);
  }
}

void DataAwareIndividualStrategy::RenderCircles(const CanvasData& data, const EnhancedRenderContext& context) {
  if (!context.efficient_renderer) return;
  
  for (const auto& circle : data.circles) {
    auto vertices = ShapeVertexGenerator::GenerateCircle(circle.center, circle.radius, circle.num_segments, circle.filled);
    
    ShapeRenderParams params;
    params.color = circle.color;
    params.thickness = circle.thickness;
    params.line_type = circle.line_type;
    params.filled = circle.filled;
    params.primitive_type = circle.filled ? GL_TRIANGLE_FAN : GL_LINE_LOOP;
    
    context.efficient_renderer->RenderShape(vertices, params);
  }
}

void DataAwareIndividualStrategy::RenderEllipses(const CanvasData& data, const EnhancedRenderContext& context) {
  if (!context.efficient_renderer) return;
  
  for (const auto& ellipse : data.ellipses) {
    auto vertices = ShapeVertexGenerator::GenerateEllipse(
        ellipse.center, ellipse.rx, ellipse.ry, ellipse.angle,
        ellipse.start_angle, ellipse.end_angle, ellipse.num_segments, ellipse.filled);
    
    ShapeRenderParams params;
    params.color = ellipse.color;
    params.thickness = ellipse.thickness;
    params.line_type = ellipse.line_type;
    params.filled = ellipse.filled;
    params.primitive_type = ellipse.filled ? GL_TRIANGLE_FAN : GL_LINE_STRIP;
    
    context.efficient_renderer->RenderShape(vertices, params);
  }
}

void DataAwareIndividualStrategy::RenderPolygons(const CanvasData& data, const EnhancedRenderContext& context) {
  if (!context.efficient_renderer) return;
  
  for (const auto& polygon : data.polygons) {
    auto vertices = ShapeVertexGenerator::GeneratePolygon(polygon.vertices);
    
    ShapeRenderParams params;
    params.color = polygon.color;
    params.thickness = polygon.thickness;
    params.line_type = polygon.line_type;
    params.filled = polygon.filled;
    params.primitive_type = polygon.filled ? GL_TRIANGLE_FAN : GL_LINE_LOOP;
    
    context.efficient_renderer->RenderShape(vertices, params);
  }
}

//==============================================================================
// AdaptiveStrategySelector Implementation  
//==============================================================================

AdaptiveStrategySelector::AdaptiveStrategySelector()
    : batched_strategy_(std::make_unique<DataAwareBatchedStrategy>()),
      individual_strategy_(std::make_unique<DataAwareIndividualStrategy>()) {
}

RenderStrategy* AdaptiveStrategySelector::SelectStrategy(const CanvasData& data, const CanvasDataManager* data_manager) {
  if (!data_manager) {
    // Fallback to individual rendering if no data manager
    return individual_strategy_.get();
  }
  
  if (ShouldUseBatching(data, data_manager)) {
    return batched_strategy_.get();
  } else {
    return individual_strategy_.get();
  }
}

bool AdaptiveStrategySelector::ShouldUseBatching(const CanvasData& data, const CanvasDataManager* data_manager) const {
  // Use batching if enabled and we have enough shapes to benefit
  if (!data_manager->IsBatchingEnabled()) {
    return false;
  }
  
  const size_t total_shapes = CalculateTotalShapes(data);
  const bool has_complex_shapes = HasComplexShapes(data);
  
  // Use batching for many simple shapes, individual rendering for few or complex shapes
  const size_t batching_threshold = has_complex_shapes ? 50 : 20;
  return total_shapes > batching_threshold;
}

size_t AdaptiveStrategySelector::CalculateTotalShapes(const CanvasData& data) const {
  return data.points.size() + data.lines.size() + data.rectangles.size() +
         data.circles.size() + data.ellipses.size() + data.polygons.size();
}

bool AdaptiveStrategySelector::HasComplexShapes(const CanvasData& data) const {
  // Ellipses and polygons are considered complex
  return !data.ellipses.empty() || !data.polygons.empty() ||
         std::any_of(data.polygons.begin(), data.polygons.end(),
                     [](const Polygon& p) { return p.vertices.size() > 6; });
}

} // namespace internal
} // namespace quickviz