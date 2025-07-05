/**
 * @file batched_render_strategy.cpp
 * @author Claude Code Assistant
 * @date 2025-01-04
 * @brief Implementation of batched rendering strategy
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "batched_render_strategy.hpp"
#include "shape_generators.hpp"

#include <iostream>
#include "glad/glad.h"
#include "renderer/shader_program.hpp"
#include "renderer/renderable/canvas.hpp"
#include "canvas_data.hpp"

namespace quickviz {

BatchedRenderStrategy::BatchedRenderStrategy(std::unordered_map<LineType, LineBatch>& line_batches, 
                                           ShapeBatch& filled_batch, 
                                           ShapeBatch& outline_batch,
                                           ShapeRenderer* shape_renderer)
  : line_batches_(line_batches), filled_shape_batch_(filled_batch), outline_shape_batch_(outline_batch), shape_renderer_(shape_renderer) {
}

bool BatchedRenderStrategy::CanHandle(const CanvasData& data) const {
  // Batched strategy can handle any data, but works best with many similar shapes
  return true;
}

void BatchedRenderStrategy::Render(const CanvasData& data, const RenderContext& context) {
  // Skip if there's no data to render
  if (data.points.empty() && data.lines.empty() && 
      data.rectangles.empty() && data.circles.empty() &&
      data.ellipses.empty() && data.polygons.empty()) {
    return;
  }

  // Use efficient batched rendering for better performance
  RenderBatches(context);
  
  // Still render points individually (they're already efficient)
  RenderPoints(data, context);
  
  // Handle non-batched shapes (ellipses, polygons) with individual rendering
  if (!data.ellipses.empty() || !data.polygons.empty()) {
    RenderIndividualShapes(data, context);
  }
}

void BatchedRenderStrategy::RenderBatches(const RenderContext& context) {
  // This method will contain the batched rendering logic
  // For now, this is a placeholder - the actual batching logic
  // will be moved from the original Canvas::RenderBatches method
  
  context.primitive_shader->Use();
  context.primitive_shader->TrySetUniform("projection", context.projection);
  context.primitive_shader->TrySetUniform("view", context.view);
  context.primitive_shader->TrySetUniform("model", glm::mat4(1.0f));
  context.primitive_shader->TrySetUniform("coordSystemTransform", context.coord_transform);
  
  // Enable OpenGL states
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  // TODO: Implement actual batch rendering logic
  // This will be moved from Canvas::RenderBatches in the next step
}

void BatchedRenderStrategy::RenderPoints(const CanvasData& data, const RenderContext& context) {
  if (data.points.empty()) return;
  
  // Setup for point rendering
  context.primitive_shader->Use();
  context.primitive_shader->TrySetUniform("projection", context.projection);
  context.primitive_shader->TrySetUniform("view", context.view);
  context.primitive_shader->TrySetUniform("model", glm::mat4(1.0f));
  context.primitive_shader->TrySetUniform("coordSystemTransform", context.coord_transform);
  context.primitive_shader->TrySetUniform("renderMode", 0); // Points mode
  
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  glBindVertexArray(context.primitive_vao);
  glEnable(GL_PROGRAM_POINT_SIZE);
  
  // Update buffer with point data
  glBindBuffer(GL_ARRAY_BUFFER, context.primitive_vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(Point) * data.points.size(), 
               data.points.data(), GL_DYNAMIC_DRAW);
  
  // Enable point attributes
  glEnableVertexAttribArray(0); // Position
  glEnableVertexAttribArray(1); // Color
  glEnableVertexAttribArray(2); // Size
  
  // Draw points
  glDrawArrays(GL_POINTS, 0, data.points.size());
  
  // Cleanup point rendering
  glDisable(GL_PROGRAM_POINT_SIZE);
  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(2);
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  
  // Update stats (cast void* back to proper type)
  if (context.render_stats) {
    // Note: This casting is safe because we control what gets passed in
    // Will be improved in Phase 2 when we extract proper interfaces
    // auto* stats = static_cast<RenderStats*>(context.render_stats);
    // stats->points_rendered = data.points.size();
    // stats->draw_calls++;
  }
}

void BatchedRenderStrategy::RenderIndividualShapes(const CanvasData& data, const RenderContext& context) {
  if (!shape_renderer_) {
    // Fallback to original individual rendering if no unified renderer available
    return;
  }
  
  if (data.ellipses.empty() && data.polygons.empty()) {
    return;
  }
  
  // Render ellipses using unified renderer
  for (const auto& ellipse : data.ellipses) {
    auto vertices = ShapeGenerators::GenerateEllipseVertices(ellipse);
    VertexLayout layout = VertexLayout::PositionOnly();
    
    RenderParams params;
    params.color = ellipse.color;
    params.thickness = ellipse.thickness;
    params.filled = ellipse.filled;
    params.primitive_type = ellipse.filled ? GL_TRIANGLE_FAN : GL_LINE_LOOP;
    params.vertex_count = vertices.size() / 3;
    
    shape_renderer_->RenderShape(vertices, layout, params);
  }
  
  // Render polygons using unified renderer
  for (const auto& polygon : data.polygons) {
    auto vertices = ShapeGenerators::GeneratePolygonVertices(polygon);
    VertexLayout layout = VertexLayout::PositionOnly();
    
    RenderParams params;
    params.color = polygon.color;
    params.thickness = polygon.thickness;
    params.filled = polygon.filled;
    params.primitive_type = polygon.filled ? GL_TRIANGLE_FAN : GL_LINE_LOOP;
    params.vertex_count = vertices.size() / 3;
    
    shape_renderer_->RenderShape(vertices, layout, params);
  }
}

} // namespace quickviz