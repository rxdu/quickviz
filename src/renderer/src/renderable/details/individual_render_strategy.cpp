/**
 * @file individual_render_strategy.cpp
 * @author Claude Code Assistant
 * @date 2025-01-04
 * @brief Implementation of individual shape rendering strategy
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "individual_render_strategy.hpp"
#include "shape_generators.hpp"

#include <iostream>
#include "glad/glad.h"
#include "renderer/shader_program.hpp"
#include "canvas_data.hpp"

namespace quickviz {

IndividualRenderStrategy::IndividualRenderStrategy(ShapeRenderer* shape_renderer)
  : shape_renderer_(shape_renderer) {
}

bool IndividualRenderStrategy::CanHandle(const CanvasData& data) const {
  // Individual strategy can handle any data
  return true;
}

void IndividualRenderStrategy::Render(const CanvasData& data, const RenderContext& context) {
  // Skip if there's no data to render
  if (data.points.empty() && data.lines.empty() && 
      data.rectangles.empty() && data.circles.empty() &&
      data.ellipses.empty() && data.polygons.empty()) {
    return;
  }

  SetupCommonRenderState(context);
  
  // Render each type of shape individually
  RenderPoints(data, context);
  RenderLines(data, context);
  RenderRectangles(data, context);
  RenderCircles(data, context);
  RenderEllipses(data, context);
  RenderPolygons(data, context);
  
  CleanupRenderState();
}

void IndividualRenderStrategy::SetupCommonRenderState(const RenderContext& context) {
  // Setup common rendering state for primitives
  context.primitive_shader->Use();
  context.primitive_shader->TrySetUniform("projection", context.projection);
  context.primitive_shader->TrySetUniform("view", context.view);
  context.primitive_shader->TrySetUniform("model", glm::mat4(1.0f));
  context.primitive_shader->TrySetUniform("coordSystemTransform", context.coord_transform);
  context.primitive_shader->TrySetUniform("lineType", 0); // Default to solid line
  context.primitive_shader->TrySetUniform("thickness", 1.0f); // Default thickness
  context.primitive_shader->TrySetUniform("uColor", glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)); // Default to white

  // Enable depth test and blending
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void IndividualRenderStrategy::CleanupRenderState() {
  // Clean up OpenGL state
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glBindVertexArray(0);
  glUseProgram(0);
}

void IndividualRenderStrategy::RenderPoints(const CanvasData& data, const RenderContext& context) {
  // TODO: Move point rendering logic from original Canvas implementation
  // For now this is a placeholder
}

void IndividualRenderStrategy::RenderLines(const CanvasData& data, const RenderContext& context) {
  // TODO: Move line rendering logic from original Canvas implementation
  // For now this is a placeholder
}

void IndividualRenderStrategy::RenderRectangles(const CanvasData& data, const RenderContext& context) {
  // TODO: Move rectangle rendering logic from original Canvas implementation
  // For now this is a placeholder
}

void IndividualRenderStrategy::RenderCircles(const CanvasData& data, const RenderContext& context) {
  // TODO: Move circle rendering logic from original Canvas implementation
  // For now this is a placeholder
}

void IndividualRenderStrategy::RenderEllipses(const CanvasData& data, const RenderContext& context) {
  if (!shape_renderer_ || data.ellipses.empty()) {
    return;
  }
  
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
}

void IndividualRenderStrategy::RenderPolygons(const CanvasData& data, const RenderContext& context) {
  if (!shape_renderer_ || data.polygons.empty()) {
    return;
  }
  
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