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
#include <algorithm>
#include "glad/glad.h"
#include "gldraw/shader_program.hpp"
#include "canvas_data.hpp"

namespace quickviz {

// Helper enums and structures for sequence-ordered rendering
enum class PrimitiveType {
  kPoint,
  kLine,
  kRectangle,
  kCircle,
  kEllipse,
  kPolygon
};

struct PrimitiveRef {
  PrimitiveType type;
  size_t index;
  uint32_t sequence_number;
};

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
  
  // Create a sorted list of all primitives by sequence number
  std::vector<PrimitiveRef> sorted_primitives;
  
  // Collect all primitives with their sequence numbers
  for (size_t i = 0; i < data.points.size(); ++i) {
    sorted_primitives.push_back({PrimitiveType::kPoint, i, data.points[i].sequence_number});
  }
  for (size_t i = 0; i < data.lines.size(); ++i) {
    sorted_primitives.push_back({PrimitiveType::kLine, i, data.lines[i].sequence_number});
  }
  for (size_t i = 0; i < data.rectangles.size(); ++i) {
    sorted_primitives.push_back({PrimitiveType::kRectangle, i, data.rectangles[i].sequence_number});
  }
  for (size_t i = 0; i < data.circles.size(); ++i) {
    sorted_primitives.push_back({PrimitiveType::kCircle, i, data.circles[i].sequence_number});
  }
  for (size_t i = 0; i < data.ellipses.size(); ++i) {
    sorted_primitives.push_back({PrimitiveType::kEllipse, i, data.ellipses[i].sequence_number});
  }
  for (size_t i = 0; i < data.polygons.size(); ++i) {
    sorted_primitives.push_back({PrimitiveType::kPolygon, i, data.polygons[i].sequence_number});
  }
  
  // Sequence numbers are correctly assigned
  
  // Sort by sequence number to maintain draw order
  std::sort(sorted_primitives.begin(), sorted_primitives.end(),
            [](const PrimitiveRef& a, const PrimitiveRef& b) {
              return a.sequence_number < b.sequence_number;
            });
  
  // Render primitives in sequence order
  for (const auto& primitive_ref : sorted_primitives) {
    switch (primitive_ref.type) {
      case PrimitiveType::kPoint:
        RenderSinglePoint(data.points[primitive_ref.index], context);
        break;
      case PrimitiveType::kLine:
        RenderSingleLine(data.lines[primitive_ref.index], context);
        break;
      case PrimitiveType::kRectangle:
        RenderSingleRectangle(data.rectangles[primitive_ref.index], context);
        break;
      case PrimitiveType::kCircle:
        RenderSingleCircle(data.circles[primitive_ref.index], context);
        break;
      case PrimitiveType::kEllipse:
        RenderSingleEllipse(data.ellipses[primitive_ref.index], context);
        break;
      case PrimitiveType::kPolygon:
        RenderSinglePolygon(data.polygons[primitive_ref.index], context);
        break;
    }
  }
  
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
  if (data.points.empty()) return;
  
  context.primitive_shader->TrySetUniform("renderMode", 0); // Point rendering mode
  glBindVertexArray(context.primitive_vao);
  glBindBuffer(GL_ARRAY_BUFFER, context.primitive_vbo);
  
  for (const auto& point : data.points) {
    Point vertices[] = {point};
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
    glPointSize(point.size);
    glDrawArrays(GL_POINTS, 0, 1);
  }
  glPointSize(1.0f); // Reset
}

void IndividualRenderStrategy::RenderLines(const CanvasData& data, const RenderContext& context) {
  if (data.lines.empty()) return;
  
  context.primitive_shader->TrySetUniform("renderMode", 1); // Line rendering mode
  glBindVertexArray(context.primitive_vao);
  glBindBuffer(GL_ARRAY_BUFFER, context.primitive_vbo);
  
  for (const auto& line : data.lines) {
    context.primitive_shader->TrySetUniform("lineType", static_cast<int>(line.line_type));
    Point vertices[] = {
      {line.start, line.color, line.thickness},
      {line.end, line.color, line.thickness}
    };
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
    glLineWidth(line.thickness);
    glDrawArrays(GL_LINES, 0, 2);
  }
  glLineWidth(1.0f); // Reset
}

void IndividualRenderStrategy::RenderRectangles(const CanvasData& data, const RenderContext& context) {
  if (data.rectangles.empty()) return;
  
  glBindVertexArray(context.primitive_vao);
  glBindBuffer(GL_ARRAY_BUFFER, context.primitive_vbo);
  
  for (const auto& rect : data.rectangles) {
    // Generate rectangle vertices
    glm::vec3 corners[4] = {
      {rect.position.x, rect.position.y, 0.0f}, // bottom-left
      {rect.position.x + rect.width, rect.position.y, 0.0f}, // bottom-right
      {rect.position.x + rect.width, rect.position.y + rect.height, 0.0f}, // top-right
      {rect.position.x, rect.position.y + rect.height, 0.0f} // top-left
    };
    
    Point vertices[4];
    for (int i = 0; i < 4; i++) {
      vertices[i] = {corners[i], rect.color, rect.thickness};
    }
    
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
    
    if (rect.filled) {
      context.primitive_shader->TrySetUniform("renderMode", 2); // Filled shapes
      glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    } else {
      context.primitive_shader->TrySetUniform("renderMode", 3); // Outlined shapes
      context.primitive_shader->TrySetUniform("lineType", static_cast<int>(rect.line_type));
      glLineWidth(rect.thickness);
      glDrawArrays(GL_LINE_LOOP, 0, 4);
      glLineWidth(1.0f);
    }
  }
}

void IndividualRenderStrategy::RenderCircles(const CanvasData& data, const RenderContext& context) {
  if (data.circles.empty()) return;
  
  glBindVertexArray(context.primitive_vao);
  glBindBuffer(GL_ARRAY_BUFFER, context.primitive_vbo);
  
  for (const auto& circle : data.circles) {
    const int segments = circle.num_segments;
    std::vector<Point> vertices;
    
    if (circle.filled) {
      // Add center point for triangle fan
      vertices.push_back({{circle.center.x, circle.center.y, 0.0f}, circle.color, circle.thickness});
    }
    
    // Generate circle points
    for (int i = 0; i <= segments; i++) {
      float angle = 2.0f * M_PI * i / segments;
      float x = circle.center.x + circle.radius * cos(angle);
      float y = circle.center.y + circle.radius * sin(angle);
      vertices.push_back({{x, y, 0.0f}, circle.color, circle.thickness});
    }
    
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Point), vertices.data(), GL_DYNAMIC_DRAW);
    
    if (circle.filled) {
      context.primitive_shader->TrySetUniform("renderMode", 2); // Filled shapes
      glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.size());
    } else {
      context.primitive_shader->TrySetUniform("renderMode", 3); // Outlined shapes
      context.primitive_shader->TrySetUniform("lineType", static_cast<int>(circle.line_type));
      glLineWidth(circle.thickness);
      glDrawArrays(GL_LINE_LOOP, 1, segments); // Skip center point for outline
      glLineWidth(1.0f);
    }
  }
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

// Single primitive rendering methods for sequence-ordered rendering
void IndividualRenderStrategy::RenderSinglePoint(const Point& point, const RenderContext& context) {
  context.primitive_shader->TrySetUniform("renderMode", 0); // Point rendering mode
  glBindVertexArray(context.primitive_vao);
  glBindBuffer(GL_ARRAY_BUFFER, context.primitive_vbo);
  
  Point vertices[] = {point};
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
  glPointSize(point.size);
  glDrawArrays(GL_POINTS, 0, 1);
  glPointSize(1.0f); // Reset
}

void IndividualRenderStrategy::RenderSingleLine(const Line& line, const RenderContext& context) {
  context.primitive_shader->TrySetUniform("renderMode", 1); // Line rendering mode
  context.primitive_shader->TrySetUniform("lineType", static_cast<int>(line.line_type));
  glBindVertexArray(context.primitive_vao);
  glBindBuffer(GL_ARRAY_BUFFER, context.primitive_vbo);
  
  Point vertices[] = {
    {line.start, line.color, line.thickness},
    {line.end, line.color, line.thickness}
  };
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
  glLineWidth(line.thickness);
  glDrawArrays(GL_LINES, 0, 2);
  glLineWidth(1.0f); // Reset
}

void IndividualRenderStrategy::RenderSingleRectangle(const Rectangle& rect, const RenderContext& context) {
  glBindVertexArray(context.primitive_vao);
  glBindBuffer(GL_ARRAY_BUFFER, context.primitive_vbo);
  
  // Generate rectangle vertices
  glm::vec3 corners[4] = {
    {rect.position.x, rect.position.y, 0.0f}, // bottom-left
    {rect.position.x + rect.width, rect.position.y, 0.0f}, // bottom-right
    {rect.position.x + rect.width, rect.position.y + rect.height, 0.0f}, // top-right
    {rect.position.x, rect.position.y + rect.height, 0.0f} // top-left
  };
  
  Point vertices[4];
  for (int i = 0; i < 4; i++) {
    vertices[i] = {corners[i], rect.color, rect.thickness};
  }
  
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
  
  if (rect.filled) {
    context.primitive_shader->TrySetUniform("renderMode", 2); // Filled shapes
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
  } else {
    context.primitive_shader->TrySetUniform("renderMode", 3); // Outlined shapes
    context.primitive_shader->TrySetUniform("lineType", static_cast<int>(rect.line_type));
    glLineWidth(rect.thickness);
    glDrawArrays(GL_LINE_LOOP, 0, 4);
    glLineWidth(1.0f);
  }
}

void IndividualRenderStrategy::RenderSingleCircle(const Circle& circle, const RenderContext& context) {
  glBindVertexArray(context.primitive_vao);
  glBindBuffer(GL_ARRAY_BUFFER, context.primitive_vbo);
  
  const int segments = circle.num_segments;
  std::vector<Point> vertices;
  
  // Use sequence number to determine Z depth for proper layering
  // Higher sequence numbers should be closer to camera (higher Z)
  float z_depth = circle.sequence_number * 0.001f;
  
  if (circle.filled) {
    // Add center point for triangle fan
    vertices.push_back({{circle.center.x, circle.center.y, z_depth}, circle.color, circle.thickness});
  }
  
  // Generate circle points
  for (int i = 0; i <= segments; i++) {
    float angle = 2.0f * M_PI * i / segments;
    float x = circle.center.x + circle.radius * cos(angle);
    float y = circle.center.y + circle.radius * sin(angle);
    vertices.push_back({{x, y, z_depth}, circle.color, circle.thickness});
  }
  
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Point), vertices.data(), GL_DYNAMIC_DRAW);
  
  if (circle.filled) {
    context.primitive_shader->TrySetUniform("renderMode", 2); // Filled shapes
    glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.size());
  } else {
    context.primitive_shader->TrySetUniform("renderMode", 3); // Outlined shapes
    context.primitive_shader->TrySetUniform("lineType", static_cast<int>(circle.line_type));
    glLineWidth(circle.thickness);
    glDrawArrays(GL_LINE_LOOP, 1, segments); // Skip center point for outline
    glLineWidth(1.0f);
  }
}

void IndividualRenderStrategy::RenderSingleEllipse(const Ellipse& ellipse, const RenderContext& context) {
  if (!shape_renderer_) {
    return;
  }
  
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

void IndividualRenderStrategy::RenderSinglePolygon(const Polygon& polygon, const RenderContext& context) {
  if (!shape_renderer_) {
    return;
  }
  
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

} // namespace quickviz