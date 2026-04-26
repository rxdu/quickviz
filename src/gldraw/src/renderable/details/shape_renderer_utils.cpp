/**
 * @file shape_renderer_utils.cpp
 * @author Canvas Refactoring Phase 1.3
 * @date 2025-01-11
 * @brief Implementation of consolidated shape rendering utilities
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "shape_renderer_utils.hpp"
#include <cmath>
#include <algorithm>
#include "glad/glad.h"
#include "gldraw/shader_program.hpp"

namespace quickviz {
namespace internal {

//==============================================================================
// ShapeVertexGenerator Implementation
//==============================================================================

std::vector<float> ShapeVertexGenerator::GenerateCircle(const glm::vec3& center, float radius, 
                                                       int segments, bool filled) {
  std::vector<float> vertices;
  vertices.reserve((segments + 2) * 3);

  if (filled) {
    // Add center vertex for filled circles
    vertices.insert(vertices.end(), {center.x, center.y, center.z});
  }

  // Generate perimeter vertices
  for (int i = 0; i <= segments; i++) {
    float angle = 2.0f * M_PI * i / segments;
    float x = center.x + radius * std::cos(angle);
    float y = center.y + radius * std::sin(angle);
    float z = center.z;
    vertices.insert(vertices.end(), {x, y, z});
  }

  return vertices;
}

std::vector<float> ShapeVertexGenerator::GenerateEllipse(const glm::vec3& center, float rx, float ry,
                                                        float angle, float start_angle, float end_angle,
                                                        int segments, bool filled) {
  std::vector<float> vertices;
  vertices.reserve((segments + 2) * 3);

  if (filled) {
    // Add center vertex for filled ellipses
    vertices.insert(vertices.end(), {center.x, center.y, center.z});
  }

  // Generate ellipse vertices
  for (int i = 0; i <= segments; i++) {
    float t = start_angle + (end_angle - start_angle) * i / segments;
    
    // Generate point on unit ellipse
    float x_local = rx * std::cos(t);
    float y_local = ry * std::sin(t);
    
    // Apply rotation
    float x_rotated = x_local * std::cos(angle) - y_local * std::sin(angle);
    float y_rotated = x_local * std::sin(angle) + y_local * std::cos(angle);
    
    // Translate to final position
    float x = center.x + x_rotated;
    float y = center.y + y_rotated;
    float z = center.z;
    
    vertices.insert(vertices.end(), {x, y, z});
  }

  return vertices;
}

std::vector<float> ShapeVertexGenerator::GenerateRectangle(const glm::vec3& position, 
                                                          float width, float height) {
  return {
    position.x, position.y, position.z,                           // Bottom-left
    position.x + width, position.y, position.z,                  // Bottom-right
    position.x + width, position.y + height, position.z,         // Top-right
    position.x, position.y + height, position.z                  // Top-left
  };
}

std::vector<float> ShapeVertexGenerator::GenerateLine(const glm::vec3& start, const glm::vec3& end) {
  return {
    start.x, start.y, start.z,
    end.x, end.y, end.z
  };
}

std::vector<float> ShapeVertexGenerator::GeneratePolygon(const std::vector<glm::vec3>& vertices) {
  std::vector<float> result;
  result.reserve(vertices.size() * 3);
  
  for (const auto& vertex : vertices) {
    result.insert(result.end(), {vertex.x, vertex.y, vertex.z});
  }
  
  return result;
}

//==============================================================================
// EfficientShapeRenderer Implementation
//==============================================================================

EfficientShapeRenderer::EfficientShapeRenderer(std::shared_ptr<OpenGLResourcePool> resource_pool,
                                              void* shader_program)
    : resource_pool_(resource_pool), shader_program_(shader_program) {
}

void EfficientShapeRenderer::RenderShape(const std::vector<float>& vertices, 
                                        const ShapeRenderParams& params) {
  if (vertices.empty() || !resource_pool_) {
    return;
  }

  // Acquire resources from pool
  auto resources = resource_pool_->Acquire();

  // Configure vertex data
  ConfigureVertexAttributes(resources, vertices);

  // Set up shader uniforms
  SetupShaderUniforms(params);

  // Perform rendering
  unsigned int vertex_count = vertices.size() / 3;
  
  if (params.primitive_type == GL_LINES || params.primitive_type == GL_LINE_LOOP || 
      params.primitive_type == GL_LINE_STRIP) {
    glLineWidth(params.thickness);
    glDrawArrays(params.primitive_type, 0, vertex_count);
    glLineWidth(1.0f); // Reset to default
  } else {
    glDrawArrays(params.primitive_type, 0, vertex_count);
  }

  // Cleanup
  glDisableVertexAttribArray(0);
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // Return resources to pool
  resource_pool_->Release(resources);
}

void EfficientShapeRenderer::SetupRenderingState(const glm::mat4& projection, 
                                                 const glm::mat4& view,
                                                 const glm::mat4& coord_transform) {
  if (!shader_program_) return;

  auto* shader = static_cast<ShaderProgram*>(shader_program_);
  shader->Use();
  shader->TrySetUniform("projection", projection);
  shader->TrySetUniform("view", view);
  shader->TrySetUniform("model", glm::mat4(1.0f));
  shader->TrySetUniform("coordSystemTransform", coord_transform);

  // Enable common OpenGL state
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void EfficientShapeRenderer::CleanupRenderingState() {
  // Reset OpenGL state
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glBindVertexArray(0);
  glUseProgram(0);
}

void EfficientShapeRenderer::SetupShaderUniforms(const ShapeRenderParams& params) {
  if (!shader_program_) return;

  auto* shader = static_cast<ShaderProgram*>(shader_program_);

  // Set color uniform
  shader->TrySetUniform("uniformColor", params.color);

  // Set rendering mode based on primitive type
  if (params.primitive_type == GL_LINES || params.primitive_type == GL_LINE_LOOP || 
      params.primitive_type == GL_LINE_STRIP) {
    shader->TrySetUniform("renderMode", 1); // Lines mode
    shader->TrySetUniform("lineType", static_cast<int>(params.line_type));
  } else {
    shader->TrySetUniform("renderMode", 2); // Filled shapes mode
    shader->TrySetUniform("lineType", 0); // Solid for filled shapes
  }
}

void EfficientShapeRenderer::ConfigureVertexAttributes(const OpenGLResourcePool::TempResources& resources,
                                                      const std::vector<float>& vertices) {
  // Bind and configure vertex array object
  glBindVertexArray(resources.vao);
  glBindBuffer(GL_ARRAY_BUFFER, resources.vbo);
  
  // Upload vertex data
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), 
              vertices.data(), GL_DYNAMIC_DRAW);
  
  // Configure vertex attributes (position only for now)
  glEnableVertexAttribArray(0); // Position
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
}

} // namespace internal
} // namespace quickviz