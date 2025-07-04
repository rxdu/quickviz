/**
 * @file shape_renderer.cpp
 * @author Claude Code Assistant
 * @date 2025-01-04
 * @brief Implementation of unified shape renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "shape_renderer.hpp"
#include "renderer/shader_program.hpp"
#include <iostream>

namespace quickviz {

// TempGLResources implementation
TempGLResources::TempGLResources() {
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
}

TempGLResources::~TempGLResources() {
  Cleanup();
}

TempGLResources::TempGLResources(TempGLResources&& other) noexcept
  : vao_(other.vao_), vbo_(other.vbo_) {
  other.vao_ = 0;
  other.vbo_ = 0;
}

TempGLResources& TempGLResources::operator=(TempGLResources&& other) noexcept {
  if (this != &other) {
    Cleanup();
    vao_ = other.vao_;
    vbo_ = other.vbo_;
    other.vao_ = 0;
    other.vbo_ = 0;
  }
  return *this;
}

void TempGLResources::Bind() {
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
}

void TempGLResources::UploadVertices(const std::vector<float>& vertices) {
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), 
               vertices.data(), GL_STATIC_DRAW);
}

void TempGLResources::SetupVertexAttributes() {
  // Position attribute (location 0)
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);
  
  // Make sure other attributes are disabled
  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(2);
}

void TempGLResources::SetupDefaultAttributes(const glm::vec4& color) {
  // Set default values for disabled attributes
  glVertexAttrib4f(1, color.r, color.g, color.b, color.a);  // Color attribute
  glVertexAttrib1f(2, 1.0f);  // Size attribute
}

void TempGLResources::Cleanup() {
  if (vao_ != 0) {
    glDeleteVertexArrays(1, &vao_);
    vao_ = 0;
  }
  if (vbo_ != 0) {
    glDeleteBuffers(1, &vbo_);
    vbo_ = 0;
  }
}

// ShapeRenderer implementation
ShapeRenderer::ShapeRenderer(ShaderProgram* shader) : shader_(shader) {
  if (!shader_) {
    throw std::invalid_argument("ShapeRenderer requires valid shader program");
  }
}

void ShapeRenderer::RenderShape(const std::vector<float>& vertices,
                               const VertexLayout& layout,
                               const RenderParams& params) {
  if (vertices.empty()) return;
  
  // Create temporary OpenGL resources
  TempGLResources resources;
  resources.Bind();
  resources.UploadVertices(vertices);
  
  // Setup vertex attributes based on layout
  SetupVertexAttributes(layout);
  resources.SetupDefaultAttributes(params.color);
  
  // Set shader uniforms
  SetShaderUniforms(params);
  
  // Render the primitive
  RenderPrimitive(params);
  
  // Cleanup (handled automatically by TempGLResources destructor)
  glDisableVertexAttribArray(0);
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void ShapeRenderer::RenderShape(const std::function<std::vector<float>()>& vertex_generator,
                               const VertexLayout& layout,
                               const RenderParams& params) {
  auto vertices = vertex_generator();
  RenderShape(vertices, layout, params);
}

void ShapeRenderer::SetShaderUniforms(const RenderParams& params) {
  shader_->TrySetUniform("uColor", params.color);
  
  if (params.filled) {
    shader_->TrySetUniform("renderMode", 2); // Filled shapes mode
    shader_->TrySetUniform("lineType", 0); // Solid fill
  } else {
    shader_->TrySetUniform("renderMode", 3); // Outline shapes mode
    shader_->TrySetUniform("lineType", params.line_type);
    shader_->TrySetUniform("thickness", params.thickness);
  }
}

void ShapeRenderer::SetupVertexAttributes(const VertexLayout& layout) {
  if (layout.has_position) {
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
  }
  
  // For now, we only support position-only layouts
  // Color and size attributes are handled via uniforms and default attributes
  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(2);
}

void ShapeRenderer::RenderPrimitive(const RenderParams& params) {
  if (params.filled) {
    glDrawArrays(params.primitive_type, 0, params.vertex_count);
  } else {
    glLineWidth(params.thickness);
    glDrawArrays(params.primitive_type, 0, params.vertex_count);
    glLineWidth(1.0f); // Reset line width
  }
}

// RenderParamFactory implementation
namespace RenderParamFactory {

RenderParams FilledShape(const glm::vec4& color, float thickness) {
  RenderParams params;
  params.color = color;
  params.thickness = thickness;
  params.filled = true;
  params.primitive_type = GL_TRIANGLE_FAN;
  return params;
}

RenderParams OutlineShape(const glm::vec4& color, float thickness, int line_type) {
  RenderParams params;
  params.color = color;
  params.thickness = thickness;
  params.filled = false;
  params.line_type = line_type;
  params.primitive_type = GL_LINE_LOOP;
  return params;
}

RenderParams Points(const glm::vec4& color, float size) {
  RenderParams params;
  params.color = color;
  params.thickness = size;
  params.filled = true;
  params.primitive_type = GL_POINTS;
  return params;
}

RenderParams Lines(const glm::vec4& color, float thickness, int line_type) {
  RenderParams params;
  params.color = color;
  params.thickness = thickness;
  params.filled = false;
  params.line_type = line_type;
  params.primitive_type = GL_LINES;
  return params;
}

} // namespace RenderParamFactory

} // namespace quickviz