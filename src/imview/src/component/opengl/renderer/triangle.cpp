/**
 * @file triangle.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-05
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/component/opengl/renderer/triangle.hpp"

#include <iostream>

#include "glad/glad.h"
#include <glm/gtc/type_ptr.hpp>

namespace quickviz {
namespace {
std::string vertex_shader_source = R"(
#version 330 core

layout(location = 0) in vec3 aPos;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform mat4 coordSystemTransform;
uniform vec3 triangleColor;

out vec4 vertexColor;

void main() {
    gl_Position = projection * view * model * coordSystemTransform * vec4(aPos, 1.0);
    vertexColor = vec4(triangleColor, 0.5);
}
)";

std::string fragment_shader_source = R"(
#version 330 core

in vec4 vertexColor;
out vec4 FragColor;

void main() {
    FragColor = vertexColor;
}
)";
}  // namespace

///////////////////////////////////////////////////////////////////////////////

Triangle::Triangle(float size, const glm::vec3& color)
    : size_(size), color_(color) {
  AllocateGpuResources();
}

Triangle::~Triangle() {
  ReleaseGpuResources();
}

void Triangle::SetSize(float size) {
  if (size <= 0.0f) {
    std::cerr << "Warning: Triangle size must be positive. Using default value." << std::endl;
    return;
  }
  
  size_ = size;
  
  // Update the vertices with the new size
  float half_size = size_ / 2.0f;
  
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  
  // Triangle vertices in X-Y plane (X-Z plane in OpenGL with coordinate transform)
  glm::vec3 vertices[3] = {
    glm::vec3(0.0f, -half_size, 0.0f),         // Bottom center
    glm::vec3(-half_size, half_size, 0.0f),    // Top left
    glm::vec3(half_size, half_size, 0.0f)      // Top right
  };
  
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  glBindVertexArray(0);
}

void Triangle::SetColor(const glm::vec3& color) {
  color_ = color;
}

void Triangle::AllocateGpuResources() {
  // Compile and link shaders
  Shader vertex_shader(vertex_shader_source.c_str(), Shader::Type::kVertex);
  Shader fragment_shader(fragment_shader_source.c_str(), Shader::Type::kFragment);
  shader_.AttachShader(vertex_shader);
  shader_.AttachShader(fragment_shader);
  
  if (!shader_.LinkProgram()) {
    std::cerr << "ERROR::TRIANGLE::SHADER_PROGRAM_LINKING_FAILED" << std::endl;
    throw std::runtime_error("Shader program linking failed");
  }
  
  // Create and set up VAO and VBO
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  
  glBindVertexArray(vao_);
  
  // Set up VBO for vertices
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  
  // Triangle vertices in X-Y plane (X-Z plane in OpenGL with coordinate transform)
  float half_size = size_ / 2.0f;
  glm::vec3 vertices[3] = {
    glm::vec3(0.0f, -half_size, 0.0f),         // Bottom center
    glm::vec3(-half_size, half_size, 0.0f),    // Top left
    glm::vec3(half_size, half_size, 0.0f)      // Top right
  };
  
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  
  // Set up vertex attributes
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(0);
  
  // Unbind
  glBindVertexArray(0);
}

void Triangle::ReleaseGpuResources() {
  glDeleteVertexArrays(1, &vao_);
  glDeleteBuffers(1, &vbo_);
}

void Triangle::OnDraw(const glm::mat4& projection, const glm::mat4& view, 
                     const glm::mat4& coord_transform) {
  shader_.Use();
  shader_.SetUniform("projection", projection);
  shader_.SetUniform("view", view);
  shader_.SetUniform("model", glm::mat4(1.0f));
  shader_.SetUniform("coordSystemTransform", coord_transform);
  shader_.SetUniform("triangleColor", color_);
  
  glBindVertexArray(vao_);
  
  // Enable blending for transparency
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  // Draw the triangle
  glDrawArrays(GL_TRIANGLES, 0, 3);
  
  // Disable blending
  glDisable(GL_BLEND);
  
  glBindVertexArray(0);
}
}  // namespace quickviz