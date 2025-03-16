/*
 * @file triangle.cpp
 * @date 11/4/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/component/opengl/renderer/triangle.hpp"

#include "glad/glad.h"

#include <iostream>

namespace quickviz {
namespace {
std::string vertex_shader_source = R"(
#version 330 core

layout(location = 0) in vec3 aPos;
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

void main() {
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
)";

std::string fragment_shader_source = R"(
#version 330 core

out vec4 FragColor;
uniform vec3 lineColor;
uniform float lineAlpha;

void main() {
    FragColor = vec4(lineColor, lineAlpha);
}
)";
}  // namespace

///////////////////////////////////////////////////////////////////////////////

Triangle::Triangle(float size, glm::vec3 color) : size_(size), color_(color) {
  AllocateGpuResources();
}

Triangle::~Triangle() { ReleaseGpuResources(); }

void Triangle::AllocateGpuResources() {
  Shader vertex_shader(vertex_shader_source.c_str(), Shader::Type::kVertex);
  Shader fragment_shader(fragment_shader_source.c_str(),
                         Shader::Type::kFragment);
  shader_.AttachShader(vertex_shader);
  shader_.AttachShader(fragment_shader);
  if (!shader_.LinkProgram()) {
    std::cout << "ERROR::GRID::SHADER_PROGRAM_LINKING_FAILED" << std::endl;
    throw std::runtime_error("Shader program linking failed");
  }
  GenerateTriangle();
}

void Triangle::ReleaseGpuResources() {
  glDeleteVertexArrays(1, &vao_);
  glDeleteBuffers(1, &vbo_);
}

void Triangle::SetColor(const glm::vec3& color, float alpha) {
  color_ = color;
  alpha_ = alpha;
}

void Triangle::OnDraw(const glm::mat4& projection, const glm::mat4& view) {
  shader_.Use();
  shader_.SetUniform("projection", projection);
  shader_.SetUniform("view", view);
  shader_.SetUniform("model", glm::mat4(1.0f));  // Identity matrix for the grid
  shader_.SetUniform("lineColor", color_);
  shader_.SetUniform("lineAlpha", alpha_);

  glBindVertexArray(vao_);
  glDrawArrays(GL_TRIANGLES, 0, vertices_.size());
  glBindVertexArray(0);
}

void Triangle::GenerateTriangle() {
  // clang-format off
  vertices_ = {
    glm::vec3(-0.5f, 0.0f, 0.0f),
    glm::vec3(0.0f, 0.866f, 0.0f),
    glm::vec3(0.5f, 0.0f, 0.0f)
  };
  // clang-format on

  // create vertex array object
  glGenVertexArrays(1, &vao_);
  glBindVertexArray(vao_);

  // create vertex buffer object
  glGenBuffers(1, &vbo_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3),
               vertices_.data(), GL_STATIC_DRAW);

  // set vertex attribute pointers
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(0);

  // unbind vbo and vao
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}
}  // namespace quickviz