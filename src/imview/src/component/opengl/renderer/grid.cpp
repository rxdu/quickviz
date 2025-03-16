/*
 * @file grid.cpp
 * @date 11/2/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/component/opengl/renderer/grid.hpp"

#include <iostream>

#include "glad/glad.h"

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

Grid::Grid(float grid_size, float spacing, glm::vec3 color)
    : grid_size_(grid_size), spacing_(spacing), color_(color) {
  AllocateGpuResources();
}

Grid::~Grid() { ReleaseGpuResources(); }

void Grid::AllocateGpuResources() {
  Shader vertex_shader(vertex_shader_source.c_str(), Shader::Type::kVertex);
  Shader fragment_shader(fragment_shader_source.c_str(),
                         Shader::Type::kFragment);
  shader_.AttachShader(vertex_shader);
  shader_.AttachShader(fragment_shader);
  if (!shader_.LinkProgram()) {
    std::cout << "ERROR::GRID::SHADER_PROGRAM_LINKING_FAILED" << std::endl;
    throw std::runtime_error("Shader program linking failed");
  }
  GenerateGrid();
}

void Grid::ReleaseGpuResources() {
  glDeleteVertexArrays(1, &vao_);
  glDeleteBuffers(1, &vbo_);
}

void Grid::SetLineColor(const glm::vec3& color, float alpha) {
  color_ = color;
  alpha_ = alpha;
}

void Grid::OnDraw(const glm::mat4& projection, const glm::mat4& view) {
  shader_.Use();
  shader_.SetUniform("projection", projection);
  shader_.SetUniform("view", view);
  shader_.SetUniform("model", glm::mat4(1.0f));  // Identity matrix for the grid
  shader_.SetUniform("lineColor", color_);
  shader_.SetUniform("lineAlpha", alpha_);

  glBindVertexArray(vao_);
  glDrawArrays(GL_LINES, 0, vertices_.size());
  glBindVertexArray(0);
}

void Grid::GenerateGrid() {
  // Always generate grid in X-Z plane for consistency between 2D and 3D modes
  float half_grid_size = grid_size_ / 2.0f;
  
  // Generate grid in X-Z plane (y=0)
  for (float x = -half_grid_size; x <= half_grid_size; x += spacing_) {
    vertices_.emplace_back(x, 0.0f, -half_grid_size);
    vertices_.emplace_back(x, 0.0f, half_grid_size);
  }

  for (float z = -half_grid_size; z <= half_grid_size; z += spacing_) {
    vertices_.emplace_back(-half_grid_size, 0.0f, z);
    vertices_.emplace_back(half_grid_size, 0.0f, z);
  }

  // set up VAO and VBO
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);

  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3),
               vertices_.data(), GL_STATIC_DRAW);

  // set vertex attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(0);

  glBindVertexArray(0);  // Unbind VAO
}
}  // namespace quickviz