/**
 * @file grid.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-05
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "renderer/renderable/grid.hpp"

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
uniform vec3 gridColor;

out vec4 vertexColor;

void main() {
    gl_Position = projection * view * model * coordSystemTransform * vec4(aPos, 1.0);
    vertexColor = vec4(gridColor, 0.5);
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

Grid::Grid(float size, float spacing, const glm::vec3& color)
    : size_(size), spacing_(spacing), color_(color) {
  AllocateGpuResources();
}

Grid::~Grid() { ReleaseGpuResources(); }

void Grid::SetSize(float size) {
  if (size <= 0.0f) {
    std::cerr << "Warning: Grid size must be positive. Using default value."
              << std::endl;
    return;
  }

  size_ = size;
  GenerateGrid();

  // Update the VBO with the new data
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3),
               vertices_.data(), GL_STATIC_DRAW);
  glBindVertexArray(0);
}

void Grid::SetSpacing(float spacing) {
  if (spacing <= 0.0f) {
    std::cerr << "Warning: Grid spacing must be positive. Using default value."
              << std::endl;
    return;
  }

  spacing_ = spacing;
  GenerateGrid();

  // Update the VBO with the new data
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3),
               vertices_.data(), GL_STATIC_DRAW);
  glBindVertexArray(0);
}

void Grid::SetColor(const glm::vec3& color) { color_ = color; }

void Grid::AllocateGpuResources() {
  // Compile and link shaders
  Shader vertex_shader(vertex_shader_source.c_str(), Shader::Type::kVertex);
  Shader fragment_shader(fragment_shader_source.c_str(),
                         Shader::Type::kFragment);
  shader_.AttachShader(vertex_shader);
  shader_.AttachShader(fragment_shader);

  if (!shader_.LinkProgram()) {
    std::cerr << "ERROR::GRID::SHADER_PROGRAM_LINKING_FAILED" << std::endl;
    throw std::runtime_error("Shader program linking failed");
  }

  // Generate the grid
  GenerateGrid();

  // Create and set up VAO and VBO
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);

  glBindVertexArray(vao_);

  // Set up VBO for vertices
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3),
               vertices_.data(), GL_STATIC_DRAW);

  // Set up vertex attributes
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(0);

  // Unbind
  glBindVertexArray(0);
}

void Grid::ReleaseGpuResources() {
  glDeleteVertexArrays(1, &vao_);
  glDeleteBuffers(1, &vbo_);
}

void Grid::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                  const glm::mat4& coord_transform) {
  shader_.Use();
  shader_.SetUniform("projection", projection);
  shader_.SetUniform("view", view);
  shader_.SetUniform("model", glm::mat4(1.0f));
  shader_.SetUniform("coordSystemTransform", coord_transform);
  shader_.SetUniform("gridColor", color_);

  glBindVertexArray(vao_);

  // Enable blending for transparency
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Draw the grid lines
  glDrawArrays(GL_LINES, 0, vertices_.size());

  // Disable blending
  glDisable(GL_BLEND);

  glBindVertexArray(0);
}

void Grid::GenerateGrid() {
  vertices_.clear();

  // Calculate the number of lines in each direction
  int num_lines = static_cast<int>(size_ / spacing_) * 2 + 1;
  float half_size = size_ / 2.0f;

  // Generate grid lines on the XY plane (XZ plane in OpenGL with Y=0, to be
  // transformed later in OpenGL shader)
  for (int i = 0; i < num_lines; ++i) {
    float pos = -half_size + i * spacing_;

    // Only draw lines within the grid area
    if (pos >= -half_size && pos <= half_size) {
      // Horizontal lines (along X axis, varying Z which is Y in standard
      // coordinates)
      vertices_.push_back(glm::vec3(-half_size, pos, 0.0f));  // Left point
      vertices_.push_back(glm::vec3(half_size, pos, 0.0f));   // Right point

      // Vertical lines (along Z axis which is Y in standard coordinates,
      // varying X)
      vertices_.push_back(glm::vec3(pos, -half_size, 0.0f));  // Bottom point
      vertices_.push_back(glm::vec3(pos, half_size, 0.0f));   // Top point
    }
  }

  // Add a border around the grid (slightly thicker lines)
  // Bottom border (min Y in standard coordinates, min Z in OpenGL)
  vertices_.push_back(glm::vec3(-half_size, -half_size, 0.0f));
  vertices_.push_back(glm::vec3(half_size, -half_size, 0.0f));

  // Top border (max Y in standard coordinates, max Z in OpenGL)
  vertices_.push_back(glm::vec3(-half_size, half_size, 0.0f));
  vertices_.push_back(glm::vec3(half_size, half_size, 0.0f));

  // Left border (min X)
  vertices_.push_back(glm::vec3(-half_size, -half_size, 0.0f));
  vertices_.push_back(glm::vec3(-half_size, half_size, 0.0f));

  // Right border (max X)
  vertices_.push_back(glm::vec3(half_size, -half_size, 0.0f));
  vertices_.push_back(glm::vec3(half_size, half_size, 0.0f));
}
}  // namespace quickviz