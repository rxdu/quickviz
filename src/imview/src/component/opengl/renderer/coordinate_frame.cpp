/**
 * @file coordinate_frame.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/component/opengl/renderer/coordinate_frame.hpp"

#include <iostream>

#include "glad/glad.h"
#include "imgui.h"
#include <glm/gtc/type_ptr.hpp>

namespace quickviz {
namespace {
std::string vertex_shader_source = R"(
#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

out vec3 vertexColor;

void main() {
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    vertexColor = aColor;
}
)";

std::string fragment_shader_source = R"(
#version 330 core

in vec3 vertexColor;
out vec4 FragColor;

void main() {
    FragColor = vec4(vertexColor, 1.0);
}
)";
}  // namespace

CoordinateFrame::CoordinateFrame(float axis_length)
    : axis_length_(axis_length) {
  AllocateGpuResources();
}

CoordinateFrame::~CoordinateFrame() {
  ReleaseGpuResources();
}

void CoordinateFrame::SetAxisLength(float length) {
  if (length <= 0.0f) {
    std::cerr << "Warning: Axis length must be positive. Using default value." << std::endl;
    return;
  }
  
  axis_length_ = length;
  
  // Regenerate the axes with the new length
  vertices_.clear();
  colors_.clear();
  indices_.clear();
  GenerateAxes();
  
  // Update the VBO with the new data
  glBindVertexArray(vao_);
  
  // Update vertex positions
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, 
               vertices_.size() * sizeof(glm::vec3) + colors_.size() * sizeof(glm::vec3),
               nullptr, GL_STATIC_DRAW);
  glBufferSubData(GL_ARRAY_BUFFER, 0, 
                  vertices_.size() * sizeof(glm::vec3), vertices_.data());
  glBufferSubData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3),
                  colors_.size() * sizeof(glm::vec3), colors_.data());
  
  // Update indices
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(unsigned int),
               indices_.data(), GL_STATIC_DRAW);
  
  glBindVertexArray(0);
}

void CoordinateFrame::SetShowLabels(bool show) {
  // Method kept for API compatibility but does nothing
}

void CoordinateFrame::AllocateGpuResources() {
  // Compile and link shaders
  Shader vertex_shader(vertex_shader_source.c_str(), Shader::Type::kVertex);
  Shader fragment_shader(fragment_shader_source.c_str(), Shader::Type::kFragment);
  shader_.AttachShader(vertex_shader);
  shader_.AttachShader(fragment_shader);
  
  if (!shader_.LinkProgram()) {
    std::cerr << "ERROR::COORDINATE_FRAME::SHADER_PROGRAM_LINKING_FAILED" << std::endl;
    throw std::runtime_error("Shader program linking failed");
  }
  
  // Generate the coordinate axes
  GenerateAxes();
  
  // Create and set up VAO, VBO, and EBO
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &ebo_);
  
  glBindVertexArray(vao_);
  
  // Set up VBO for vertices and colors
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  
  // Allocate buffer for both vertices and colors
  glBufferData(GL_ARRAY_BUFFER, 
               vertices_.size() * sizeof(glm::vec3) + colors_.size() * sizeof(glm::vec3),
               nullptr, GL_STATIC_DRAW);
  
  // Upload vertices and colors to the buffer
  glBufferSubData(GL_ARRAY_BUFFER, 0, 
                  vertices_.size() * sizeof(glm::vec3), vertices_.data());
  glBufferSubData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3),
                  colors_.size() * sizeof(glm::vec3), colors_.data());
  
  // Set up EBO for indices
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(unsigned int),
               indices_.data(), GL_STATIC_DRAW);
  
  // Set up vertex attributes
  // Position attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(0);
  
  // Color attribute
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 
                        (void*)(vertices_.size() * sizeof(glm::vec3)));
  glEnableVertexAttribArray(1);
  
  // Unbind
  glBindVertexArray(0);
}

void CoordinateFrame::ReleaseGpuResources() {
  glDeleteVertexArrays(1, &vao_);
  glDeleteBuffers(1, &vbo_);
  glDeleteBuffers(1, &ebo_);
}

void CoordinateFrame::OnDraw(const glm::mat4& projection, const glm::mat4& view) {
  shader_.Use();
  shader_.SetUniform("projection", projection);
  shader_.SetUniform("view", view);
  shader_.SetUniform("model", glm::mat4(1.0f));  // Identity matrix
  
  glBindVertexArray(vao_);
  
  // Draw the axis lines (first 6 indices are for the 3 lines)
  glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT, 0);
  
  // Draw the arrow heads as triangles (remaining indices are for the 3 triangles)
  glDrawElements(GL_TRIANGLES, indices_.size() - 6, GL_UNSIGNED_INT, (void*)(6 * sizeof(unsigned int)));
  
  glBindVertexArray(0);
}

void CoordinateFrame::GenerateAxes() {
  // Create vertices for the coordinate axes with arrows
  const float arrow_size = axis_length_ * 0.1f; // Size of arrow head
  
  // X-axis line
  vertices_.push_back(glm::vec3(0.0f, 0.0f, 0.0f));                  // 0: Origin
  vertices_.push_back(glm::vec3(axis_length_, 0.0f, 0.0f));          // 1: X-axis endpoint
  
  // X-axis arrow head (triangle) - rotated 90 degrees around X axis
  vertices_.push_back(glm::vec3(axis_length_, 0.0f, 0.0f));          // 2: Arrow base
  vertices_.push_back(glm::vec3(axis_length_ - arrow_size, 0.0f, arrow_size/2.0f));  // 3: Arrow top
  vertices_.push_back(glm::vec3(axis_length_ - arrow_size, 0.0f, -arrow_size/2.0f)); // 4: Arrow bottom
  
  // Y-axis line
  vertices_.push_back(glm::vec3(0.0f, 0.0f, 0.0f));                  // 5: Origin
  vertices_.push_back(glm::vec3(0.0f, axis_length_, 0.0f));          // 6: Y-axis endpoint
  
  // Y-axis arrow head (triangle) - rotated 90 degrees around Y axis
  vertices_.push_back(glm::vec3(0.0f, axis_length_, 0.0f));          // 7: Arrow base
  vertices_.push_back(glm::vec3(0.0f, axis_length_ - arrow_size, arrow_size/2.0f));  // 8: Arrow right
  vertices_.push_back(glm::vec3(0.0f, axis_length_ - arrow_size, -arrow_size/2.0f)); // 9: Arrow left
  
  // Z-axis line
  vertices_.push_back(glm::vec3(0.0f, 0.0f, 0.0f));                  // 10: Origin
  vertices_.push_back(glm::vec3(0.0f, 0.0f, axis_length_));          // 11: Z-axis endpoint
  
  // Z-axis arrow head (triangle) - rotated 90 degrees around Z axis
  vertices_.push_back(glm::vec3(0.0f, 0.0f, axis_length_));          // 12: Arrow base
  vertices_.push_back(glm::vec3(arrow_size/2.0f, 0.0f, axis_length_ - arrow_size));  // 13: Arrow top
  vertices_.push_back(glm::vec3(-arrow_size/2.0f, 0.0f, axis_length_ - arrow_size)); // 14: Arrow bottom
  
  // Colors for each vertex
  // X-axis colors (red)
  colors_.push_back(glm::vec3(1.0f, 0.0f, 0.0f)); // Origin
  colors_.push_back(glm::vec3(1.0f, 0.0f, 0.0f)); // X-axis endpoint
  colors_.push_back(glm::vec3(1.0f, 0.0f, 0.0f)); // Arrow base
  colors_.push_back(glm::vec3(1.0f, 0.0f, 0.0f)); // Arrow top
  colors_.push_back(glm::vec3(1.0f, 0.0f, 0.0f)); // Arrow bottom
  
  // Y-axis colors (green)
  colors_.push_back(glm::vec3(0.0f, 1.0f, 0.0f)); // Origin
  colors_.push_back(glm::vec3(0.0f, 1.0f, 0.0f)); // Y-axis endpoint
  colors_.push_back(glm::vec3(0.0f, 1.0f, 0.0f)); // Arrow base
  colors_.push_back(glm::vec3(0.0f, 1.0f, 0.0f)); // Arrow right
  colors_.push_back(glm::vec3(0.0f, 1.0f, 0.0f)); // Arrow left
  
  // Z-axis colors (blue)
  colors_.push_back(glm::vec3(0.0f, 0.0f, 1.0f)); // Origin
  colors_.push_back(glm::vec3(0.0f, 0.0f, 1.0f)); // Z-axis endpoint
  colors_.push_back(glm::vec3(0.0f, 0.0f, 1.0f)); // Arrow base
  colors_.push_back(glm::vec3(0.0f, 0.0f, 1.0f)); // Arrow top
  colors_.push_back(glm::vec3(0.0f, 0.0f, 1.0f)); // Arrow bottom
  
  // Indices for drawing lines
  // X-axis line
  indices_.push_back(0);
  indices_.push_back(1);
  
  // Y-axis line
  indices_.push_back(5);
  indices_.push_back(6);
  
  // Z-axis line
  indices_.push_back(10);
  indices_.push_back(11);
  
  // Indices for drawing arrow heads as triangles
  // X-axis arrow head
  indices_.push_back(2);
  indices_.push_back(3);
  indices_.push_back(4);
  
  // Y-axis arrow head
  indices_.push_back(7);
  indices_.push_back(8);
  indices_.push_back(9);
  
  // Z-axis arrow head
  indices_.push_back(12);
  indices_.push_back(13);
  indices_.push_back(14);
}

}  // namespace quickviz