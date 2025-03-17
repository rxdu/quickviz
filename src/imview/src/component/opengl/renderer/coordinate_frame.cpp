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
#include <cmath>

#include "glad/glad.h"
#include "imgui.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace quickviz {
namespace {
std::string vertex_shader_source = R"(
#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform mat4 coordSystemTransform;

out vec3 vertexColor;

void main() {
    gl_Position = projection * view * model * coordSystemTransform * vec4(aPos, 1.0);
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

CoordinateFrame::CoordinateFrame(float axis_length, bool is_2d_mode)
    : axis_length_(axis_length), is_2d_mode_(is_2d_mode) {
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

void CoordinateFrame::Set2DMode(bool is_2d) {
  if (is_2d_mode_ != is_2d) {
    is_2d_mode_ = is_2d;
    
    // Regenerate the axes with the new mode
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
}

void CoordinateFrame::SetPosition(const glm::vec3& position) {
  position_ = position;
  UpdateModelMatrix();
}

void CoordinateFrame::SetOrientation(const glm::quat& orientation) {
  orientation_ = orientation;
  UpdateModelMatrix();
}

void CoordinateFrame::SetPose(const glm::vec3& position, const glm::quat& orientation) {
  position_ = position;
  orientation_ = orientation;
  UpdateModelMatrix();
}

void CoordinateFrame::UpdateModelMatrix() {
  // Create a rotation matrix from the quaternion
  glm::mat4 rotation_matrix = glm::mat4_cast(orientation_);
  
  // Create a translation matrix
  glm::mat4 translation_matrix = glm::translate(glm::mat4(1.0f), position_);
  
  // Combine rotation and translation
  model_matrix_ = translation_matrix * rotation_matrix;
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
  
  // Initialize model matrix
  UpdateModelMatrix();
  
  // Unbind
  glBindVertexArray(0);
}

void CoordinateFrame::ReleaseGpuResources() {
  glDeleteVertexArrays(1, &vao_);
  glDeleteBuffers(1, &vbo_);
  glDeleteBuffers(1, &ebo_);
}

void CoordinateFrame::OnDraw(const glm::mat4& projection, const glm::mat4& view, 
                            const glm::mat4& coord_transform) {
  shader_.Use();
  shader_.SetUniform("projection", projection);
  shader_.SetUniform("view", view);
  shader_.SetUniform("model", model_matrix_);
  shader_.SetUniform("coordSystemTransform", coord_transform);
  
  glBindVertexArray(vao_);
  
  // Calculate the number of line indices to draw
  int num_line_indices = is_2d_mode_ ? 4 : 6; // 2 lines (4 indices) for 2D, 3 lines (6 indices) for 3D
  
  // Draw the axis lines
  glDrawElements(GL_LINES, num_line_indices, GL_UNSIGNED_INT, 0);
  
  // Draw the cone-shaped arrowheads as triangles
  int num_triangle_indices = indices_.size() - num_line_indices;
  if (num_triangle_indices > 0) {
    glDrawElements(GL_TRIANGLES, num_triangle_indices, GL_UNSIGNED_INT, 
                  (void*)(num_line_indices * sizeof(unsigned int)));
  }
  
  glBindVertexArray(0);
}

void CoordinateFrame::GenerateAxes() {
  // Create vertices for the coordinate axes with cone-shaped arrows
  const float arrow_size = axis_length_ * 0.1f; // Size of arrow head
  const int num_segments = 12; // Number of segments for the cone base
  
  // X-axis line (red)
  vertices_.push_back(glm::vec3(0.0f, 0.0f, 0.0f));                  // 0: Origin
  vertices_.push_back(glm::vec3(axis_length_, 0.0f, 0.0f));          // 1: X-axis endpoint
  
  // X-axis colors (red)
  colors_.push_back(glm::vec3(1.0f, 0.0f, 0.0f)); // Origin
  colors_.push_back(glm::vec3(1.0f, 0.0f, 0.0f)); // X-axis endpoint
  
  // Y-axis line (green)
  vertices_.push_back(glm::vec3(0.0f, 0.0f, 0.0f));                  // 2: Origin
  vertices_.push_back(glm::vec3(0.0f, axis_length_, 0.0f));          // 3: Y-axis endpoint
  
  // Y-axis colors (green)
  colors_.push_back(glm::vec3(0.0f, 1.0f, 0.0f)); // Origin
  colors_.push_back(glm::vec3(0.0f, 1.0f, 0.0f)); // Y-axis endpoint
  
  // Indices for drawing lines
  // X-axis line
  indices_.push_back(0);
  indices_.push_back(1);
  
  // Y-axis line
  indices_.push_back(2);
  indices_.push_back(3);
  
  // Z-axis line (blue)
  vertices_.push_back(glm::vec3(0.0f, 0.0f, 0.0f));                  // 4: Origin
  vertices_.push_back(glm::vec3(0.0f, 0.0f, axis_length_));          // 5: Z-axis endpoint
  
  // Z-axis colors (blue)
  colors_.push_back(glm::vec3(0.0f, 0.0f, 1.0f)); // Origin
  colors_.push_back(glm::vec3(0.0f, 0.0f, 1.0f)); // Z-axis endpoint
  
  // Z-axis line
  indices_.push_back(4);
  indices_.push_back(5);
  
  // Create cone vertices for X-axis arrowhead
  int x_cone_base_start_idx = vertices_.size();
  glm::vec3 x_cone_tip(axis_length_, 0.0f, 0.0f);
  glm::vec3 x_cone_center(axis_length_ - arrow_size, 0.0f, 0.0f);
  
  // Add the cone tip
  vertices_.push_back(x_cone_tip);
  colors_.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
  
  // Add vertices for the cone base
  for (int i = 0; i < num_segments; ++i) {
    float angle = 2.0f * M_PI * i / num_segments;
    float y = arrow_size/2.0f * sin(angle);
    float z = arrow_size/2.0f * cos(angle);
    
    vertices_.push_back(glm::vec3(axis_length_ - arrow_size, y, z));
    colors_.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
  }
  
  // Add indices for the cone triangles
  for (int i = 0; i < num_segments; ++i) {
    // Cone tip to current and next base vertex
    indices_.push_back(x_cone_base_start_idx); // Tip
    indices_.push_back(x_cone_base_start_idx + 1 + i);
    indices_.push_back(x_cone_base_start_idx + 1 + (i + 1) % num_segments);
  }
  
  // Create cone vertices for Y-axis arrowhead
  int y_cone_base_start_idx = vertices_.size();
  glm::vec3 y_cone_tip(0.0f, axis_length_, 0.0f);
  glm::vec3 y_cone_center(0.0f, axis_length_ - arrow_size, 0.0f);
  
  // Add the cone tip
  vertices_.push_back(y_cone_tip);
  colors_.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
  
  // Add vertices for the cone base
  for (int i = 0; i < num_segments; ++i) {
    float angle = 2.0f * M_PI * i / num_segments;
    float x = arrow_size/2.0f * sin(angle);
    float z = arrow_size/2.0f * cos(angle);
    
    vertices_.push_back(glm::vec3(x, axis_length_ - arrow_size, z));
    colors_.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
  }
  
  // Add indices for the cone triangles
  for (int i = 0; i < num_segments; ++i) {
    // Cone tip to current and next base vertex
    indices_.push_back(y_cone_base_start_idx); // Tip
    indices_.push_back(y_cone_base_start_idx + 1 + i);
    indices_.push_back(y_cone_base_start_idx + 1 + (i + 1) % num_segments);
  }
  
  // Create cone vertices for Z-axis arrowhead
  int z_cone_base_start_idx = vertices_.size();
  glm::vec3 z_cone_tip(0.0f, 0.0f, axis_length_);
  glm::vec3 z_cone_center(0.0f, 0.0f, axis_length_ - arrow_size);
  
  // Add the cone tip
  vertices_.push_back(z_cone_tip);
  colors_.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
  
  // Add vertices for the cone base
  for (int i = 0; i < num_segments; ++i) {
    float angle = 2.0f * M_PI * i / num_segments;
    float x = arrow_size/2.0f * sin(angle);
    float y = arrow_size/2.0f * cos(angle);
    
    vertices_.push_back(glm::vec3(x, y, axis_length_ - arrow_size));
    colors_.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
  }
  
  // Add indices for the cone triangles
  for (int i = 0; i < num_segments; ++i) {
    // Cone tip to current and next base vertex
    indices_.push_back(z_cone_base_start_idx); // Tip
    indices_.push_back(z_cone_base_start_idx + 1 + i);
    indices_.push_back(z_cone_base_start_idx + 1 + (i + 1) % num_segments);
  }
}
}  // namespace quickviz