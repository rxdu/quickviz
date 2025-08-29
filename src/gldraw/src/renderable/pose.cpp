/**
 * @file pose.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Implementation of 6-DOF pose visualization
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/pose.hpp"

#include <iostream>
#include <chrono>
#include <algorithm>

#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "gldraw/shader.hpp"

namespace quickviz {

namespace {

const char* kFrameVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 vertexColor;

uniform mat4 mvp;
uniform mat4 model;

void main() {
    vertexColor = aColor;
    gl_Position = mvp * model * vec4(aPos, 1.0);
}
)";

const char* kFrameFragmentShader = R"(
#version 330 core
in vec3 vertexColor;
out vec4 FragColor;

uniform float alpha;

void main() {
    FragColor = vec4(vertexColor, alpha);
}
)";

const char* kTrailVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 vertexColor;

uniform mat4 mvp;
uniform mat4 coordTransform;

void main() {
    vertexColor = aColor;
    gl_Position = mvp * coordTransform * vec4(aPos, 1.0);
}
)";

const char* kTrailFragmentShader = R"(
#version 330 core
in vec3 vertexColor;
out vec4 FragColor;

uniform float alpha;

void main() {
    FragColor = vec4(vertexColor, alpha);
}
)";

float getCurrentTime() {
  auto now = std::chrono::steady_clock::now();
  auto duration = now.time_since_epoch();
  return std::chrono::duration<float>(duration).count();
}

} // namespace

Pose::Pose() 
    : position_(0.0f, 0.0f, 0.0f)
    , orientation_(1.0f, 0.0f, 0.0f, 0.0f) // Identity quaternion
    , model_matrix_(1.0f)
    , axis_length_(1.0f)
    , x_axis_color_(1.0f, 0.0f, 0.0f)  // Red for X
    , y_axis_color_(0.0f, 1.0f, 0.0f)  // Green for Y
    , z_axis_color_(0.0f, 0.0f, 1.0f)  // Blue for Z
    , axis_width_(2.0f)
    , show_frame_(true)
    , scale_(1.0f)
    , alpha_(1.0f)
    , trail_mode_(TrailMode::kNone)
    , max_trail_points_(100)
    , trail_color_(0.8f, 0.8f, 0.0f)   // Yellow
    , trail_width_(1.5f)
    , trail_fade_time_(10.0f)
    , frame_vao_(0), frame_vbo_(0), frame_ebo_(0)
    , trail_vao_(0), trail_vbo_(0), trail_ebo_(0)
    , needs_frame_update_(true)
    , needs_trail_update_(true) {
  UpdateModelMatrix();
}

Pose::Pose(const glm::vec3& position, const glm::quat& orientation)
    : Pose() {
  position_ = position;
  orientation_ = orientation;
  UpdateModelMatrix();
  AddTrailPoint(position_, orientation_);
}

Pose::~Pose() {
  if (IsGpuResourcesAllocated()) {
    ReleaseGpuResources();
  }
}

void Pose::SetPose(const glm::vec3& position, const glm::quat& orientation) {
  position_ = position;
  orientation_ = orientation;
  UpdateModelMatrix();
  if (trail_mode_ != TrailMode::kNone) {
    AddTrailPoint(position_, orientation_);
  }
}

void Pose::SetPosition(const glm::vec3& position) {
  position_ = position;
  UpdateModelMatrix();
  if (trail_mode_ != TrailMode::kNone) {
    AddTrailPoint(position_, orientation_);
  }
}

void Pose::SetOrientation(const glm::quat& orientation) {
  orientation_ = orientation;
  UpdateModelMatrix();
  if (trail_mode_ != TrailMode::kNone) {
    AddTrailPoint(position_, orientation_);
  }
}

void Pose::UpdatePose(const glm::vec3& position, const glm::quat& orientation) {
  SetPose(position, orientation);
}

void Pose::SetAxisLength(float length) {
  if (length > 0.0f) {
    axis_length_ = length;
    needs_frame_update_ = true;
  }
}

void Pose::SetAxisColors(const glm::vec3& x_color, const glm::vec3& y_color, const glm::vec3& z_color) {
  x_axis_color_ = x_color;
  y_axis_color_ = y_color;
  z_axis_color_ = z_color;
  needs_frame_update_ = true;
}

void Pose::SetAxisWidth(float width) {
  if (width > 0.0f) {
    axis_width_ = width;
  }
}

void Pose::SetShowFrame(bool show) {
  show_frame_ = show;
}

void Pose::SetTrailMode(TrailMode mode) {
  if (trail_mode_ != mode) {
    trail_mode_ = mode;
    if (mode == TrailMode::kNone) {
      ClearTrail();
    } else if (trail_positions_.empty() && mode != TrailMode::kNone) {
      AddTrailPoint(position_, orientation_);
    }
    needs_trail_update_ = true;
  }
}

void Pose::SetTrailLength(size_t max_points) {
  if (max_points > 0) {
    max_trail_points_ = max_points;
    while (trail_positions_.size() > max_trail_points_) {
      trail_positions_.pop_front();
    }
    needs_trail_update_ = true;
  }
}

void Pose::SetTrailColor(const glm::vec3& color) {
  trail_color_ = color;
  needs_trail_update_ = true;
}

void Pose::SetTrailWidth(float width) {
  if (width > 0.0f) {
    trail_width_ = width;
  }
}

void Pose::SetTrailFadeTime(float seconds) {
  if (seconds > 0.0f) {
    trail_fade_time_ = seconds;
    needs_trail_update_ = true;
  }
}

void Pose::ClearTrail() {
  trail_positions_.clear();
  needs_trail_update_ = true;
}

void Pose::SetScale(float scale) {
  if (scale > 0.0f) {
    scale_ = scale;
    needs_frame_update_ = true;
  }
}

void Pose::SetTransparency(float alpha) {
  alpha_ = glm::clamp(alpha, 0.0f, 1.0f);
}

void Pose::GenerateFrameGeometry() {
  frame_vertices_.clear();
  frame_colors_.clear();
  frame_indices_.clear();

  float length = axis_length_ * scale_;

  // X axis (red)
  frame_vertices_.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
  frame_vertices_.push_back(glm::vec3(length, 0.0f, 0.0f));
  frame_colors_.push_back(x_axis_color_);
  frame_colors_.push_back(x_axis_color_);

  // Y axis (green)
  frame_vertices_.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
  frame_vertices_.push_back(glm::vec3(0.0f, length, 0.0f));
  frame_colors_.push_back(y_axis_color_);
  frame_colors_.push_back(y_axis_color_);

  // Z axis (blue)
  frame_vertices_.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
  frame_vertices_.push_back(glm::vec3(0.0f, 0.0f, length));
  frame_colors_.push_back(z_axis_color_);
  frame_colors_.push_back(z_axis_color_);

  // Line indices
  frame_indices_ = {0, 1, 2, 3, 4, 5};
}

void Pose::GenerateTrailGeometry() {
  trail_vertices_.clear();
  trail_vertex_colors_.clear();
  trail_indices_.clear();

  if (trail_positions_.empty() || trail_mode_ == TrailMode::kNone) {
    return;
  }

  float current_time = getCurrentTime();

  switch (trail_mode_) {
    case TrailMode::kLine: {
      for (size_t i = 0; i < trail_positions_.size(); ++i) {
        trail_vertices_.push_back(trail_positions_[i].position);
        trail_vertex_colors_.push_back(trail_color_);
        
        if (i > 0) {
          trail_indices_.push_back(static_cast<uint32_t>(i - 1));
          trail_indices_.push_back(static_cast<uint32_t>(i));
        }
      }
      break;
    }
    
    case TrailMode::kDots: {
      for (size_t i = 0; i < trail_positions_.size(); ++i) {
        trail_vertices_.push_back(trail_positions_[i].position);
        trail_vertex_colors_.push_back(trail_color_);
      }
      // No indices needed for point rendering
      break;
    }
    
    case TrailMode::kFading: {
      for (size_t i = 0; i < trail_positions_.size(); ++i) {
        trail_vertices_.push_back(trail_positions_[i].position);
        
        // Calculate fade alpha based on age
        float age = current_time - trail_positions_[i].timestamp;
        float fade_alpha = std::max(0.0f, 1.0f - (age / trail_fade_time_));
        
        glm::vec3 faded_color = trail_color_ * fade_alpha;
        trail_vertex_colors_.push_back(faded_color);
        
        if (i > 0) {
          trail_indices_.push_back(static_cast<uint32_t>(i - 1));
          trail_indices_.push_back(static_cast<uint32_t>(i));
        }
      }
      break;
    }
    
    case TrailMode::kArrows: {
      // For now, render as dots - full arrow implementation would be more complex
      for (size_t i = 0; i < trail_positions_.size(); ++i) {
        trail_vertices_.push_back(trail_positions_[i].position);
        trail_vertex_colors_.push_back(trail_color_);
      }
      break;
    }
    
    default:
      break;
  }
}

void Pose::UpdateModelMatrix() {
  glm::mat4 translation = glm::translate(glm::mat4(1.0f), position_);
  glm::mat4 rotation = glm::mat4_cast(orientation_);
  model_matrix_ = translation * rotation;
}

void Pose::AddTrailPoint(const glm::vec3& position, const glm::quat& orientation) {
  if (trail_mode_ == TrailMode::kNone) return;

  TrailPoint point;
  point.position = position;
  point.orientation = orientation;
  point.timestamp = getCurrentTime();

  trail_positions_.push_back(point);

  // Remove old points if we exceed max length
  while (trail_positions_.size() > max_trail_points_) {
    trail_positions_.pop_front();
  }

  // Remove old points for fading mode
  if (trail_mode_ == TrailMode::kFading) {
    float current_time = getCurrentTime();
    while (!trail_positions_.empty() && 
           (current_time - trail_positions_.front().timestamp) > trail_fade_time_) {
      trail_positions_.pop_front();
    }
  }

  needs_trail_update_ = true;
}

void Pose::UpdateFrameBuffers() {
  if (!needs_frame_update_ || !IsGpuResourcesAllocated()) return;

  GenerateFrameGeometry();

  // Update frame VAO
  glBindVertexArray(frame_vao_);
  
  glBindBuffer(GL_ARRAY_BUFFER, frame_vbo_);
  glBufferData(GL_ARRAY_BUFFER, 
               (frame_vertices_.size() * sizeof(glm::vec3)) + (frame_colors_.size() * sizeof(glm::vec3)),
               nullptr, GL_DYNAMIC_DRAW);
  
  // Upload vertices
  glBufferSubData(GL_ARRAY_BUFFER, 0, frame_vertices_.size() * sizeof(glm::vec3), frame_vertices_.data());
  // Upload colors
  glBufferSubData(GL_ARRAY_BUFFER, frame_vertices_.size() * sizeof(glm::vec3), 
                  frame_colors_.size() * sizeof(glm::vec3), frame_colors_.data());

  // Position attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(0);
  
  // Color attribute
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 
                        (void*)(frame_vertices_.size() * sizeof(glm::vec3)));
  glEnableVertexAttribArray(1);

  // Update element buffer
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, frame_ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, frame_indices_.size() * sizeof(uint32_t), 
               frame_indices_.data(), GL_DYNAMIC_DRAW);

  glBindVertexArray(0);
  needs_frame_update_ = false;
}

void Pose::UpdateTrailBuffers() {
  if (!needs_trail_update_ || !IsGpuResourcesAllocated() || trail_mode_ == TrailMode::kNone) return;

  GenerateTrailGeometry();

  if (trail_vertices_.empty()) return;

  // Update trail VAO
  glBindVertexArray(trail_vao_);
  
  glBindBuffer(GL_ARRAY_BUFFER, trail_vbo_);
  glBufferData(GL_ARRAY_BUFFER, 
               (trail_vertices_.size() * sizeof(glm::vec3)) + (trail_vertex_colors_.size() * sizeof(glm::vec3)),
               nullptr, GL_DYNAMIC_DRAW);
  
  // Upload vertices
  glBufferSubData(GL_ARRAY_BUFFER, 0, trail_vertices_.size() * sizeof(glm::vec3), trail_vertices_.data());
  // Upload colors
  glBufferSubData(GL_ARRAY_BUFFER, trail_vertices_.size() * sizeof(glm::vec3), 
                  trail_vertex_colors_.size() * sizeof(glm::vec3), trail_vertex_colors_.data());

  // Position attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(0);
  
  // Color attribute
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 
                        (void*)(trail_vertices_.size() * sizeof(glm::vec3)));
  glEnableVertexAttribArray(1);

  // Update element buffer for line modes
  if (!trail_indices_.empty()) {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, trail_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, trail_indices_.size() * sizeof(uint32_t), 
                 trail_indices_.data(), GL_DYNAMIC_DRAW);
  }

  glBindVertexArray(0);
  needs_trail_update_ = false;
}

void Pose::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;

  try {
    // Compile frame shader
    Shader frame_vs(kFrameVertexShader, Shader::Type::kVertex);
    Shader frame_fs(kFrameFragmentShader, Shader::Type::kFragment);
    if (!frame_vs.Compile() || !frame_fs.Compile()) {
      throw std::runtime_error("Frame shader compilation failed");
    }
    frame_shader_.AttachShader(frame_vs);
    frame_shader_.AttachShader(frame_fs);
    if (!frame_shader_.LinkProgram()) {
      throw std::runtime_error("Frame shader linking failed");
    }

    // Compile trail shader
    Shader trail_vs(kTrailVertexShader, Shader::Type::kVertex);
    Shader trail_fs(kTrailFragmentShader, Shader::Type::kFragment);
    if (!trail_vs.Compile() || !trail_fs.Compile()) {
      throw std::runtime_error("Trail shader compilation failed");
    }
    trail_shader_.AttachShader(trail_vs);
    trail_shader_.AttachShader(trail_fs);
    if (!trail_shader_.LinkProgram()) {
      throw std::runtime_error("Trail shader linking failed");
    }

    // Create OpenGL objects
    glGenVertexArrays(1, &frame_vao_);
    glGenBuffers(1, &frame_vbo_);
    glGenBuffers(1, &frame_ebo_);
    
    glGenVertexArrays(1, &trail_vao_);
    glGenBuffers(1, &trail_vbo_);
    glGenBuffers(1, &trail_ebo_);

    UpdateFrameBuffers();
    UpdateTrailBuffers();

  } catch (const std::exception& e) {
    std::cerr << "Pose::AllocateGpuResources: " << e.what() << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void Pose::ReleaseGpuResources() noexcept {
  if (frame_vao_ != 0) {
    glDeleteVertexArrays(1, &frame_vao_);
    frame_vao_ = 0;
  }
  if (frame_vbo_ != 0) {
    glDeleteBuffers(1, &frame_vbo_);
    frame_vbo_ = 0;
  }
  if (frame_ebo_ != 0) {
    glDeleteBuffers(1, &frame_ebo_);
    frame_ebo_ = 0;
  }
  if (trail_vao_ != 0) {
    glDeleteVertexArrays(1, &trail_vao_);
    trail_vao_ = 0;
  }
  if (trail_vbo_ != 0) {
    glDeleteBuffers(1, &trail_vbo_);
    trail_vbo_ = 0;
  }
  if (trail_ebo_ != 0) {
    glDeleteBuffers(1, &trail_ebo_);
    trail_ebo_ = 0;
  }
}

void Pose::OnDraw(const glm::mat4& projection, const glm::mat4& view, const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }

  UpdateFrameBuffers();
  UpdateTrailBuffers();

  glm::mat4 mvp = projection * view * coord_transform;

  // Draw coordinate frame
  if (show_frame_) {
    frame_shader_.Use();
    frame_shader_.SetUniform("mvp", mvp);
    frame_shader_.SetUniform("model", model_matrix_);
    frame_shader_.SetUniform("alpha", alpha_);

    glLineWidth(axis_width_);
    glBindVertexArray(frame_vao_);
    glDrawElements(GL_LINES, static_cast<GLsizei>(frame_indices_.size()), GL_UNSIGNED_INT, nullptr);
    glLineWidth(1.0f);
  }

  // Draw trail
  if (trail_mode_ != TrailMode::kNone && !trail_vertices_.empty()) {
    trail_shader_.Use();
    trail_shader_.SetUniform("mvp", mvp);
    trail_shader_.SetUniform("coordTransform", coord_transform);
    trail_shader_.SetUniform("alpha", alpha_);

    glBindVertexArray(trail_vao_);

    switch (trail_mode_) {
      case TrailMode::kLine:
      case TrailMode::kFading:
        glLineWidth(trail_width_);
        glDrawElements(GL_LINES, static_cast<GLsizei>(trail_indices_.size()), GL_UNSIGNED_INT, nullptr);
        glLineWidth(1.0f);
        break;
      
      case TrailMode::kDots:
      case TrailMode::kArrows:
        glPointSize(trail_width_ * 3.0f);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(trail_vertices_.size()));
        glPointSize(1.0f);
        break;
      
      default:
        break;
    }
  }

  glBindVertexArray(0);
}

} // namespace quickviz