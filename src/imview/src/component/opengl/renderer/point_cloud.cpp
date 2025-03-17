/**
 * @file point_cloud.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/component/opengl/renderer/point_cloud.hpp"

#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <stdexcept>
#include <iostream>
#include <mutex>

#include "imview/component/opengl/shader.hpp"

namespace quickviz {

namespace {
// Simplified shader for debugging
const char* vertex_shader_source = R"(
    #version 330 core
    layout (location = 0) in vec3 aPosition;
    layout (location = 1) in vec3 aColor;
    
    uniform mat4 projection;
    uniform mat4 view;
    uniform mat4 coord_transform;
    uniform float pointSize;
    
    out vec3 vColor;
    
    void main() {
        gl_Position = projection * view * coord_transform * vec4(aPosition, 1.0);
        gl_PointSize = pointSize;  // Hardcoded point size for testing
        vColor = aColor;
    }
)";

const char* fragment_shader_source = R"(
    #version 330 core
    in vec3 vColor;
    
    out vec4 FragColor;
    
    uniform float opacity;
    
    void main() {
        FragColor = vec4(vColor, opacity);  // Use opacity uniform
    }
)";
}  // namespace

PointCloud::PointCloud() { AllocateGpuResources(); }

PointCloud::~PointCloud() { ReleaseGpuResources(); }

void PointCloud::AllocateGpuResources() {
  try {
    // Create and compile shaders using the Shader class
    Shader vertexShader(vertex_shader_source, Shader::Type::kVertex);
    Shader fragmentShader(fragment_shader_source, Shader::Type::kFragment);

    // Print shader source for debugging
    std::cout << "Vertex Shader:" << std::endl;
    vertexShader.Print();
    std::cout << "Fragment Shader:" << std::endl;
    fragmentShader.Print();

    if (!vertexShader.Compile()) {
      throw std::runtime_error(
          "Failed to compile vertex shader for point cloud");
    }

    if (!fragmentShader.Compile()) {
      throw std::runtime_error(
          "Failed to compile fragment shader for point cloud");
    }

    // Create shader program
    shader_.AttachShader(vertexShader);
    shader_.AttachShader(fragmentShader);

    if (!shader_.LinkProgram()) {
      throw std::runtime_error("Failed to link point cloud shader program");
    }

    // Create VAO and VBOs
    glGenVertexArrays(1, &vao_);
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      throw std::runtime_error("Failed to generate VAO: OpenGL error " +
                               std::to_string(err));
    }

    glGenBuffers(1, &position_vbo_);
    err = glGetError();
    if (err != GL_NO_ERROR) {
      throw std::runtime_error(
          "Failed to generate position VBO: OpenGL error " +
          std::to_string(err));
    }

    glGenBuffers(1, &color_vbo_);
    err = glGetError();
    if (err != GL_NO_ERROR) {
      throw std::runtime_error("Failed to generate color VBO: OpenGL error " +
                               std::to_string(err));
    }

    glBindVertexArray(vao_);
    err = glGetError();
    if (err != GL_NO_ERROR) {
      throw std::runtime_error("Failed to bind VAO: OpenGL error " +
                               std::to_string(err));
    }

    // Setup position VBO
    glBindBuffer(GL_ARRAY_BUFFER, position_vbo_);
    err = glGetError();
    if (err != GL_NO_ERROR) {
      throw std::runtime_error("Failed to bind position VBO: OpenGL error " +
                               std::to_string(err));
    }

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    err = glGetError();
    if (err != GL_NO_ERROR) {
      throw std::runtime_error(
          "Failed to set position attribute pointer: OpenGL error " +
          std::to_string(err));
    }

    glEnableVertexAttribArray(0);
    err = glGetError();
    if (err != GL_NO_ERROR) {
      throw std::runtime_error(
          "Failed to enable position attribute: OpenGL error " +
          std::to_string(err));
    }

    // Setup color VBO
    glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
    err = glGetError();
    if (err != GL_NO_ERROR) {
      throw std::runtime_error("Failed to bind color VBO: OpenGL error " +
                               std::to_string(err));
    }

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    err = glGetError();
    if (err != GL_NO_ERROR) {
      throw std::runtime_error(
          "Failed to set color attribute pointer: OpenGL error " +
          std::to_string(err));
    }

    glEnableVertexAttribArray(1);
    err = glGetError();
    if (err != GL_NO_ERROR) {
      throw std::runtime_error(
          "Failed to enable color attribute: OpenGL error " +
          std::to_string(err));
    }

    // Unbind the current buffer
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Unbind the VAO
    glBindVertexArray(0);

    std::cout << "Point cloud graphics resources initialized successfully"
              << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error initializing point cloud resources: " << e.what()
              << std::endl;
    // Clean up any resources that were created
    if (vao_ != 0) {
      glDeleteVertexArrays(1, &vao_);
      vao_ = 0;
    }
    if (position_vbo_ != 0) {
      glDeleteBuffers(1, &position_vbo_);
      position_vbo_ = 0;
    }
    if (color_vbo_ != 0) {
      glDeleteBuffers(1, &color_vbo_);
      color_vbo_ = 0;
    }
    throw;
  }
}

void PointCloud::ReleaseGpuResources() {
  if (vao_) glDeleteVertexArrays(1, &vao_);
  if (position_vbo_) glDeleteBuffers(1, &position_vbo_);
  if (color_vbo_) glDeleteBuffers(1, &color_vbo_);

  vao_ = 0;
  position_vbo_ = 0;
  color_vbo_ = 0;
}

void PointCloud::SetPoints(const std::vector<glm::vec4>& points,
                           ColorMode color_mode) {
  if (points.empty()) {
    return;
  }

  // Store the points in the pending updates queue
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pending_updates_.push({points, 0, color_mode, false});
  }

  // Signal that there's a pending update
  has_pending_update_ = true;
}

void PointCloud::SetPoints(std::vector<glm::vec4>&& points,
                           ColorMode color_mode) {
  if (points.empty()) {
    return;
  }

  // Move the points into the pending updates queue
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pending_updates_.push({std::move(points), 0, color_mode, false});
  }

  // Signal that there's a pending update
  has_pending_update_ = true;
}

void PointCloud::PreallocateBuffers(size_t max_points) {
  if (max_points == 0) {
    std::cerr << "Cannot preallocate buffers with zero size" << std::endl;
    return;
  }

  try {
    // Resize internal vectors to maximum capacity
    points_.resize(max_points);
    colors_.resize(max_points);

    // Check if OpenGL buffers are initialized
    if (position_vbo_ == 0 || color_vbo_ == 0) {
      std::cerr << "OpenGL buffers not initialized. Make sure this is called "
                   "from a thread with an active OpenGL context."
                << std::endl;
      return;
    }

    // Preallocate GPU buffers
    glBindBuffer(GL_ARRAY_BUFFER, position_vbo_);
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      std::cerr << "OpenGL error in PreallocateBuffers (bind position buffer): "
                << err << std::endl;
      return;
    }

    glBufferData(GL_ARRAY_BUFFER, max_points * sizeof(glm::vec3), nullptr,
                 GL_DYNAMIC_DRAW);
    err = glGetError();
    if (err != GL_NO_ERROR) {
      std::cerr
          << "OpenGL error in PreallocateBuffers (allocate position buffer): "
          << err << std::endl;
      return;
    }

    // Unbind position buffer
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
    err = glGetError();
    if (err != GL_NO_ERROR) {
      std::cerr << "OpenGL error in PreallocateBuffers (bind color buffer): "
                << err << std::endl;
      return;
    }

    glBufferData(GL_ARRAY_BUFFER, max_points * sizeof(glm::vec3), nullptr,
                 GL_DYNAMIC_DRAW);
    err = glGetError();
    if (err != GL_NO_ERROR) {
      std::cerr
          << "OpenGL error in PreallocateBuffers (allocate color buffer): "
          << err << std::endl;
      return;
    }

    // Unbind color buffer
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    buffer_capacity_ = max_points;
    // Only reset active_points_ if buffers weren't previously preallocated
    if (!buffers_preallocated_) {
      active_points_ = 0;
    }
    buffers_preallocated_ = true;

    std::cout << "Preallocated buffers for " << max_points << " points"
              << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error in PreallocateBuffers: " << e.what() << std::endl;
    throw;
  }
}

void PointCloud::UpdatePointSubset(const std::vector<glm::vec4>& points,
                                   size_t offset, ColorMode color_mode) {
  if (points.empty()) {
    return;
  }

  // Queue a subset update
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pending_updates_.push({points, offset, color_mode, true});
  }

  // Signal that there's a pending update
  has_pending_update_ = true;
}

void PointCloud::UpdateColors(ColorMode color_mode, size_t start_idx,
                              size_t count) {
  switch (color_mode) {
    case ColorMode::kStatic:
      // Use default color for all points
      for (size_t i = start_idx; i < start_idx + count; ++i) {
        colors_[i] = default_color_;
      }
      break;
    case ColorMode::kHeightField:
      // Use z-coordinate as height field
      for (size_t i = start_idx; i < start_idx + count; ++i) {
        float t = (points_[i].z - min_scalar_) / (max_scalar_ - min_scalar_);
        t = std::max(0.0f, std::min(1.0f, t));

        // Simple rainbow colormap
        colors_[i] = glm::vec3(
            std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.75f)),  // Red
            std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.5f)),   // Green
            std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.25f))   // Blue
        );
      }
      break;
    case ColorMode::kScalarField:
      // For scalar field, we need the original vec4 points
      // This is handled in the calling functions
      break;
  }
}

bool PointCloud::ShouldUseBufferMapping(size_t point_count) const {
  switch (buffer_update_strategy_) {
    case BufferUpdateStrategy::kAuto:
      // Use mapping for large point clouds or when point size is large
      return (point_count > buffer_update_threshold_) || (point_size_ > 5.0f);
    case BufferUpdateStrategy::kBufferSubData:
      return false;
    case BufferUpdateStrategy::kMapBuffer:
      return true;
    default:
      return false;
  }
}

void PointCloud::UpdateBufferWithSubData(uint32_t buffer, const void* data,
                                         size_t size_bytes,
                                         size_t offset_bytes) {
  if (buffer == 0 || data == nullptr || size_bytes == 0) {
    std::cerr << "Invalid parameters for UpdateBufferWithSubData" << std::endl;
    return;
  }

  glBindBuffer(GL_ARRAY_BUFFER, buffer);
  glBufferSubData(GL_ARRAY_BUFFER, offset_bytes, size_bytes, data);
  // Unbind buffer to reset state
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PointCloud::UpdateBufferWithMapping(uint32_t buffer, const void* data,
                                         size_t size_bytes,
                                         size_t offset_bytes) {
  if (buffer == 0 || data == nullptr || size_bytes == 0) {
    std::cerr << "Invalid parameters for UpdateBufferWithMapping" << std::endl;
    return;
  }

  glBindBuffer(GL_ARRAY_BUFFER, buffer);

  // Map the buffer
  GLbitfield access = GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_RANGE_BIT;
  void* mapped_buffer =
      glMapBufferRange(GL_ARRAY_BUFFER, offset_bytes, size_bytes, access);

  if (mapped_buffer) {
    // Copy data to the mapped buffer
    std::memcpy(mapped_buffer, data, size_bytes);

    // Unmap the buffer
    glUnmapBuffer(GL_ARRAY_BUFFER);
  } else {
    std::cerr << "Failed to map buffer for writing" << std::endl;
    // Fall back to glBufferSubData if mapping fails
    glBufferSubData(GL_ARRAY_BUFFER, offset_bytes, size_bytes, data);
  }

  // Unbind buffer to reset state
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PointCloud::OnDraw(const glm::mat4& projection, const glm::mat4& view, 
                       const glm::mat4& coord_transform) {
  // Process any pending updates first
  if (has_pending_update_) {
    ProcessPendingUpdates();
  }

  if (points_.empty() || active_points_ == 0) {
    return;
  }

  try {
    // Check if OpenGL buffers are initialized
    if (vao_ == 0 || position_vbo_ == 0 || color_vbo_ == 0) {
      std::cerr << "OpenGL buffers not initialized. Make sure this is called "
                   "from a thread with an active OpenGL context."
                << std::endl;
      return;
    }

    if (needs_update_) {
      // Determine if we should use buffer mapping based on point count and size
      bool use_mapping =
          buffers_preallocated_ && ShouldUseBufferMapping(active_points_);

      // Update position buffer
      if (buffers_preallocated_) {
        size_t position_data_size = active_points_ * sizeof(glm::vec3);

        if (use_mapping) {
          UpdateBufferWithMapping(position_vbo_, points_.data(),
                                  position_data_size);
        } else {
          UpdateBufferWithSubData(position_vbo_, points_.data(),
                                  position_data_size);
        }
      } else {
        // Fall back to glBufferData when not preallocated
        glBindBuffer(GL_ARRAY_BUFFER, position_vbo_);
        GLenum err = glGetError();
        if (err != GL_NO_ERROR) {
          std::cerr << "OpenGL error in OnDraw (bind position buffer): " << err
                    << std::endl;
          return;
        }

        glBufferData(GL_ARRAY_BUFFER, points_.size() * sizeof(glm::vec3),
                     points_.data(), GL_STATIC_DRAW);
        err = glGetError();
        if (err != GL_NO_ERROR) {
          std::cerr << "OpenGL error in OnDraw (allocate position buffer): "
                    << err << std::endl;
          return;
        }

        // Unbind buffer
        glBindBuffer(GL_ARRAY_BUFFER, 0);
      }

      // Update color buffer
      if (buffers_preallocated_) {
        size_t color_data_size = active_points_ * sizeof(glm::vec3);

        if (use_mapping) {
          UpdateBufferWithMapping(color_vbo_, colors_.data(), color_data_size);
        } else {
          UpdateBufferWithSubData(color_vbo_, colors_.data(), color_data_size);
        }
      } else {
        // Fall back to glBufferData when not preallocated
        glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
        GLenum err = glGetError();
        if (err != GL_NO_ERROR) {
          std::cerr << "OpenGL error in OnDraw (bind color buffer): " << err
                    << std::endl;
          return;
        }

        glBufferData(GL_ARRAY_BUFFER, colors_.size() * sizeof(glm::vec3),
                     colors_.data(), GL_STATIC_DRAW);
        err = glGetError();
        if (err != GL_NO_ERROR) {
          std::cerr << "OpenGL error in OnDraw (allocate color buffer): " << err
                    << std::endl;
          return;
        }

        // Unbind buffer
        glBindBuffer(GL_ARRAY_BUFFER, 0);
      }

      needs_update_ = false;
    }

    // Get the current appearance settings
    float current_point_size = point_size_;
    float current_opacity = opacity_;
    PointRenderMode current_render_mode;

    {
      std::lock_guard<std::mutex> lock(appearance_mutex_);
      current_render_mode = render_mode_;
    }

    shader_.Use();
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      std::cerr << "OpenGL error in OnDraw (use shader): " << err << std::endl;
      return;
    }

    // Use TrySetUniform instead of SetUniform to avoid exceptions
    shader_.TrySetUniform("projection", projection);
    shader_.TrySetUniform("view", view);
    shader_.TrySetUniform("coord_transform", coord_transform);
    shader_.TrySetUniform("pointSize", current_point_size);
    shader_.TrySetUniform("opacity", current_opacity);

    // Enable point size - this is critical for the shader to control point size
    glEnable(GL_PROGRAM_POINT_SIZE);
    err = glGetError();
    if (err != GL_NO_ERROR) {
      std::cerr << "OpenGL error in OnDraw (enable program point size): " << err
                << std::endl;
      // Continue anyway, as this might not be fatal
    }

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Enable blending if opacity is less than 1.0
    if (current_opacity < 1.0f) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    // Draw points
    glBindVertexArray(vao_);
    err = glGetError();
    if (err != GL_NO_ERROR) {
      std::cerr << "OpenGL error in OnDraw (bind VAO): " << err << std::endl;
      return;
    }

    if (current_render_mode == PointRenderMode::Points) {
      glDrawArrays(GL_POINTS, 0, active_points_);
      err = glGetError();
      if (err != GL_NO_ERROR) {
        std::cerr << "OpenGL error in OnDraw (draw points): " << err
                  << std::endl;
        // Continue anyway, as we've already done most of the work
      }
    } else if (current_render_mode == PointRenderMode::Spheres) {
      // Sphere rendering would go here
      // For now, fall back to points
      glDrawArrays(GL_POINTS, 0, active_points_);
      err = glGetError();
      if (err != GL_NO_ERROR) {
        std::cerr << "OpenGL error in OnDraw (draw spheres): " << err
                  << std::endl;
        // Continue anyway, as we've already done most of the work
      }
    }

    // Unbind VAO
    glBindVertexArray(0);

    // Disable states we enabled
    if (current_opacity < 1.0f) {
      glDisable(GL_BLEND);
    }
    glDisable(GL_PROGRAM_POINT_SIZE);
    glDisable(GL_DEPTH_TEST);

    // Unbind shader program
    glUseProgram(0);
  } catch (const std::exception& e) {
    std::cerr << "Error in OnDraw: " << e.what() << std::endl;
  }
}

void PointCloud::ProcessPendingUpdates() {
  if (!has_pending_update_) {
    return;
  }

  // Process all pending updates
  while (true) {
    // Get the next pending update
    PendingUpdate update;
    bool has_update = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (pending_updates_.empty()) {
        break;
      }

      update = std::move(pending_updates_.front());
      pending_updates_.pop();
      has_update = true;
    }

    if (!has_update) {
      break;
    }

    // Process the update
    if (update.is_subset) {
      // Handle subset update
      if (!buffers_preallocated_) {
        std::cerr << "Cannot update subset without preallocated buffers. Call "
                     "PreallocateBuffers first."
                  << std::endl;
        continue;
      }

      if (update.offset + update.points.size() > buffer_capacity_) {
        std::cerr << "Update exceeds buffer capacity. Offset: " << update.offset
                  << ", Points: " << update.points.size()
                  << ", Capacity: " << buffer_capacity_ << std::endl;
        continue;
      }

      // Extract the xyz components from vec4 points and update the subset
      for (size_t i = 0; i < update.points.size(); ++i) {
        if (update.offset + i < points_.size()) {
          points_[update.offset + i] = glm::vec3(
              update.points[i].x, update.points[i].y, update.points[i].z);
        }
      }

      // Update colors for the subset
      if (update.color_mode == ColorMode::kScalarField) {
        // Handle scalar field separately since we need the w component
        for (size_t i = 0; i < update.points.size(); ++i) {
          if (update.offset + i < colors_.size()) {
            float scalar_value = update.points[i].w;

            // Ensure scalar value is valid
            if (std::isnan(scalar_value) || std::isinf(scalar_value)) {
              scalar_value = 0.0f;
            }

            float t =
                (scalar_value - min_scalar_) / (max_scalar_ - min_scalar_);
            t = std::max(0.0f, std::min(1.0f, t));

            // Simple rainbow colormap
            colors_[update.offset + i] = glm::vec3(
                std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.75f)),  // Red
                std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.5f)),   // Green
                std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.25f))   // Blue
            );
          }
        }
      } else {
        UpdateColors(update.color_mode, update.offset, update.points.size());
      }

      // Update active point count if needed
      active_points_ =
          std::max(active_points_, update.offset + update.points.size());
    } else {
      // Handle full update
      // If buffers aren't preallocated or the new data exceeds capacity, resize
      if (!buffers_preallocated_ || update.points.size() > buffer_capacity_) {
        // Resize vectors to match input size
        points_.resize(update.points.size());
        colors_.resize(update.points.size());
        active_points_ = update.points.size();
      } else {
        // Update only the active portion of preallocated buffers
        active_points_ = update.points.size();
      }

      // Extract the xyz components from vec4 points
      for (size_t i = 0; i < update.points.size(); ++i) {
        points_[i] = glm::vec3(update.points[i].x, update.points[i].y,
                               update.points[i].z);
      }

      // Update colors based on the color mode
      if (update.color_mode == ColorMode::kScalarField) {
        // Handle scalar field separately since we need the w component
        for (size_t i = 0; i < update.points.size(); ++i) {
          float scalar_value = update.points[i].w;

          // Ensure scalar value is valid
          if (std::isnan(scalar_value) || std::isinf(scalar_value)) {
            scalar_value = 0.0f;
          }

          float t = (scalar_value - min_scalar_) / (max_scalar_ - min_scalar_);
          t = std::max(0.0f, std::min(1.0f, t));

          // Simple rainbow colormap
          colors_[i] = glm::vec3(
              std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.75f)),  // Red
              std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.5f)),   // Green
              std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.25f))   // Blue
          );
        }
      } else {
        UpdateColors(update.color_mode, 0, update.points.size());
      }
    }

    // Mark that we need to update the OpenGL buffers
    needs_update_ = true;
  }

  // Reset the flag if all updates have been processed
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (pending_updates_.empty()) {
      has_pending_update_ = false;
    }
  }
}

void PointCloud::SetPointSize(float size) { point_size_ = size; }

void PointCloud::SetDefaultColor(const glm::vec3& color) {
  std::lock_guard<std::mutex> lock(appearance_mutex_);
  default_color_ = color;
}

void PointCloud::SetOpacity(float opacity) { opacity_ = opacity; }

void PointCloud::SetScalarRange(float min_val, float max_val) {
  min_scalar_ = min_val;
  max_scalar_ = max_val;
}

void PointCloud::SetRenderMode(PointRenderMode mode) { render_mode_ = mode; }

}  // namespace quickviz
