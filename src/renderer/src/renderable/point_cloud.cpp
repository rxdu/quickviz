/**
 * @file point_cloud.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "renderer/renderable/point_cloud.hpp"

#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <stdexcept>
#include <iostream>
#include <algorithm>

#include "renderer/shader.hpp"

namespace quickviz {

namespace {
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
        gl_PointSize = pointSize;
        vColor = aColor;
    }
)";

const char* fragment_shader_source = R"(
    #version 330 core
    in vec3 vColor;
    
    out vec4 FragColor;
    
    uniform float opacity;
    
    void main() {
        FragColor = vec4(vColor, opacity);
    }
)";
}  // namespace

PointCloud::PointCloud() { AllocateGpuResources(); }

PointCloud::~PointCloud() { ReleaseGpuResources(); }

void PointCloud::AllocateGpuResources() {
  try {
    // Create and compile shaders
    Shader vertexShader(vertex_shader_source, Shader::Type::kVertex);
    Shader fragmentShader(fragment_shader_source, Shader::Type::kFragment);

    // IMPORTANT: Compile shaders before linking
    if (!vertexShader.Compile()) {
      std::cerr << "ERROR::POINT_CLOUD::VERTEX_SHADER_COMPILATION_FAILED" << std::endl;
      throw std::runtime_error("Vertex shader compilation failed");
    }

    if (!fragmentShader.Compile()) {
      std::cerr << "ERROR::POINT_CLOUD::FRAGMENT_SHADER_COMPILATION_FAILED" << std::endl;
      throw std::runtime_error("Fragment shader compilation failed");
    }

    // Create shader program
    shader_.AttachShader(vertexShader);
    shader_.AttachShader(fragmentShader);

    if (!shader_.LinkProgram()) {
      throw std::runtime_error("Failed to link point cloud shader program");
    }

    // Create VAO and VBOs
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &position_vbo_);
    glGenBuffers(1, &color_vbo_);

    glBindVertexArray(vao_);

    // Setup position VBO
    glBindBuffer(GL_ARRAY_BUFFER, position_vbo_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    // Setup color VBO
    glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);

    // Unbind buffers
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

  } catch (const std::exception& e) {
    std::cerr << "Error initializing point cloud resources: " << e.what() << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void PointCloud::ReleaseGpuResources() noexcept {
  if (vao_) glDeleteVertexArrays(1, &vao_);
  if (position_vbo_) glDeleteBuffers(1, &position_vbo_);
  if (color_vbo_) glDeleteBuffers(1, &color_vbo_);

  vao_ = 0;
  position_vbo_ = 0;
  color_vbo_ = 0;
}

void PointCloud::SetPoints(const std::vector<glm::vec4>& points, ColorMode color_mode) {
  if (points.empty()) return;

  // Resize vectors if needed
  if (!buffers_preallocated_ || points.size() > buffer_capacity_) {
    points_.resize(points.size());
    colors_.resize(points.size());
    active_points_ = points.size();
  } else {
    active_points_ = points.size();
  }

  // Extract xyz components and handle scalar field
  if (color_mode == ColorMode::kScalarField) {
    for (size_t i = 0; i < points.size(); ++i) {
      points_[i] = glm::vec3(points[i].x, points[i].y, points[i].z);
      float t = (points[i].w - min_scalar_) / (max_scalar_ - min_scalar_);
      t = std::max(0.0f, std::min(1.0f, t));
      colors_[i] = glm::vec3(
          std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.75f)),  // Red
          std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.5f)),   // Green
          std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.25f))   // Blue
      );
    }
  } else {
    for (size_t i = 0; i < points.size(); ++i) {
      points_[i] = glm::vec3(points[i].x, points[i].y, points[i].z);
    }
    UpdateColors(color_mode);
  }
  needs_update_ = true;
}

void PointCloud::SetPoints(std::vector<glm::vec4>&& points, ColorMode color_mode) {
  if (points.empty()) return;

  // Resize vectors if needed
  if (!buffers_preallocated_ || points.size() > buffer_capacity_) {
    points_.resize(points.size());
    colors_.resize(points.size());
    active_points_ = points.size();
  } else {
    active_points_ = points.size();
  }

  // Extract xyz components and handle scalar field with move semantics
  if (color_mode == ColorMode::kScalarField) {
    for (size_t i = 0; i < points.size(); ++i) {
      points_[i] = glm::vec3(points[i].x, points[i].y, points[i].z);
      float t = (points[i].w - min_scalar_) / (max_scalar_ - min_scalar_);
      t = std::max(0.0f, std::min(1.0f, t));
      colors_[i] = glm::vec3(
          std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.75f)),  // Red
          std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.5f)),   // Green
          std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.25f))   // Blue
      );
    }
  } else {
    for (size_t i = 0; i < points.size(); ++i) {
      points_[i] = glm::vec3(points[i].x, points[i].y, points[i].z);
    }
    UpdateColors(color_mode);
  }
  needs_update_ = true;
}

void PointCloud::SetPoints(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& colors) {
  if (points.empty() || colors.empty()) return;
  if (points.size() != colors.size()) {
    std::cerr << "Error: Points and colors vectors must have the same size" << std::endl;
    return;
  }

  // Resize vectors if needed
  if (!buffers_preallocated_ || points.size() > buffer_capacity_) {
    points_.resize(points.size());
    colors_.resize(points.size());
    active_points_ = points.size();
  } else {
    active_points_ = points.size();
  }

  // Copy points and colors directly
  for (size_t i = 0; i < points.size(); ++i) {
    points_[i] = points[i];
    colors_[i] = colors[i];
  }
  
  needs_update_ = true;
}

void PointCloud::SetPoints(std::vector<glm::vec3>&& points, std::vector<glm::vec3>&& colors) {
  if (points.empty() || colors.empty()) return;
  if (points.size() != colors.size()) {
    std::cerr << "Error: Points and colors vectors must have the same size" << std::endl;
    return;
  }

  // Resize vectors if needed
  if (!buffers_preallocated_ || points.size() > buffer_capacity_) {
    points_.resize(points.size());
    colors_.resize(points.size());
    active_points_ = points.size();
  } else {
    active_points_ = points.size();
  }

  // Move points and colors directly
  for (size_t i = 0; i < points.size(); ++i) {
    points_[i] = std::move(points[i]);
    colors_[i] = std::move(colors[i]);
  }
  
  needs_update_ = true;
}

void PointCloud::PreallocateBuffers(size_t max_points) {
  if (max_points == 0) {
    std::cerr << "Cannot preallocate buffers with zero size" << std::endl;
    return;
  }

  try {
    points_.resize(max_points);
    colors_.resize(max_points);

    glBindBuffer(GL_ARRAY_BUFFER, position_vbo_);
    glBufferData(GL_ARRAY_BUFFER, max_points * sizeof(glm::vec3), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
    glBufferData(GL_ARRAY_BUFFER, max_points * sizeof(glm::vec3), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    buffer_capacity_ = max_points;
    if (!buffers_preallocated_) {
      active_points_ = 0;
    }
    buffers_preallocated_ = true;

  } catch (const std::exception& e) {
    std::cerr << "Error in PreallocateBuffers: " << e.what() << std::endl;
    throw;
  }
}

void PointCloud::UpdateColors(ColorMode color_mode) {
  switch (color_mode) {
    case ColorMode::kStatic:
      std::fill(colors_.begin(), colors_.begin() + active_points_, default_color_);
      break;
    case ColorMode::kHeightField:
      for (size_t i = 0; i < active_points_; ++i) {
        float t = (points_[i].z - min_scalar_) / (max_scalar_ - min_scalar_);
        t = std::max(0.0f, std::min(1.0f, t));
        colors_[i] = glm::vec3(
            std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.75f)),  // Red
            std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.5f)),   // Green
            std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.25f))   // Blue
        );
      }
      break;
    case ColorMode::kScalarField:
      // Handled in SetPoints
      break;
    case ColorMode::kRGB:
      // RGB colors are set directly via SetPoints(points, colors)
      // No additional processing needed
      break;
  }
}

bool PointCloud::ShouldUseBufferMapping(size_t point_count) const {
  switch (buffer_update_strategy_) {
    case BufferUpdateStrategy::kAuto:
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
                                       size_t size_bytes, size_t offset_bytes) {
  glBindBuffer(GL_ARRAY_BUFFER, buffer);
  glBufferSubData(GL_ARRAY_BUFFER, offset_bytes, size_bytes, data);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PointCloud::UpdateBufferWithMapping(uint32_t buffer, const void* data,
                                       size_t size_bytes, size_t offset_bytes) {
  glBindBuffer(GL_ARRAY_BUFFER, buffer);
  GLbitfield access = GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_RANGE_BIT;
  void* mapped_buffer = glMapBufferRange(GL_ARRAY_BUFFER, offset_bytes, size_bytes, access);

  if (mapped_buffer) {
    std::memcpy(mapped_buffer, data, size_bytes);
    glUnmapBuffer(GL_ARRAY_BUFFER);
  } else {
    glBufferSubData(GL_ARRAY_BUFFER, offset_bytes, size_bytes, data);
  }
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PointCloud::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                       const glm::mat4& coord_transform) {
  if (points_.empty() || active_points_ == 0) return;

  try {
    if (needs_update_) {
      bool use_mapping = buffers_preallocated_ && ShouldUseBufferMapping(active_points_);
      size_t data_size = active_points_ * sizeof(glm::vec3);

      if (buffers_preallocated_) {
        if (use_mapping) {
          UpdateBufferWithMapping(position_vbo_, points_.data(), data_size);
          UpdateBufferWithMapping(color_vbo_, colors_.data(), data_size);
        } else {
          UpdateBufferWithSubData(position_vbo_, points_.data(), data_size);
          UpdateBufferWithSubData(color_vbo_, colors_.data(), data_size);
        }
      } else {
        glBindBuffer(GL_ARRAY_BUFFER, position_vbo_);
        glBufferData(GL_ARRAY_BUFFER, data_size, points_.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
        glBufferData(GL_ARRAY_BUFFER, data_size, colors_.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
      }
      needs_update_ = false;
    }

    shader_.Use();
    shader_.TrySetUniform("projection", projection);
    shader_.TrySetUniform("view", view);
    shader_.TrySetUniform("coord_transform", coord_transform);
    shader_.TrySetUniform("pointSize", point_size_);
    shader_.TrySetUniform("opacity", opacity_);

    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);
    if (opacity_ < 1.0f) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    glBindVertexArray(vao_);
    glDrawArrays(GL_POINTS, 0, active_points_);
    glBindVertexArray(0);

    if (opacity_ < 1.0f) {
      glDisable(GL_BLEND);
    }
    glDisable(GL_PROGRAM_POINT_SIZE);
    glDisable(GL_DEPTH_TEST);
    glUseProgram(0);

  } catch (const std::exception& e) {
    std::cerr << "Error in OnDraw: " << e.what() << std::endl;
  }
  
  // Apply layer effects after base rendering
  ApplyLayerEffects(projection, view, coord_transform);
}

// Layer management implementations
std::shared_ptr<PointLayer> PointCloud::CreateLayer(const std::string& name, int priority) {
  return layer_manager_.CreateLayer(name, priority);
}

std::shared_ptr<PointLayer> PointCloud::GetLayer(const std::string& name) {
  return layer_manager_.GetLayer(name);
}

bool PointCloud::RemoveLayer(const std::string& name) {
  return layer_manager_.RemoveLayer(name);
}

void PointCloud::ClearAllLayers() {
  layer_manager_.ClearAllLayers();
}

void PointCloud::HighlightPoints(const std::vector<size_t>& point_indices, 
                                const glm::vec3& color,
                                const std::string& layer_name,
                                float size_multiplier) {
  auto layer = layer_manager_.GetLayer(layer_name);
  if (!layer) {
    layer = layer_manager_.CreateLayer(layer_name, 100); // High priority for highlights
  }
  
  layer->SetPoints(point_indices);
  layer->SetColor(color);
  layer->SetPointSizeMultiplier(size_multiplier);
  layer->SetHighlightMode(PointLayer::HighlightMode::kColorAndSize);
  layer->SetVisible(true);
}

void PointCloud::HighlightPoint(size_t point_index, 
                               const glm::vec3& color,
                               const std::string& layer_name,
                               float size_multiplier) {
  HighlightPoints({point_index}, color, layer_name, size_multiplier);
}

void PointCloud::ClearHighlights(const std::string& layer_name) {
  auto layer = layer_manager_.GetLayer(layer_name);
  if (layer) {
    layer->ClearPoints();
  }
}

void PointCloud::SetSelectedPoints(const std::vector<size_t>& point_indices, 
                                  const glm::vec3& selection_color) {
  selected_points_ = point_indices;
  
  // Create or update selection layer
  auto selection_layer = layer_manager_.GetLayer("selection");
  if (!selection_layer) {
    selection_layer = layer_manager_.CreateLayer("selection", 200); // Very high priority
  }
  
  selection_layer->SetPoints(point_indices);
  selection_layer->SetColor(selection_color);
  selection_layer->SetHighlightMode(PointLayer::HighlightMode::kColorAndSize);
  selection_layer->SetPointSizeMultiplier(1.8f);
  selection_layer->SetVisible(true);
}

void PointCloud::AddToSelection(const std::vector<size_t>& point_indices) {
  // Add to internal selection list
  selected_points_.insert(selected_points_.end(), point_indices.begin(), point_indices.end());
  
  // Remove duplicates
  std::sort(selected_points_.begin(), selected_points_.end());
  selected_points_.erase(std::unique(selected_points_.begin(), selected_points_.end()), 
                        selected_points_.end());
  
  SetSelectedPoints(selected_points_);
}

void PointCloud::RemoveFromSelection(const std::vector<size_t>& point_indices) {
  for (size_t idx : point_indices) {
    selected_points_.erase(std::remove(selected_points_.begin(), selected_points_.end(), idx),
                          selected_points_.end());
  }
  
  SetSelectedPoints(selected_points_);
}

void PointCloud::ClearSelection() {
  selected_points_.clear();
  auto selection_layer = layer_manager_.GetLayer("selection");
  if (selection_layer) {
    selection_layer->ClearPoints();
  }
}

std::vector<glm::vec4> PointCloud::GetPointsAs4D() const {
  std::vector<glm::vec4> points_4d;
  points_4d.reserve(points_.size());
  
  for (const auto& point : points_) {
    points_4d.push_back(glm::vec4(point, 1.0f)); // w = 1.0f as default
  }
  
  return points_4d;
}

void PointCloud::ApplyLayerEffects(const glm::mat4& projection, const glm::mat4& view, 
                                  const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) return;
  
  auto render_data = layer_manager_.GenerateRenderData();
  if (render_data.empty()) return;
  
  // Enable blending for layer effects
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
  
  shader_.Use();
  shader_.SetUniform("projection", projection);
  shader_.SetUniform("view", view);
  shader_.SetUniform("coord_transform", coord_transform);
  shader_.SetUniform("opacity", 1.0f); // Layer opacity handled separately
  
  glBindVertexArray(vao_);
  
  // Render each layer
  for (const auto& layer_data : render_data) {
    if (layer_data.point_indices.empty()) continue;
    
    // Set layer-specific uniforms
    float layer_point_size = point_size_ * layer_data.point_size_multiplier;
    shader_.SetUniform("pointSize", layer_point_size);
    
    // For now, render all points with layer color
    // TODO: Implement proper per-point layer rendering with index buffers
    shader_.TrySetUniform("layerColor", layer_data.color);
    shader_.TrySetUniform("layerOpacity", layer_data.opacity);
    
    // This is a simplified version - ideally we'd use index buffers
    // to render only the points in this layer
    for (size_t idx : layer_data.point_indices) {
      if (idx < active_points_) {
        glDrawArrays(GL_POINTS, idx, 1);
      }
    }
  }
  
  glBindVertexArray(0);
  glDisable(GL_BLEND);
  glDisable(GL_PROGRAM_POINT_SIZE);
  glDisable(GL_DEPTH_TEST);
  glUseProgram(0);
}

}  // namespace quickviz
