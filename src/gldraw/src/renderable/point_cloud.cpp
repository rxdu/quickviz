/**
 * @file point_cloud.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/point_cloud.hpp"

#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <numeric>

#include "gldraw/shader.hpp"

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
    uniform vec3 layerColor;
    uniform float layerOpacity;
    uniform bool useLayerColor;
    uniform bool useSphereMode;
    uniform bool useSphereSurfaceHighlight;
    uniform bool useOutlineMode;
    
    void main() {
        // Create circular points by discarding pixels outside circle radius
        vec2 coord = gl_PointCoord - vec2(0.5);
        float distance = length(coord);
        if (distance > 0.5) {
            discard;
        }
        
        vec3 finalColor = vColor;
        float finalOpacity = opacity;
        
        // Handle outline mode - only color pixels near the edge
        if (useOutlineMode && useLayerColor) {
            float outlineThickness = 0.15; // Width of the outline ring
            if (distance > (0.5 - outlineThickness)) {
                finalColor = layerColor;
                finalOpacity = layerOpacity;
            }
        } else if (useLayerColor) {
            // Surface fill mode - color entire surface
            finalColor = layerColor;
            finalOpacity = layerOpacity;
        }
        
        if (useSphereMode) {
            // Calculate 3D sphere normal and lighting for sphere mode
            float z = sqrt(1.0 - 4.0 * distance * distance);
            vec3 normal = normalize(vec3(coord * 2.0, z));
            
            // Simple directional lighting
            vec3 lightDir = normalize(vec3(0.5, 0.5, 1.0));
            float diffuse = max(dot(normal, lightDir), 0.0);
            
            // Add ambient lighting and specular highlight
            float ambient = 0.3;
            vec3 viewDir = vec3(0.0, 0.0, 1.0);
            vec3 reflectDir = reflect(-lightDir, normal);
            float specular = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
            
            // Combine lighting
            float lighting = ambient + diffuse * 0.7 + specular * 0.3;
            
            // Apply lighting to create 3D appearance
            finalColor *= lighting;
            
            // Add depth-based darkening for more 3D effect
            float depthFactor = z * 0.2 + 0.8;
            finalColor *= depthFactor;
        }
        
        FragColor = vec4(finalColor, finalOpacity);
    }
)";

// ID buffer picking shaders
const char* id_vertex_shader_source = R"(
    #version 330 core
    layout(location = 0) in vec3 a_position;
    uniform mat4 u_mvp;
    uniform float u_point_size_px;
    uniform int u_base_id;
    flat out int v_id;
    void main() {
      gl_Position = u_mvp * vec4(a_position, 1.0);
      gl_PointSize = u_point_size_px;
      v_id = u_base_id + gl_VertexID;
    }
)";

const char* id_fragment_shader_source = R"(
    #version 330 core
    layout(location = 0) out vec4 o_color;
    flat in int v_id;

    vec3 encode24(int id) {
      int r =  id        & 255;
      int g = (id >> 8 ) & 255;
      int b = (id >> 16) & 255;
      return vec3(float(r), float(g), float(b)) / 255.0;
    }

    void main() {
      vec2 p = gl_PointCoord * 2.0 - 1.0;
      if (dot(p, p) > 1.0) discard;
      o_color = vec4(encode24(v_id), 1.0);
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

    // Create and compile ID buffer shaders
    Shader idVertexShader(id_vertex_shader_source, Shader::Type::kVertex);
    Shader idFragmentShader(id_fragment_shader_source, Shader::Type::kFragment);

    if (!idVertexShader.Compile()) {
      std::cerr << "ERROR::POINT_CLOUD::ID_VERTEX_SHADER_COMPILATION_FAILED" << std::endl;
      throw std::runtime_error("ID vertex shader compilation failed");
    }

    if (!idFragmentShader.Compile()) {
      std::cerr << "ERROR::POINT_CLOUD::ID_FRAGMENT_SHADER_COMPILATION_FAILED" << std::endl;
      throw std::runtime_error("ID fragment shader compilation failed");
    }

    // Create ID shader program
    id_shader_.AttachShader(idVertexShader);
    id_shader_.AttachShader(idFragmentShader);

    if (!id_shader_.LinkProgram()) {
      throw std::runtime_error("Failed to link point cloud ID shader program");
    }

    // Create VAO and VBOs
    glGenVertexArrays(1, &vao_);
    glGenVertexArrays(1, &id_vao_);  // Dedicated VAO for ID rendering
    glGenBuffers(1, &position_vbo_);
    glGenBuffers(1, &color_vbo_);
    glGenBuffers(1, &id_vbo_);

    // Setup main VAO for normal rendering
    glBindVertexArray(vao_);
    // Setup position VBO
    glBindBuffer(GL_ARRAY_BUFFER, position_vbo_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);
    // Setup color VBO
    glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);
    // Setup ID VBO for point indices
    glBindBuffer(GL_ARRAY_BUFFER, id_vbo_);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(2);
    glBindVertexArray(0);

    // Setup dedicated ID VAO - exactly like reference implementation
    glBindVertexArray(id_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, position_vbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 
                         reinterpret_cast<void*>(0));
    glBindVertexArray(0);

    // Unbind buffers
    glBindBuffer(GL_ARRAY_BUFFER, 0);

  } catch (const std::exception& e) {
    std::cerr << "Error initializing point cloud resources: " << e.what() << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void PointCloud::ReleaseGpuResources() noexcept {
  if (vao_) glDeleteVertexArrays(1, &vao_);
  if (id_vao_) glDeleteVertexArrays(1, &id_vao_);
  if (position_vbo_) glDeleteBuffers(1, &position_vbo_);
  if (color_vbo_) glDeleteBuffers(1, &color_vbo_);
  if (id_vbo_) glDeleteBuffers(1, &id_vbo_);
  
  // Clean up layer index buffers
  CleanupLayerIndexBuffers();

  vao_ = 0;
  id_vao_ = 0;
  position_vbo_ = 0;
  color_vbo_ = 0;
  id_vbo_ = 0;
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
    glBindBuffer(GL_ARRAY_BUFFER, id_vbo_);
    glBufferData(GL_ARRAY_BUFFER, max_points * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
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
        
        // Update ID buffer with point indices (0, 1, 2, ...)
        std::vector<float> point_ids(active_points_);
        std::iota(point_ids.begin(), point_ids.end(), 0.0f);
        
        
        if (use_mapping) {
          UpdateBufferWithMapping(id_vbo_, point_ids.data(), active_points_ * sizeof(float));
        } else {
          UpdateBufferWithSubData(id_vbo_, point_ids.data(), active_points_ * sizeof(float));
        }
      } else {
        glBindBuffer(GL_ARRAY_BUFFER, position_vbo_);
        glBufferData(GL_ARRAY_BUFFER, data_size, points_.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
        glBufferData(GL_ARRAY_BUFFER, data_size, colors_.data(), GL_STATIC_DRAW);
        
        // Update ID buffer with point indices (0, 1, 2, ...)
        std::vector<float> point_ids(active_points_);
        std::iota(point_ids.begin(), point_ids.end(), 0.0f);
        
        
        glBindBuffer(GL_ARRAY_BUFFER, id_vbo_);
        glBufferData(GL_ARRAY_BUFFER, active_points_ * sizeof(float), point_ids.data(), GL_STATIC_DRAW);
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
      }
      needs_update_ = false;
    }

    // Choose shader and rendering mode based on render mode
    if (render_mode_ == PointMode::kIdBuffer) {
      // ID buffer rendering mode for GPU picking
      id_shader_.Use();
      
      // Compute MVP matrix exactly like reference
      glm::mat4 mvp = projection * view * coord_transform;
      id_shader_.TrySetUniform("u_mvp", mvp);
      id_shader_.TrySetUniform("u_point_size_px", point_size_);
      id_shader_.TrySetUniform("u_base_id", 1);  // Reserve 0 for background
      
      
      glEnable(GL_PROGRAM_POINT_SIZE);
      glEnable(GL_DEPTH_TEST);
      
      // Use main VAO for ID buffer rendering
      glBindVertexArray(vao_);
      glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(active_points_));
      
      glBindVertexArray(0);
      glDisable(GL_PROGRAM_POINT_SIZE);
      glDisable(GL_DEPTH_TEST);
      glUseProgram(0);
    } else {
      // Normal rendering mode
      shader_.Use();
      shader_.TrySetUniform("projection", projection);
      shader_.TrySetUniform("view", view);
      shader_.TrySetUniform("coord_transform", coord_transform);
      shader_.TrySetUniform("pointSize", point_size_);
      shader_.TrySetUniform("opacity", opacity_);
      shader_.TrySetUniform("useLayerColor", false);
      shader_.TrySetUniform("useSphereMode", render_mode_ == PointMode::kSphere);
      shader_.TrySetUniform("useSphereSurfaceHighlight", false);

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
    }

  } catch (const std::exception& e) {
    std::cerr << "Error in OnDraw: " << e.what() << std::endl;
  }
  
  // Apply layer effects only during normal rendering (not ID buffer mode)
  if (render_mode_ != PointMode::kIdBuffer) {
    ApplyLayerEffects(projection, view, coord_transform);
  }
}

// Layer management implementations
std::shared_ptr<PointLayer> PointCloud::CreateLayer(const std::string& name, int priority) {
  return layer_manager_.CreateLayer(name, priority);
}

std::shared_ptr<PointLayer> PointCloud::GetLayer(const std::string& name) {
  return layer_manager_.GetLayer(name);
}

bool PointCloud::RemoveLayer(const std::string& name) {
  // Remove the index buffer for this layer
  auto it = layer_index_buffers_.find(name);
  if (it != layer_index_buffers_.end()) {
    if (it->second.ebo != 0) {
      glDeleteBuffers(1, &it->second.ebo);
    }
    layer_index_buffers_.erase(it);
  }
  
  return layer_manager_.RemoveLayer(name);
}

void PointCloud::ClearAllLayers() {
  CleanupLayerIndexBuffers();
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
  
  // Invalidate the layer's rendering cache
  InvalidateLayerBuffer(layer_name);
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
    
    // Invalidate the layer's rendering cache
    InvalidateLayerBuffer(layer_name);
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
  
  auto layers = layer_manager_.GetLayersByPriority();
  if (layers.empty()) return;
  
  // Enable blending for layer effects
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glDisable(GL_DEPTH_TEST);  // Disable depth test so layers always render on top
  
  shader_.Use();
  shader_.SetUniform("projection", projection);
  shader_.SetUniform("view", view);
  shader_.SetUniform("coord_transform", coord_transform);
  shader_.SetUniform("opacity", 1.0f); // Layer opacity handled separately
  shader_.TrySetUniform("useSphereMode", render_mode_ == PointMode::kSphere);
  
  glBindVertexArray(vao_);
  
  // Render each layer using index buffers for efficient batch rendering
  for (const auto& layer : layers) {
    if (!layer || !layer->IsVisible() || layer->GetPointCount() == 0) continue;
    
    const std::string& layer_name = layer->GetName();
    
    // Update index buffer if needed
    std::vector<size_t> indices_vec(layer->GetPointIndices().begin(), 
                                    layer->GetPointIndices().end());
    UpdateLayerIndexBuffer(layer_name, indices_vec);
    
    // Get the index buffer for this layer
    auto& buffer_info = layer_index_buffers_[layer_name];
    if (buffer_info.ebo == 0 || buffer_info.count == 0) continue;
    
    // Set layer-specific uniforms
    // Only apply size multiplier for highlight modes that affect size
    float layer_point_size = point_size_;
    if (layer->GetHighlightMode() == PointLayer::HighlightMode::kColorAndSize ||
        layer->GetHighlightMode() == PointLayer::HighlightMode::kSizeIncrease) {
      layer_point_size = point_size_ * layer->GetPointSizeMultiplier();
    }
    shader_.SetUniform("pointSize", layer_point_size);
    shader_.TrySetUniform("layerColor", layer->GetColor());
    shader_.TrySetUniform("layerOpacity", layer->GetOpacity());
    shader_.TrySetUniform("useLayerColor", true);
    
    // Enable sphere surface highlighting if using sphere mode and appropriate highlight mode
    bool useSphereSurface = (render_mode_ == PointMode::kSphere) && 
                            (layer->GetHighlightMode() == PointLayer::HighlightMode::kSphereSurface);
    shader_.TrySetUniform("useSphereSurfaceHighlight", useSphereSurface);
    
    // Enable outline mode for kColorAndSize and kSizeIncrease modes
    bool useOutlineMode = (layer->GetHighlightMode() == PointLayer::HighlightMode::kColorAndSize) ||
                          (layer->GetHighlightMode() == PointLayer::HighlightMode::kSizeIncrease);
    shader_.TrySetUniform("useOutlineMode", useOutlineMode);
    
    // Set blending mode based on highlight mode
    // Surface modes completely replace underlying colors
    // Outline modes blend with underlying colors
    if (layer->GetHighlightMode() == PointLayer::HighlightMode::kSphereSurface) {
      glBlendFunc(GL_ONE, GL_ZERO); // Replace mode - completely overwrites
    } else {
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Standard alpha blending
    }
    
    // Bind the index buffer and draw all points in one call
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer_info.ebo);
    glDrawElements(GL_POINTS, buffer_info.count, GL_UNSIGNED_INT, nullptr);
  }
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
  glDisable(GL_BLEND);
  glDisable(GL_PROGRAM_POINT_SIZE);
  glUseProgram(0);
}

void PointCloud::UpdateLayerIndexBuffer(const std::string& layer_name, 
                                       const std::vector<size_t>& indices) {
  // Find or create buffer info for this layer
  auto& buffer_info = layer_index_buffers_[layer_name];
  
  // Check if we need to update
  if (!buffer_info.needs_update && buffer_info.count == indices.size()) {
    return;
  }
  
  // Create buffer if it doesn't exist
  if (buffer_info.ebo == 0) {
    glGenBuffers(1, &buffer_info.ebo);
  }
  
  // Convert size_t indices to GLuint (OpenGL expects unsigned int)
  std::vector<GLuint> gl_indices;
  gl_indices.reserve(indices.size());
  for (size_t idx : indices) {
    if (idx < active_points_) {
      gl_indices.push_back(static_cast<GLuint>(idx));
    }
  }
  
  // Update the buffer data
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer_info.ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, 
               gl_indices.size() * sizeof(GLuint),
               gl_indices.data(), 
               GL_DYNAMIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  
  buffer_info.count = gl_indices.size();
  buffer_info.needs_update = false;
}

void PointCloud::InvalidateLayerBuffer(const std::string& layer_name) {
  auto it = layer_index_buffers_.find(layer_name);
  if (it != layer_index_buffers_.end()) {
    it->second.needs_update = true;
  }
  // If buffer doesn't exist yet, it will be created on next render with correct data
}

void PointCloud::CleanupLayerIndexBuffers() {
  for (auto& [name, buffer_info] : layer_index_buffers_) {
    if (buffer_info.ebo != 0) {
      glDeleteBuffers(1, &buffer_info.ebo);
      buffer_info.ebo = 0;
    }
  }
  layer_index_buffers_.clear();
}

// ID buffer encoding/decoding methods
glm::vec3 PointCloud::EncodePointId(size_t point_index) {
  // Encode point index + 1 (reserve 0 for background)
  uint32_t id = static_cast<uint32_t>(point_index + 1);
  
  // Extract RGB components (24-bit encoding)
  uint8_t r = (id >> 16) & 0xFF;
  uint8_t g = (id >> 8) & 0xFF;
  uint8_t b = id & 0xFF;
  
  // Convert to normalized float values
  return glm::vec3(r / 255.0f, g / 255.0f, b / 255.0f);
}

size_t PointCloud::DecodePointId(const glm::vec3& color) {
  // Convert normalized float to 8-bit values
  uint8_t r = static_cast<uint8_t>(color.r * 255.0f + 0.5f);
  uint8_t g = static_cast<uint8_t>(color.g * 255.0f + 0.5f);
  uint8_t b = static_cast<uint8_t>(color.b * 255.0f + 0.5f);
  
  return DecodePointId(r, g, b);
}

size_t PointCloud::DecodePointId(uint8_t r, uint8_t g, uint8_t b) {
  // Decode using reference implementation approach (little-endian packing)
  uint32_t id = (uint32_t(r) << 0) | (uint32_t(g) << 8) | (uint32_t(b) << 16);
  
  if (id == 0u) {
    return SIZE_MAX; // No hit (background)
  } else {
    // We used base_id = 1, actual vertex index = id - 1
    uint32_t vertex_index = id - 1u;
    return static_cast<size_t>(vertex_index);
  }
}

size_t PointCloud::PickPointAt(float screen_x, float screen_y,
                              float screen_width, float screen_height,
                              const glm::mat4& projection,
                              const glm::mat4& view,
                              const glm::mat4& coord_transform) const {
  // This method delegates to the scene manager's GPU picking system
  // The scene manager maintains the ID buffer rendering pipeline
  // Convert screen coordinates to framebuffer coordinates if needed
  int pixel_x = static_cast<int>(screen_x);
  int pixel_y = static_cast<int>(screen_height - screen_y); // Flip Y for OpenGL
  
  // Note: This is a placeholder implementation. The actual picking logic
  // should be coordinated through the GlSceneManager's PickPointAtPixel method
  // which maintains the ID buffer rendering pipeline.
  
  // For now, return SIZE_MAX to indicate no point found
  // The SelectionManager will call GlSceneManager::PickPointAtPixel instead
  return SIZE_MAX;
}

}  // namespace quickviz
