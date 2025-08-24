/**
 * @file sensor_coverage.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-24
 * @brief Implementation of sensor coverage renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/sensor_coverage.hpp"

#include <glad/glad.h>
#include "gldraw/shader.hpp"
#include <cmath>
#include <algorithm>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

namespace quickviz {

namespace {
const std::string kSurfaceVertexShader = R"glsl(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec3 aColor;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uCoordTransform;
uniform mat4 uModel;

out vec3 FragPos;
out vec3 Normal;
out vec3 VertexColor;

void main() {
    vec4 worldPos = uCoordTransform * uModel * vec4(aPos, 1.0);
    gl_Position = uProjection * uView * worldPos;
    
    FragPos = worldPos.xyz;
    Normal = normalize(mat3(transpose(inverse(uCoordTransform * uModel))) * aNormal);
    VertexColor = aColor;
}
)glsl";

const std::string kSurfaceFragmentShader = R"glsl(
#version 330 core
in vec3 FragPos;
in vec3 Normal;
in vec3 VertexColor;

uniform float uAlpha;
uniform bool uPulsing;
uniform float uPulseIntensity;

out vec4 FragColor;

void main() {
    vec3 color = VertexColor;
    
    if (uPulsing) {
        color = color * (1.0 + uPulseIntensity * 0.3);
    }
    
    // Simple lighting
    vec3 lightDir = normalize(vec3(1.0, 1.0, 1.0));
    float diff = max(dot(Normal, lightDir), 0.3);
    color = color * diff;
    
    FragColor = vec4(color, uAlpha);
}
)glsl";

const std::string kLineVertexShader = R"glsl(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uCoordTransform;
uniform mat4 uModel;

out vec3 VertexColor;

void main() {
    gl_Position = uProjection * uView * uCoordTransform * uModel * vec4(aPos, 1.0);
    VertexColor = aColor;
}
)glsl";

const std::string kLineFragmentShader = R"glsl(
#version 330 core
in vec3 VertexColor;

uniform float uAlpha;

out vec4 FragColor;

void main() {
    FragColor = vec4(VertexColor, uAlpha);
}
)glsl";
}

SensorCoverage::SensorCoverage()
    : sensor_type_(SensorType::kLidar)
    , coverage_type_(CoverageType::k2DRings)
    , sensor_position_(glm::vec3(0.0f))
    , sensor_direction_(glm::vec3(1.0f, 0.0f, 0.0f))
    , sensor_up_(glm::vec3(0.0f, 0.0f, 1.0f))
    , min_range_(0.1f)
    , max_range_(10.0f)
    , horizontal_fov_(2.0f * M_PI)  // 360 degrees for LIDAR
    , vertical_fov_(glm::radians(30.0f))
    , min_angle_(0.0f)
    , max_angle_(2.0f * M_PI)
    , detection_probability_(0.95f)
    , beam_width_(glm::radians(1.0f))
    , visualization_mode_(VisualizationMode::kTransparent)
    , color_(glm::vec3(0.0f, 0.8f, 0.2f))
    , near_color_(glm::vec3(0.2f, 1.0f, 0.4f))
    , far_color_(glm::vec3(0.0f, 0.4f, 0.1f))
    , alpha_(0.3f)
    , outline_color_(glm::vec3(0.0f, 0.6f, 0.0f))
    , range_ring_count_(5)
    , range_ring_spacing_(2.0f)
    , show_range_labels_(false)
    , angular_resolution_(glm::radians(1.0f))
    , beam_count_(1)
    , scan_pattern_enabled_(false)
    , scan_speed_(1.0f)
    , blind_spot_direction_(glm::vec3(0.0f))
    , blind_spot_angle_(0.0f)
    , blind_spot_depth_(0.0f)
    , detection_threshold_(0.1f)
    , radial_segments_(32)
    , angular_segments_(64)
    , lod_enabled_(false)
    , lod_distance_threshold_(20.0f)
    , adaptive_resolution_(false)
    , pulsing_enabled_(false)
    , pulsing_frequency_(1.0f)
    , scan_enabled_(false)
    , scan_period_(3.0f)
    , fade_with_distance_(true)
    , fade_start_ratio_(0.8f)
    , time_parameter_(0.0f)
    , transform_matrix_(glm::mat4(1.0f))
    , main_vao_(0), main_vbo_(0), main_normal_vbo_(0), main_color_vbo_(0), main_ebo_(0)
    , ring_vao_(0), ring_vbo_(0), ring_color_vbo_(0), ring_ebo_(0)
    , outline_vao_(0), outline_vbo_(0), outline_ebo_(0)
    , needs_geometry_update_(true)
    , needs_transform_update_(true) {
}

SensorCoverage::SensorCoverage(SensorType sensor_type) : SensorCoverage() {
  SetSensorType(sensor_type);
}

SensorCoverage::~SensorCoverage() {
  ReleaseGpuResources();
}

void SensorCoverage::SetSensorType(SensorType type) {
  sensor_type_ = type;
  
  // Configure default parameters based on sensor type
  switch (type) {
    case SensorType::kLidar:
      coverage_type_ = CoverageType::k2DRings;
      horizontal_fov_ = 2.0f * M_PI;  // 360 degrees
      vertical_fov_ = glm::radians(30.0f);
      min_range_ = 0.1f;
      max_range_ = 100.0f;
      angular_resolution_ = glm::radians(0.25f);
      color_ = glm::vec3(0.0f, 0.8f, 0.2f);
      break;
      
    case SensorType::kCamera:
      coverage_type_ = CoverageType::k3DCone;
      horizontal_fov_ = glm::radians(60.0f);
      vertical_fov_ = glm::radians(45.0f);
      min_range_ = 0.1f;
      max_range_ = 50.0f;
      color_ = glm::vec3(0.2f, 0.4f, 1.0f);
      break;
      
    case SensorType::kRadar:
      coverage_type_ = CoverageType::k3DCone;
      horizontal_fov_ = glm::radians(120.0f);
      vertical_fov_ = glm::radians(20.0f);
      min_range_ = 1.0f;
      max_range_ = 200.0f;
      color_ = glm::vec3(1.0f, 0.4f, 0.0f);
      break;
      
    case SensorType::kSonar:
      coverage_type_ = CoverageType::k3DCone;
      horizontal_fov_ = glm::radians(30.0f);
      vertical_fov_ = glm::radians(30.0f);
      min_range_ = 0.05f;
      max_range_ = 5.0f;
      color_ = glm::vec3(0.8f, 0.0f, 0.8f);
      break;
      
    case SensorType::kProximity:
      coverage_type_ = CoverageType::k3DSphere;
      horizontal_fov_ = 2.0f * M_PI;
      vertical_fov_ = M_PI;
      min_range_ = 0.01f;
      max_range_ = 1.0f;
      color_ = glm::vec3(1.0f, 1.0f, 0.2f);
      break;
      
    default:
      break;
  }
  
  needs_geometry_update_ = true;
}

void SensorCoverage::SetCoverageType(CoverageType type) {
  if (coverage_type_ != type) {
    coverage_type_ = type;
    needs_geometry_update_ = true;
  }
}

void SensorCoverage::SetSensorPosition(const glm::vec3& position) {
  sensor_position_ = position;
  needs_transform_update_ = true;
}

void SensorCoverage::SetSensorOrientation(const glm::vec3& direction, const glm::vec3& up) {
  sensor_direction_ = glm::normalize(direction);
  sensor_up_ = glm::normalize(up);
  needs_transform_update_ = true;
}

void SensorCoverage::SetRange(float min_range, float max_range) {
  min_range_ = std::max(0.01f, min_range);
  max_range_ = std::max(min_range_ + 0.01f, max_range);
  needs_geometry_update_ = true;
}

void SensorCoverage::SetAngularCoverage(float horizontal_fov, float vertical_fov) {
  horizontal_fov_ = std::clamp(horizontal_fov, 0.01f, static_cast<float>(2.0f * M_PI));
  vertical_fov_ = std::clamp(vertical_fov, 0.01f, static_cast<float>(M_PI));
  needs_geometry_update_ = true;
}

void SensorCoverage::SetAngularLimits(float min_angle, float max_angle) {
  min_angle_ = min_angle;
  max_angle_ = max_angle;
  needs_geometry_update_ = true;
}

void SensorCoverage::SetVisualizationMode(VisualizationMode mode) {
  visualization_mode_ = mode;
  needs_geometry_update_ = true;
}

void SensorCoverage::SetColor(const glm::vec3& color) {
  color_ = color;
  needs_geometry_update_ = true;
}

void SensorCoverage::SetRangeColors(const glm::vec3& near_color, const glm::vec3& far_color) {
  near_color_ = near_color;
  far_color_ = far_color;
  needs_geometry_update_ = true;
}

void SensorCoverage::SetTransparency(float alpha) {
  alpha_ = std::clamp(alpha, 0.0f, 1.0f);
}

void SensorCoverage::SetRangeRingCount(int count) {
  range_ring_count_ = std::max(1, std::min(20, count));
  needs_geometry_update_ = true;
}

void SensorCoverage::SetTimeParameter(float time) {
  time_parameter_ = time;
  if (pulsing_enabled_ || scan_enabled_) {
    needs_transform_update_ = true;
  }
}

void SensorCoverage::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;
  
  // Initialize shaders
  try {
    Shader surface_vs(kSurfaceVertexShader.c_str(), Shader::Type::kVertex);
    Shader surface_fs(kSurfaceFragmentShader.c_str(), Shader::Type::kFragment);
    if (!surface_vs.Compile() || !surface_fs.Compile()) {
      throw std::runtime_error("Surface shader compilation failed");
    }
    surface_shader_.AttachShader(surface_vs);
    surface_shader_.AttachShader(surface_fs);
    if (!surface_shader_.LinkProgram()) {
      throw std::runtime_error("Surface shader linking failed");
    }
    
    Shader ring_vs(kLineVertexShader.c_str(), Shader::Type::kVertex);
    Shader ring_fs(kLineFragmentShader.c_str(), Shader::Type::kFragment);
    if (!ring_vs.Compile() || !ring_fs.Compile()) {
      throw std::runtime_error("Ring shader compilation failed");
    }
    ring_shader_.AttachShader(ring_vs);
    ring_shader_.AttachShader(ring_fs);
    if (!ring_shader_.LinkProgram()) {
      throw std::runtime_error("Ring shader linking failed");
    }
    
    Shader outline_vs(kLineVertexShader.c_str(), Shader::Type::kVertex);
    Shader outline_fs(kLineFragmentShader.c_str(), Shader::Type::kFragment);
    if (!outline_vs.Compile() || !outline_fs.Compile()) {
      throw std::runtime_error("Outline shader compilation failed");
    }
    outline_shader_.AttachShader(outline_vs);
    outline_shader_.AttachShader(outline_fs);
    if (!outline_shader_.LinkProgram()) {
      throw std::runtime_error("Outline shader linking failed");
    }
  } catch (const std::exception& e) {
    std::cerr << "SensorCoverage::AllocateGpuResources: " << e.what() << std::endl;
    throw;
  }
  
  // Generate main surface VAO and buffers
  glGenVertexArrays(1, &main_vao_);
  glGenBuffers(1, &main_vbo_);
  glGenBuffers(1, &main_normal_vbo_);
  glGenBuffers(1, &main_color_vbo_);
  glGenBuffers(1, &main_ebo_);
  
  glBindVertexArray(main_vao_);
  
  // Vertex positions
  glBindBuffer(GL_ARRAY_BUFFER, main_vbo_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glEnableVertexAttribArray(0);
  
  // Normals
  glBindBuffer(GL_ARRAY_BUFFER, main_normal_vbo_);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glEnableVertexAttribArray(1);
  
  // Colors
  glBindBuffer(GL_ARRAY_BUFFER, main_color_vbo_);
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glEnableVertexAttribArray(2);
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, main_ebo_);
  
  // Generate range ring VAO and buffers
  glGenVertexArrays(1, &ring_vao_);
  glGenBuffers(1, &ring_vbo_);
  glGenBuffers(1, &ring_color_vbo_);
  glGenBuffers(1, &ring_ebo_);
  
  glBindVertexArray(ring_vao_);
  
  glBindBuffer(GL_ARRAY_BUFFER, ring_vbo_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glEnableVertexAttribArray(0);
  
  glBindBuffer(GL_ARRAY_BUFFER, ring_color_vbo_);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glEnableVertexAttribArray(1);
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ring_ebo_);
  
  // Generate outline VAO and buffers
  glGenVertexArrays(1, &outline_vao_);
  glGenBuffers(1, &outline_vbo_);
  glGenBuffers(1, &outline_ebo_);
  
  glBindVertexArray(outline_vao_);
  
  glBindBuffer(GL_ARRAY_BUFFER, outline_vbo_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glEnableVertexAttribArray(0);
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, outline_ebo_);
  
  glBindVertexArray(0);
  
  needs_geometry_update_ = true;
  needs_transform_update_ = true;
}

void SensorCoverage::ReleaseGpuResources() noexcept {
  if (main_vao_ != 0) {
    glDeleteVertexArrays(1, &main_vao_);
    glDeleteBuffers(1, &main_vbo_);
    glDeleteBuffers(1, &main_normal_vbo_);
    glDeleteBuffers(1, &main_color_vbo_);
    glDeleteBuffers(1, &main_ebo_);
    main_vao_ = main_vbo_ = main_normal_vbo_ = main_color_vbo_ = main_ebo_ = 0;
  }
  
  if (ring_vao_ != 0) {
    glDeleteVertexArrays(1, &ring_vao_);
    glDeleteBuffers(1, &ring_vbo_);
    glDeleteBuffers(1, &ring_color_vbo_);
    glDeleteBuffers(1, &ring_ebo_);
    ring_vao_ = ring_vbo_ = ring_color_vbo_ = ring_ebo_ = 0;
  }
  
  if (outline_vao_ != 0) {
    glDeleteVertexArrays(1, &outline_vao_);
    glDeleteBuffers(1, &outline_vbo_);
    glDeleteBuffers(1, &outline_ebo_);
    outline_vao_ = outline_vbo_ = outline_ebo_ = 0;
  }
}

void SensorCoverage::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                           const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }
  
  // Update geometry if needed
  if (needs_geometry_update_) {
    UpdateGeometry();
    needs_geometry_update_ = false;
  }
  
  // Update transform if needed
  if (needs_transform_update_) {
    UpdateTransformMatrix();
    needs_transform_update_ = false;
  }
  
  // Enable blending for transparency
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  // Draw main coverage surface
  if (visualization_mode_ != VisualizationMode::kRangeRings && !surface_vertices_.empty()) {
    surface_shader_.Use();
    surface_shader_.SetUniform("uProjection", projection);
    surface_shader_.SetUniform("uView", view);
    surface_shader_.SetUniform("uCoordTransform", coord_transform);
    surface_shader_.SetUniform("uModel", transform_matrix_);
    surface_shader_.SetUniform("uAlpha", alpha_);
    surface_shader_.SetUniform("uPulsing", pulsing_enabled_);
    
    if (pulsing_enabled_) {
      float pulse_intensity = std::sin(2.0f * M_PI * pulsing_frequency_ * time_parameter_);
      surface_shader_.SetUniform("uPulseIntensity", pulse_intensity);
    }
    
    glBindVertexArray(main_vao_);
    
    // Update buffers
    glBindBuffer(GL_ARRAY_BUFFER, main_vbo_);
    glBufferData(GL_ARRAY_BUFFER, surface_vertices_.size() * sizeof(glm::vec3),
                 surface_vertices_.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ARRAY_BUFFER, main_normal_vbo_);
    glBufferData(GL_ARRAY_BUFFER, surface_normals_.size() * sizeof(glm::vec3),
                 surface_normals_.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ARRAY_BUFFER, main_color_vbo_);
    glBufferData(GL_ARRAY_BUFFER, surface_colors_.size() * sizeof(glm::vec3),
                 surface_colors_.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, main_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, surface_indices_.size() * sizeof(uint32_t),
                 surface_indices_.data(), GL_DYNAMIC_DRAW);
    
    if (visualization_mode_ == VisualizationMode::kWireframe) {
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      glLineWidth(2.0f);
    }
    
    glDrawElements(GL_TRIANGLES, surface_indices_.size(), GL_UNSIGNED_INT, nullptr);
    
    if (visualization_mode_ == VisualizationMode::kWireframe) {
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      glLineWidth(1.0f);
    }
  }
  
  // Draw range rings
  if ((visualization_mode_ == VisualizationMode::kRangeRings || 
       coverage_type_ == CoverageType::k2DRings) && !ring_vertices_.empty()) {
    
    glLineWidth(2.0f);
    
    ring_shader_.Use();
    ring_shader_.SetUniform("uProjection", projection);
    ring_shader_.SetUniform("uView", view);
    ring_shader_.SetUniform("uCoordTransform", coord_transform);
    ring_shader_.SetUniform("uModel", transform_matrix_);
    ring_shader_.SetUniform("uAlpha", alpha_);
    
    glBindVertexArray(ring_vao_);
    
    glBindBuffer(GL_ARRAY_BUFFER, ring_vbo_);
    glBufferData(GL_ARRAY_BUFFER, ring_vertices_.size() * sizeof(glm::vec3),
                 ring_vertices_.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ARRAY_BUFFER, ring_color_vbo_);
    glBufferData(GL_ARRAY_BUFFER, ring_colors_.size() * sizeof(glm::vec3),
                 ring_colors_.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ring_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, ring_indices_.size() * sizeof(uint32_t),
                 ring_indices_.data(), GL_DYNAMIC_DRAW);
    
    glDrawElements(GL_LINES, ring_indices_.size(), GL_UNSIGNED_INT, nullptr);
    
    glLineWidth(1.0f);
  }
  
  glBindVertexArray(0);
  glDisable(GL_BLEND);
}

void SensorCoverage::UpdateGeometry() {
  // Clear previous geometry
  surface_vertices_.clear();
  surface_normals_.clear();
  surface_colors_.clear();
  surface_indices_.clear();
  ring_vertices_.clear();
  ring_colors_.clear();
  ring_indices_.clear();
  outline_vertices_.clear();
  outline_indices_.clear();
  
  // Generate geometry based on coverage type
  switch (coverage_type_) {
    case CoverageType::k2DRings:
      Generate2DRings();
      break;
    case CoverageType::k3DCone:
      Generate3DCone();
      break;
    case CoverageType::k3DSphere:
      Generate3DSphere();
      break;
    case CoverageType::k3DCylinder:
      Generate3DCylinder();
      break;
    case CoverageType::kSectorSlice:
      GenerateSectorSlice();
      break;
    case CoverageType::kMultiBeam:
      GenerateMultiBeam();
      break;
  }
  
  // Generate range rings if needed
  if (visualization_mode_ == VisualizationMode::kRangeRings || range_ring_count_ > 0) {
    GenerateRangeRings();
  }
}

void SensorCoverage::Generate2DRings() {
  // Generate concentric rings for LIDAR-style sensors
  int num_segments = angular_segments_;
  
  for (int ring = 0; ring < range_ring_count_; ++ring) {
    float range = min_range_ + (max_range_ - min_range_) * (ring + 1) / range_ring_count_;
    float t = static_cast<float>(ring) / (range_ring_count_ - 1);
    glm::vec3 ring_color = glm::mix(near_color_, far_color_, t);
    
    // Generate ring vertices
    uint32_t ring_center_idx = ring_vertices_.size();
    for (int i = 0; i <= num_segments; ++i) {
      float angle = horizontal_fov_ * i / num_segments + min_angle_;
      
      // Check if angle is within FOV
      if (angle > max_angle_) continue;
      
      glm::vec3 vertex(
        range * std::cos(angle),
        range * std::sin(angle),
        0.0f
      );
      
      ring_vertices_.push_back(vertex);
      ring_colors_.push_back(ring_color);
      
      // Create line segment
      if (i > 0) {
        ring_indices_.push_back(ring_vertices_.size() - 2);
        ring_indices_.push_back(ring_vertices_.size() - 1);
      }
    }
    
    // Close ring if full circle
    if (horizontal_fov_ >= 2.0f * M_PI - 0.1f) {
      ring_indices_.push_back(ring_vertices_.size() - 1);
      ring_indices_.push_back(ring_center_idx);
    }
  }
  
  // Add radial lines
  int num_radial_lines = 8;
  for (int i = 0; i <= num_radial_lines; ++i) {
    float t = static_cast<float>(i) / num_radial_lines;
    float angle = min_angle_ + t * (max_angle_ - min_angle_);
    if (angle < min_angle_ || angle > max_angle_) continue;
    
    glm::vec3 inner_point(
      min_range_ * std::cos(angle),
      min_range_ * std::sin(angle),
      0.0f
    );
    
    glm::vec3 outer_point(
      max_range_ * std::cos(angle),
      max_range_ * std::sin(angle),
      0.0f
    );
    
    uint32_t start_idx = ring_vertices_.size();
    ring_vertices_.push_back(inner_point);
    ring_vertices_.push_back(outer_point);
    ring_colors_.push_back(outline_color_);
    ring_colors_.push_back(outline_color_);
    
    ring_indices_.push_back(start_idx);
    ring_indices_.push_back(start_idx + 1);
  }
}

void SensorCoverage::Generate3DCone() {
  // Generate simple camera frustum with 4 flat sides
  float half_h_fov = horizontal_fov_ * 0.5f;
  float half_v_fov = vertical_fov_ * 0.5f;
  
  // Calculate frustum corners at max range (in local coordinate system: +Z is forward)
  float far_width = max_range_ * std::tan(half_h_fov);
  float far_height = max_range_ * std::tan(half_v_fov);
  
  // Calculate frustum corners at min range  
  float near_width = min_range_ * std::tan(half_h_fov);
  float near_height = min_range_ * std::tan(half_v_fov);
  
  // Define 8 frustum vertices (4 near + 4 far)
  std::vector<glm::vec3> frustum_vertices = {
    // Near face (at min_range_, in +Z direction)
    glm::vec3(-near_width, -near_height, min_range_),  // 0: bottom-left
    glm::vec3( near_width, -near_height, min_range_),  // 1: bottom-right
    glm::vec3( near_width,  near_height, min_range_),  // 2: top-right
    glm::vec3(-near_width,  near_height, min_range_),  // 3: top-left
    
    // Far face (at max_range_, in +Z direction)
    glm::vec3(-far_width, -far_height, max_range_),    // 4: bottom-left
    glm::vec3( far_width, -far_height, max_range_),    // 5: bottom-right
    glm::vec3( far_width,  far_height, max_range_),    // 6: top-right
    glm::vec3(-far_width,  far_height, max_range_)     // 7: top-left
  };
  
  // Add vertices to surface
  for (const auto& vertex : frustum_vertices) {
    surface_vertices_.push_back(vertex);
    surface_normals_.push_back(glm::normalize(vertex));
    float t = (glm::length(vertex) - min_range_) / (max_range_ - min_range_);
    surface_colors_.push_back(glm::mix(near_color_, far_color_, t));
  }
  
  // Define frustum faces (triangles) - add indices directly
  std::vector<uint32_t> face_indices = {
    // Far face (2 triangles)
    4, 5, 6,  4, 6, 7,
    
    // Right face (2 triangles)  
    1, 5, 6,  1, 6, 2,
    
    // Left face (2 triangles)
    0, 4, 7,  0, 7, 3,
    
    // Top face (2 triangles)
    2, 6, 7,  2, 7, 3,
    
    // Bottom face (2 triangles)
    0, 1, 5,  0, 5, 4
  };
  
  // Add triangle indices
  for (uint32_t index : face_indices) {
    surface_indices_.push_back(index);
  }
}

void SensorCoverage::Generate3DSphere() {
  // Generate spherical coverage for omnidirectional sensors
  int rings = radial_segments_;
  int sectors = angular_segments_;
  
  for (int ring = 0; ring <= rings; ++ring) {
    float phi = M_PI * ring / rings;  // From 0 to π
    float cos_phi = std::cos(phi);
    float sin_phi = std::sin(phi);
    
    for (int sector = 0; sector <= sectors; ++sector) {
      float theta = 2.0f * M_PI * sector / sectors;  // From 0 to 2π
      float cos_theta = std::cos(theta);
      float sin_theta = std::sin(theta);
      
      // Sphere vertex at max range
      glm::vec3 vertex(
        max_range_ * sin_phi * cos_theta,
        max_range_ * sin_phi * sin_theta,
        max_range_ * cos_phi
      );
      
      glm::vec3 normal = glm::normalize(vertex);
      
      // Color gradient from center
      float t = 0.7f; // Most of sphere at far color
      glm::vec3 color = glm::mix(near_color_, far_color_, t);
      
      surface_vertices_.push_back(vertex);
      surface_normals_.push_back(normal);
      surface_colors_.push_back(color);
      
      // Create triangles
      if (ring > 0 && sector > 0) {
        uint32_t current = ring * (sectors + 1) + sector;
        uint32_t left = current - 1;
        uint32_t up = current - (sectors + 1);
        uint32_t up_left = up - 1;
        
        // First triangle
        surface_indices_.push_back(up_left);
        surface_indices_.push_back(up);
        surface_indices_.push_back(current);
        
        // Second triangle
        surface_indices_.push_back(up_left);
        surface_indices_.push_back(current);
        surface_indices_.push_back(left);
      }
    }
  }
}

void SensorCoverage::Generate3DCylinder() {
  // Simplified cylindrical coverage
  Generate2DRings();  // Use ring generation as base
  
  // Add vertical extent
  for (size_t i = 0; i < ring_vertices_.size(); ++i) {
    glm::vec3 top_vertex = ring_vertices_[i];
    top_vertex.z += vertical_fov_;  // Use vertical FOV as height
    
    ring_vertices_.push_back(top_vertex);
    ring_colors_.push_back(ring_colors_[i]);
    
    // Connect bottom to top
    ring_indices_.push_back(i);
    ring_indices_.push_back(ring_vertices_.size() - 1);
  }
}

void SensorCoverage::GenerateSectorSlice() {
  // Similar to 3D cone but limited angular range
  Generate3DCone();
}

void SensorCoverage::GenerateMultiBeam() {
  // Generate multiple beam patterns
  for (int beam = 0; beam < beam_count_; ++beam) {
    float beam_angle = beam_angles_.empty() ? 
      (horizontal_fov_ * beam / beam_count_ - horizontal_fov_ * 0.5f) : 
      beam_angles_[beam % beam_angles_.size()];
    
    // Simple beam as thin cone
    int segments = 16;
    for (int i = 0; i <= segments; ++i) {
      float angle = beam_angle + beam_width_ * (i - segments/2) / segments;
      
      glm::vec3 near_point(
        min_range_ * std::cos(angle),
        min_range_ * std::sin(angle),
        0.0f
      );
      
      glm::vec3 far_point(
        max_range_ * std::cos(angle),
        max_range_ * std::sin(angle),
        0.0f
      );
      
      uint32_t start_idx = ring_vertices_.size();
      ring_vertices_.push_back(near_point);
      ring_vertices_.push_back(far_point);
      ring_colors_.push_back(color_);
      ring_colors_.push_back(color_);
      
      ring_indices_.push_back(start_idx);
      ring_indices_.push_back(start_idx + 1);
    }
  }
}

void SensorCoverage::GenerateRangeRings() {
  // Range rings already generated in Generate2DRings for most cases
  if (coverage_type_ != CoverageType::k2DRings) {
    // Generate additional range indicators for 3D sensors
    for (int ring = 1; ring <= range_ring_count_; ++ring) {
      float range = min_range_ + (max_range_ - min_range_) * ring / range_ring_count_;
      
      // Create a simple circle at this range
      int segments = 32;
      for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        
        glm::vec3 vertex(
          range * std::cos(angle),
          range * std::sin(angle),
          0.0f
        );
        
        ring_vertices_.push_back(vertex);
        ring_colors_.push_back(outline_color_);
        
        if (i > 0) {
          ring_indices_.push_back(ring_vertices_.size() - 2);
          ring_indices_.push_back(ring_vertices_.size() - 1);
        }
      }
    }
  }
}

void SensorCoverage::UpdateTransformMatrix() {
  // Create transform matrix from sensor position and orientation
  transform_matrix_ = glm::mat4(1.0f);
  
  // Translation
  transform_matrix_ = glm::translate(transform_matrix_, sensor_position_);
  
  // Rotation to align with sensor direction
  glm::vec3 forward = sensor_direction_;
  glm::vec3 right = glm::normalize(glm::cross(forward, sensor_up_));
  glm::vec3 up = glm::cross(right, forward);
  
  glm::mat4 rotation_matrix = glm::mat4(
    right.x, right.y, right.z, 0.0f,
    up.x, up.y, up.z, 0.0f,
    -forward.x, -forward.y, -forward.z, 0.0f,  // Negative for right-handed system
    0.0f, 0.0f, 0.0f, 1.0f
  );
  
  transform_matrix_ = transform_matrix_ * rotation_matrix;
}

bool SensorCoverage::IsPointInCoverage(const glm::vec3& point) const {
  // Transform point to sensor local coordinates
  glm::vec3 local_point = point - sensor_position_;
  
  // Check distance
  float distance = glm::length(local_point);
  if (distance < min_range_ || distance > max_range_) {
    return false;
  }
  
  // Check angular coverage based on sensor type
  switch (coverage_type_) {
    case CoverageType::k3DSphere:
      return true;  // Omnidirectional
      
    case CoverageType::k2DRings:
    case CoverageType::kSectorSlice: {
      float angle = std::atan2(local_point.y, local_point.x);
      return (angle >= min_angle_ && angle <= max_angle_);
    }
    
    default:
      // For 3D cones, check both horizontal and vertical angles
      glm::vec3 direction = glm::normalize(local_point);
      float h_angle = std::atan2(direction.y, direction.x);
      float v_angle = std::asin(direction.z);
      
      return (std::abs(h_angle) <= horizontal_fov_ * 0.5f && 
              std::abs(v_angle) <= vertical_fov_ * 0.5f);
  }
}

glm::vec3 SensorCoverage::ComputeDetectionColor(float distance, float angle) const {
  float distance_factor = (distance - min_range_) / (max_range_ - min_range_);
  
  if (fade_with_distance_) {
    float fade_factor = 1.0f;
    if (distance_factor > fade_start_ratio_) {
      fade_factor = 1.0f - (distance_factor - fade_start_ratio_) / (1.0f - fade_start_ratio_);
    }
    
    glm::vec3 base_color = glm::mix(near_color_, far_color_, distance_factor);
    return base_color * fade_factor;
  }
  
  return glm::mix(near_color_, far_color_, distance_factor);
}

} // namespace quickviz