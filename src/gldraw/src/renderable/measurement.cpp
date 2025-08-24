/**
 * @file measurement.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-24
 * @brief Implementation of measurement renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/measurement.hpp"
#include "gldraw/renderable/text3d.hpp"

#include <glad/glad.h>
#include "gldraw/shader.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

namespace quickviz {

namespace {
const std::string kLineVertexShader = R"glsl(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uCoordTransform;

out vec3 fragColor;

void main() {
    gl_Position = uProjection * uView * uCoordTransform * vec4(aPos, 1.0);
    fragColor = aColor;
}
)glsl";

const std::string kLineFragmentShader = R"glsl(
#version 330 core
in vec3 fragColor;
out vec4 FragColor;

uniform float uAlpha;
uniform bool uGlowEnabled;
uniform float uGlowIntensity;

void main() {
    vec3 color = fragColor;
    if (uGlowEnabled) {
        color = color * (1.0 + uGlowIntensity);
    }
    FragColor = vec4(color, uAlpha);
}
)glsl";
}

Measurement::Measurement()
    : measurement_type_(MeasurementType::kDistance)
    , color_(glm::vec3(1.0f, 1.0f, 0.0f))
    , line_width_(2.0f)
    , line_style_(LineStyle::kSolid)
    , show_arrows_(true)
    , arrow_size_(0.1f)
    , show_extensions_(false)
    , extension_length_(0.5f)
    , highlighted_(false)
    , highlight_color_(glm::vec3(1.0f, 1.0f, 0.0f))
    , alpha_(1.0f)
    , glow_enabled_(false)
    , glow_intensity_(1.0f)
    , show_label_(true)
    , label_position_(LabelPosition::kCenter)
    , label_offset_(glm::vec3(0.0f))
    , label_color_(glm::vec3(0.0f, 0.0f, 0.0f))
    , label_bg_color_(glm::vec3(1.0f, 1.0f, 1.0f))
    , label_bg_alpha_(0.8f)
    , label_scale_(1.0f)
    , auto_update_(true)
    , precision_(2)
    , units_("m")
    , arc_radius_(1.0f)
    , arc_resolution_(32)
    , show_arc_ticks_(true)
    , arc_tick_count_(5)
    , main_vao_(0), main_vbo_(0), main_color_vbo_(0), main_ebo_(0)
    , arrow_vao_(0), arrow_vbo_(0), arrow_color_vbo_(0), arrow_ebo_(0)
    , needs_geometry_update_(true)
    , needs_label_update_(true) {
  
  label_text_obj_ = std::make_unique<Text3D>();
}

Measurement::Measurement(MeasurementType type) : Measurement() {
  measurement_type_ = type;
}

Measurement::~Measurement() {
  ReleaseGpuResources();
}

void Measurement::SetMeasurementType(MeasurementType type) {
  if (measurement_type_ != type) {
    measurement_type_ = type;
    needs_geometry_update_ = true;
    needs_label_update_ = true;
  }
}

void Measurement::SetPoints(const std::vector<glm::vec3>& points) {
  measurement_points_ = points;
  needs_geometry_update_ = true;
  if (auto_update_) {
    needs_label_update_ = true;
  }
}

void Measurement::SetTwoPointDistance(const glm::vec3& start, const glm::vec3& end) {
  measurement_type_ = MeasurementType::kDistance;
  measurement_points_ = {start, end};
  needs_geometry_update_ = true;
  if (auto_update_) {
    needs_label_update_ = true;
  }
}

void Measurement::SetThreePointAngle(const glm::vec3& vertex, const glm::vec3& point1, const glm::vec3& point2) {
  measurement_type_ = MeasurementType::kAngle;
  measurement_points_ = {vertex, point1, point2};
  needs_geometry_update_ = true;
  if (auto_update_) {
    needs_label_update_ = true;
  }
}

void Measurement::SetRadius(const glm::vec3& center, const glm::vec3& point_on_circle) {
  measurement_type_ = MeasurementType::kRadius;
  measurement_points_ = {center, point_on_circle};
  needs_geometry_update_ = true;
  if (auto_update_) {
    needs_label_update_ = true;
  }
}

void Measurement::SetDiameter(const glm::vec3& center, const glm::vec3& point1, const glm::vec3& point2) {
  measurement_type_ = MeasurementType::kDiameter;
  measurement_points_ = {center, point1, point2};
  needs_geometry_update_ = true;
  if (auto_update_) {
    needs_label_update_ = true;
  }
}

void Measurement::SetCoordinate(const glm::vec3& point, const glm::vec3& direction, float range) {
  measurement_type_ = MeasurementType::kCoordinate;
  measurement_points_ = {point, point + glm::normalize(direction) * range};
  needs_geometry_update_ = true;
  if (auto_update_) {
    needs_label_update_ = true;
  }
}

void Measurement::SetColor(const glm::vec3& color) {
  color_ = color;
  needs_geometry_update_ = true;
}

void Measurement::SetLineWidth(float width) {
  line_width_ = std::max(0.1f, width);
}

void Measurement::SetLineStyle(LineStyle style) {
  if (line_style_ != style) {
    line_style_ = style;
    needs_geometry_update_ = true;
  }
}

void Measurement::SetArrowStyle(bool show_arrows, float arrow_size) {
  if (show_arrows_ != show_arrows || arrow_size_ != arrow_size) {
    show_arrows_ = show_arrows;
    arrow_size_ = std::max(0.01f, arrow_size);
    needs_geometry_update_ = true;
  }
}

void Measurement::SetExtensionLines(bool show_extensions, float extension_length) {
  if (show_extensions_ != show_extensions || extension_length_ != extension_length) {
    show_extensions_ = show_extensions;
    extension_length_ = std::max(0.0f, extension_length);
    needs_geometry_update_ = true;
  }
}

void Measurement::SetShowLabel(bool show) {
  if (show_label_ != show) {
    show_label_ = show;
    needs_label_update_ = true;
  }
}

void Measurement::SetLabelText(const std::string& text) {
  if (label_text_ != text) {
    label_text_ = text;
    needs_label_update_ = true;
  }
}

void Measurement::SetLabelPosition(LabelPosition position) {
  if (label_position_ != position) {
    label_position_ = position;
    needs_label_update_ = true;
  }
}

void Measurement::SetLabelOffset(const glm::vec3& offset) {
  label_offset_ = offset;
  needs_label_update_ = true;
}

void Measurement::SetLabelScale(float scale) {
  label_scale_ = std::max(0.1f, scale);
  needs_label_update_ = true;
}

void Measurement::SetLabelColor(const glm::vec3& color) {
  label_color_ = color;
  needs_label_update_ = true;
}

void Measurement::SetLabelBackgroundColor(const glm::vec3& color, float alpha) {
  label_bg_color_ = color;
  label_bg_alpha_ = std::clamp(alpha, 0.0f, 1.0f);
  needs_label_update_ = true;
}

void Measurement::SetPrecision(int decimal_places) {
  precision_ = std::max(0, std::min(6, decimal_places));
  if (auto_update_) {
    needs_label_update_ = true;
  }
}

void Measurement::SetUnits(const std::string& unit_string) {
  units_ = unit_string;
  if (auto_update_) {
    needs_label_update_ = true;
  }
}

void Measurement::SetAutoUpdate(bool auto_update) {
  auto_update_ = auto_update;
}

void Measurement::SetArcRadius(float radius) {
  arc_radius_ = std::max(0.1f, radius);
  if (measurement_type_ == MeasurementType::kAngle) {
    needs_geometry_update_ = true;
  }
}

void Measurement::SetArcResolution(int segments) {
  arc_resolution_ = std::max(8, std::min(128, segments));
  if (measurement_type_ == MeasurementType::kAngle) {
    needs_geometry_update_ = true;
  }
}

void Measurement::SetShowArcTicks(bool show_ticks, int tick_count) {
  show_arc_ticks_ = show_ticks;
  arc_tick_count_ = std::max(2, std::min(20, tick_count));
  if (measurement_type_ == MeasurementType::kAngle) {
    needs_geometry_update_ = true;
  }
}

void Measurement::SetHighlight(bool highlighted, const glm::vec3& highlight_color) {
  highlighted_ = highlighted;
  highlight_color_ = highlight_color;
  needs_geometry_update_ = true;
}

void Measurement::SetTransparency(float alpha) {
  alpha_ = std::clamp(alpha, 0.0f, 1.0f);
}

void Measurement::SetGlow(bool enable_glow, float intensity) {
  glow_enabled_ = enable_glow;
  glow_intensity_ = std::max(0.0f, intensity);
}

std::string Measurement::GetLabelText() const {
  if (!label_text_.empty()) {
    return label_text_;
  }
  
  // Auto-generate label based on measurement type
  if (auto_update_) {
    switch (measurement_type_) {
      case MeasurementType::kDistance:
        return FormatValue(GetDistanceValue()) + " " + units_;
      case MeasurementType::kAngle:
        return FormatValue(GetAngleValueDegrees()) + "°";
      case MeasurementType::kRadius:
        return "R " + FormatValue(GetDistanceValue()) + " " + units_;
      case MeasurementType::kDiameter:
        return "Ø " + FormatValue(GetDistanceValue()) + " " + units_;
      default:
        return FormatValue(GetDistanceValue()) + " " + units_;
    }
  }
  
  return "";
}

float Measurement::GetDistanceValue() const {
  if (measurement_points_.size() < 2) return 0.0f;
  
  switch (measurement_type_) {
    case MeasurementType::kDistance:
    case MeasurementType::kRadius:
    case MeasurementType::kCoordinate:
      return glm::length(measurement_points_[1] - measurement_points_[0]);
    
    case MeasurementType::kDiameter:
      if (measurement_points_.size() >= 3) {
        return glm::length(measurement_points_[2] - measurement_points_[1]);
      }
      return 0.0f;
    
    case MeasurementType::kMultiSegment: {
      float total_distance = 0.0f;
      for (size_t i = 1; i < measurement_points_.size(); ++i) {
        total_distance += glm::length(measurement_points_[i] - measurement_points_[i-1]);
      }
      return total_distance;
    }
    
    default:
      return 0.0f;
  }
}

float Measurement::GetAngleValue() const {
  if (measurement_type_ != MeasurementType::kAngle || measurement_points_.size() < 3) {
    return 0.0f;
  }
  
  glm::vec3 vertex = measurement_points_[0];
  glm::vec3 v1 = glm::normalize(measurement_points_[1] - vertex);
  glm::vec3 v2 = glm::normalize(measurement_points_[2] - vertex);
  
  float dot_product = glm::clamp(glm::dot(v1, v2), -1.0f, 1.0f);
  return std::acos(dot_product);
}

float Measurement::GetAngleValueDegrees() const {
  return GetAngleValue() * 180.0f / M_PI;
}

void Measurement::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;
  
  // Initialize shaders
  try {
    Shader line_vs(kLineVertexShader.c_str(), Shader::Type::kVertex);
    Shader line_fs(kLineFragmentShader.c_str(), Shader::Type::kFragment);
    if (!line_vs.Compile() || !line_fs.Compile()) {
      throw std::runtime_error("Line shader compilation failed");
    }
    line_shader_.AttachShader(line_vs);
    line_shader_.AttachShader(line_fs);
    if (!line_shader_.LinkProgram()) {
      throw std::runtime_error("Line shader linking failed");
    }
    
    Shader arrow_vs(kLineVertexShader.c_str(), Shader::Type::kVertex);
    Shader arrow_fs(kLineFragmentShader.c_str(), Shader::Type::kFragment);
    if (!arrow_vs.Compile() || !arrow_fs.Compile()) {
      throw std::runtime_error("Arrow shader compilation failed");
    }
    arrow_shader_.AttachShader(arrow_vs);
    arrow_shader_.AttachShader(arrow_fs);
    if (!arrow_shader_.LinkProgram()) {
      throw std::runtime_error("Arrow shader linking failed");
    }
  } catch (const std::exception& e) {
    std::cerr << "Measurement::AllocateGpuResources: " << e.what() << std::endl;
    throw;
  }
  
  // Generate main line VAO and buffers
  glGenVertexArrays(1, &main_vao_);
  glGenBuffers(1, &main_vbo_);
  glGenBuffers(1, &main_color_vbo_);
  glGenBuffers(1, &main_ebo_);
  
  glBindVertexArray(main_vao_);
  
  // Vertex positions
  glBindBuffer(GL_ARRAY_BUFFER, main_vbo_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glEnableVertexAttribArray(0);
  
  // Vertex colors
  glBindBuffer(GL_ARRAY_BUFFER, main_color_vbo_);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glEnableVertexAttribArray(1);
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, main_ebo_);
  
  // Generate arrow VAO and buffers
  glGenVertexArrays(1, &arrow_vao_);
  glGenBuffers(1, &arrow_vbo_);
  glGenBuffers(1, &arrow_color_vbo_);
  glGenBuffers(1, &arrow_ebo_);
  
  glBindVertexArray(arrow_vao_);
  
  // Arrow vertex positions
  glBindBuffer(GL_ARRAY_BUFFER, arrow_vbo_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glEnableVertexAttribArray(0);
  
  // Arrow vertex colors
  glBindBuffer(GL_ARRAY_BUFFER, arrow_color_vbo_);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glEnableVertexAttribArray(1);
  
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, arrow_ebo_);
  
  glBindVertexArray(0);
  
  // Initialize label
  if (label_text_obj_) {
    label_text_obj_->AllocateGpuResources();
  }
  
  needs_geometry_update_ = true;
  needs_label_update_ = true;
}

void Measurement::ReleaseGpuResources() noexcept {
  if (main_vao_ != 0) {
    glDeleteVertexArrays(1, &main_vao_);
    glDeleteBuffers(1, &main_vbo_);
    glDeleteBuffers(1, &main_color_vbo_);
    glDeleteBuffers(1, &main_ebo_);
    main_vao_ = main_vbo_ = main_color_vbo_ = main_ebo_ = 0;
  }
  
  if (arrow_vao_ != 0) {
    glDeleteVertexArrays(1, &arrow_vao_);
    glDeleteBuffers(1, &arrow_vbo_);
    glDeleteBuffers(1, &arrow_color_vbo_);
    glDeleteBuffers(1, &arrow_ebo_);
    arrow_vao_ = arrow_vbo_ = arrow_color_vbo_ = arrow_ebo_ = 0;
  }
  
  if (label_text_obj_) {
    label_text_obj_->ReleaseGpuResources();
  }
}

void Measurement::OnDraw(const glm::mat4& projection, const glm::mat4& view, 
                        const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }
  
  // Update geometry if needed
  if (needs_geometry_update_) {
    UpdateGeometry();
    needs_geometry_update_ = false;
  }
  
  // Update label if needed
  if (needs_label_update_) {
    UpdateLabel();
    needs_label_update_ = false;
  }
  
  // Enable line width
  glLineWidth(line_width_);
  
  // Enable blending for transparency
  if (alpha_ < 1.0f) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }
  
  // Draw main lines
  if (!line_vertices_.empty()) {
    line_shader_.Use();
    line_shader_.SetUniform("uProjection", projection);
    line_shader_.SetUniform("uView", view);
    line_shader_.SetUniform("uCoordTransform", coord_transform);
    line_shader_.SetUniform("uAlpha", alpha_);
    line_shader_.SetUniform("uGlowEnabled", glow_enabled_);
    line_shader_.SetUniform("uGlowIntensity", glow_intensity_);
    
    glBindVertexArray(main_vao_);
    
    // Update buffers
    glBindBuffer(GL_ARRAY_BUFFER, main_vbo_);
    glBufferData(GL_ARRAY_BUFFER, line_vertices_.size() * sizeof(glm::vec3),
                 line_vertices_.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ARRAY_BUFFER, main_color_vbo_);
    glBufferData(GL_ARRAY_BUFFER, line_colors_.size() * sizeof(glm::vec3),
                 line_colors_.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, main_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, line_indices_.size() * sizeof(uint32_t),
                 line_indices_.data(), GL_DYNAMIC_DRAW);
    
    glDrawElements(GL_LINES, line_indices_.size(), GL_UNSIGNED_INT, nullptr);
  }
  
  // Draw arrows
  if (show_arrows_ && !arrow_vertices_.empty()) {
    arrow_shader_.Use();
    arrow_shader_.SetUniform("uProjection", projection);
    arrow_shader_.SetUniform("uView", view);
    arrow_shader_.SetUniform("uCoordTransform", coord_transform);
    arrow_shader_.SetUniform("uAlpha", alpha_);
    arrow_shader_.SetUniform("uGlowEnabled", glow_enabled_);
    arrow_shader_.SetUniform("uGlowIntensity", glow_intensity_);
    
    glBindVertexArray(arrow_vao_);
    
    // Update arrow buffers
    glBindBuffer(GL_ARRAY_BUFFER, arrow_vbo_);
    glBufferData(GL_ARRAY_BUFFER, arrow_vertices_.size() * sizeof(glm::vec3),
                 arrow_vertices_.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ARRAY_BUFFER, arrow_color_vbo_);
    glBufferData(GL_ARRAY_BUFFER, arrow_colors_.size() * sizeof(glm::vec3),
                 arrow_colors_.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, arrow_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, arrow_indices_.size() * sizeof(uint32_t),
                 arrow_indices_.data(), GL_DYNAMIC_DRAW);
    
    glDrawElements(GL_TRIANGLES, arrow_indices_.size(), GL_UNSIGNED_INT, nullptr);
  }
  
  // Draw label
  if (show_label_ && label_text_obj_) {
    label_text_obj_->OnDraw(projection, view, coord_transform);
  }
  
  glBindVertexArray(0);
  
  // Restore line width
  glLineWidth(1.0f);
  
  if (alpha_ < 1.0f) {
    glDisable(GL_BLEND);
  }
}

void Measurement::UpdateGeometry() {
  // Clear previous geometry
  line_vertices_.clear();
  line_colors_.clear();
  line_indices_.clear();
  arrow_vertices_.clear();
  arrow_colors_.clear();
  arrow_indices_.clear();
  
  if (measurement_points_.empty()) return;
  
  // Generate geometry based on measurement type
  switch (measurement_type_) {
    case MeasurementType::kDistance:
      GenerateDistanceGeometry();
      break;
    case MeasurementType::kAngle:
      GenerateAngleGeometry();
      break;
    case MeasurementType::kRadius:
      GenerateRadiusGeometry();
      break;
    case MeasurementType::kDiameter:
      GenerateDiameterGeometry();
      break;
    case MeasurementType::kCoordinate:
      GenerateCoordinateGeometry();
      break;
    case MeasurementType::kMultiSegment:
      GenerateMultiSegmentGeometry();
      break;
  }
  
  // Generate arrows if enabled (but not for angle measurements)
  if (show_arrows_ && measurement_type_ != MeasurementType::kAngle) {
    GenerateArrows();
  }
  
  // Generate extension lines if enabled
  if (show_extensions_) {
    GenerateExtensionLines();
  }
}

void Measurement::GenerateDistanceGeometry() {
  if (measurement_points_.size() < 2) return;
  
  glm::vec3 start = measurement_points_[0];
  glm::vec3 end = measurement_points_[1];
  
  GenerateLineWithStyle(start, end, line_vertices_, line_indices_);
  
  // Add colors
  glm::vec3 color = highlighted_ ? highlight_color_ : color_;
  for (size_t i = line_colors_.size(); i < line_vertices_.size(); ++i) {
    line_colors_.push_back(color);
  }
}

void Measurement::GenerateAngleGeometry() {
  if (measurement_points_.size() < 3) return;
  
  glm::vec3 vertex = measurement_points_[0];
  glm::vec3 point1 = measurement_points_[1];
  glm::vec3 point2 = measurement_points_[2];
  
  glm::vec3 v1 = glm::normalize(point1 - vertex);
  glm::vec3 v2 = glm::normalize(point2 - vertex);
  
  float angle = GetAngleValue();
  glm::vec3 color = highlighted_ ? highlight_color_ : color_;
  
  // Generate arc
  uint32_t start_index = line_vertices_.size();
  
  // Create arc by interpolating between the two vectors
  for (int i = 0; i <= arc_resolution_; ++i) {
    float t = static_cast<float>(i) / arc_resolution_;
    float current_angle = angle * t;
    
    // Rotate v1 towards v2
    glm::vec3 axis = glm::cross(v1, v2);
    if (glm::length(axis) < 1e-6f) {
      axis = glm::vec3(0, 0, 1); // Fallback axis for parallel vectors
    } else {
      axis = glm::normalize(axis);
    }
    
    float cos_a = std::cos(current_angle);
    float sin_a = std::sin(current_angle);
    glm::mat3 rotation_matrix = cos_a * glm::mat3(1.0f) + 
                               sin_a * glm::mat3(0, axis.z, -axis.y, -axis.z, 0, axis.x, axis.y, -axis.x, 0) +
                               (1 - cos_a) * glm::outerProduct(axis, axis);
    
    glm::vec3 arc_point = vertex + (rotation_matrix * v1) * arc_radius_;
    line_vertices_.push_back(arc_point);
    line_colors_.push_back(color);
    
    if (i > 0) {
      line_indices_.push_back(start_index + i - 1);
      line_indices_.push_back(start_index + i);
    }
  }
  
  // Add construction lines from vertex to arc endpoints (no arrows)
  if (arc_radius_ > 0.1f) {
    // Line from vertex to arc start (along v1 direction)
    glm::vec3 arc_start = vertex + v1 * arc_radius_;
    uint32_t line_start = line_vertices_.size();
    line_vertices_.push_back(vertex);
    line_vertices_.push_back(arc_start);
    line_colors_.push_back(color);
    line_colors_.push_back(color);
    line_indices_.push_back(line_start);
    line_indices_.push_back(line_start + 1);
    
    // Line from vertex to arc end (along v2 direction)
    glm::vec3 arc_end = vertex + v2 * arc_radius_;
    line_start = line_vertices_.size();
    line_vertices_.push_back(vertex);
    line_vertices_.push_back(arc_end);
    line_colors_.push_back(color);
    line_colors_.push_back(color);
    line_indices_.push_back(line_start);
    line_indices_.push_back(line_start + 1);
  }
  
  // Add tick marks if enabled
  if (show_arc_ticks_ && arc_tick_count_ > 2) {
    // Distribute ticks evenly across the angle, including endpoints
    for (int tick = 0; tick < arc_tick_count_; ++tick) {
      float t = static_cast<float>(tick) / (arc_tick_count_ - 1);
      float tick_angle = angle * t;
      
      glm::vec3 axis = glm::normalize(glm::cross(v1, v2));
      float cos_a = std::cos(tick_angle);
      float sin_a = std::sin(tick_angle);
      glm::mat3 rotation_matrix = cos_a * glm::mat3(1.0f) + 
                                 sin_a * glm::mat3(0, axis.z, -axis.y, -axis.z, 0, axis.x, axis.y, -axis.x, 0) +
                                 (1 - cos_a) * glm::outerProduct(axis, axis);
      
      glm::vec3 tick_dir = rotation_matrix * v1;
      glm::vec3 tick_start = vertex + tick_dir * (arc_radius_ * 0.95f);
      glm::vec3 tick_end = vertex + tick_dir * (arc_radius_ * 1.05f);
      
      uint32_t tick_start_idx = line_vertices_.size();
      line_vertices_.push_back(tick_start);
      line_vertices_.push_back(tick_end);
      line_colors_.push_back(color);
      line_colors_.push_back(color);
      line_indices_.push_back(tick_start_idx);
      line_indices_.push_back(tick_start_idx + 1);
    }
  }
}

void Measurement::GenerateRadiusGeometry() {
  if (measurement_points_.size() < 2) return;
  
  glm::vec3 center = measurement_points_[0];
  glm::vec3 point = measurement_points_[1];
  
  GenerateLineWithStyle(center, point, line_vertices_, line_indices_);
  
  // Add colors
  glm::vec3 color = highlighted_ ? highlight_color_ : color_;
  for (size_t i = line_colors_.size(); i < line_vertices_.size(); ++i) {
    line_colors_.push_back(color);
  }
}

void Measurement::GenerateDiameterGeometry() {
  if (measurement_points_.size() < 3) return;
  
  glm::vec3 point1 = measurement_points_[1];
  glm::vec3 point2 = measurement_points_[2];
  
  GenerateLineWithStyle(point1, point2, line_vertices_, line_indices_);
  
  // Add colors
  glm::vec3 color = highlighted_ ? highlight_color_ : color_;
  for (size_t i = line_colors_.size(); i < line_vertices_.size(); ++i) {
    line_colors_.push_back(color);
  }
}

void Measurement::GenerateCoordinateGeometry() {
  GenerateDistanceGeometry(); // Same as distance for basic implementation
}

void Measurement::GenerateMultiSegmentGeometry() {
  if (measurement_points_.size() < 2) return;
  
  glm::vec3 color = highlighted_ ? highlight_color_ : color_;
  
  for (size_t i = 1; i < measurement_points_.size(); ++i) {
    glm::vec3 start = measurement_points_[i-1];
    glm::vec3 end = measurement_points_[i];
    
    GenerateLineWithStyle(start, end, line_vertices_, line_indices_);
    
    // Add colors for this segment
    line_colors_.push_back(color);
    line_colors_.push_back(color);
  }
}

void Measurement::GenerateArrows() {
  if (measurement_points_.size() < 2) return;
  
  glm::vec3 color = highlighted_ ? highlight_color_ : color_;
  
  for (size_t i = 1; i < measurement_points_.size(); ++i) {
    glm::vec3 start = measurement_points_[i-1];
    glm::vec3 end = measurement_points_[i];
    glm::vec3 direction = glm::normalize(end - start);
    
    // Create simple arrow head at the end point
    glm::vec3 perpendicular;
    if (std::abs(direction.y) < 0.9f) {
      perpendicular = glm::normalize(glm::cross(direction, glm::vec3(0, 1, 0)));
    } else {
      perpendicular = glm::normalize(glm::cross(direction, glm::vec3(1, 0, 0)));
    }
    
    glm::vec3 up = glm::cross(perpendicular, direction);
    
    // Arrow head vertices
    glm::vec3 arrow_base = end - direction * arrow_size_;
    glm::vec3 arrow_left = arrow_base + perpendicular * arrow_size_ * 0.5f;
    glm::vec3 arrow_right = arrow_base - perpendicular * arrow_size_ * 0.5f;
    glm::vec3 arrow_up = arrow_base + up * arrow_size_ * 0.5f;
    glm::vec3 arrow_down = arrow_base - up * arrow_size_ * 0.5f;
    
    uint32_t base_idx = arrow_vertices_.size();
    
    // Add arrow vertices
    arrow_vertices_.insert(arrow_vertices_.end(), {end, arrow_left, arrow_right, arrow_up, arrow_down});
    
    // Add colors
    for (int j = 0; j < 5; ++j) {
      arrow_colors_.push_back(color);
    }
    
    // Add arrow triangles
    arrow_indices_.insert(arrow_indices_.end(), {
      base_idx, base_idx + 1, base_idx + 2,  // Left-right triangle
      base_idx, base_idx + 3, base_idx + 4   // Up-down triangle
    });
  }
}

void Measurement::GenerateExtensionLines() {
  // Implementation for extension lines - simplified for now
  // This would add perpendicular lines at measurement endpoints
}

void Measurement::GenerateLineWithStyle(const glm::vec3& start, const glm::vec3& end, 
                                       std::vector<glm::vec3>& vertices, std::vector<uint32_t>& indices) {
  uint32_t start_idx = vertices.size();
  
  switch (line_style_) {
    case LineStyle::kSolid:
      vertices.push_back(start);
      vertices.push_back(end);
      indices.push_back(start_idx);
      indices.push_back(start_idx + 1);
      break;
      
    case LineStyle::kDashed: {
      glm::vec3 direction = end - start;
      float total_length = glm::length(direction);
      glm::vec3 unit_dir = direction / total_length;
      
      float dash_length = 0.1f;
      float gap_length = 0.05f;
      float segment_length = dash_length + gap_length;
      
      int num_segments = static_cast<int>(total_length / segment_length);
      
      for (int i = 0; i < num_segments; ++i) {
        float t1 = i * segment_length;
        float t2 = std::min(t1 + dash_length, total_length);
        
        if (t2 > t1) {
          glm::vec3 dash_start = start + unit_dir * t1;
          glm::vec3 dash_end = start + unit_dir * t2;
          
          uint32_t dash_start_idx = vertices.size();
          vertices.push_back(dash_start);
          vertices.push_back(dash_end);
          indices.push_back(dash_start_idx);
          indices.push_back(dash_start_idx + 1);
        }
      }
      break;
    }
    
    default:
      // Fallback to solid line
      vertices.push_back(start);
      vertices.push_back(end);
      indices.push_back(start_idx);
      indices.push_back(start_idx + 1);
      break;
  }
}

glm::vec3 Measurement::ComputeLabelPosition() {
  if (measurement_points_.empty()) return glm::vec3(0.0f);
  
  glm::vec3 position;
  
  switch (label_position_) {
    case LabelPosition::kCenter: {
      // Compute centroid of all measurement points
      glm::vec3 centroid(0.0f);
      for (const auto& point : measurement_points_) {
        centroid += point;
      }
      position = centroid / static_cast<float>(measurement_points_.size());
      break;
    }
    
    case LabelPosition::kAbove:
    case LabelPosition::kBelow: {
      // Position above or below the midpoint
      if (measurement_points_.size() >= 2) {
        glm::vec3 midpoint = (measurement_points_[0] + measurement_points_[1]) * 0.5f;
        glm::vec3 direction = glm::normalize(measurement_points_[1] - measurement_points_[0]);
        glm::vec3 perpendicular = glm::cross(direction, glm::vec3(0, 0, 1));
        if (glm::length(perpendicular) < 1e-6f) {
          perpendicular = glm::cross(direction, glm::vec3(0, 1, 0));
        }
        perpendicular = glm::normalize(perpendicular);
        
        float offset_distance = 0.2f;
        if (label_position_ == LabelPosition::kBelow) {
          offset_distance = -offset_distance;
        }
        
        position = midpoint + perpendicular * offset_distance;
      } else {
        position = measurement_points_[0];
      }
      break;
    }
    
    case LabelPosition::kStart:
      position = measurement_points_[0];
      break;
      
    case LabelPosition::kEnd:
      position = measurement_points_.back();
      break;
      
    case LabelPosition::kCustom:
    default:
      // Use first point as fallback
      position = measurement_points_[0];
      break;
  }
  
  return position + label_offset_;
}

void Measurement::UpdateLabel() {
  if (!label_text_obj_ || !show_label_) return;
  
  // Set label text
  std::string text = GetLabelText();
  if (!text.empty()) {
    label_text_obj_->SetText(text);
    label_text_obj_->SetPosition(ComputeLabelPosition());
    label_text_obj_->SetColor(label_color_);
    label_text_obj_->SetScale(label_scale_);
    label_text_obj_->SetAlignment(Text3D::Alignment::kCenter, Text3D::VerticalAlignment::kMiddle);
  }
}

std::string Measurement::FormatValue(float value) const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision_) << value;
  return oss.str();
}

} // namespace quickviz