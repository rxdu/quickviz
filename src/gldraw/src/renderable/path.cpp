/**
 * @file path.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Implementation of path renderer for trajectory visualization
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/path.hpp"

#include <iostream>
#include <algorithm>
#include <cmath>

#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "gldraw/shader.hpp"

namespace quickviz {

namespace {

const char* kPathVertexShader = R"(
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

const char* kPathFragmentShader = R"(
#version 330 core
in vec3 vertexColor;
out vec4 FragColor;

uniform float alpha;
uniform bool glowEnabled;
uniform float glowIntensity;

void main() {
    vec3 color = vertexColor;
    
    if (glowEnabled) {
        // Simple glow effect by brightening the color
        color = color * (1.0 + glowIntensity * 0.5);
    }
    
    FragColor = vec4(color, alpha);
}
)";

const char* kArrowVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 vertexColor;

uniform mat4 mvp;
uniform mat4 coordTransform;
uniform mat4 arrowTransform;

void main() {
    vertexColor = aColor;
    gl_Position = mvp * coordTransform * arrowTransform * vec4(aPos, 1.0);
}
)";

const char* kArrowFragmentShader = R"(
#version 330 core
in vec3 vertexColor;
out vec4 FragColor;

uniform float alpha;

void main() {
    FragColor = vec4(vertexColor, alpha);
}
)";

} // namespace

Path::Path()
    : path_type_(PathType::kLineSegments)
    , line_width_(2.0f)
    , subdivisions_(20)
    , tension_(0.5f)
    , color_mode_(ColorMode::kUniform)
    , base_color_(0.0f, 0.8f, 1.0f)  // Cyan
    , gradient_start_(0.0f, 1.0f, 0.0f)  // Green
    , gradient_end_(1.0f, 0.0f, 0.0f)    // Red
    , color_range_(-1.0f, 1.0f)
    , arrow_mode_(ArrowMode::kNone)
    , arrow_size_(0.2f)
    , arrow_spacing_(1.0f)
    , arrow_color_(1.0f, 1.0f, 0.0f)  // Yellow
    , animation_progress_(1.0f)
    , glow_enabled_(false)
    , glow_intensity_(1.0f)
    , alpha_(1.0f)
    , path_vao_(0), path_vbo_(0), path_color_vbo_(0), path_ebo_(0)
    , arrow_vao_(0), arrow_vbo_(0), arrow_color_vbo_(0), arrow_ebo_(0)
    , needs_geometry_update_(true)
    , needs_color_update_(true)
    , needs_arrow_update_(true) {
}

Path::Path(const std::vector<glm::vec3>& points) : Path() {
  SetPoints(points);
}

Path::~Path() {
  if (IsGpuResourcesAllocated()) {
    ReleaseGpuResources();
  }
}

void Path::SetPoints(const std::vector<glm::vec3>& points) {
  control_points_ = points;
  needs_geometry_update_ = true;
  needs_color_update_ = true;
  needs_arrow_update_ = true;
}

void Path::AddPoint(const glm::vec3& point) {
  control_points_.push_back(point);
  needs_geometry_update_ = true;
  needs_color_update_ = true;
  needs_arrow_update_ = true;
}

void Path::InsertPoint(size_t index, const glm::vec3& point) {
  if (index <= control_points_.size()) {
    control_points_.insert(control_points_.begin() + index, point);
    needs_geometry_update_ = true;
    needs_color_update_ = true;
    needs_arrow_update_ = true;
  }
}

void Path::RemovePoint(size_t index) {
  if (index < control_points_.size()) {
    control_points_.erase(control_points_.begin() + index);
    needs_geometry_update_ = true;
    needs_color_update_ = true;
    needs_arrow_update_ = true;
  }
}

void Path::ClearPath() {
  control_points_.clear();
  path_vertices_.clear();
  path_colors_.clear();
  path_indices_.clear();
  needs_geometry_update_ = true;
  needs_color_update_ = true;
  needs_arrow_update_ = true;
}

void Path::SetPathType(PathType type) {
  if (path_type_ != type) {
    path_type_ = type;
    needs_geometry_update_ = true;
  }
}

void Path::SetLineWidth(float width) {
  if (width > 0.0f) {
    line_width_ = width;
  }
}

void Path::SetSubdivisions(int subdivisions) {
  if (subdivisions > 1 && subdivisions != subdivisions_) {
    subdivisions_ = subdivisions;
    needs_geometry_update_ = true;
  }
}

void Path::SetTension(float tension) {
  tension_ = glm::clamp(tension, 0.0f, 1.0f);
  if (path_type_ == PathType::kSpline) {
    needs_geometry_update_ = true;
  }
}

void Path::SetColorMode(ColorMode mode) {
  if (color_mode_ != mode) {
    color_mode_ = mode;
    needs_color_update_ = true;
  }
}

void Path::SetColor(const glm::vec3& color) {
  base_color_ = color;
  if (color_mode_ == ColorMode::kUniform) {
    needs_color_update_ = true;
  }
}

void Path::SetColorGradient(const glm::vec3& start_color, const glm::vec3& end_color) {
  gradient_start_ = start_color;
  gradient_end_ = end_color;
  if (color_mode_ == ColorMode::kGradient) {
    needs_color_update_ = true;
  }
}

void Path::SetColors(const std::vector<glm::vec3>& colors) {
  custom_colors_ = colors;
  if (color_mode_ == ColorMode::kCustom) {
    needs_color_update_ = true;
  }
}

void Path::SetColorRange(const glm::vec2& range) {
  color_range_ = range;
  if (color_mode_ == ColorMode::kVelocity || color_mode_ == ColorMode::kTime || color_mode_ == ColorMode::kCost) {
    needs_color_update_ = true;
  }
}

void Path::SetScalarValues(const std::vector<float>& values) {
  scalar_values_ = values;
  if (color_mode_ == ColorMode::kVelocity || color_mode_ == ColorMode::kTime || color_mode_ == ColorMode::kCost) {
    needs_color_update_ = true;
  }
}

void Path::SetArrowMode(ArrowMode mode) {
  if (arrow_mode_ != mode) {
    arrow_mode_ = mode;
    needs_arrow_update_ = true;
  }
}

void Path::SetArrowSize(float size) {
  if (size > 0.0f && size != arrow_size_) {
    arrow_size_ = size;
    needs_arrow_update_ = true;
  }
}

void Path::SetArrowSpacing(float spacing) {
  if (spacing > 0.0f && spacing != arrow_spacing_) {
    arrow_spacing_ = spacing;
    needs_arrow_update_ = true;
  }
}

void Path::SetArrowColor(const glm::vec3& color) {
  arrow_color_ = color;
  needs_arrow_update_ = true;
}

void Path::SetAnimationProgress(float progress) {
  animation_progress_ = glm::clamp(progress, 0.0f, 1.0f);
  needs_geometry_update_ = true;
}

void Path::SetGlowEffect(bool enable, float intensity) {
  glow_enabled_ = enable;
  glow_intensity_ = intensity;
}

void Path::SetTransparency(float alpha) {
  alpha_ = glm::clamp(alpha, 0.0f, 1.0f);
}

void Path::GeneratePathGeometry() {
  path_vertices_.clear();
  path_indices_.clear();

  if (control_points_.size() < 2) {
    return;
  }

  switch (path_type_) {
    case PathType::kLineSegments:
      GenerateLineSegments();
      break;
    case PathType::kSmoothCurve:
      GenerateSmoothCurve();
      break;
    case PathType::kBezierCurve:
      GenerateBezierCurve();
      break;
    case PathType::kSpline:
      GenerateSplineCurve();
      break;
  }

  needs_geometry_update_ = false;
  needs_color_update_ = true;
}

void Path::GenerateLineSegments() {
  // Simple line segments connecting control points
  size_t end_index = static_cast<size_t>(control_points_.size() * animation_progress_);
  end_index = std::min(end_index, control_points_.size());

  for (size_t i = 0; i < end_index; ++i) {
    path_vertices_.push_back(control_points_[i]);
  }

  // Generate line indices
  for (size_t i = 0; i < path_vertices_.size() - 1; ++i) {
    path_indices_.push_back(static_cast<uint32_t>(i));
    path_indices_.push_back(static_cast<uint32_t>(i + 1));
  }
}

void Path::GenerateSmoothCurve() {
  if (control_points_.size() < 3) {
    GenerateLineSegments();
    return;
  }

  // Simple smooth interpolation between points using subdivision
  float total_segments = static_cast<float>((control_points_.size() - 1) * subdivisions_);
  size_t end_segments = static_cast<size_t>(total_segments * animation_progress_);

  for (size_t seg = 0; seg <= end_segments; ++seg) {
    float t = static_cast<float>(seg) / total_segments * (control_points_.size() - 1);
    size_t idx = static_cast<size_t>(t);
    float local_t = t - idx;

    if (idx >= control_points_.size() - 1) {
      path_vertices_.push_back(control_points_.back());
      break;
    }

    // Linear interpolation between control points
    glm::vec3 point = glm::mix(control_points_[idx], control_points_[idx + 1], local_t);
    path_vertices_.push_back(point);
  }

  // Generate line indices
  for (size_t i = 0; i < path_vertices_.size() - 1; ++i) {
    path_indices_.push_back(static_cast<uint32_t>(i));
    path_indices_.push_back(static_cast<uint32_t>(i + 1));
  }
}

void Path::GenerateBezierCurve() {
  if (control_points_.size() < 2) {
    return;
  }

  // Simple quadratic Bezier for now - could be extended to cubic
  size_t total_points = subdivisions_ + 1;
  size_t end_points = static_cast<size_t>(total_points * animation_progress_);

  for (size_t i = 0; i <= end_points; ++i) {
    float t = static_cast<float>(i) / subdivisions_;
    glm::vec3 point = InterpolateBezier(control_points_, t);
    path_vertices_.push_back(point);
  }

  // Generate line indices
  for (size_t i = 0; i < path_vertices_.size() - 1; ++i) {
    path_indices_.push_back(static_cast<uint32_t>(i));
    path_indices_.push_back(static_cast<uint32_t>(i + 1));
  }
}

void Path::GenerateSplineCurve() {
  if (control_points_.size() < 3) {
    GenerateLineSegments();
    return;
  }

  // Catmull-Rom spline interpolation
  size_t total_segments = (control_points_.size() - 1) * subdivisions_;
  size_t end_segments = static_cast<size_t>(total_segments * animation_progress_);

  for (size_t seg = 0; seg <= end_segments; ++seg) {
    float t = static_cast<float>(seg) / total_segments;
    glm::vec3 point = InterpolateSpline(control_points_, t);
    path_vertices_.push_back(point);
  }

  // Generate line indices
  for (size_t i = 0; i < path_vertices_.size() - 1; ++i) {
    path_indices_.push_back(static_cast<uint32_t>(i));
    path_indices_.push_back(static_cast<uint32_t>(i + 1));
  }
}

void Path::GenerateArrows() {
  arrow_vertices_.clear();
  arrow_colors_.clear();
  arrow_indices_.clear();

  if (arrow_mode_ == ArrowMode::kNone || path_vertices_.size() < 2) {
    return;
  }

  std::vector<size_t> arrow_positions;

  switch (arrow_mode_) {
    case ArrowMode::kEndpoints:
      if (!path_vertices_.empty()) {
        arrow_positions.push_back(0);
        arrow_positions.push_back(path_vertices_.size() - 1);
      }
      break;
    
    case ArrowMode::kRegular: {
      float path_length = GetTotalLength();
      if (path_length > 0.0f) {
        for (float dist = 0.0f; dist <= path_length; dist += arrow_spacing_) {
          // Find position index for this distance - simplified
          size_t pos = static_cast<size_t>((dist / path_length) * (path_vertices_.size() - 1));
          arrow_positions.push_back(std::min(pos, path_vertices_.size() - 1));
        }
      }
      break;
    }
    
    case ArrowMode::kAll:
      for (size_t i = 0; i < path_vertices_.size(); ++i) {
        arrow_positions.push_back(i);
      }
      break;
    
    default:
      break;
  }

  // Generate 3D pyramidal arrows
  for (size_t pos : arrow_positions) {
    if (pos >= path_vertices_.size() - 1) continue;

    glm::vec3 position = path_vertices_[pos];
    glm::vec3 direction = glm::normalize(path_vertices_[pos + 1] - path_vertices_[pos]);

    // Create orthogonal basis vectors for the arrow
    glm::vec3 up = glm::vec3(0, 0, 1);
    glm::vec3 right = glm::cross(direction, up);
    
    // Handle case where direction is parallel to up vector
    if (glm::length(right) < 0.1f) {
      up = glm::vec3(0, 1, 0);
      right = glm::cross(direction, up);
    }
    
    right = glm::normalize(right) * arrow_size_ * 0.4f;
    up = glm::normalize(glm::cross(right, direction)) * arrow_size_ * 0.4f;
    glm::vec3 forward = direction * arrow_size_;

    // Create 3D pyramid arrow with 5 vertices
    size_t base_idx = arrow_vertices_.size();
    
    // Tip of the arrow (pointing in direction of motion)
    arrow_vertices_.push_back(position + forward);
    
    // Base vertices forming a square base
    arrow_vertices_.push_back(position + right + up);     // Top-right base
    arrow_vertices_.push_back(position - right + up);     // Top-left base  
    arrow_vertices_.push_back(position - right - up);     // Bottom-left base
    arrow_vertices_.push_back(position + right - up);     // Bottom-right base

    // Add colors for all vertices
    for (int i = 0; i < 5; ++i) {
      arrow_colors_.push_back(arrow_color_);
    }

    // Create triangular faces for the 3D pyramid
    uint32_t tip = static_cast<uint32_t>(base_idx);
    uint32_t tr = static_cast<uint32_t>(base_idx + 1);  // top-right
    uint32_t tl = static_cast<uint32_t>(base_idx + 2);  // top-left
    uint32_t bl = static_cast<uint32_t>(base_idx + 3);  // bottom-left
    uint32_t br = static_cast<uint32_t>(base_idx + 4);  // bottom-right

    // Side faces (4 triangular faces from tip to base edges)
    // Right face
    arrow_indices_.push_back(tip);
    arrow_indices_.push_back(tr);
    arrow_indices_.push_back(br);
    
    // Top face  
    arrow_indices_.push_back(tip);
    arrow_indices_.push_back(tl);
    arrow_indices_.push_back(tr);
    
    // Left face
    arrow_indices_.push_back(tip);
    arrow_indices_.push_back(bl);
    arrow_indices_.push_back(tl);
    
    // Bottom face
    arrow_indices_.push_back(tip);
    arrow_indices_.push_back(br);
    arrow_indices_.push_back(bl);

    // Base faces (2 triangular faces to close the base)
    // Base triangle 1
    arrow_indices_.push_back(tr);
    arrow_indices_.push_back(tl);
    arrow_indices_.push_back(bl);
    
    // Base triangle 2  
    arrow_indices_.push_back(tr);
    arrow_indices_.push_back(bl);
    arrow_indices_.push_back(br);
  }

  needs_arrow_update_ = false;
}

void Path::ComputePathColors() {
  path_colors_.clear();
  if (path_vertices_.empty()) {
    return;
  }

  switch (color_mode_) {
    case ColorMode::kUniform:
      for (size_t i = 0; i < path_vertices_.size(); ++i) {
        path_colors_.push_back(base_color_);
      }
      break;

    case ColorMode::kGradient: {
      for (size_t i = 0; i < path_vertices_.size(); ++i) {
        float t = static_cast<float>(i) / std::max(1.0f, static_cast<float>(path_vertices_.size() - 1));
        glm::vec3 color = glm::mix(gradient_start_, gradient_end_, t);
        path_colors_.push_back(color);
      }
      break;
    }

    case ColorMode::kVelocity:
    case ColorMode::kTime:
    case ColorMode::kCost: {
      for (size_t i = 0; i < path_vertices_.size(); ++i) {
        float value = 0.0f;
        if (i < scalar_values_.size()) {
          value = scalar_values_[i];
        } else if (!scalar_values_.empty()) {
          // Use last available value
          value = scalar_values_.back();
        }
        path_colors_.push_back(ColorFromScalar(value));
      }
      break;
    }

    case ColorMode::kCustom: {
      for (size_t i = 0; i < path_vertices_.size(); ++i) {
        if (i < custom_colors_.size()) {
          path_colors_.push_back(custom_colors_[i]);
        } else if (!custom_colors_.empty()) {
          path_colors_.push_back(custom_colors_.back());
        } else {
          path_colors_.push_back(base_color_);
        }
      }
      break;
    }
  }

  needs_color_update_ = false;
}

glm::vec3 Path::InterpolateSpline(const std::vector<glm::vec3>& points, float t) const {
  if (points.size() < 2) return glm::vec3(0.0f);
  if (points.size() == 2) return glm::mix(points[0], points[1], t);

  // Catmull-Rom spline interpolation
  float segment_t = t * (points.size() - 1);
  int segment = static_cast<int>(segment_t);
  float local_t = segment_t - segment;

  segment = glm::clamp(segment, 0, static_cast<int>(points.size()) - 2);

  // Get four control points for Catmull-Rom
  glm::vec3 p0 = (segment > 0) ? points[segment - 1] : points[0];
  glm::vec3 p1 = points[segment];
  glm::vec3 p2 = points[segment + 1];
  glm::vec3 p3 = (segment < static_cast<int>(points.size()) - 2) ? points[segment + 2] : points.back();

  // Catmull-Rom interpolation
  float t2 = local_t * local_t;
  float t3 = t2 * local_t;

  return 0.5f * ((2.0f * p1) +
                 (-p0 + p2) * local_t +
                 (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                 (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
}

glm::vec3 Path::InterpolateBezier(const std::vector<glm::vec3>& points, float t) const {
  if (points.size() < 2) return glm::vec3(0.0f);
  if (points.size() == 2) return glm::mix(points[0], points[1], t);
  if (points.size() == 3) {
    // Quadratic Bezier
    float u = 1.0f - t;
    return u * u * points[0] + 2.0f * u * t * points[1] + t * t * points[2];
  }
  // For more points, use linear interpolation between first and last
  return glm::mix(points[0], points.back(), t);
}

glm::vec3 Path::ColorFromScalar(float value) const {
  // Normalize value to [0, 1] range
  float normalized = (value - color_range_.x) / (color_range_.y - color_range_.x);
  normalized = glm::clamp(normalized, 0.0f, 1.0f);

  // Simple color mapping: blue (low) -> green (medium) -> red (high)
  if (normalized < 0.5f) {
    return glm::mix(glm::vec3(0, 0, 1), glm::vec3(0, 1, 0), normalized * 2.0f);
  } else {
    return glm::mix(glm::vec3(0, 1, 0), glm::vec3(1, 0, 0), (normalized - 0.5f) * 2.0f);
  }
}

float Path::GetTotalLength() const {
  if (path_vertices_.size() < 2) return 0.0f;

  float length = 0.0f;
  for (size_t i = 0; i < path_vertices_.size() - 1; ++i) {
    length += glm::length(path_vertices_[i + 1] - path_vertices_[i]);
  }
  return length;
}

glm::vec3 Path::GetPointAtDistance(float distance) const {
  if (path_vertices_.empty()) return glm::vec3(0.0f);
  if (path_vertices_.size() == 1) return path_vertices_[0];

  float current_dist = 0.0f;
  for (size_t i = 0; i < path_vertices_.size() - 1; ++i) {
    float segment_length = glm::length(path_vertices_[i + 1] - path_vertices_[i]);
    if (current_dist + segment_length >= distance) {
      float t = (distance - current_dist) / segment_length;
      return glm::mix(path_vertices_[i], path_vertices_[i + 1], t);
    }
    current_dist += segment_length;
  }
  return path_vertices_.back();
}

glm::vec3 Path::GetDirectionAtDistance(float distance) const {
  if (path_vertices_.size() < 2) return glm::vec3(1, 0, 0);

  float current_dist = 0.0f;
  for (size_t i = 0; i < path_vertices_.size() - 1; ++i) {
    glm::vec3 segment = path_vertices_[i + 1] - path_vertices_[i];
    float segment_length = glm::length(segment);
    if (current_dist + segment_length >= distance) {
      return glm::normalize(segment);
    }
    current_dist += segment_length;
  }
  // Return last segment direction
  return glm::normalize(path_vertices_.back() - path_vertices_[path_vertices_.size() - 2]);
}

float Path::GetCurvatureAtDistance(float distance) const {
  // Simplified curvature calculation - could be improved
  return 0.0f; // Not implemented for now
}

void Path::UpdateGpuBuffers() {
  if (!IsGpuResourcesAllocated()) return;

  // Update path geometry
  if (!path_vertices_.empty()) {
    glBindVertexArray(path_vao_);

    glBindBuffer(GL_ARRAY_BUFFER, path_vbo_);
    glBufferData(GL_ARRAY_BUFFER, path_vertices_.size() * sizeof(glm::vec3),
                 path_vertices_.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, path_color_vbo_);
    glBufferData(GL_ARRAY_BUFFER, path_colors_.size() * sizeof(glm::vec3),
                 path_colors_.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, path_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, path_indices_.size() * sizeof(uint32_t),
                 path_indices_.data(), GL_DYNAMIC_DRAW);
  }

  // Update arrow geometry
  if (!arrow_vertices_.empty()) {
    glBindVertexArray(arrow_vao_);

    glBindBuffer(GL_ARRAY_BUFFER, arrow_vbo_);
    glBufferData(GL_ARRAY_BUFFER, arrow_vertices_.size() * sizeof(glm::vec3),
                 arrow_vertices_.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, arrow_color_vbo_);
    glBufferData(GL_ARRAY_BUFFER, arrow_colors_.size() * sizeof(glm::vec3),
                 arrow_colors_.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, arrow_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, arrow_indices_.size() * sizeof(uint32_t),
                 arrow_indices_.data(), GL_DYNAMIC_DRAW);
  }

  glBindVertexArray(0);
}

void Path::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;

  try {
    // Compile path shader
    Shader path_vs(kPathVertexShader, Shader::Type::kVertex);
    Shader path_fs(kPathFragmentShader, Shader::Type::kFragment);
    if (!path_vs.Compile() || !path_fs.Compile()) {
      throw std::runtime_error("Path shader compilation failed");
    }
    path_shader_.AttachShader(path_vs);
    path_shader_.AttachShader(path_fs);
    if (!path_shader_.LinkProgram()) {
      throw std::runtime_error("Path shader linking failed");
    }

    // Compile arrow shader
    Shader arrow_vs(kArrowVertexShader, Shader::Type::kVertex);
    Shader arrow_fs(kArrowFragmentShader, Shader::Type::kFragment);
    if (!arrow_vs.Compile() || !arrow_fs.Compile()) {
      throw std::runtime_error("Arrow shader compilation failed");
    }
    arrow_shader_.AttachShader(arrow_vs);
    arrow_shader_.AttachShader(arrow_fs);
    if (!arrow_shader_.LinkProgram()) {
      throw std::runtime_error("Arrow shader linking failed");
    }

    // Create OpenGL objects
    glGenVertexArrays(1, &path_vao_);
    glGenBuffers(1, &path_vbo_);
    glGenBuffers(1, &path_color_vbo_);
    glGenBuffers(1, &path_ebo_);

    glGenVertexArrays(1, &arrow_vao_);
    glGenBuffers(1, &arrow_vbo_);
    glGenBuffers(1, &arrow_color_vbo_);
    glGenBuffers(1, &arrow_ebo_);

  } catch (const std::exception& e) {
    std::cerr << "Path::AllocateGpuResources: " << e.what() << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void Path::ReleaseGpuResources() noexcept {
  if (path_vao_ != 0) {
    glDeleteVertexArrays(1, &path_vao_);
    path_vao_ = 0;
  }
  if (path_vbo_ != 0) {
    glDeleteBuffers(1, &path_vbo_);
    path_vbo_ = 0;
  }
  if (path_color_vbo_ != 0) {
    glDeleteBuffers(1, &path_color_vbo_);
    path_color_vbo_ = 0;
  }
  if (path_ebo_ != 0) {
    glDeleteBuffers(1, &path_ebo_);
    path_ebo_ = 0;
  }
  if (arrow_vao_ != 0) {
    glDeleteVertexArrays(1, &arrow_vao_);
    arrow_vao_ = 0;
  }
  if (arrow_vbo_ != 0) {
    glDeleteBuffers(1, &arrow_vbo_);
    arrow_vbo_ = 0;
  }
  if (arrow_color_vbo_ != 0) {
    glDeleteBuffers(1, &arrow_color_vbo_);
    arrow_color_vbo_ = 0;
  }
  if (arrow_ebo_ != 0) {
    glDeleteBuffers(1, &arrow_ebo_);
    arrow_ebo_ = 0;
  }
}

void Path::OnDraw(const glm::mat4& projection, const glm::mat4& view, const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }

  if (needs_geometry_update_) {
    GeneratePathGeometry();
  }
  if (needs_color_update_) {
    ComputePathColors();
  }
  if (needs_arrow_update_) {
    GenerateArrows();
  }

  UpdateGpuBuffers();

  if (path_vertices_.empty()) {
    return;
  }

  glm::mat4 mvp = projection * view * coord_transform;

  // Enable transparency if needed
  if (alpha_ < 1.0f) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }

  // Draw path
  path_shader_.Use();
  path_shader_.SetUniform("mvp", mvp);
  path_shader_.SetUniform("coordTransform", coord_transform);
  path_shader_.SetUniform("alpha", alpha_);
  path_shader_.SetUniform("glowEnabled", glow_enabled_);
  path_shader_.SetUniform("glowIntensity", glow_intensity_);

  glLineWidth(line_width_);
  glBindVertexArray(path_vao_);
  glDrawElements(GL_LINES, static_cast<GLsizei>(path_indices_.size()), GL_UNSIGNED_INT, nullptr);
  glLineWidth(1.0f);

  // Draw arrows
  if (arrow_mode_ != ArrowMode::kNone && !arrow_vertices_.empty()) {
    arrow_shader_.Use();
    arrow_shader_.SetUniform("mvp", mvp);
    arrow_shader_.SetUniform("coordTransform", coord_transform);
    arrow_shader_.SetUniform("arrowTransform", glm::mat4(1.0f));
    arrow_shader_.SetUniform("alpha", alpha_);

    glBindVertexArray(arrow_vao_);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(arrow_indices_.size()), GL_UNSIGNED_INT, nullptr);
  }

  glBindVertexArray(0);

  if (alpha_ < 1.0f) {
    glDisable(GL_BLEND);
  }
}

} // namespace quickviz