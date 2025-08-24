/**
 * @file uncertainty_ellipse.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-24
 * @brief Implementation of uncertainty ellipse renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/uncertainty_ellipse.hpp"

#include <glad/glad.h>
#include "gldraw/shader.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

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

uniform vec3 uLightPos;
uniform vec3 uViewPos;
uniform float uAlpha;
uniform bool uUseGradient;
uniform vec3 uGradientCenter;
uniform vec3 uGradientEdge;

out vec4 FragColor;

void main() {
    vec3 color = VertexColor;
    
    if (uUseGradient) {
        // Simple gradient based on distance from center
        float distance = length(gl_PointCoord - vec2(0.5));
        color = mix(uGradientCenter, uGradientEdge, smoothstep(0.0, 0.5, distance));
    }
    
    // Simple lighting
    vec3 lightColor = vec3(1.0);
    vec3 ambient = 0.3 * lightColor;
    
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(uLightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;
    
    vec3 viewDir = normalize(uViewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = 0.2 * spec * lightColor;
    
    vec3 result = (ambient + diffuse + specular) * color;
    FragColor = vec4(result, uAlpha);
}
)glsl";

const std::string kLineVertexShader = R"glsl(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uCoordTransform;
uniform mat4 uModel;

void main() {
    gl_Position = uProjection * uView * uCoordTransform * uModel * vec4(aPos, 1.0);
}
)glsl";

const std::string kLineFragmentShader = R"glsl(
#version 330 core
uniform vec3 uColor;
uniform float uAlpha;

out vec4 FragColor;

void main() {
    FragColor = vec4(uColor, uAlpha);
}
)glsl";

// Chi-squared values for confidence levels
const float kOneSignmaChiSquared2D = 2.279f;    // 68.27% for 2D
const float kTwoSigmaChiSquared2D = 5.991f;     // 95.45% for 2D
const float kThreeSigmaChiSquared2D = 11.829f;  // 99.73% for 2D

const float kOneSigmaChiSquared3D = 3.527f;     // 68.27% for 3D
const float kTwoSigmaChiSquared3D = 7.815f;     // 95.45% for 3D
const float kThreeSigmaChiSquared3D = 14.796f;  // 99.73% for 3D
}

UncertaintyEllipse::UncertaintyEllipse()
    : ellipse_type_(EllipseType::k2D)
    , center_(glm::vec3(0.0f))
    , covariance_2d_(glm::mat2(1.0f))
    , covariance_3d_(glm::mat3(1.0f))
    , eigen_values_2d_(glm::vec2(1.0f))
    , eigen_values_3d_(glm::vec3(1.0f))
    , eigen_vectors_2d_(glm::mat2(1.0f))
    , eigen_vectors_3d_(glm::mat3(1.0f))
    , covariance_valid_(false)
    , manual_semi_axes_(glm::vec3(1.0f))
    , manual_rotation_(glm::mat3(1.0f))
    , use_manual_specification_(true)
    , confidence_level_(ConfidenceLevel::kOneSigma)
    , custom_confidence_(68.27f)
    , sigma_multiplier_(1.0f)
    , render_mode_(RenderMode::kTransparent)
    , color_(glm::vec3(0.0f, 0.5f, 1.0f))
    , outline_color_(glm::vec3(0.0f, 0.3f, 0.8f))
    , gradient_center_color_(glm::vec3(0.2f, 0.7f, 1.0f))
    , gradient_edge_color_(glm::vec3(0.0f, 0.3f, 0.8f))
    , alpha_(0.3f)
    , outline_width_(2.0f)
    , resolution_2d_(32)
    , resolution_rings_(16)
    , resolution_sectors_(32)
    , cylindrical_height_(1.0f)
    , multi_level_enabled_(false)
    , pulsing_enabled_(false)
    , pulsing_frequency_(1.0f)
    , pulsing_amplitude_(0.2f)
    , growth_enabled_(false)
    , growth_rate_(1.0f)
    , time_parameter_(0.0f)
    , transform_matrix_(glm::mat4(1.0f))
    , main_vao_(0), main_vbo_(0), main_normal_vbo_(0), main_color_vbo_(0), main_ebo_(0)
    , outline_vao_(0), outline_vbo_(0), outline_ebo_(0)
    , needs_geometry_update_(true)
    , needs_transform_update_(true) {
}

UncertaintyEllipse::UncertaintyEllipse(EllipseType type) : UncertaintyEllipse() {
  ellipse_type_ = type;
}

UncertaintyEllipse::~UncertaintyEllipse() {
  ReleaseGpuResources();
}

void UncertaintyEllipse::SetEllipseType(EllipseType type) {
  if (ellipse_type_ != type) {
    ellipse_type_ = type;
    needs_geometry_update_ = true;
  }
}

void UncertaintyEllipse::SetCenter(const glm::vec3& center) {
  center_ = center;
  needs_transform_update_ = true;
}

void UncertaintyEllipse::SetCovarianceMatrix2D(const glm::mat2& covariance) {
  covariance_2d_ = covariance;
  use_manual_specification_ = false;
  covariance_valid_ = true;
  ComputeEigenDecomposition2D();
  needs_geometry_update_ = true;
  needs_transform_update_ = true;
}

void UncertaintyEllipse::SetCovarianceMatrix3D(const glm::mat3& covariance) {
  covariance_3d_ = covariance;
  use_manual_specification_ = false;
  covariance_valid_ = true;
  ComputeEigenDecomposition3D();
  needs_geometry_update_ = true;
  needs_transform_update_ = true;
}

void UncertaintyEllipse::SetAxisLengths2D(float semi_major, float semi_minor, float rotation_angle) {
  manual_semi_axes_ = glm::vec3(semi_major, semi_minor, 0.0f);
  
  float cos_angle = std::cos(rotation_angle);
  float sin_angle = std::sin(rotation_angle);
  manual_rotation_ = glm::mat3(
    cos_angle, -sin_angle, 0.0f,
    sin_angle, cos_angle, 0.0f,
    0.0f, 0.0f, 1.0f
  );
  
  use_manual_specification_ = true;
  covariance_valid_ = false;
  needs_geometry_update_ = true;
  needs_transform_update_ = true;
}

void UncertaintyEllipse::SetAxisLengths3D(const glm::vec3& semi_axes, const glm::mat3& rotation) {
  manual_semi_axes_ = semi_axes;
  manual_rotation_ = rotation;
  use_manual_specification_ = true;
  covariance_valid_ = false;
  needs_geometry_update_ = true;
  needs_transform_update_ = true;
}

void UncertaintyEllipse::SetConfidenceLevel(ConfidenceLevel level) {
  confidence_level_ = level;
  
  // Update custom confidence based on standard levels
  switch (level) {
    case ConfidenceLevel::kOneSigma:
      custom_confidence_ = 68.27f;
      sigma_multiplier_ = 1.0f;
      break;
    case ConfidenceLevel::kTwoSigma:
      custom_confidence_ = 95.45f;
      sigma_multiplier_ = 2.0f;
      break;
    case ConfidenceLevel::kThreeSigma:
      custom_confidence_ = 99.73f;
      sigma_multiplier_ = 3.0f;
      break;
    default:
      break;
  }
  
  needs_geometry_update_ = true;
}

void UncertaintyEllipse::SetCustomConfidence(float confidence_percentage) {
  confidence_level_ = ConfidenceLevel::kCustom;
  custom_confidence_ = std::clamp(confidence_percentage, 0.1f, 99.9f);
  
  // Convert percentage to sigma multiplier (approximation)
  if (confidence_percentage <= 68.27f) {
    sigma_multiplier_ = 1.0f;
  } else if (confidence_percentage <= 95.45f) {
    sigma_multiplier_ = 2.0f;
  } else {
    sigma_multiplier_ = 3.0f;
  }
  
  needs_geometry_update_ = true;
}

void UncertaintyEllipse::SetSigmaMultiplier(float sigma_multiplier) {
  sigma_multiplier_ = std::max(0.1f, sigma_multiplier);
  confidence_level_ = ConfidenceLevel::kCustom;
  needs_geometry_update_ = true;
}

void UncertaintyEllipse::SetRenderMode(RenderMode mode) {
  render_mode_ = mode;
  needs_geometry_update_ = true;
}

void UncertaintyEllipse::SetColor(const glm::vec3& color) {
  color_ = color;
  needs_geometry_update_ = true;
}

void UncertaintyEllipse::SetOutlineColor(const glm::vec3& outline_color) {
  outline_color_ = outline_color;
}

void UncertaintyEllipse::SetGradientColors(const glm::vec3& center_color, const glm::vec3& edge_color) {
  gradient_center_color_ = center_color;
  gradient_edge_color_ = edge_color;
  needs_geometry_update_ = true;
}

void UncertaintyEllipse::SetTransparency(float alpha) {
  alpha_ = std::clamp(alpha, 0.0f, 1.0f);
}

void UncertaintyEllipse::SetOutlineWidth(float width) {
  outline_width_ = std::max(0.5f, width);
}

void UncertaintyEllipse::SetResolution(int segments) {
  resolution_2d_ = std::max(8, std::min(128, segments));
  needs_geometry_update_ = true;
}

void UncertaintyEllipse::SetResolution3D(int rings, int sectors) {
  resolution_rings_ = std::max(4, std::min(32, rings));
  resolution_sectors_ = std::max(8, std::min(64, sectors));
  needs_geometry_update_ = true;
}

void UncertaintyEllipse::SetCylindricalHeight(float height) {
  cylindrical_height_ = std::max(0.1f, height);
  if (ellipse_type_ == EllipseType::kCylindrical) {
    needs_geometry_update_ = true;
  }
}

void UncertaintyEllipse::SetMultiLevel(bool enable_multi_level) {
  multi_level_enabled_ = enable_multi_level;
  needs_geometry_update_ = true;
}

void UncertaintyEllipse::AddConfidenceLevel(float sigma_level, const glm::vec3& color, float alpha) {
  ConfidenceLevelInfo level_info;
  level_info.sigma_level = std::max(0.1f, sigma_level);
  level_info.color = color;
  level_info.alpha = std::clamp(alpha, 0.0f, 1.0f);
  
  confidence_levels_.push_back(level_info);
  
  // Sort by sigma level (ascending)
  std::sort(confidence_levels_.begin(), confidence_levels_.end(),
           [](const ConfidenceLevelInfo& a, const ConfidenceLevelInfo& b) {
             return a.sigma_level < b.sigma_level;
           });
  
  if (multi_level_enabled_) {
    needs_geometry_update_ = true;
  }
}

void UncertaintyEllipse::ClearConfidenceLevels() {
  confidence_levels_.clear();
  needs_geometry_update_ = true;
}

void UncertaintyEllipse::SetPulsing(bool enable_pulsing, float frequency, float amplitude) {
  pulsing_enabled_ = enable_pulsing;
  pulsing_frequency_ = std::max(0.1f, frequency);
  pulsing_amplitude_ = std::clamp(amplitude, 0.0f, 1.0f);
}

void UncertaintyEllipse::SetGrowthAnimation(bool enable_growth, float growth_rate) {
  growth_enabled_ = enable_growth;
  growth_rate_ = std::max(0.1f, growth_rate);
}

void UncertaintyEllipse::SetTimeParameter(float time) {
  time_parameter_ = time;
  if (pulsing_enabled_ || growth_enabled_) {
    needs_transform_update_ = true;
  }
}

glm::vec2 UncertaintyEllipse::GetAxisLengths2D() const {
  if (use_manual_specification_) {
    return glm::vec2(manual_semi_axes_.x, manual_semi_axes_.y);
  } else if (covariance_valid_) {
    float multiplier = GetChiSquaredMultiplier();
    return glm::vec2(std::sqrt(eigen_values_2d_.x * multiplier),
                     std::sqrt(eigen_values_2d_.y * multiplier));
  }
  return glm::vec2(1.0f);
}

glm::vec3 UncertaintyEllipse::GetAxisLengths3D() const {
  if (use_manual_specification_) {
    return manual_semi_axes_;
  } else if (covariance_valid_) {
    float multiplier = GetChiSquaredMultiplier();
    return glm::vec3(std::sqrt(eigen_values_3d_.x * multiplier),
                     std::sqrt(eigen_values_3d_.y * multiplier),
                     std::sqrt(eigen_values_3d_.z * multiplier));
  }
  return glm::vec3(1.0f);
}

float UncertaintyEllipse::GetRotationAngle2D() const {
  if (use_manual_specification_) {
    return std::atan2(manual_rotation_[1][0], manual_rotation_[0][0]);
  } else if (covariance_valid_) {
    return std::atan2(eigen_vectors_2d_[1][0], eigen_vectors_2d_[0][0]);
  }
  return 0.0f;
}

glm::mat3 UncertaintyEllipse::GetRotationMatrix3D() const {
  if (use_manual_specification_) {
    return manual_rotation_;
  } else if (covariance_valid_) {
    return eigen_vectors_3d_;
  }
  return glm::mat3(1.0f);
}

void UncertaintyEllipse::AllocateGpuResources() {
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
    std::cerr << "UncertaintyEllipse::AllocateGpuResources: " << e.what() << std::endl;
    throw;
  }
  
  // Generate surface VAO and buffers
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

void UncertaintyEllipse::ReleaseGpuResources() noexcept {
  if (main_vao_ != 0) {
    glDeleteVertexArrays(1, &main_vao_);
    glDeleteBuffers(1, &main_vbo_);
    glDeleteBuffers(1, &main_normal_vbo_);
    glDeleteBuffers(1, &main_color_vbo_);
    glDeleteBuffers(1, &main_ebo_);
    main_vao_ = main_vbo_ = main_normal_vbo_ = main_color_vbo_ = main_ebo_ = 0;
  }
  
  if (outline_vao_ != 0) {
    glDeleteVertexArrays(1, &outline_vao_);
    glDeleteBuffers(1, &outline_vbo_);
    glDeleteBuffers(1, &outline_ebo_);
    outline_vao_ = outline_vbo_ = outline_ebo_ = 0;
  }
}

void UncertaintyEllipse::OnDraw(const glm::mat4& projection, const glm::mat4& view,
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
  
  // Draw filled surface
  if (render_mode_ != RenderMode::kWireframe && !vertices_.empty()) {
    surface_shader_.Use();
    surface_shader_.SetUniform("uProjection", projection);
    surface_shader_.SetUniform("uView", view);
    surface_shader_.SetUniform("uCoordTransform", coord_transform);
    surface_shader_.SetUniform("uModel", transform_matrix_);
    surface_shader_.SetUniform("uAlpha", alpha_);
    surface_shader_.SetUniform("uLightPos", glm::vec3(10.0f, 10.0f, 10.0f));
    surface_shader_.SetUniform("uViewPos", glm::vec3(0.0f, 0.0f, 10.0f));
    
    bool use_gradient = (render_mode_ == RenderMode::kGradient);
    surface_shader_.SetUniform("uUseGradient", use_gradient);
    if (use_gradient) {
      surface_shader_.SetUniform("uGradientCenter", gradient_center_color_);
      surface_shader_.SetUniform("uGradientEdge", gradient_edge_color_);
    }
    
    glBindVertexArray(main_vao_);
    
    // Update buffers
    glBindBuffer(GL_ARRAY_BUFFER, main_vbo_);
    glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3),
                 vertices_.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ARRAY_BUFFER, main_normal_vbo_);
    glBufferData(GL_ARRAY_BUFFER, normals_.size() * sizeof(glm::vec3),
                 normals_.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ARRAY_BUFFER, main_color_vbo_);
    glBufferData(GL_ARRAY_BUFFER, colors_.size() * sizeof(glm::vec3),
                 colors_.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, main_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(uint32_t),
                 indices_.data(), GL_DYNAMIC_DRAW);
    
    glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, nullptr);
  }
  
  // Draw outline
  if ((render_mode_ == RenderMode::kWireframe || render_mode_ == RenderMode::kOutlined) 
      && !outline_vertices_.empty()) {
    
    glLineWidth(outline_width_);
    
    outline_shader_.Use();
    outline_shader_.SetUniform("uProjection", projection);
    outline_shader_.SetUniform("uView", view);
    outline_shader_.SetUniform("uCoordTransform", coord_transform);
    outline_shader_.SetUniform("uModel", transform_matrix_);
    outline_shader_.SetUniform("uColor", outline_color_);
    outline_shader_.SetUniform("uAlpha", alpha_);
    
    glBindVertexArray(outline_vao_);
    
    glBindBuffer(GL_ARRAY_BUFFER, outline_vbo_);
    glBufferData(GL_ARRAY_BUFFER, outline_vertices_.size() * sizeof(glm::vec3),
                 outline_vertices_.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, outline_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, outline_indices_.size() * sizeof(uint32_t),
                 outline_indices_.data(), GL_DYNAMIC_DRAW);
    
    glDrawElements(GL_LINES, outline_indices_.size(), GL_UNSIGNED_INT, nullptr);
    
    glLineWidth(1.0f);
  }
  
  glBindVertexArray(0);
  glDisable(GL_BLEND);
}

void UncertaintyEllipse::UpdateGeometry() {
  // Clear previous geometry
  vertices_.clear();
  normals_.clear();
  colors_.clear();
  indices_.clear();
  outline_vertices_.clear();
  outline_indices_.clear();
  
  if (multi_level_enabled_ && !confidence_levels_.empty()) {
    // Generate geometry for multiple confidence levels
    for (const auto& level_info : confidence_levels_) {
      float saved_sigma = sigma_multiplier_;
      sigma_multiplier_ = level_info.sigma_level;
      
      size_t vertex_offset = vertices_.size();
      
      // Generate geometry for this level
      switch (ellipse_type_) {
        case EllipseType::k2D:
          Generate2DEllipse();
          break;
        case EllipseType::k3D:
          Generate3DEllipsoid();
          break;
        case EllipseType::kCylindrical:
          GenerateCylindricalUncertainty();
          break;
      }
      
      // Update colors for this level
      for (size_t i = vertex_offset; i < vertices_.size(); ++i) {
        if (i < colors_.size()) {
          colors_[i] = level_info.color;
        } else {
          colors_.push_back(level_info.color);
        }
      }
      
      sigma_multiplier_ = saved_sigma;
    }
  } else {
    // Generate single level geometry
    switch (ellipse_type_) {
      case EllipseType::k2D:
        Generate2DEllipse();
        break;
      case EllipseType::k3D:
        Generate3DEllipsoid();
        break;
      case EllipseType::kCylindrical:
        GenerateCylindricalUncertainty();
        break;
    }
  }
}

void UncertaintyEllipse::Generate2DEllipse() {
  glm::vec2 axes = GetAxisLengths2D();
  float a = axes.x;  // Semi-major axis
  float b = axes.y;  // Semi-minor axis
  
  uint32_t center_index = vertices_.size();
  
  // Center vertex
  vertices_.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
  normals_.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
  colors_.push_back(color_);
  
  // Generate ellipse vertices
  for (int i = 0; i <= resolution_2d_; ++i) {
    float angle = 2.0f * M_PI * i / resolution_2d_;
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);
    
    // Ellipse point
    glm::vec3 vertex(a * cos_a, b * sin_a, 0.0f);
    vertices_.push_back(vertex);
    normals_.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
    colors_.push_back(color_);
    
    // Add outline vertex
    outline_vertices_.push_back(vertex);
    
    // Create triangle with center
    if (i < resolution_2d_) {
      indices_.push_back(center_index);
      indices_.push_back(center_index + 1 + i);
      indices_.push_back(center_index + 1 + ((i + 1) % resolution_2d_));
    }
    
    // Create outline indices
    if (i > 0) {
      outline_indices_.push_back(outline_vertices_.size() - 2);
      outline_indices_.push_back(outline_vertices_.size() - 1);
    }
  }
  
  // Close outline loop
  if (!outline_vertices_.empty()) {
    outline_indices_.push_back(outline_vertices_.size() - 1);
    outline_indices_.push_back(outline_vertices_.size() - resolution_2d_);
  }
}

void UncertaintyEllipse::Generate3DEllipsoid() {
  glm::vec3 axes = GetAxisLengths3D();
  float a = axes.x;  // Semi-axis X
  float b = axes.y;  // Semi-axis Y
  float c = axes.z;  // Semi-axis Z
  
  // Generate ellipsoid vertices using spherical coordinates
  for (int ring = 0; ring <= resolution_rings_; ++ring) {
    float phi = M_PI * ring / resolution_rings_;  // From 0 to π
    float cos_phi = std::cos(phi);
    float sin_phi = std::sin(phi);
    
    for (int sector = 0; sector <= resolution_sectors_; ++sector) {
      float theta = 2.0f * M_PI * sector / resolution_sectors_;  // From 0 to 2π
      float cos_theta = std::cos(theta);
      float sin_theta = std::sin(theta);
      
      // Ellipsoid vertex
      glm::vec3 vertex(
        a * sin_phi * cos_theta,
        b * sin_phi * sin_theta,
        c * cos_phi
      );
      
      // Normal (same as vertex for ellipsoid centered at origin)
      glm::vec3 normal = glm::normalize(vertex);
      
      vertices_.push_back(vertex);
      normals_.push_back(normal);
      colors_.push_back(color_);
      
      // Add outline vertices for wireframe (only some edges)
      if (ring == 0 || ring == resolution_rings_ || sector % 4 == 0) {
        outline_vertices_.push_back(vertex);
      }
    }
  }
  
  // Generate indices for triangles
  for (int ring = 0; ring < resolution_rings_; ++ring) {
    for (int sector = 0; sector < resolution_sectors_; ++sector) {
      int current_row = ring * (resolution_sectors_ + 1);
      int next_row = (ring + 1) * (resolution_sectors_ + 1);
      
      // First triangle
      indices_.push_back(current_row + sector);
      indices_.push_back(next_row + sector);
      indices_.push_back(current_row + sector + 1);
      
      // Second triangle
      indices_.push_back(current_row + sector + 1);
      indices_.push_back(next_row + sector);
      indices_.push_back(next_row + sector + 1);
    }
  }
  
  // Generate outline indices (simplified - just some longitude/latitude lines)
  for (int i = 1; i < outline_vertices_.size(); ++i) {
    outline_indices_.push_back(i - 1);
    outline_indices_.push_back(i);
  }
}

void UncertaintyEllipse::GenerateCylindricalUncertainty() {
  // Generate 2D ellipse first
  Generate2DEllipse();
  
  // Extrude in Z direction
  size_t base_vertex_count = vertices_.size();
  
  // Add top vertices
  for (size_t i = 0; i < base_vertex_count; ++i) {
    glm::vec3 top_vertex = vertices_[i];
    top_vertex.z += cylindrical_height_;
    
    vertices_.push_back(top_vertex);
    normals_.push_back(normals_[i]);
    colors_.push_back(colors_[i]);
  }
  
  // Add side faces
  for (int i = 1; i <= resolution_2d_; ++i) {
    int current = i;
    int next = (i % resolution_2d_) + 1;
    
    // Bottom quad vertices
    uint32_t bl = current;                    // Bottom left
    uint32_t br = next;                       // Bottom right
    uint32_t tl = current + base_vertex_count;  // Top left
    uint32_t tr = next + base_vertex_count;     // Top right
    
    // Two triangles for side face
    indices_.push_back(bl);
    indices_.push_back(tl);
    indices_.push_back(br);
    
    indices_.push_back(br);
    indices_.push_back(tl);
    indices_.push_back(tr);
    
    // Add outline for vertical edges
    outline_vertices_.push_back(vertices_[bl]);
    outline_vertices_.push_back(vertices_[tl]);
    outline_indices_.push_back(outline_vertices_.size() - 2);
    outline_indices_.push_back(outline_vertices_.size() - 1);
  }
}

void UncertaintyEllipse::UpdateTransformMatrix() {
  // Start with identity
  transform_matrix_ = glm::mat4(1.0f);
  
  // Apply translation
  transform_matrix_ = glm::translate(transform_matrix_, center_);
  
  // Apply rotation
  glm::mat3 rotation_matrix = GetRotationMatrix3D();
  glm::mat4 rotation_4x4 = glm::mat4(
    rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], 0.0f,
    rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], 0.0f,
    rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f
  );
  transform_matrix_ = transform_matrix_ * rotation_4x4;
  
  // Apply animation effects
  float scale_factor = 1.0f;
  
  if (pulsing_enabled_) {
    scale_factor *= (1.0f + pulsing_amplitude_ * std::sin(2.0f * M_PI * pulsing_frequency_ * time_parameter_));
  }
  
  if (growth_enabled_) {
    scale_factor *= (1.0f + growth_rate_ * time_parameter_);
  }
  
  if (scale_factor != 1.0f) {
    transform_matrix_ = glm::scale(transform_matrix_, glm::vec3(scale_factor));
  }
}

void UncertaintyEllipse::ComputeEigenDecomposition2D() {
  // Simple 2D eigenvalue decomposition for 2x2 symmetric matrix
  float a = covariance_2d_[0][0];
  float b = covariance_2d_[0][1];
  float c = covariance_2d_[1][1];
  
  // Eigenvalues
  float trace = a + c;
  float det = a * c - b * b;
  float discriminant = trace * trace - 4.0f * det;
  
  if (discriminant < 0.0f) {
    // Fallback to identity
    eigen_values_2d_ = glm::vec2(1.0f);
    eigen_vectors_2d_ = glm::mat2(1.0f);
    return;
  }
  
  float sqrt_disc = std::sqrt(discriminant);
  eigen_values_2d_.x = (trace + sqrt_disc) * 0.5f;
  eigen_values_2d_.y = (trace - sqrt_disc) * 0.5f;
  
  // Eigenvectors
  if (std::abs(b) < 1e-10f) {
    // Already diagonal
    eigen_vectors_2d_ = glm::mat2(1.0f);
  } else {
    // First eigenvector
    glm::vec2 v1(eigen_values_2d_.x - c, b);
    v1 = glm::normalize(v1);
    
    // Second eigenvector (perpendicular)
    glm::vec2 v2(-v1.y, v1.x);
    
    eigen_vectors_2d_ = glm::mat2(v1.x, v1.y, v2.x, v2.y);
  }
}

void UncertaintyEllipse::ComputeEigenDecomposition3D() {
  // Simplified 3D eigenvalue computation (for demonstration)
  // In practice, would use a robust numerical library
  
  // Extract diagonal for simple case
  eigen_values_3d_.x = covariance_3d_[0][0];
  eigen_values_3d_.y = covariance_3d_[1][1];
  eigen_values_3d_.z = covariance_3d_[2][2];
  
  // For now, assume identity rotation (simplified)
  eigen_vectors_3d_ = glm::mat3(1.0f);
  
  // Ensure positive eigenvalues
  eigen_values_3d_ = glm::max(eigen_values_3d_, glm::vec3(1e-6f));
}

float UncertaintyEllipse::GetChiSquaredMultiplier() const {
  if (ellipse_type_ == EllipseType::k2D || ellipse_type_ == EllipseType::kCylindrical) {
    switch (confidence_level_) {
      case ConfidenceLevel::kOneSigma:
        return kOneSignmaChiSquared2D;
      case ConfidenceLevel::kTwoSigma:
        return kTwoSigmaChiSquared2D;
      case ConfidenceLevel::kThreeSigma:
        return kThreeSigmaChiSquared2D;
      case ConfidenceLevel::kCustom:
        return sigma_multiplier_ * sigma_multiplier_;  // Simplified
    }
  } else {
    switch (confidence_level_) {
      case ConfidenceLevel::kOneSigma:
        return kOneSigmaChiSquared3D;
      case ConfidenceLevel::kTwoSigma:
        return kTwoSigmaChiSquared3D;
      case ConfidenceLevel::kThreeSigma:
        return kThreeSigmaChiSquared3D;
      case ConfidenceLevel::kCustom:
        return sigma_multiplier_ * sigma_multiplier_;  // Simplified
    }
  }
  
  return 1.0f;
}

bool UncertaintyEllipse::ContainsPoint(const glm::vec3& point) const {
  // Transform point to ellipse local coordinates
  glm::vec3 local_point = point - center_;
  
  // Apply inverse rotation
  glm::mat3 inverse_rotation = glm::transpose(GetRotationMatrix3D());
  local_point = inverse_rotation * local_point;
  
  // Check if inside ellipse/ellipsoid
  if (ellipse_type_ == EllipseType::k2D) {
    glm::vec2 axes = GetAxisLengths2D();
    float normalized_dist = (local_point.x * local_point.x) / (axes.x * axes.x) +
                           (local_point.y * local_point.y) / (axes.y * axes.y);
    return normalized_dist <= 1.0f;
  } else {
    glm::vec3 axes = GetAxisLengths3D();
    float normalized_dist = (local_point.x * local_point.x) / (axes.x * axes.x) +
                           (local_point.y * local_point.y) / (axes.y * axes.y) +
                           (local_point.z * local_point.z) / (axes.z * axes.z);
    return normalized_dist <= 1.0f;
  }
}

float UncertaintyEllipse::GetMahalanobisDistance(const glm::vec3& point) const {
  glm::vec3 diff = point - center_;
  
  if (ellipse_type_ == EllipseType::k2D && covariance_valid_) {
    glm::vec2 diff_2d(diff.x, diff.y);
    glm::mat2 inv_cov = glm::inverse(covariance_2d_);
    return std::sqrt(glm::dot(diff_2d, inv_cov * diff_2d));
  } else if (covariance_valid_) {
    glm::mat3 inv_cov = glm::inverse(covariance_3d_);
    return std::sqrt(glm::dot(diff, inv_cov * diff));
  }
  
  return glm::length(diff);
}

float UncertaintyEllipse::GetProbabilityAtPoint(const glm::vec3& point) const {
  float mahal_dist = GetMahalanobisDistance(point);
  return std::exp(-0.5f * mahal_dist * mahal_dist);
}

} // namespace quickviz