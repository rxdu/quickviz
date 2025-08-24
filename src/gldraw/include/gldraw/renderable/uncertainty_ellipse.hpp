/**
 * @file uncertainty_ellipse.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-24
 * @brief Uncertainty ellipse renderer for covariance visualization
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_UNCERTAINTY_ELLIPSE_HPP
#define QUICKVIZ_UNCERTAINTY_ELLIPSE_HPP

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief Uncertainty ellipse and ellipsoid renderer for probabilistic visualization
 * 
 * Renders confidence regions, covariance ellipses, and uncertainty volumes for:
 * - Robot localization uncertainty (2D/3D position covariance)
 * - Sensor measurement uncertainty  
 * - State estimation confidence regions
 * - Multi-dimensional parameter confidence intervals
 * - Probabilistic trajectory uncertainty
 * 
 * Supports both 2D ellipses and 3D ellipsoids with:
 * - Confidence level visualization (1σ, 2σ, 3σ)
 * - Multiple rendering modes (wireframe, filled, outlined)
 * - Covariance matrix-based automatic scaling and orientation
 * - Chi-squared confidence level mapping
 * - Animation and opacity for temporal uncertainty
 * 
 * Common use cases:
 * - SLAM uncertainty visualization
 * - Kalman filter state uncertainty
 * - Monte Carlo simulation results
 * - Sensor fusion confidence regions
 * - Path planning uncertainty corridors
 */
class UncertaintyEllipse : public OpenGlObject {
public:
  enum class EllipseType {
    k2D,              // 2D ellipse (circle/ellipse in XY plane)
    k3D,              // 3D ellipsoid
    kCylindrical      // Cylindrical uncertainty (2D ellipse extruded in Z)
  };

  enum class RenderMode {
    kWireframe,       // Only outline/edges
    kFilled,          // Solid filled surface
    kOutlined,        // Filled with outline
    kTransparent,     // Semi-transparent filled
    kGradient         // Gradient from center to edge
  };

  enum class ConfidenceLevel {
    kOneSigma,        // 68.27% confidence (1 standard deviation)
    kTwoSigma,        // 95.45% confidence (2 standard deviations)
    kThreeSigma,      // 99.73% confidence (3 standard deviations)
    kCustom           // User-specified confidence level
  };

  UncertaintyEllipse();
  explicit UncertaintyEllipse(EllipseType type);
  ~UncertaintyEllipse();

  // Ellipse definition
  void SetEllipseType(EllipseType type);
  void SetCenter(const glm::vec3& center);
  void SetCovarianceMatrix2D(const glm::mat2& covariance);
  void SetCovarianceMatrix3D(const glm::mat3& covariance);
  void SetAxisLengths2D(float semi_major, float semi_minor, float rotation_angle = 0.0f);
  void SetAxisLengths3D(const glm::vec3& semi_axes, const glm::mat3& rotation = glm::mat3(1.0f));

  EllipseType GetEllipseType() const { return ellipse_type_; }
  glm::vec3 GetCenter() const { return center_; }

  // Confidence level control
  void SetConfidenceLevel(ConfidenceLevel level);
  void SetCustomConfidence(float confidence_percentage);  // 0.0 to 100.0
  void SetSigmaMultiplier(float sigma_multiplier);        // Direct sigma scaling

  ConfidenceLevel GetConfidenceLevel() const { return confidence_level_; }
  float GetCustomConfidence() const { return custom_confidence_; }
  float GetSigmaMultiplier() const { return sigma_multiplier_; }

  // Appearance settings
  void SetRenderMode(RenderMode mode);
  void SetColor(const glm::vec3& color);
  void SetOutlineColor(const glm::vec3& outline_color);
  void SetGradientColors(const glm::vec3& center_color, const glm::vec3& edge_color);
  void SetTransparency(float alpha);
  void SetOutlineWidth(float width);

  RenderMode GetRenderMode() const { return render_mode_; }
  glm::vec3 GetColor() const { return color_; }
  float GetTransparency() const { return alpha_; }

  // Geometry resolution
  void SetResolution(int segments);                       // For 2D ellipses
  void SetResolution3D(int rings, int sectors);           // For 3D ellipsoids
  void SetCylindricalHeight(float height);               // For cylindrical type

  int GetResolution() const { return resolution_2d_; }

  // Multi-level confidence visualization
  void SetMultiLevel(bool enable_multi_level);
  void AddConfidenceLevel(float sigma_level, const glm::vec3& color, float alpha = 0.3f);
  void ClearConfidenceLevels();

  bool GetMultiLevel() const { return multi_level_enabled_; }
  size_t GetConfidenceLevelCount() const { return confidence_levels_.size(); }

  // Animation and effects
  void SetPulsing(bool enable_pulsing, float frequency = 1.0f, float amplitude = 0.2f);
  void SetGrowthAnimation(bool enable_growth, float growth_rate = 1.0f);
  void SetTimeParameter(float time);  // For animations

  bool GetPulsing() const { return pulsing_enabled_; }
  bool GetGrowthAnimation() const { return growth_enabled_; }

  // Statistical utilities
  glm::vec2 GetAxisLengths2D() const;                    // Returns (semi_major, semi_minor)
  glm::vec3 GetAxisLengths3D() const;                    // Returns (a, b, c) semi-axes
  float GetRotationAngle2D() const;                      // Rotation of major axis from X-axis
  glm::mat3 GetRotationMatrix3D() const;                 // 3D rotation matrix
  
  // Probability calculations
  bool ContainsPoint(const glm::vec3& point) const;      // Check if point is inside ellipse
  float GetProbabilityAtPoint(const glm::vec3& point) const;  // Probability density at point
  float GetMahalanobisDistance(const glm::vec3& point) const; // Mahalanobis distance

  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return main_vao_ != 0; }

private:
  struct ConfidenceLevelInfo {
    float sigma_level;
    glm::vec3 color;
    float alpha;
  };

  void UpdateGeometry();
  void Generate2DEllipse();
  void Generate3DEllipsoid();
  void GenerateCylindricalUncertainty();
  void UpdateTransformMatrix();
  void ComputeEigenDecomposition2D();
  void ComputeEigenDecomposition3D();
  float GetChiSquaredMultiplier() const;

  // Core properties
  EllipseType ellipse_type_;
  glm::vec3 center_;
  
  // Covariance and scaling
  glm::mat2 covariance_2d_;
  glm::mat3 covariance_3d_;
  glm::vec2 eigen_values_2d_;         // 2D eigenvalues (semi-axes squared)
  glm::vec3 eigen_values_3d_;         // 3D eigenvalues (semi-axes squared)
  glm::mat2 eigen_vectors_2d_;        // 2D eigenvectors (rotation matrix)
  glm::mat3 eigen_vectors_3d_;        // 3D eigenvectors (rotation matrix)
  bool covariance_valid_;

  // Manual axis specification (when not using covariance)
  glm::vec3 manual_semi_axes_;        // Semi-axis lengths
  glm::mat3 manual_rotation_;         // Manual rotation matrix
  bool use_manual_specification_;

  // Confidence level
  ConfidenceLevel confidence_level_;
  float custom_confidence_;           // Custom confidence percentage
  float sigma_multiplier_;            // Direct sigma scaling factor

  // Appearance
  RenderMode render_mode_;
  glm::vec3 color_;
  glm::vec3 outline_color_;
  glm::vec3 gradient_center_color_;
  glm::vec3 gradient_edge_color_;
  float alpha_;
  float outline_width_;

  // Geometry resolution
  int resolution_2d_;                 // Number of segments for 2D ellipse
  int resolution_rings_;              // Number of rings for 3D ellipsoid
  int resolution_sectors_;            // Number of sectors for 3D ellipsoid
  float cylindrical_height_;          // Height for cylindrical uncertainty

  // Multi-level confidence
  bool multi_level_enabled_;
  std::vector<ConfidenceLevelInfo> confidence_levels_;

  // Animation
  bool pulsing_enabled_;
  float pulsing_frequency_;
  float pulsing_amplitude_;
  bool growth_enabled_;
  float growth_rate_;
  float time_parameter_;

  // Computed transform
  glm::mat4 transform_matrix_;

  // Geometry data
  std::vector<glm::vec3> vertices_;
  std::vector<glm::vec3> normals_;
  std::vector<glm::vec3> colors_;
  std::vector<uint32_t> indices_;
  
  std::vector<glm::vec3> outline_vertices_;
  std::vector<uint32_t> outline_indices_;

  // OpenGL resources
  uint32_t main_vao_, main_vbo_, main_normal_vbo_, main_color_vbo_, main_ebo_;
  uint32_t outline_vao_, outline_vbo_, outline_ebo_;
  ShaderProgram surface_shader_;
  ShaderProgram outline_shader_;

  // Update flags
  bool needs_geometry_update_;
  bool needs_transform_update_;
};

} // namespace quickviz

#endif // QUICKVIZ_UNCERTAINTY_ELLIPSE_HPP