/**
 * @file sensor_coverage.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-24
 * @brief Sensor coverage renderer for range rings and 3D coverage volumes
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_SENSOR_COVERAGE_HPP
#define QUICKVIZ_SENSOR_COVERAGE_HPP

#include <vector>
#include <glm/glm.hpp>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief Sensor coverage renderer for robotics sensor visualization
 * 
 * Renders sensor coverage patterns, detection zones, and range information:
 * - Range rings and distance markers
 * - 3D coverage cones and volumes
 * - Field-of-view visualization
 * - Multi-range sensor patterns
 * - Detection probability heat maps
 * - Occlusion and blind spot visualization
 * 
 * Supports various sensor types:
 * - LIDAR: Range rings, angular resolution, blind spots
 * - Camera: Field-of-view cones, depth ranges
 * - Radar: Detection cones, range-azimuth patterns
 * - Sonar: Acoustic beam patterns, multi-beam arrays
 * - Proximity: Simple range spheres and detection zones
 * 
 * Common use cases:
 * - Robot sensor planning and validation
 * - Sensor fusion coverage analysis
 * - Surveillance system design
 * - Autonomous vehicle sensor layouts
 * - Multi-robot sensor coordination
 */
class SensorCoverage : public OpenGlObject {
public:
  enum class SensorType {
    kLidar,           // Rotating laser scanner
    kCamera,          // Perspective camera
    kRadar,           // Radio detection and ranging
    kSonar,           // Sound navigation and ranging
    kProximity,       // Simple proximity sensor
    kCustom           // User-defined pattern
  };

  enum class CoverageType {
    k2DRings,         // 2D range rings (top-down view)
    k3DCone,          // 3D cone or frustum
    k3DSphere,        // Spherical coverage (omnidirectional)
    k3DCylinder,      // Cylindrical detection zone
    kSectorSlice,     // Angular sector slice
    kMultiBeam        // Multiple beam patterns
  };

  enum class VisualizationMode {
    kWireframe,       // Only outlines and edges
    kSolid,           // Filled surfaces
    kTransparent,     // Semi-transparent volumes
    kHeatMap,         // Detection probability gradient
    kRangeRings,      // Concentric range indicators
    kBeamPattern      // Detailed beam visualization
  };

  SensorCoverage();
  explicit SensorCoverage(SensorType sensor_type);
  ~SensorCoverage();

  // Sensor configuration
  void SetSensorType(SensorType type);
  void SetCoverageType(CoverageType type);
  void SetSensorPosition(const glm::vec3& position);
  void SetSensorOrientation(const glm::vec3& direction, const glm::vec3& up = glm::vec3(0, 0, 1));

  SensorType GetSensorType() const { return sensor_type_; }
  CoverageType GetCoverageType() const { return coverage_type_; }
  glm::vec3 GetSensorPosition() const { return sensor_position_; }

  // Range and coverage parameters
  void SetRange(float min_range, float max_range);
  void SetAngularCoverage(float horizontal_fov, float vertical_fov = 0.0f);
  void SetAngularLimits(float min_angle, float max_angle);  // For partial coverage
  void SetDetectionProbability(float probability);
  void SetBeamWidth(float width);

  float GetMinRange() const { return min_range_; }
  float GetMaxRange() const { return max_range_; }
  float GetHorizontalFOV() const { return horizontal_fov_; }
  float GetVerticalFOV() const { return vertical_fov_; }

  // Visualization settings
  void SetVisualizationMode(VisualizationMode mode);
  void SetColor(const glm::vec3& color);
  void SetRangeColors(const glm::vec3& near_color, const glm::vec3& far_color);
  void SetTransparency(float alpha);
  void SetOutlineColor(const glm::vec3& outline_color);

  VisualizationMode GetVisualizationMode() const { return visualization_mode_; }
  glm::vec3 GetColor() const { return color_; }
  float GetTransparency() const { return alpha_; }

  // Range ring configuration
  void SetRangeRingCount(int count);
  void SetRangeRingSpacing(float spacing);  // Regular spacing
  void SetCustomRangeRings(const std::vector<float>& ranges);
  void SetShowRangeLabels(bool show_labels);

  int GetRangeRingCount() const { return range_ring_count_; }

  // Angular resolution and beam patterns
  void SetAngularResolution(float resolution_degrees);
  void SetBeamCount(int beam_count);  // For multi-beam sensors
  void SetBeamAngles(const std::vector<float>& angles);  // Custom beam directions
  void SetScanPattern(bool enable_scan, float scan_speed = 1.0f);

  float GetAngularResolution() const { return angular_resolution_; }
  int GetBeamCount() const { return beam_count_; }

  // Detection zone modifiers
  void SetBlindSpot(const glm::vec3& direction, float angle_width, float depth);
  void AddOcclusionZone(const glm::vec3& center, float radius);
  void SetDetectionThreshold(float threshold);  // Minimum detectable signal
  void ClearOcclusionZones();

  size_t GetOcclusionZoneCount() const { return occlusion_zones_.size(); }

  // Performance and detail levels
  void SetResolution(int radial_segments, int angular_segments);
  void SetLevelOfDetail(bool enable_lod, float distance_threshold = 20.0f);
  void SetAdaptiveResolution(bool enable_adaptive);

  // Animation and effects
  void SetPulsingEffect(bool enable_pulsing, float frequency = 1.0f);
  void SetScanAnimation(bool enable_scan, float scan_period = 3.0f);
  void SetFadeWithDistance(bool enable_fade, float fade_start_ratio = 0.8f);
  void SetTimeParameter(float time);  // For animations

  bool GetPulsingEffect() const { return pulsing_enabled_; }
  bool GetScanAnimation() const { return scan_enabled_; }

  // Sensor-specific utilities
  bool IsPointInCoverage(const glm::vec3& point) const;
  float GetDetectionProbabilityAtPoint(const glm::vec3& point) const;
  glm::vec3 GetPointOnCoverageBoundary(float azimuth, float elevation = 0.0f) const;
  std::vector<glm::vec3> GetCoverageBoundaryPoints(int num_points) const;

  // Intersection and overlap analysis
  bool OverlapsWith(const SensorCoverage& other) const;
  std::vector<glm::vec3> GetOverlapBoundary(const SensorCoverage& other) const;
  float GetCoverageVolume() const;
  float GetCoverageArea() const;  // For 2D sensors

  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return main_vao_ != 0; }

private:
  struct OcclusionZone {
    glm::vec3 center;
    float radius;
  };

  void UpdateGeometry();
  void Generate2DRings();
  void Generate3DCone();
  void Generate3DSphere();
  void Generate3DCylinder();
  void GenerateSectorSlice();
  void GenerateMultiBeam();
  void GenerateRangeRings();
  void GenerateBeamPattern();
  void ApplyHeatMapColoring();
  void UpdateTransformMatrix();
  glm::vec3 ComputeDetectionColor(float distance, float angle) const;
  bool IsPointOccluded(const glm::vec3& point) const;

  // Sensor properties
  SensorType sensor_type_;
  CoverageType coverage_type_;
  glm::vec3 sensor_position_;
  glm::vec3 sensor_direction_;
  glm::vec3 sensor_up_;

  // Coverage parameters
  float min_range_;
  float max_range_;
  float horizontal_fov_;        // Horizontal field of view (radians)
  float vertical_fov_;          // Vertical field of view (radians)
  float min_angle_;             // Minimum angle limit
  float max_angle_;             // Maximum angle limit
  float detection_probability_;
  float beam_width_;

  // Visualization
  VisualizationMode visualization_mode_;
  glm::vec3 color_;
  glm::vec3 near_color_;
  glm::vec3 far_color_;
  float alpha_;
  glm::vec3 outline_color_;

  // Range rings
  int range_ring_count_;
  float range_ring_spacing_;
  std::vector<float> custom_range_rings_;
  bool show_range_labels_;

  // Angular properties
  float angular_resolution_;
  int beam_count_;
  std::vector<float> beam_angles_;
  bool scan_pattern_enabled_;
  float scan_speed_;

  // Detection zones
  glm::vec3 blind_spot_direction_;
  float blind_spot_angle_;
  float blind_spot_depth_;
  std::vector<OcclusionZone> occlusion_zones_;
  float detection_threshold_;

  // Geometry resolution
  int radial_segments_;
  int angular_segments_;
  bool lod_enabled_;
  float lod_distance_threshold_;
  bool adaptive_resolution_;

  // Animation
  bool pulsing_enabled_;
  float pulsing_frequency_;
  bool scan_enabled_;
  float scan_period_;
  bool fade_with_distance_;
  float fade_start_ratio_;
  float time_parameter_;

  // Computed transform
  glm::mat4 transform_matrix_;

  // Geometry data
  std::vector<glm::vec3> surface_vertices_;
  std::vector<glm::vec3> surface_normals_;
  std::vector<glm::vec3> surface_colors_;
  std::vector<uint32_t> surface_indices_;

  std::vector<glm::vec3> ring_vertices_;
  std::vector<glm::vec3> ring_colors_;
  std::vector<uint32_t> ring_indices_;

  std::vector<glm::vec3> outline_vertices_;
  std::vector<uint32_t> outline_indices_;

  // OpenGL resources
  uint32_t main_vao_, main_vbo_, main_normal_vbo_, main_color_vbo_, main_ebo_;
  uint32_t ring_vao_, ring_vbo_, ring_color_vbo_, ring_ebo_;
  uint32_t outline_vao_, outline_vbo_, outline_ebo_;

  ShaderProgram surface_shader_;
  ShaderProgram ring_shader_;
  ShaderProgram outline_shader_;

  // Update flags
  bool needs_geometry_update_;
  bool needs_transform_update_;
};

} // namespace quickviz

#endif // QUICKVIZ_SENSOR_COVERAGE_HPP