/**
 * @file path.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Path renderer for trajectory and motion planning visualization
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_PATH_HPP
#define QUICKVIZ_PATH_HPP

#include <vector>
#include <deque>
#include <glm/glm.hpp>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief Path renderer for trajectory and motion planning visualization
 * 
 * Renders smooth paths, trajectories, and motion planning results with:
 * - Line segments, smooth curves (Bezier, spline)
 * - Color encoding for velocity, time, cost, or other scalar values
 * - Directional indicators (arrows) along the path
 * - Multiple path segments with different properties
 * - Animated path tracing and growth effects
 * 
 * Common use cases:
 * - Robot trajectory visualization
 * - Motion planning results
 * - Path following visualization  
 * - Multi-waypoint navigation
 * - Obstacle avoidance paths
 */
class Path : public OpenGlObject {
public:
  enum class PathType {
    kLineSegments,    // Simple line segments between points
    kSmoothCurve,     // Smooth interpolated curve through points
    kBezierCurve,     // Bezier curve with control points
    kSpline           // Catmull-Rom spline interpolation
  };

  enum class ColorMode {
    kUniform,         // Single color for entire path
    kGradient,        // Interpolated color gradient along path
    kVelocity,        // Color encoded by velocity magnitude  
    kTime,            // Color encoded by time parameter
    kCost,            // Color encoded by cost/weight values
    kCustom           // User-provided colors per segment
  };

  enum class ArrowMode {
    kNone,            // No directional arrows
    kEndpoints,       // Arrows at start and end only
    kRegular,         // Arrows at regular intervals
    kCurvature,       // Arrows at high curvature points
    kAll              // Arrow at every path point
  };

  Path();
  explicit Path(const std::vector<glm::vec3>& points);
  ~Path();

  // Path definition
  void SetPoints(const std::vector<glm::vec3>& points);
  void AddPoint(const glm::vec3& point);
  void InsertPoint(size_t index, const glm::vec3& point);
  void RemovePoint(size_t index);
  void ClearPath();

  const std::vector<glm::vec3>& GetPoints() const { return control_points_; }
  size_t GetPointCount() const { return control_points_.size(); }

  // Path properties
  void SetPathType(PathType type);
  void SetLineWidth(float width);
  void SetSubdivisions(int subdivisions);  // For smooth curves
  void SetTension(float tension);          // For spline curves

  PathType GetPathType() const { return path_type_; }
  float GetLineWidth() const { return line_width_; }

  // Color control
  void SetColorMode(ColorMode mode);
  void SetColor(const glm::vec3& color);
  void SetColorGradient(const glm::vec3& start_color, const glm::vec3& end_color);
  void SetColors(const std::vector<glm::vec3>& colors);  // Custom colors per point
  void SetColorRange(const glm::vec2& range);            // Min/max for encoded values
  void SetScalarValues(const std::vector<float>& values); // For velocity/time/cost encoding

  ColorMode GetColorMode() const { return color_mode_; }
  glm::vec3 GetColor() const { return base_color_; }

  // Directional arrows
  void SetArrowMode(ArrowMode mode);
  void SetArrowSize(float size);
  void SetArrowSpacing(float spacing);     // Distance between arrows for kRegular mode
  void SetArrowColor(const glm::vec3& color);

  ArrowMode GetArrowMode() const { return arrow_mode_; }
  float GetArrowSize() const { return arrow_size_; }

  // Animation and effects
  void SetAnimationProgress(float progress);  // 0.0 to 1.0 for path tracing
  void SetGlowEffect(bool enable, float intensity = 1.0f);
  void SetTransparency(float alpha);

  float GetAnimationProgress() const { return animation_progress_; }
  bool GetGlowEffect() const { return glow_enabled_; }
  float GetTransparency() const { return alpha_; }

  // Path analysis
  float GetTotalLength() const;
  glm::vec3 GetPointAtDistance(float distance) const;
  glm::vec3 GetDirectionAtDistance(float distance) const;
  float GetCurvatureAtDistance(float distance) const;

  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return path_vao_ != 0; }

private:
  void GeneratePathGeometry();
  void GenerateLineSegments();
  void GenerateSmoothCurve();
  void GenerateBezierCurve();
  void GenerateSplineCurve();
  void GenerateArrows();
  void ComputePathColors();
  void UpdateGpuBuffers();

  // Utility functions
  glm::vec3 InterpolateSpline(const std::vector<glm::vec3>& points, float t) const;
  glm::vec3 InterpolateBezier(const std::vector<glm::vec3>& points, float t) const;
  glm::vec3 ColorFromScalar(float value) const;
  std::vector<float> ComputeSegmentLengths() const;

  // Control points
  std::vector<glm::vec3> control_points_;
  
  // Path properties
  PathType path_type_;
  float line_width_;
  int subdivisions_;
  float tension_;

  // Generated path geometry
  std::vector<glm::vec3> path_vertices_;
  std::vector<glm::vec3> path_colors_;
  std::vector<uint32_t> path_indices_;

  // Color properties
  ColorMode color_mode_;
  glm::vec3 base_color_;
  glm::vec3 gradient_start_;
  glm::vec3 gradient_end_;
  std::vector<glm::vec3> custom_colors_;
  std::vector<float> scalar_values_;
  glm::vec2 color_range_;

  // Arrow properties
  ArrowMode arrow_mode_;
  float arrow_size_;
  float arrow_spacing_;
  glm::vec3 arrow_color_;
  std::vector<glm::vec3> arrow_vertices_;
  std::vector<glm::vec3> arrow_colors_;
  std::vector<uint32_t> arrow_indices_;

  // Animation and effects
  float animation_progress_;
  bool glow_enabled_;
  float glow_intensity_;
  float alpha_;

  // OpenGL resources
  uint32_t path_vao_, path_vbo_, path_color_vbo_, path_ebo_;
  uint32_t arrow_vao_, arrow_vbo_, arrow_color_vbo_, arrow_ebo_;
  ShaderProgram path_shader_;
  ShaderProgram arrow_shader_;

  bool needs_geometry_update_;
  bool needs_color_update_;
  bool needs_arrow_update_;
};

} // namespace quickviz

#endif // QUICKVIZ_PATH_HPP