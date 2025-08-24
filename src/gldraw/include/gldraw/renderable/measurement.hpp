/**
 * @file measurement.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-24
 * @brief Measurement renderer for distance lines, angle arcs, and dimensional callouts
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_MEASUREMENT_HPP
#define QUICKVIZ_MEASUREMENT_HPP

#include <vector>
#include <string>
#include <memory>
#include <glm/glm.hpp>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

class Text3D; // Forward declaration

/**
 * @brief Measurement renderer for engineering and robotics visualization
 * 
 * Provides various measurement primitives:
 * - Distance measurements with extension lines
 * - Angular measurements with arcs and degree labels
 * - Dimensional callouts with arrows and text
 * - Coordinate measurements with tick marks
 * - Multi-segment measurements with cumulative totals
 * 
 * Common use cases:
 * - Engineering drawings and CAD visualization
 * - Robot workspace measurements
 * - Sensor range and field-of-view annotations
 * - Path length and trajectory analysis
 * - Clearance and tolerance visualization
 */
class Measurement : public OpenGlObject {
public:
  enum class MeasurementType {
    kDistance,        // Linear distance between two points
    kAngle,           // Angular measurement between three points or two vectors
    kRadius,          // Radius measurement from center to point
    kDiameter,        // Diameter measurement through center
    kCoordinate,      // Single coordinate value with tick marks
    kMultiSegment     // Chain of connected distance measurements
  };

  enum class LabelPosition {
    kCenter,          // Label at measurement center
    kAbove,           // Label above the measurement line
    kBelow,           // Label below the measurement line
    kStart,           // Label near start point
    kEnd,             // Label near end point
    kCustom           // User-specified position
  };

  enum class LineStyle {
    kSolid,           // Solid line
    kDashed,          // Dashed line
    kDotted,          // Dotted line
    kDashDot          // Alternating dash-dot pattern
  };

  Measurement();
  explicit Measurement(MeasurementType type);
  ~Measurement();

  // Measurement definition
  void SetMeasurementType(MeasurementType type);
  void SetPoints(const std::vector<glm::vec3>& points);
  void SetTwoPointDistance(const glm::vec3& start, const glm::vec3& end);
  void SetThreePointAngle(const glm::vec3& vertex, const glm::vec3& point1, const glm::vec3& point2);
  void SetRadius(const glm::vec3& center, const glm::vec3& point_on_circle);
  void SetDiameter(const glm::vec3& center, const glm::vec3& point1, const glm::vec3& point2);
  void SetCoordinate(const glm::vec3& point, const glm::vec3& direction, float range);

  MeasurementType GetMeasurementType() const { return measurement_type_; }
  const std::vector<glm::vec3>& GetPoints() const { return measurement_points_; }

  // Appearance settings
  void SetColor(const glm::vec3& color);
  void SetLineWidth(float width);
  void SetLineStyle(LineStyle style);
  void SetArrowStyle(bool show_arrows, float arrow_size = 0.1f);
  void SetExtensionLines(bool show_extensions, float extension_length = 0.5f);

  glm::vec3 GetColor() const { return color_; }
  float GetLineWidth() const { return line_width_; }

  // Label control
  void SetShowLabel(bool show);
  void SetLabelText(const std::string& text);
  void SetLabelPosition(LabelPosition position);
  void SetLabelOffset(const glm::vec3& offset);
  void SetLabelScale(float scale);
  void SetLabelColor(const glm::vec3& color);
  void SetLabelBackgroundColor(const glm::vec3& color, float alpha = 0.8f);

  bool GetShowLabel() const { return show_label_; }
  std::string GetLabelText() const;
  LabelPosition GetLabelPosition() const { return label_position_; }

  // Precision and units
  void SetPrecision(int decimal_places);
  void SetUnits(const std::string& unit_string);
  void SetAutoUpdate(bool auto_update);  // Automatically update label when points change

  int GetPrecision() const { return precision_; }
  std::string GetUnits() const { return units_; }

  // Arc settings (for angle measurements)
  void SetArcRadius(float radius);
  void SetArcResolution(int segments);
  void SetShowArcTicks(bool show_ticks, int tick_count = 5);

  float GetArcRadius() const { return arc_radius_; }

  // Measurement values
  float GetDistanceValue() const;
  float GetAngleValue() const;       // Returns angle in radians
  float GetAngleValueDegrees() const; // Returns angle in degrees

  // Visual effects
  void SetHighlight(bool highlighted, const glm::vec3& highlight_color = glm::vec3(1.0f, 1.0f, 0.0f));
  void SetTransparency(float alpha);
  void SetGlow(bool enable_glow, float intensity = 1.0f);

  bool GetHighlight() const { return highlighted_; }
  float GetTransparency() const { return alpha_; }

  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return main_vao_ != 0; }

private:
  void UpdateGeometry();
  void UpdateLabel();
  void GenerateDistanceGeometry();
  void GenerateAngleGeometry();
  void GenerateRadiusGeometry();
  void GenerateDiameterGeometry();
  void GenerateCoordinateGeometry();
  void GenerateMultiSegmentGeometry();
  void GenerateArrows();
  void GenerateExtensionLines();
  void GenerateLineWithStyle(const glm::vec3& start, const glm::vec3& end, 
                            std::vector<glm::vec3>& vertices, std::vector<uint32_t>& indices);
  glm::vec3 ComputeLabelPosition();
  std::string FormatValue(float value) const;

  // Measurement data
  MeasurementType measurement_type_;
  std::vector<glm::vec3> measurement_points_;
  
  // Appearance
  glm::vec3 color_;
  float line_width_;
  LineStyle line_style_;
  bool show_arrows_;
  float arrow_size_;
  bool show_extensions_;
  float extension_length_;
  bool highlighted_;
  glm::vec3 highlight_color_;
  float alpha_;
  bool glow_enabled_;
  float glow_intensity_;

  // Label properties
  bool show_label_;
  std::string label_text_;
  LabelPosition label_position_;
  glm::vec3 label_offset_;
  glm::vec3 label_color_;
  glm::vec3 label_bg_color_;
  float label_bg_alpha_;
  float label_scale_;
  bool auto_update_;
  int precision_;
  std::string units_;

  // Arc properties (for angles)
  float arc_radius_;
  int arc_resolution_;
  bool show_arc_ticks_;
  int arc_tick_count_;

  // Geometry data
  std::vector<glm::vec3> line_vertices_;
  std::vector<glm::vec3> line_colors_;
  std::vector<uint32_t> line_indices_;
  
  std::vector<glm::vec3> arrow_vertices_;
  std::vector<glm::vec3> arrow_colors_;
  std::vector<uint32_t> arrow_indices_;

  // OpenGL resources
  uint32_t main_vao_, main_vbo_, main_color_vbo_, main_ebo_;
  uint32_t arrow_vao_, arrow_vbo_, arrow_color_vbo_, arrow_ebo_;
  ShaderProgram line_shader_;
  ShaderProgram arrow_shader_;

  // Text label
  std::unique_ptr<Text3D> label_text_obj_;

  // Update flags
  bool needs_geometry_update_;
  bool needs_label_update_;
};

} // namespace quickviz

#endif // QUICKVIZ_MEASUREMENT_HPP