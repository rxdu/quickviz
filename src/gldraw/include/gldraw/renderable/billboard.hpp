/**
 * @file billboard.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Modern Billboard primitive for high-quality screen-aligned text labels
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_BILLBOARD_HPP
#define QUICKVIZ_BILLBOARD_HPP

#include <string>
#include <memory>
#include <vector>
#include <glm/glm.hpp>

#include "gldraw/renderable/geometric_primitive.hpp"
#include "gldraw/font_renderer.hpp"
#include "../shader_program.hpp"

namespace quickviz {

/**
 * @brief Modern Billboard primitive for professional screen-aligned text
 * 
 * Provides high-quality text rendering using STB TrueType with:
 * - High-quality font atlas rendering with proper kerning
 * - Multiple billboard modes for 3D annotation
 * - Unified selection support via GeometricPrimitive base
 * - Material-based highlighting and visual feedback
 * - OpenSans font integration
 * 
 * Replaces the primitive Text3D implementation with modern font rendering.
 */
class Billboard : public GeometricPrimitive {
public:
  enum class Mode {
    kSphere,     // Always face camera (both rotation axes)
    kCylinder,   // Face camera horizontally only (vertical axis rotation)
    kFixed       // Fixed orientation in world space
  };
  
  enum class Alignment {
    kLeft,
    kCenter,
    kRight
  };
  
  enum class VerticalAlignment {
    kTop,
    kMiddle,
    kBottom
  };

  Billboard();
  explicit Billboard(const std::string& text);
  ~Billboard();

  // Text content and properties
  void SetText(const std::string& text);
  void SetAlignment(Alignment align, VerticalAlignment vertical_align = VerticalAlignment::kMiddle);
  void SetFontSize(float size_pixels); // Size in screen pixels
  
  // Billboard behavior
  void SetBillboardMode(Mode mode);
  
  // Background and visual effects
  void SetBackgroundEnabled(bool enabled);
  void SetBackgroundColor(const glm::vec4& color);
  void SetBackgroundPadding(float padding);
  void SetOutlineEnabled(bool enabled);
  void SetOutlineColor(const glm::vec3& color);
  void SetOutlineWidth(float width);
  
  // =================================================================
  // GeometricPrimitive Interface Implementation
  // =================================================================
  
  // Transform interface
  void SetPosition(const glm::vec3& position);
  glm::vec3 GetPosition() const { return position_; }
  void SetTransform(const glm::mat4& transform) override;
  glm::mat4 GetTransform() const override;
  
  // Geometry calculations
  float GetVolume() const override { return 0.0f; } // 2D primitive
  float GetSurfaceArea() const override;
  glm::vec3 GetCentroid() const override { return position_; }
  std::pair<glm::vec3, glm::vec3> GetBoundingBox() const override;
  
  // OpenGL resource management
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_ != 0; }

  // Utility methods
  const std::string& GetText() const { return text_; }
  glm::vec2 GetTextDimensions() const; // Screen-space dimensions in pixels
  float GetFontSize() const { return font_size_; }

protected:
  // =================================================================
  // Template Method Implementation
  // =================================================================
  
  void PrepareShaders(const glm::mat4& mvp_matrix, const glm::mat4& model_matrix) override;
  void RenderSolid() override;
  void RenderWireframe() override;
  void RenderPoints() override;
  
  // Override ID rendering for billboard-specific geometry
  void RenderIdBuffer(const glm::mat4& mvp_matrix) override;

private:
  void GenerateGeometry();
  void UpdateGpuBuffers();
  void SetupShaders();
  glm::mat4 CalculateBillboardMatrix(const glm::mat4& view_matrix) const;
  glm::vec2 CalculateTextOffset() const;
  void InitializeFontRenderer();

  // Text properties
  std::string text_;
  glm::vec3 position_ = glm::vec3(0.0f);
  float font_size_ = 16.0f; // Size in pixels
  
  // Font rendering
  static std::shared_ptr<FontRenderer> font_renderer_;
  static bool font_renderer_initialized_;
  
  // Alignment
  Alignment alignment_ = Alignment::kCenter;
  VerticalAlignment vertical_alignment_ = VerticalAlignment::kMiddle;
  
  // Billboard behavior
  Mode billboard_mode_ = Mode::kSphere;
  
  // Visual effects
  bool background_enabled_ = false;
  glm::vec4 background_color_ = glm::vec4(0.0f, 0.0f, 0.0f, 0.8f);
  float background_padding_ = 4.0f; // Padding in pixels
  bool outline_enabled_ = false;
  glm::vec3 outline_color_ = glm::vec3(0.0f);
  float outline_width_ = 1.0f;
  
  // OpenGL resources
  uint32_t vao_ = 0;
  uint32_t vbo_ = 0;
  uint32_t ebo_ = 0;
  uint32_t texture_ = 0; // Font atlas texture
  
  // Geometry data
  std::vector<glm::vec3> vertices_;
  std::vector<glm::vec2> tex_coords_;
  std::vector<uint32_t> indices_;
  
  // Shaders
  ShaderProgram billboard_shader_;
  ShaderProgram background_shader_;
  
  // Cached text metrics
  mutable glm::vec2 text_dimensions_ = glm::vec2(0.0f);
  mutable bool text_dimensions_dirty_ = true;
  
  // Billboard transformation matrices (stored during PrepareShaders)
  mutable glm::mat4 stored_mvp_matrix_ = glm::mat4(1.0f);
  mutable glm::mat4 stored_model_matrix_ = glm::mat4(1.0f);
};

} // namespace quickviz

#endif // QUICKVIZ_BILLBOARD_HPP