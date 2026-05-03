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

#include "scene/interface/opengl_object.hpp"
#include "scene/font_renderer.hpp"
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
class Billboard : public OpenGlObject {
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
  // OpenGlObject Interface Implementation
  // =================================================================
  
  // Transform interface
  void SetPosition(const glm::vec3& position);
  glm::vec3 GetPosition() const { return position_; }
  void SetTransform(const glm::mat4& transform);
  glm::mat4 GetTransform() const;
  
  // Appearance settings
  void SetColor(const glm::vec3& color);
  void SetWireframeColor(const glm::vec3& color);
  void SetOpacity(float opacity);
  
  // OpenGL resource management
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_ != 0; }
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  
  // Selection interface
  bool SupportsSelection() const override { return true; }
  void SetHighlighted(bool highlighted) override;
  std::pair<glm::vec3, glm::vec3> GetBoundingBox() const override;
  
  // ID rendering support for GPU selection
  bool SupportsIdRendering() const override { return true; }
  void SetIdRenderMode(bool enabled) override { id_render_mode_ = enabled; }
  void SetIdColor(const glm::vec3& color) override { id_color_ = color; }

  // Utility methods
  const std::string& GetText() const { return text_; }
  glm::vec2 GetTextDimensions() const; // Screen-space dimensions in pixels
  float GetFontSize() const { return font_size_; }
  
  // Scaling configuration
  void SetPixelsToWorldScale(float scale) { pixels_to_world_scale_ = scale; GenerateGeometry(); }
  float GetPixelsToWorldScale() const { return pixels_to_world_scale_; }

private:
  // Internal rendering methods
  void DrawBillboard(const glm::mat4& mvp);
  void DrawIdBuffer(const glm::mat4& mvp);
  void GenerateGeometry();
  void UpdateGpuBuffers();
  void SetupShaders();
  glm::mat4 CalculateBillboardMatrix(const glm::mat4& view_matrix) const;
  glm::vec2 CalculateTextOffset() const;

  // Text properties
  std::string text_;
  glm::vec3 position_ = glm::vec3(0.0f);
  float font_size_ = 16.0f; // Size in pixels
  
  // Font rendering - local instance per Billboard
  std::unique_ptr<FontRenderer> font_renderer_;
  
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
  
  // Text appearance
  glm::vec3 color_ = glm::vec3(1.0f, 1.0f, 1.0f);  // White by default
  glm::vec3 wireframe_color_ = glm::vec3(1.0f, 1.0f, 0.0f);  // Yellow wireframe
  float opacity_ = 1.0f;
  
  // Selection state
  bool is_highlighted_ = false;
  glm::vec3 original_color_;
  glm::vec3 original_wireframe_color_;
  
  // ID rendering for GPU selection
  bool id_render_mode_ = false;
  glm::vec3 id_color_{0.0f};
  
  // Shaders
  ShaderProgram billboard_shader_;
  ShaderProgram background_shader_;
  ShaderProgram id_shader_;
  
  // Cached text metrics
  mutable glm::vec2 text_dimensions_ = glm::vec2(0.0f);
  mutable bool text_dimensions_dirty_ = true;
  
  // Scaling configuration
  float pixels_to_world_scale_ = 0.008f;  // Conversion factor: 1 pixel = 0.008 world units
  
  // Dirty flag for geometry updates
  bool needs_update_ = true;
};

} // namespace quickviz

#endif // QUICKVIZ_BILLBOARD_HPP