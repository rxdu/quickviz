/**
 * @file text3d.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief High-quality 3D text renderer using signed distance field fonts
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_TEXT3D_HPP
#define QUICKVIZ_TEXT3D_HPP

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <glm/glm.hpp>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

// Forward declarations (removed ImFont as no longer needed)

namespace quickviz {

/**
 * @brief High-quality 3D text renderer using signed distance field fonts
 * 
 * Renders crisp, scalable text in 3D space with proper typography metrics.
 * Uses a pre-rendered SDF font atlas for professional appearance suitable
 * for scientific visualization. Supports billboard functionality, alignment,
 * outlining, shadows, and background rectangles.
 * 
 * Features:
 * - Signed distance field rendering for crisp text at any scale
 * - Professional typography with proper metrics and spacing
 * - Multiple billboard modes for 3D annotation
 * - Text alignment and positioning options
 * - Visual effects: outlines, shadows, backgrounds
 * - Optimized for scientific and robotics visualization
 */
class Text3D : public OpenGlObject {
public:
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
  
  enum class BillboardMode {
    kNone,      // Fixed orientation in world space
    kSphere,    // Always face camera (both rotation axes)
    kCylinder   // Face camera horizontally only (vertical axis rotation)
  };

  Text3D();
  ~Text3D();

  // Text content and properties
  void SetText(const std::string& text);
  void SetPosition(const glm::vec3& position);
  void SetColor(const glm::vec3& color);
  void SetBackgroundColor(const glm::vec4& color); // With alpha for transparency
  void SetScale(float scale);
  
  // Alignment and orientation
  void SetAlignment(Alignment align, VerticalAlignment vertical_align = VerticalAlignment::kMiddle);
  void SetBillboardMode(BillboardMode mode);
  void SetRotation(const glm::vec3& rotation); // Euler angles (only used if billboard disabled)
  
  // Visual effects
  void SetOutline(bool enabled, const glm::vec3& outline_color = glm::vec3(0.0f), float outline_width = 0.1f);
  void SetShadow(bool enabled, const glm::vec2& shadow_offset = glm::vec2(0.02f, -0.02f), 
                 const glm::vec3& shadow_color = glm::vec3(0.0f));
  void SetBackgroundEnabled(bool enabled);
  void SetBackgroundPadding(float padding);
  
  // Font properties (simplified - using bitmap font approach)
  void SetFontSize(float size);
  
  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_ != 0; }

  // Utility methods
  const std::string& GetText() const { return text_; }
  const glm::vec3& GetPosition() const { return position_; }
  float GetScale() const { return scale_; }
  glm::vec3 GetTextDimensions() const; // Estimated text dimensions in world units

private:
  void GenerateTextGeometry();
  void SetupTextShader();
  void SetupBackgroundShader();
  glm::mat4 CalculateBillboardMatrix(const glm::mat4& view) const;
  glm::vec2 CalculateTextOffset() const;

  // Text properties
  std::string text_;
  glm::vec3 position_;
  glm::vec3 color_;
  glm::vec4 background_color_;
  float scale_;
  float font_size_;
  
  // Alignment and orientation
  Alignment alignment_;
  VerticalAlignment vertical_alignment_;
  BillboardMode billboard_mode_;
  glm::vec3 rotation_;
  
  // Visual effects
  bool outline_enabled_;
  glm::vec3 outline_color_;
  float outline_width_;
  bool shadow_enabled_;
  glm::vec2 shadow_offset_;
  glm::vec3 shadow_color_;
  bool background_enabled_;
  float background_padding_;
  
  // OpenGL resources
  unsigned int vao_, vbo_, ebo_;
  unsigned int background_vao_, background_vbo_;
  ShaderProgram text_shader_;
  ShaderProgram background_shader_;
  
  // Geometry data
  std::vector<glm::vec3> vertices_;
  std::vector<glm::vec2> tex_coords_;
  std::vector<uint32_t> indices_;
  std::vector<glm::vec3> background_vertices_;
  
  // SDF font system
  struct Character {
    float advance;      // Horizontal advance
    float bearing_x;    // Bearing X (offset from baseline)
    float bearing_y;    // Bearing Y (offset from baseline)
    float width;        // Glyph width
    float height;       // Glyph height
    glm::vec4 uv_rect;  // UV coordinates in atlas (x, y, w, h)
  };
  
  static constexpr int kFontAtlasSize = 512;
  static constexpr float kFontSize = 48.0f;  // Size used to generate the atlas
  
  unsigned int font_atlas_;
  std::unordered_map<char, Character> characters_;
  bool font_loaded_;
  
  void LoadBitmapFont();
  void LoadBitmapFontFallback();
  void DrawCleanCharacter(unsigned char* atlas_data, int atlas_size,
                         int char_x, int char_y, int char_width, int char_height, 
                         char character);
  Character GetCharacter(char c) const;
};

} // namespace quickviz

#endif // QUICKVIZ_TEXT3D_HPP