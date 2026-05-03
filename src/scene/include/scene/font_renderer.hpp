/**
 * @file font_renderer.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Font rendering system using STB TrueType for high-quality text
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_FONT_RENDERER_HPP
#define QUICKVIZ_FONT_RENDERER_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <glm/glm.hpp>

namespace quickviz {

/**
 * @brief Font renderer using STB TrueType for high-quality text rendering
 * 
 * This class manages font loading, glyph atlas generation, and provides
 * utilities for text rendering with proper kerning and metrics.
 */
class FontRenderer {
public:
  struct GlyphInfo {
    float advance_x;      // Advance to next character
    float bearing_x;      // Offset from baseline to left of glyph
    float bearing_y;      // Offset from baseline to top of glyph  
    float width;          // Width of glyph
    float height;         // Height of glyph
    float tex_x0, tex_y0; // Texture coordinates (top-left)
    float tex_x1, tex_y1; // Texture coordinates (bottom-right)
  };
  
  struct TextMetrics {
    float width;          // Total width of text
    float height;         // Total height of text
    float ascent;         // Distance from baseline to top
    float descent;        // Distance from baseline to bottom
  };

  FontRenderer();
  ~FontRenderer();
  
  // Initialize with font data
  bool Initialize(const unsigned char* font_data, size_t data_size, float font_size = 24.0f);
  bool InitializeFromFile(const std::string& font_path, float font_size = 24.0f);
  
  // Initialize with built-in OpenSans Bold font
  bool InitializeWithOpenSans(float font_size = 24.0f);
  
  // Get font atlas texture
  unsigned int GetAtlasTexture() const { return atlas_texture_; }
  int GetAtlasWidth() const { return atlas_width_; }
  int GetAtlasHeight() const { return atlas_height_; }
  
  // Text metrics
  TextMetrics GetTextMetrics(const std::string& text) const;
  float GetLineHeight() const { return line_height_; }
  
  // Get glyph information
  const GlyphInfo* GetGlyph(char c) const;
  
  // Generate vertex data for text rendering
  struct TextVertex {
    glm::vec3 position;
    glm::vec2 tex_coord;
  };
  
  std::vector<TextVertex> GenerateTextVertices(
    const std::string& text,
    const glm::vec3& position = glm::vec3(0.0f),
    float scale = 1.0f
  ) const;
  
  // Check if initialized
  bool IsInitialized() const { return initialized_; }

private:
  void GenerateAtlas(const unsigned char* font_data, float font_size);
  void CreateGlyphInfo(int codepoint, int x, int y, int w, int h, 
                       float advance, float lsb, float baseline);
  
  bool initialized_ = false;
  unsigned int atlas_texture_ = 0;
  int atlas_width_ = 512;
  int atlas_height_ = 512;
  float font_size_ = 24.0f;
  float line_height_ = 0.0f;
  float ascent_ = 0.0f;
  float descent_ = 0.0f;
  
  std::unordered_map<char, GlyphInfo> glyphs_;
  std::vector<unsigned char> atlas_data_;
  
  // STB TrueType font data
  void* stb_font_info_ = nullptr;  // stbtt_fontinfo*
  std::vector<unsigned char> font_buffer_;
};

} // namespace quickviz

#endif // QUICKVIZ_FONT_RENDERER_HPP