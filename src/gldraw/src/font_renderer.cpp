/**
 * @file font_renderer.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Font rendering implementation using STB TrueType
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/font_renderer.hpp"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>
#include <cassert>

#include "glad/glad.h"
#include "core/fonts/opensans_bold.hpp"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

// Extracted STB decompression implementation (from stb.h deprecated)
// This avoids dependency on deprecated code that might be removed
using stb_uint = unsigned int;
using stb_uchar = unsigned char;

// Forward declarations
static stb_uint stb_decompress_length(stb_uchar *input);
static stb_uint stb_decompress(stb_uchar *output, stb_uchar *input, stb_uint length);
static stb_uint stb_adler32(stb_uint adler32, stb_uchar *buffer, stb_uint buflen);

// Implementation
static stb_uint stb_decompress_length(stb_uchar *input)
{
   return (input[8] << 24) + (input[9] << 16) + (input[10] << 8) + input[11];
}

static stb_uint stb_adler32(stb_uint adler32, stb_uchar *buffer, stb_uint buflen)
{
   const unsigned long ADLER_MOD = 65521;
   unsigned long s1 = adler32 & 0xffff, s2 = adler32 >> 16;
   unsigned long blocklen, i;

   blocklen = buflen % 5552;
   while (buflen) {
      for (i=0; i + 7 < blocklen; i += 8) {
         s1 += buffer[0], s2 += s1;
         s1 += buffer[1], s2 += s1;
         s1 += buffer[2], s2 += s1;
         s1 += buffer[3], s2 += s1;
         s1 += buffer[4], s2 += s1;
         s1 += buffer[5], s2 += s1;
         s1 += buffer[6], s2 += s1;
         s1 += buffer[7], s2 += s1;

         buffer += 8;
      }

      for (; i < blocklen; ++i)
         s1 += *buffer++, s2 += s1;

      s1 %= ADLER_MOD, s2 %= ADLER_MOD;
      buflen -= blocklen;
      blocklen = 5552;
   }
   return (s2 << 16) + s1;
}

// Decompressor state
static unsigned char *stb__barrier;
static unsigned char *stb__barrier2;
static unsigned char *stb__barrier3;
static unsigned char *stb__barrier4;
static stb_uchar *stb__dout;

static void stb__match(stb_uchar *data, stb_uint length)
{
   // INVERSE of memmove... write each byte before copying the next...
   if (stb__dout + length > stb__barrier) { stb__dout += length; return; }
   if (data < stb__barrier4) { stb__dout = stb__barrier+1; return; }
   while (length--) *stb__dout++ = *data++;
}

static void stb__lit(stb_uchar *data, stb_uint length)
{
   if (stb__dout + length > stb__barrier) { stb__dout += length; return; }
   if (data < stb__barrier2) { stb__dout = stb__barrier+1; return; }
   memcpy(stb__dout, data, length);
   stb__dout += length;
}

#define stb__in2(x)   ((i[x] << 8) + i[(x)+1])
#define stb__in3(x)   ((i[x] << 16) + stb__in2((x)+1))
#define stb__in4(x)   ((i[x] << 24) + stb__in3((x)+1))

static stb_uchar *stb_decompress_token(stb_uchar *i)
{
   if (*i >= 0x20) { // use fewer if's for cases that expand small
      if (*i >= 0x80)       stb__match(stb__dout-i[1]-1, i[0] - 0x80 + 1), i += 2;
      else if (*i >= 0x40)  stb__match(stb__dout-(stb__in2(0) - 0x4000 + 1), i[2]+1), i += 3;
      else /* *i >= 0x20 */ stb__lit(i+1, i[0] - 0x20 + 1), i += 1 + (i[0] - 0x20 + 1);
   } else { // more ifs for cases that expand large, since overhead is amortized
      if (*i >= 0x18)       stb__match(stb__dout-(stb__in3(0) - 0x180000 + 1), i[3]+1), i += 4;
      else if (*i >= 0x10)  stb__match(stb__dout-(stb__in3(0) - 0x100000 + 1), stb__in2(3)+1), i += 5;
      else if (*i >= 0x08)  stb__lit(i+2, stb__in2(0) - 0x0800 + 1), i += 2 + (stb__in2(0) - 0x0800 + 1);
      else if (*i == 0x07)  stb__lit(i+3, stb__in2(1) + 1), i += 3 + (stb__in2(1) + 1);
      else if (*i == 0x06)  stb__match(stb__dout-(stb__in3(1)+1), i[4]+1), i += 5;
      else if (*i == 0x04)  stb__match(stb__dout-(stb__in3(1)+1), stb__in2(4)+1), i += 6;
   }
   return i;
}

static stb_uint stb_decompress(stb_uchar *output, stb_uchar *i, stb_uint length)
{
   stb_uint olen;
   if (stb__in4(0) != 0x57bC0000) return 0;
   if (stb__in4(4) != 0)          return 0; // error! stream is > 4GB
   olen = stb_decompress_length(i);
   stb__barrier2 = i;
   stb__barrier3 = i+length;
   stb__barrier = output + olen;
   stb__barrier4 = output;
   i += 16;

   stb__dout = output;
   while (1) {
      stb_uchar *old_i = i;
      i = stb_decompress_token(i);
      if (i == old_i) {
         if (*i == 0x05 && i[1] == 0xfa) {
            if (stb__dout != output + olen) return 0;
            if (stb_adler32(1, output, olen) != (stb_uint) stb__in4(2))
               return 0;
            return olen;
         } else {
            return 0; // NOTREACHED
         }
      }
      if (stb__dout > output + olen)
         return 0;
   }
}

namespace quickviz {

FontRenderer::FontRenderer() {
  stb_font_info_ = new stbtt_fontinfo();
}

FontRenderer::~FontRenderer() {
  // Only delete OpenGL resources if there's still a valid context
  // This prevents crashes during program exit when context is already destroyed
  if (atlas_texture_ != 0) {
    try {
      // Check if we can make OpenGL calls safely
      GLint current_texture = 0;
      glGetIntegerv(GL_TEXTURE_BINDING_2D, &current_texture);
      glDeleteTextures(1, &atlas_texture_);
    } catch (...) {
      // Ignore OpenGL errors during destruction
    }
  }
  
  // Safe cleanup of STB font info
  if (stb_font_info_ != nullptr) {
    delete static_cast<stbtt_fontinfo*>(stb_font_info_);
    stb_font_info_ = nullptr;
  }
}

bool FontRenderer::Initialize(const unsigned char* font_data, size_t data_size, float font_size) {
  if (initialized_) {
    return true;
  }
  
  // Copy font data
  font_buffer_.resize(data_size);
  std::memcpy(font_buffer_.data(), font_data, data_size);
  
  // Initialize STB TrueType
  auto* font_info = static_cast<stbtt_fontinfo*>(stb_font_info_);
  if (!stbtt_InitFont(font_info, font_buffer_.data(), 0)) {
    std::cerr << "FontRenderer: Failed to initialize font" << std::endl;
    return false;
  }
  
  font_size_ = font_size;
  GenerateAtlas(font_buffer_.data(), font_size);
  
  initialized_ = true;
  return true;
}

bool FontRenderer::InitializeFromFile(const std::string& font_path, float font_size) {
  std::ifstream file(font_path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    std::cerr << "FontRenderer: Failed to open font file: " << font_path << std::endl;
    return false;
  }
  
  size_t file_size = file.tellg();
  file.seekg(0, std::ios::beg);
  
  std::vector<unsigned char> buffer(file_size);
  if (!file.read(reinterpret_cast<char*>(buffer.data()), file_size)) {
    std::cerr << "FontRenderer: Failed to read font file: " << font_path << std::endl;
    return false;
  }
  
  return Initialize(buffer.data(), buffer.size(), font_size);
}

bool FontRenderer::InitializeWithOpenSans(float font_size) {
  // Decompress the embedded font data
  unsigned int compressed_size = OpenSansBold_compressed_size;
  const unsigned char* compressed_data = reinterpret_cast<const unsigned char*>(OpenSansBold_compressed_data);
  
  // Get decompressed size (cast away const for STB API)
  unsigned int decompressed_size = stb_decompress_length(const_cast<stb_uchar*>(compressed_data));
  if (decompressed_size == 0) {
    std::cerr << "FontRenderer: Invalid compressed font data" << std::endl;
    return false;
  }
  
  // Allocate buffer for decompressed data
  std::vector<unsigned char> decompressed_data(decompressed_size);
  
  // Decompress (cast away const for STB API)
  unsigned int actual_size = stb_decompress(decompressed_data.data(), const_cast<stb_uchar*>(compressed_data), compressed_size);
  
  if (actual_size != decompressed_size) {
    std::cerr << "FontRenderer: Failed to decompress font data (expected: " 
              << decompressed_size << ", got: " << actual_size << ")" << std::endl;
    return false;
  }
  
  // Initialize with decompressed font data
  bool success = Initialize(decompressed_data.data(), decompressed_size, font_size);
  if (!success) {
    std::cerr << "FontRenderer: Failed to initialize with OpenSans Bold" << std::endl;
  }
  return success;
}

void FontRenderer::GenerateAtlas(const unsigned char* font_data, float font_size) {
  auto* font_info = static_cast<stbtt_fontinfo*>(stb_font_info_);
  
  // Calculate scale for pixel height
  float scale = stbtt_ScaleForPixelHeight(font_info, font_size);
  
  // Get font metrics
  int ascent, descent, line_gap;
  stbtt_GetFontVMetrics(font_info, &ascent, &descent, &line_gap);
  ascent_ = ascent * scale;
  descent_ = descent * scale;
  line_height_ = (ascent - descent + line_gap) * scale;
  
  // Generate atlas for ASCII printable characters (32-126)
  const int first_char = 32;
  const int char_count = 95;
  
  // Allocate atlas bitmap
  atlas_width_ = 512;
  atlas_height_ = 512;
  atlas_data_.resize(atlas_width_ * atlas_height_);
  std::fill(atlas_data_.begin(), atlas_data_.end(), 0);
  
  // Pack glyphs into atlas
  int x = 0;
  int y = 0;
  int row_height = 0;
  
  for (int c = first_char; c < first_char + char_count; ++c) {
    int advance, lsb;
    stbtt_GetCodepointHMetrics(font_info, c, &advance, &lsb);
    
    int x0, y0, x1, y1;
    stbtt_GetCodepointBitmapBox(font_info, c, scale, scale, &x0, &y0, &x1, &y1);
    
    int glyph_width = x1 - x0;
    int glyph_height = y1 - y0;
    
    // Move to next row if needed
    if (x + glyph_width >= atlas_width_) {
      x = 0;
      y += row_height + 1;
      row_height = 0;
    }
    
    // Check if we've run out of space
    if (y + glyph_height >= atlas_height_) {
      std::cerr << "FontRenderer: Atlas too small for font size" << std::endl;
      break;
    }
    
    // Render glyph to atlas
    if (glyph_width > 0 && glyph_height > 0) {
      stbtt_MakeCodepointBitmap(font_info, 
                                atlas_data_.data() + y * atlas_width_ + x,
                                glyph_width, glyph_height, atlas_width_,
                                scale, scale, c);
    }
    
    // Store glyph info
    CreateGlyphInfo(c, x, y, glyph_width, glyph_height, 
                   advance * scale, lsb * scale, -y0);
    
    // Update position
    x += glyph_width + 1;
    row_height = std::max(row_height, glyph_height);
  }
  
  // Create OpenGL texture
  glGenTextures(1, &atlas_texture_);
  glBindTexture(GL_TEXTURE_2D, atlas_texture_);
  
  // Upload atlas to GPU
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, atlas_width_, atlas_height_, 0, 
              GL_RED, GL_UNSIGNED_BYTE, atlas_data_.data());
  
  // Set texture parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  
  // Generate mipmaps for better quality at different scales
  glGenerateMipmap(GL_TEXTURE_2D);
  
  glBindTexture(GL_TEXTURE_2D, 0);
}

void FontRenderer::CreateGlyphInfo(int codepoint, int x, int y, int w, int h,
                                  float advance, float lsb, float baseline) {
  GlyphInfo info;
  info.advance_x = advance;
  info.bearing_x = lsb;
  info.bearing_y = baseline;
  info.width = w;
  info.height = h;
  info.tex_x0 = static_cast<float>(x) / atlas_width_;
  info.tex_y0 = static_cast<float>(y) / atlas_height_;
  info.tex_x1 = static_cast<float>(x + w) / atlas_width_;
  info.tex_y1 = static_cast<float>(y + h) / atlas_height_;
  
  glyphs_[static_cast<char>(codepoint)] = info;
}

FontRenderer::TextMetrics FontRenderer::GetTextMetrics(const std::string& text) const {
  TextMetrics metrics;
  metrics.width = 0.0f;
  metrics.height = line_height_;
  metrics.ascent = ascent_;
  metrics.descent = descent_;
  
  for (char c : text) {
    auto it = glyphs_.find(c);
    if (it != glyphs_.end()) {
      metrics.width += it->second.advance_x;
    }
  }
  
  return metrics;
}

const FontRenderer::GlyphInfo* FontRenderer::GetGlyph(char c) const {
  auto it = glyphs_.find(c);
  if (it != glyphs_.end()) {
    return &it->second;
  }
  
  // Return space glyph for unknown characters
  auto space_it = glyphs_.find(' ');
  if (space_it != glyphs_.end()) {
    return &space_it->second;
  }
  
  return nullptr;
}

std::vector<FontRenderer::TextVertex> FontRenderer::GenerateTextVertices(
    const std::string& text,
    const glm::vec3& position,
    float scale) const {
  
  std::vector<TextVertex> vertices;
  vertices.reserve(text.length() * 6); // 6 vertices per character (2 triangles)
  
  float x = position.x;
  float y = position.y;
  
  for (char c : text) {
    const GlyphInfo* glyph = GetGlyph(c);
    if (!glyph) continue;
    
    if (c != ' ' && glyph->width > 0 && glyph->height > 0) {
      // Apply scale to all glyph dimensions
      float x0 = x + glyph->bearing_x * scale;
      float y0 = y + glyph->bearing_y * scale;
      float x1 = x0 + glyph->width * scale;
      float y1 = y0 - glyph->height * scale;
      
      // First triangle - normal texture coordinates
      vertices.push_back({glm::vec3(x0, y0, position.z), glm::vec2(glyph->tex_x0, glyph->tex_y0)});
      vertices.push_back({glm::vec3(x1, y0, position.z), glm::vec2(glyph->tex_x1, glyph->tex_y0)});
      vertices.push_back({glm::vec3(x0, y1, position.z), glm::vec2(glyph->tex_x0, glyph->tex_y1)});
      
      // Second triangle - normal texture coordinates
      vertices.push_back({glm::vec3(x1, y0, position.z), glm::vec2(glyph->tex_x1, glyph->tex_y0)});
      vertices.push_back({glm::vec3(x1, y1, position.z), glm::vec2(glyph->tex_x1, glyph->tex_y1)});
      vertices.push_back({glm::vec3(x0, y1, position.z), glm::vec2(glyph->tex_x0, glyph->tex_y1)});
    }
    
    x += glyph->advance_x * scale;  // Apply scale to character advance
  }
  
  return vertices;
}

} // namespace quickviz