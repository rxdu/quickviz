/**
 * @file text3d.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Clean, professional 3D text renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/text3d.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include "glad/glad.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "gldraw/shader.hpp"

namespace quickviz {

Text3D::Text3D()
    : position_(0.0f),
      color_(1.0f),
      background_color_(0.0f, 0.0f, 0.0f, 0.8f),
      scale_(1.0f),
      font_size_(2.0f),
      alignment_(Alignment::kLeft),
      vertical_alignment_(VerticalAlignment::kMiddle),
      billboard_mode_(BillboardMode::kNone),
      rotation_(0.0f),
      outline_enabled_(false),
      outline_color_(0.0f),
      outline_width_(0.1f),
      shadow_enabled_(false),
      shadow_offset_(0.02f, -0.02f),
      shadow_color_(0.0f),
      background_enabled_(false),
      background_padding_(0.1f),
      vao_(0), vbo_(0), ebo_(0),
      background_vao_(0), background_vbo_(0),
      font_atlas_(0),
      font_loaded_(false) {
  AllocateGpuResources();
}

Text3D::~Text3D() {
  ReleaseGpuResources();
}

void Text3D::SetText(const std::string& text) {
  text_ = text;
  GenerateTextGeometry();
}

void Text3D::SetPosition(const glm::vec3& position) {
  position_ = position;
}

void Text3D::SetColor(const glm::vec3& color) {
  color_ = color;
}

void Text3D::SetBackgroundColor(const glm::vec4& color) {
  background_color_ = color;
}

void Text3D::SetScale(float scale) {
  scale_ = scale;
}

void Text3D::SetAlignment(Alignment align, VerticalAlignment vertical_align) {
  alignment_ = align;
  vertical_alignment_ = vertical_align;
}

void Text3D::SetBillboardMode(BillboardMode mode) {
  billboard_mode_ = mode;
}

void Text3D::SetRotation(const glm::vec3& rotation) {
  rotation_ = rotation;
}

void Text3D::SetOutline(bool enabled, const glm::vec3& outline_color, float outline_width) {
  outline_enabled_ = enabled;
  outline_color_ = outline_color;
  outline_width_ = outline_width;
}

void Text3D::SetShadow(bool enabled, const glm::vec2& shadow_offset, const glm::vec3& shadow_color) {
  shadow_enabled_ = enabled;
  shadow_offset_ = shadow_offset;
  shadow_color_ = shadow_color;
}

void Text3D::SetBackgroundEnabled(bool enabled) {
  background_enabled_ = enabled;
}

void Text3D::SetBackgroundPadding(float padding) {
  background_padding_ = padding;
}

void Text3D::SetFontSize(float size) {
  font_size_ = size;
}

void Text3D::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) {
    return;
  }
  
  // Load font
  LoadBitmapFont();
  
  // Setup shaders
  SetupTextShader();
  SetupBackgroundShader();
  
  // Generate OpenGL objects
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &ebo_);
  
  glGenVertexArrays(1, &background_vao_);
  glGenBuffers(1, &background_vbo_);
  
}

void Text3D::ReleaseGpuResources() noexcept {
  if (vao_ != 0) {
    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &ebo_);
    vao_ = vbo_ = ebo_ = 0;
  }
  
  if (background_vao_ != 0) {
    glDeleteVertexArrays(1, &background_vao_);
    glDeleteBuffers(1, &background_vbo_);
    background_vao_ = background_vbo_ = 0;
  }
  
  if (font_atlas_ != 0) {
    glDeleteTextures(1, &font_atlas_);
    font_atlas_ = 0;
  }
}

void Text3D::OnDraw(const glm::mat4& projection, const glm::mat4& view, const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    return;
  }
  if (text_.empty()) {
    return;
  }
  
  // Generate geometry if needed
  if (vertices_.empty()) {
    GenerateTextGeometry();
  }
  
  // Calculate model matrix
  glm::mat4 model = coord_transform * glm::translate(glm::mat4(1.0f), position_);
  
  // Apply billboard transformation if enabled
  if (billboard_mode_ != BillboardMode::kNone) {
    model *= CalculateBillboardMatrix(view);
  } else {
    // Apply rotation
    model = glm::rotate(model, rotation_.x, glm::vec3(1, 0, 0));
    model = glm::rotate(model, rotation_.y, glm::vec3(0, 1, 0));
    model = glm::rotate(model, rotation_.z, glm::vec3(0, 0, 1));
  }
  
  model = glm::scale(model, glm::vec3(scale_));
  
  // Draw text
  if (!vertices_.empty() && font_atlas_ != 0) {
    text_shader_.Use();
    text_shader_.SetUniform("uProjection", projection);
    text_shader_.SetUniform("uView", view);
    text_shader_.SetUniform("uModel", model);
    text_shader_.SetUniform("uTextColor", color_);
    
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, font_atlas_);
    text_shader_.SetUniform("uFontAtlas", 0);
    
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
  }
}

glm::vec3 Text3D::GetTextDimensions() const {
  if (text_.empty()) return glm::vec3(0.0f);
  
  float width = 0.0f;
  float height = font_size_;
  
  for (char c : text_) {
    auto ch = GetCharacter(c);
    width += ch.advance;
  }
  
  return glm::vec3(width * scale_, height * scale_, 0.0f);
}

void Text3D::GenerateTextGeometry() {
  vertices_.clear();
  tex_coords_.clear();
  indices_.clear();
  
  if (text_.empty()) return;
  
  float x = 0.0f;
  float y = 0.0f;
  uint32_t vertex_offset = 0;
  
  // Calculate text offset for alignment
  glm::vec2 offset = CalculateTextOffset();
  x += offset.x;
  y += offset.y;
  
  for (char c : text_) {
    if (c == '\n') {
      y += font_size_ * 1.2f; // Line spacing (positive since we flip Y later)
      x = offset.x;
      continue;
    }
    
    auto ch = GetCharacter(c);
    
    float xpos = x + ch.bearing_x;
    float ypos = y - (ch.height - ch.bearing_y);
    float w = ch.width;
    float h = ch.height;
    
    // Add vertices for this character (flip Y to correct orientation)
    vertices_.push_back(glm::vec3(xpos, -(ypos + h), 0.0f));     // Top-left
    vertices_.push_back(glm::vec3(xpos, -ypos, 0.0f));           // Bottom-left
    vertices_.push_back(glm::vec3(xpos + w, -ypos, 0.0f));       // Bottom-right
    vertices_.push_back(glm::vec3(xpos + w, -(ypos + h), 0.0f)); // Top-right
    
    // Add texture coordinates (flip V to match flipped vertices)
    tex_coords_.push_back(glm::vec2(ch.uv_rect.x, ch.uv_rect.y + ch.uv_rect.w));           // Top-left (flipped)
    tex_coords_.push_back(glm::vec2(ch.uv_rect.x, ch.uv_rect.y));                           // Bottom-left (flipped)
    tex_coords_.push_back(glm::vec2(ch.uv_rect.x + ch.uv_rect.z, ch.uv_rect.y));           // Bottom-right (flipped)
    tex_coords_.push_back(glm::vec2(ch.uv_rect.x + ch.uv_rect.z, ch.uv_rect.y + ch.uv_rect.w)); // Top-right (flipped)
    
    // Add indices for two triangles
    indices_.push_back(vertex_offset + 0);
    indices_.push_back(vertex_offset + 1);
    indices_.push_back(vertex_offset + 2);
    
    indices_.push_back(vertex_offset + 0);
    indices_.push_back(vertex_offset + 2);
    indices_.push_back(vertex_offset + 3);
    
    vertex_offset += 4;
    x += ch.advance;
  }
  
  // Update GPU buffers
  if (IsGpuResourcesAllocated() && !vertices_.empty()) {
    std::vector<float> vertex_data;
    for (size_t i = 0; i < vertices_.size(); ++i) {
      vertex_data.push_back(vertices_[i].x);
      vertex_data.push_back(vertices_[i].y);
      vertex_data.push_back(vertices_[i].z);
      vertex_data.push_back(tex_coords_[i].x);
      vertex_data.push_back(tex_coords_[i].y);
    }
    
    glBindVertexArray(vao_);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, vertex_data.size() * sizeof(float), vertex_data.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(uint32_t), indices_.data(), GL_DYNAMIC_DRAW);
    
    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    // Texture coordinate attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    
    glBindVertexArray(0);
  }
}

void Text3D::SetupTextShader() {
  const std::string vertex_shader_source = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoord;

out vec2 TexCoord;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uModel;

void main() {
    gl_Position = uProjection * uView * uModel * vec4(aPos, 1.0);
    TexCoord = aTexCoord;
}
)";

  const std::string fragment_shader_source = R"(
#version 330 core
in vec2 TexCoord;
out vec4 FragColor;

uniform sampler2D uFontAtlas;
uniform vec3 uTextColor;

void main() {
    float alpha = texture(uFontAtlas, TexCoord).r;
    if (alpha < 0.1) discard;
    FragColor = vec4(uTextColor, alpha);
}
)";

  Shader vertex_shader(vertex_shader_source.c_str(), Shader::Type::kVertex);
  if (!vertex_shader.Compile()) {
    std::cerr << "ERROR::TEXT3D::VERTEX_SHADER_COMPILATION_FAILED" << std::endl;
    throw std::runtime_error("Vertex shader compilation failed");
  }

  Shader fragment_shader(fragment_shader_source.c_str(), Shader::Type::kFragment);
  if (!fragment_shader.Compile()) {
    std::cerr << "ERROR::TEXT3D::FRAGMENT_SHADER_COMPILATION_FAILED" << std::endl;
    throw std::runtime_error("Fragment shader compilation failed");
  }

  text_shader_.AttachShader(vertex_shader);
  text_shader_.AttachShader(fragment_shader);
  
  if (!text_shader_.LinkProgram()) {
    std::cerr << "ERROR::TEXT3D::SHADER_PROGRAM_LINKING_FAILED" << std::endl;
    throw std::runtime_error("Shader program linking failed");
  }
}

void Text3D::SetupBackgroundShader() {
  // Simple background shader (not implemented for now)
}

glm::mat4 Text3D::CalculateBillboardMatrix(const glm::mat4& view) const {
  glm::mat4 billboard(1.0f);
  
  if (billboard_mode_ == BillboardMode::kSphere) {
    // Full spherical billboard - face camera completely
    glm::mat3 view_rotation(view);
    billboard = glm::mat4(glm::transpose(view_rotation));
  } else if (billboard_mode_ == BillboardMode::kCylinder) {
    // Cylindrical billboard - only rotate around Y axis
    glm::vec3 camera_pos = glm::vec3(glm::inverse(view)[3]);
    glm::vec3 to_camera = glm::normalize(camera_pos - position_);
    to_camera.y = 0.0f; // Remove Y component
    if (glm::length(to_camera) > 0.0f) {
      to_camera = glm::normalize(to_camera);
      float angle = atan2(to_camera.x, to_camera.z);
      billboard = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0, 1, 0));
    }
  }
  
  return billboard;
}

glm::vec2 Text3D::CalculateTextOffset() const {
  glm::vec2 offset(0.0f);
  
  if (alignment_ == Alignment::kCenter) {
    float width = 0.0f;
    for (char c : text_) {
      auto ch = GetCharacter(c);
      width += ch.advance;
    }
    offset.x = -width * 0.5f;
  } else if (alignment_ == Alignment::kRight) {
    float width = 0.0f;
    for (char c : text_) {
      auto ch = GetCharacter(c);
      width += ch.advance;
    }
    offset.x = -width;
  }
  
  if (vertical_alignment_ == VerticalAlignment::kMiddle) {
    offset.y = font_size_ * 0.5f;
  } else if (vertical_alignment_ == VerticalAlignment::kTop) {
    offset.y = 0.0f;
  } else if (vertical_alignment_ == VerticalAlignment::kBottom) {
    offset.y = font_size_;
  }
  
  return offset;
}

void Text3D::LoadBitmapFont() {
  if (font_loaded_) return;
  
  
  // Create a high-resolution 2048x2048 atlas for smooth text rendering
  constexpr int atlas_size = 2048;  // Increased from 512 for higher resolution
  constexpr int chars_per_row = 16;
  constexpr int char_size = atlas_size / chars_per_row;  // 128 pixels per character
  
  std::vector<unsigned char> atlas_data(atlas_size * atlas_size, 0);
  characters_.clear();
  
  // Create clean characters for A-Z, 0-9, and basic punctuation
  const std::string charset = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 .-/";
  
  for (size_t i = 0; i < charset.size(); ++i) {
    char c = charset[i];
    int row = i / chars_per_row;
    int col = i % chars_per_row;
    
    int char_x = col * char_size;
    int char_y = row * char_size;
    
    // Draw clean character
    DrawCleanCharacter(atlas_data.data(), atlas_size, char_x, char_y, char_size, char_size, c);
    
    // Create character data - scale down for reasonable 3D size
    Character ch;
    float scale_factor = 0.0075f; // Scale from 128-pixel char_size to ~1 unit (was 0.03f for 32px)
    ch.advance = char_size * 0.15f * scale_factor;  // Reduced by half from 0.3f for very tight letter spacing
    ch.bearing_x = char_size * 0.1f * scale_factor;
    ch.bearing_y = char_size * 0.8f * scale_factor;
    ch.width = char_size * 0.8f * scale_factor;
    ch.height = char_size * 0.8f * scale_factor;
    ch.uv_rect = glm::vec4(
      static_cast<float>(char_x) / atlas_size,
      static_cast<float>(char_y) / atlas_size,
      static_cast<float>(char_size) / atlas_size,
      static_cast<float>(char_size) / atlas_size
    );
    
    characters_[c] = ch;
  }
  
  // Create OpenGL texture
  glGenTextures(1, &font_atlas_);
  glBindTexture(GL_TEXTURE_2D, font_atlas_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, atlas_size, atlas_size, 0, GL_RED, GL_UNSIGNED_BYTE, atlas_data.data());
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  
  font_loaded_ = true;
}

void Text3D::DrawCleanCharacter(unsigned char* atlas_data, int atlas_size,
                               int char_x, int char_y, int char_width, int char_height, 
                               char character) {
  // Clear character area first
  for (int y = 0; y < char_height; ++y) {
    for (int x = 0; x < char_width; ++x) {
      int atlas_idx = (char_y + y) * atlas_size + (char_x + x);
      atlas_data[atlas_idx] = 0;
    }
  }
  
  // Don't draw anything for space
  if (character == ' ') return;
  
  // Use a consistent 5x7 grid system for all characters
  const int grid_w = 5;
  const int grid_h = 7;
  const int pixel_size = 3;  // Size of each grid pixel
  const int start_x = (char_width - grid_w * pixel_size) / 2;  // Center horizontally
  const int start_y = (char_height - grid_h * pixel_size) / 2; // Center vertically
  
  // Helper to draw a solid pixel at grid position
  auto DrawGridPixel = [&](int gx, int gy) {
    if (gx >= 0 && gx < grid_w && gy >= 0 && gy < grid_h) {
      // Draw a solid 3x3 block for each grid pixel with proper spacing
      for (int dy = 0; dy < pixel_size; dy++) {
        for (int dx = 0; dx < pixel_size; dx++) {
          int x = start_x + gx * pixel_size + dx;
          int y = start_y + gy * pixel_size + dy;
          if (x >= 0 && x < char_width && y >= 0 && y < char_height) {
            int atlas_idx = (char_y + y) * atlas_size + (char_x + x);
            atlas_data[atlas_idx] = 255;
          }
        }
      }
    }
  };
  
  // Define consistent 5x7 bitmap patterns for each character
  switch (character) {
    case 'A':
      // Pattern for A (5x7 grid)
      DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0);
      DrawGridPixel(0, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2); DrawGridPixel(1, 2); DrawGridPixel(2, 2); DrawGridPixel(3, 2); DrawGridPixel(4, 2);
      DrawGridPixel(0, 3); DrawGridPixel(4, 3);
      DrawGridPixel(0, 4); DrawGridPixel(4, 4);
      DrawGridPixel(0, 5); DrawGridPixel(4, 5);
      DrawGridPixel(0, 6); DrawGridPixel(4, 6);
      break;
      
    case 'B':
      DrawGridPixel(0, 0); DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0);
      DrawGridPixel(0, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2); DrawGridPixel(1, 2); DrawGridPixel(2, 2); DrawGridPixel(3, 2);
      DrawGridPixel(0, 3); DrawGridPixel(4, 3);
      DrawGridPixel(0, 4); DrawGridPixel(4, 4);
      DrawGridPixel(0, 5); DrawGridPixel(4, 5);
      DrawGridPixel(0, 6); DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6);
      break;
      
    case 'C':
      DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0);
      DrawGridPixel(0, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2);
      DrawGridPixel(0, 3);
      DrawGridPixel(0, 4);
      DrawGridPixel(0, 5); DrawGridPixel(4, 5);
      DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6);
      break;
      
    case 'D':
      DrawGridPixel(0, 0); DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0);
      DrawGridPixel(0, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2); DrawGridPixel(4, 2);
      DrawGridPixel(0, 3); DrawGridPixel(4, 3);
      DrawGridPixel(0, 4); DrawGridPixel(4, 4);
      DrawGridPixel(0, 5); DrawGridPixel(4, 5);
      DrawGridPixel(0, 6); DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6);
      break;
      
    case 'E':
      DrawGridPixel(0, 0); DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0); DrawGridPixel(4, 0);
      DrawGridPixel(0, 1);
      DrawGridPixel(0, 2); DrawGridPixel(1, 2); DrawGridPixel(2, 2); DrawGridPixel(3, 2);
      DrawGridPixel(0, 3);
      DrawGridPixel(0, 4);
      DrawGridPixel(0, 5);
      DrawGridPixel(0, 6); DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6); DrawGridPixel(4, 6);
      break;
      
    case 'F':
      DrawGridPixel(0, 0); DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0); DrawGridPixel(4, 0);
      DrawGridPixel(0, 1);
      DrawGridPixel(0, 2); DrawGridPixel(1, 2); DrawGridPixel(2, 2); DrawGridPixel(3, 2);
      DrawGridPixel(0, 3);
      DrawGridPixel(0, 4);
      DrawGridPixel(0, 5);
      DrawGridPixel(0, 6);
      break;
      
    case 'G':
      DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0); DrawGridPixel(4, 0);
      DrawGridPixel(0, 1);
      DrawGridPixel(0, 2);
      DrawGridPixel(0, 3); DrawGridPixel(2, 3); DrawGridPixel(3, 3); DrawGridPixel(4, 3);
      DrawGridPixel(0, 4); DrawGridPixel(4, 4);
      DrawGridPixel(0, 5); DrawGridPixel(4, 5);
      DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6); DrawGridPixel(4, 6);
      break;
      
    case 'H':
      DrawGridPixel(0, 0); DrawGridPixel(4, 0);
      DrawGridPixel(0, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2); DrawGridPixel(1, 2); DrawGridPixel(2, 2); DrawGridPixel(3, 2); DrawGridPixel(4, 2);
      DrawGridPixel(0, 3); DrawGridPixel(4, 3);
      DrawGridPixel(0, 4); DrawGridPixel(4, 4);
      DrawGridPixel(0, 5); DrawGridPixel(4, 5);
      DrawGridPixel(0, 6); DrawGridPixel(4, 6);
      break;
      
    case 'I':
      DrawGridPixel(0, 0); DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0); DrawGridPixel(4, 0);
      DrawGridPixel(2, 1);
      DrawGridPixel(2, 2);
      DrawGridPixel(2, 3);
      DrawGridPixel(2, 4);
      DrawGridPixel(2, 5);
      DrawGridPixel(0, 6); DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6); DrawGridPixel(4, 6);
      break;
      
    case 'L':
      DrawGridPixel(0, 0);
      DrawGridPixel(0, 1);
      DrawGridPixel(0, 2);
      DrawGridPixel(0, 3);
      DrawGridPixel(0, 4);
      DrawGridPixel(0, 5);
      DrawGridPixel(0, 6); DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6); DrawGridPixel(4, 6);
      break;
      
    case 'M':
      DrawGridPixel(0, 0); DrawGridPixel(4, 0);
      DrawGridPixel(0, 1); DrawGridPixel(1, 1); DrawGridPixel(3, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2); DrawGridPixel(2, 2); DrawGridPixel(4, 2);
      DrawGridPixel(0, 3); DrawGridPixel(4, 3);
      DrawGridPixel(0, 4); DrawGridPixel(4, 4);
      DrawGridPixel(0, 5); DrawGridPixel(4, 5);
      DrawGridPixel(0, 6); DrawGridPixel(4, 6);
      break;
      
    case 'N':
      DrawGridPixel(0, 0); DrawGridPixel(4, 0);
      DrawGridPixel(0, 1); DrawGridPixel(1, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2); DrawGridPixel(2, 2); DrawGridPixel(4, 2);
      DrawGridPixel(0, 3); DrawGridPixel(3, 3); DrawGridPixel(4, 3);
      DrawGridPixel(0, 4); DrawGridPixel(4, 4);
      DrawGridPixel(0, 5); DrawGridPixel(4, 5);
      DrawGridPixel(0, 6); DrawGridPixel(4, 6);
      break;
      
    case 'O':
      DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0);
      DrawGridPixel(0, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2); DrawGridPixel(4, 2);
      DrawGridPixel(0, 3); DrawGridPixel(4, 3);
      DrawGridPixel(0, 4); DrawGridPixel(4, 4);
      DrawGridPixel(0, 5); DrawGridPixel(4, 5);
      DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6);
      break;
      
    case 'P':
      DrawGridPixel(0, 0); DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0);
      DrawGridPixel(0, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2); DrawGridPixel(1, 2); DrawGridPixel(2, 2); DrawGridPixel(3, 2);
      DrawGridPixel(0, 3);
      DrawGridPixel(0, 4);
      DrawGridPixel(0, 5);
      DrawGridPixel(0, 6);
      break;
      
    case 'R':
      DrawGridPixel(0, 0); DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0);
      DrawGridPixel(0, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2); DrawGridPixel(1, 2); DrawGridPixel(2, 2); DrawGridPixel(3, 2);
      DrawGridPixel(0, 3); DrawGridPixel(2, 3);
      DrawGridPixel(0, 4); DrawGridPixel(3, 4);
      DrawGridPixel(0, 5); DrawGridPixel(4, 5);
      DrawGridPixel(0, 6); DrawGridPixel(4, 6);
      break;
      
    case 'S':
      DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0); DrawGridPixel(4, 0);
      DrawGridPixel(0, 1);
      DrawGridPixel(1, 2); DrawGridPixel(2, 2); DrawGridPixel(3, 2);
      DrawGridPixel(4, 3);
      DrawGridPixel(4, 4);
      DrawGridPixel(4, 5);
      DrawGridPixel(0, 6); DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6);
      break;
      
    case 'T':
      DrawGridPixel(0, 0); DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0); DrawGridPixel(4, 0);
      DrawGridPixel(2, 1);
      DrawGridPixel(2, 2);
      DrawGridPixel(2, 3);
      DrawGridPixel(2, 4);
      DrawGridPixel(2, 5);
      DrawGridPixel(2, 6);
      break;
      
    case 'U':
      DrawGridPixel(0, 0); DrawGridPixel(4, 0);
      DrawGridPixel(0, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2); DrawGridPixel(4, 2);
      DrawGridPixel(0, 3); DrawGridPixel(4, 3);
      DrawGridPixel(0, 4); DrawGridPixel(4, 4);
      DrawGridPixel(0, 5); DrawGridPixel(4, 5);
      DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6);
      break;
      
    case 'V':
      DrawGridPixel(0, 0); DrawGridPixel(4, 0);
      DrawGridPixel(0, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2); DrawGridPixel(4, 2);
      DrawGridPixel(0, 3); DrawGridPixel(4, 3);
      DrawGridPixel(1, 4); DrawGridPixel(3, 4);
      DrawGridPixel(1, 5); DrawGridPixel(3, 5);
      DrawGridPixel(2, 6);
      break;
      
    case 'W':
      DrawGridPixel(0, 0); DrawGridPixel(4, 0);
      DrawGridPixel(0, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2); DrawGridPixel(4, 2);
      DrawGridPixel(0, 3); DrawGridPixel(2, 3); DrawGridPixel(4, 3);
      DrawGridPixel(0, 4); DrawGridPixel(2, 4); DrawGridPixel(4, 4);
      DrawGridPixel(0, 5); DrawGridPixel(1, 5); DrawGridPixel(3, 5); DrawGridPixel(4, 5);
      DrawGridPixel(1, 6); DrawGridPixel(3, 6);
      break;
      
    case 'X':
      DrawGridPixel(0, 0); DrawGridPixel(4, 0);
      DrawGridPixel(1, 1); DrawGridPixel(3, 1);
      DrawGridPixel(2, 2);
      DrawGridPixel(2, 3);
      DrawGridPixel(2, 4);
      DrawGridPixel(1, 5); DrawGridPixel(3, 5);
      DrawGridPixel(0, 6); DrawGridPixel(4, 6);
      break;
      
    case 'Y':
      DrawGridPixel(0, 0); DrawGridPixel(4, 0);
      DrawGridPixel(1, 1); DrawGridPixel(3, 1);
      DrawGridPixel(2, 2);
      DrawGridPixel(2, 3);
      DrawGridPixel(2, 4);
      DrawGridPixel(2, 5);
      DrawGridPixel(2, 6);
      break;
      
    case 'Z':
      DrawGridPixel(0, 0); DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0); DrawGridPixel(4, 0);
      DrawGridPixel(3, 1);
      DrawGridPixel(3, 2);
      DrawGridPixel(2, 3);
      DrawGridPixel(1, 4);
      DrawGridPixel(1, 5);
      DrawGridPixel(0, 6); DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6); DrawGridPixel(4, 6);
      break;
      
    // Numbers
    case '0':
      DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0);
      DrawGridPixel(0, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2); DrawGridPixel(4, 2);
      DrawGridPixel(0, 3); DrawGridPixel(4, 3);
      DrawGridPixel(0, 4); DrawGridPixel(4, 4);
      DrawGridPixel(0, 5); DrawGridPixel(4, 5);
      DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6);
      break;
      
    case '1':
      DrawGridPixel(1, 0);
      DrawGridPixel(2, 0);
      DrawGridPixel(2, 1);
      DrawGridPixel(2, 2);
      DrawGridPixel(2, 3);
      DrawGridPixel(2, 4);
      DrawGridPixel(2, 5);
      DrawGridPixel(0, 6); DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6); DrawGridPixel(4, 6);
      break;
      
    case '2':
      DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0);
      DrawGridPixel(0, 1); DrawGridPixel(4, 1);
      DrawGridPixel(4, 2);
      DrawGridPixel(3, 3);
      DrawGridPixel(2, 4);
      DrawGridPixel(1, 5);
      DrawGridPixel(0, 6); DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6); DrawGridPixel(4, 6);
      break;
      
    case '3':
      DrawGridPixel(0, 0); DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0);
      DrawGridPixel(4, 1);
      DrawGridPixel(1, 2); DrawGridPixel(2, 2); DrawGridPixel(3, 2);
      DrawGridPixel(4, 3);
      DrawGridPixel(4, 4);
      DrawGridPixel(4, 5);
      DrawGridPixel(0, 6); DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6);
      break;
      
    case '4':
      DrawGridPixel(0, 0); DrawGridPixel(3, 0);
      DrawGridPixel(0, 1); DrawGridPixel(3, 1);
      DrawGridPixel(0, 2); DrawGridPixel(1, 2); DrawGridPixel(2, 2); DrawGridPixel(3, 2); DrawGridPixel(4, 2);
      DrawGridPixel(3, 3);
      DrawGridPixel(3, 4);
      DrawGridPixel(3, 5);
      DrawGridPixel(3, 6);
      break;
      
    case '5':
      DrawGridPixel(0, 0); DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0); DrawGridPixel(4, 0);
      DrawGridPixel(0, 1);
      DrawGridPixel(0, 2); DrawGridPixel(1, 2); DrawGridPixel(2, 2); DrawGridPixel(3, 2);
      DrawGridPixel(4, 3);
      DrawGridPixel(4, 4);
      DrawGridPixel(4, 5);
      DrawGridPixel(0, 6); DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6);
      break;
      
    // Special characters
    case '.':
      DrawGridPixel(2, 5);
      DrawGridPixel(2, 6);
      break;
      
    case '-':
      DrawGridPixel(1, 3); DrawGridPixel(2, 3); DrawGridPixel(3, 3);
      break;
      
    case '/':
      DrawGridPixel(4, 0);
      DrawGridPixel(3, 1);
      DrawGridPixel(3, 2);
      DrawGridPixel(2, 3);
      DrawGridPixel(1, 4);
      DrawGridPixel(1, 5);
      DrawGridPixel(0, 6);
      break;
      
    default:
      // Draw a simple rectangle for unknown characters
      DrawGridPixel(0, 0); DrawGridPixel(1, 0); DrawGridPixel(2, 0); DrawGridPixel(3, 0); DrawGridPixel(4, 0);
      DrawGridPixel(0, 1); DrawGridPixel(4, 1);
      DrawGridPixel(0, 2); DrawGridPixel(4, 2);
      DrawGridPixel(0, 3); DrawGridPixel(4, 3);
      DrawGridPixel(0, 4); DrawGridPixel(4, 4);
      DrawGridPixel(0, 5); DrawGridPixel(4, 5);
      DrawGridPixel(0, 6); DrawGridPixel(1, 6); DrawGridPixel(2, 6); DrawGridPixel(3, 6); DrawGridPixel(4, 6);
      break;
  }
}

Text3D::Character Text3D::GetCharacter(char c) const {
  auto it = characters_.find(c);
  if (it != characters_.end()) {
    return it->second;
  }
  
  // Return space character for unknown characters
  auto space_it = characters_.find(' ');
  if (space_it != characters_.end()) {
    return space_it->second;
  }
  
  // Fallback character
  Character fallback;
  fallback.advance = 0.145f;  // Reduced by half from 0.29f for very tight spacing
  fallback.bearing_x = 0.0f;
  fallback.bearing_y = 0.0f;
  fallback.width = 20.0f;
  fallback.height = 24.0f;
  fallback.uv_rect = glm::vec4(0.0f, 0.0f, 1.0f / 16.0f, 1.0f / 16.0f);
  return fallback;
}

} // namespace quickviz