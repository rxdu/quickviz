/**
 * @file billboard.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Modern Billboard primitive implementation
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/billboard.hpp"

#include <iostream>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "glad/glad.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "gldraw/shader.hpp"

namespace quickviz {

namespace {
// ID rendering shaders for GPU-based selection
const char* id_vertex_shader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 uMVP;

void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

const char* id_fragment_shader = R"(
#version 330 core
out vec4 FragColor;

uniform vec3 uIdColor;

void main() {
    FragColor = vec4(uIdColor, 1.0);
}
)";
}  // anonymous namespace

Billboard::Billboard() {
  // Set default material properties
  SetColor(glm::vec3(1.0f, 1.0f, 1.0f));  // White text by default
  SetOpacity(1.0f);

  // Initialize local font renderer with moderate resolution for smooth/blurry
  // appearance Use 24px for the atlas - when scaled up it will appear smooth
  // and slightly blurred
  const float atlas_font_size = 24.0f;
  font_renderer_ = std::make_unique<FontRenderer>();
  if (!font_renderer_->InitializeWithOpenSans(atlas_font_size)) {
    std::cerr << "Warning: Failed to initialize FontRenderer for Billboard"
              << std::endl;
  }

  AllocateGpuResources();
}

Billboard::Billboard(const std::string& text) : Billboard() { SetText(text); }

void Billboard::SetColor(const glm::vec3& color) { color_ = color; }

void Billboard::SetWireframeColor(const glm::vec3& color) {
  wireframe_color_ = color;
}

void Billboard::SetOpacity(float opacity) {
  opacity_ = std::max(0.0f, std::min(1.0f, opacity));
}

void Billboard::SetHighlighted(bool highlighted) {
  if (is_highlighted_ == highlighted) return;

  is_highlighted_ = highlighted;

  if (highlighted) {
    // Save original values
    original_color_ = color_;
    original_wireframe_color_ = wireframe_color_;
  } else {
    // Restore original values
    color_ = original_color_;
    wireframe_color_ = original_wireframe_color_;
  }
}

Billboard::~Billboard() { ReleaseGpuResources(); }

void Billboard::SetText(const std::string& text) {
  if (text_ != text) {
    text_ = text;
    text_dimensions_dirty_ = true;
    GenerateGeometry();
  }
}

void Billboard::SetAlignment(Alignment align,
                             VerticalAlignment vertical_align) {
  if (alignment_ != align || vertical_alignment_ != vertical_align) {
    alignment_ = align;
    vertical_alignment_ = vertical_align;
    GenerateGeometry();
  }
}

void Billboard::SetFontSize(float size_pixels) {
  if (font_size_ != size_pixels) {
    font_size_ = std::max(1.0f, size_pixels);
    text_dimensions_dirty_ = true;

    // FontRenderer always uses high-resolution atlas (48px) for quality
    // The font_size_ parameter only affects the visual scaling in
    // GenerateGeometry() No need to reinitialize the FontRenderer

    GenerateGeometry();
  }
}

void Billboard::SetBillboardMode(Mode mode) { billboard_mode_ = mode; }

void Billboard::SetBackgroundEnabled(bool enabled) {
  background_enabled_ = enabled;
}

void Billboard::SetBackgroundColor(const glm::vec4& color) {
  background_color_ = color;
}

void Billboard::SetBackgroundPadding(float padding) {
  background_padding_ = std::max(0.0f, padding);
  GenerateGeometry();
}

void Billboard::SetOutlineEnabled(bool enabled) { outline_enabled_ = enabled; }

void Billboard::SetOutlineColor(const glm::vec3& color) {
  outline_color_ = color;
}

void Billboard::SetOutlineWidth(float width) {
  outline_width_ = std::max(0.0f, width);
}

void Billboard::SetPosition(const glm::vec3& position) { position_ = position; }

void Billboard::SetTransform(const glm::mat4& transform) {
  // Extract position from transform matrix
  position_ = glm::vec3(transform[3]);
}

glm::mat4 Billboard::GetTransform() const {
  return glm::translate(glm::mat4(1.0f), position_);
}

std::pair<glm::vec3, glm::vec3> Billboard::GetBoundingBox() const {
  // Billboard bounding box using same scaling as GenerateGeometry()
  float world_scale = pixels_to_world_scale_ * (font_size_ / 24.0f);

  // Get actual text dimensions for accurate bounding box
  glm::vec2 text_dims = GetTextDimensions();
  glm::vec3 half_size(text_dims.x * world_scale * 0.5f,
                      text_dims.y * world_scale * 0.5f, 0.01f);
  return {position_ - half_size, position_ + half_size};
}

glm::vec2 Billboard::GetTextDimensions() const {
  if (text_dimensions_dirty_) {
    text_dimensions_ = glm::vec2(0.0f);

    if (!text_.empty() && font_renderer_ && font_renderer_->IsInitialized()) {
      // Calculate text size using FontRenderer
      FontRenderer::TextMetrics metrics = font_renderer_->GetTextMetrics(text_);
      text_dimensions_ = glm::vec2(metrics.width, metrics.height);
    }

    text_dimensions_dirty_ = false;
  }

  return text_dimensions_;
}

void Billboard::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) {
    return;
  }

  try {
    // Setup billboard shader
    SetupShaders();

    // Compile and link ID shader for GPU selection
    Shader id_vs(id_vertex_shader, Shader::Type::kVertex);
    Shader id_fs(id_fragment_shader, Shader::Type::kFragment);
    if (!id_vs.Compile() || !id_fs.Compile()) {
      throw std::runtime_error("ID shader compilation failed");
    }
    id_shader_.AttachShader(id_vs);
    id_shader_.AttachShader(id_fs);
    if (!id_shader_.LinkProgram()) {
      throw std::runtime_error("ID shader linking failed");
    }

    // Generate OpenGL objects
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ebo_);

    GenerateGeometry();

  } catch (const std::exception& e) {
    std::cerr << "Billboard::AllocateGpuResources: " << e.what() << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void Billboard::ReleaseGpuResources() noexcept {
  if (vao_ != 0) {
    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &ebo_);
    vao_ = vbo_ = ebo_ = 0;
  }

  if (texture_ != 0) {
    glDeleteTextures(1, &texture_);
    texture_ = 0;
  }
}

void Billboard::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                       const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }

  if (text_.empty() || !font_renderer_ || !font_renderer_->IsInitialized()) {
    return;
  }

  if (needs_update_) {
    GenerateGeometry();
    needs_update_ = false;
  }

  // Calculate transformation matrices
  glm::mat4 model =
      GetTransform();  // This contains the billboard's world position
  glm::mat4 view_matrix = view * coord_transform;
  glm::mat4 billboard_matrix = CalculateBillboardMatrix(view_matrix);

  // Combine position transform with billboard rotation transform
  glm::mat4 world_transform = model * billboard_matrix;
  glm::mat4 mvp = projection * view * coord_transform * world_transform;

  // Handle ID rendering mode for GPU selection
  if (id_render_mode_) {
    DrawIdBuffer(mvp);
    return;
  }

  // Draw billboard text
  DrawBillboard(mvp);
}

void Billboard::DrawBillboard(const glm::mat4& mvp) {
  if (vertices_.empty() || indices_.empty()) return;

  glBindVertexArray(vao_);

  billboard_shader_.Use();
  billboard_shader_.SetUniform("uMVP", mvp);
  billboard_shader_.SetUniform(
      "uColor", is_highlighted_ ? glm::vec3(1.0f, 1.0f, 0.0f) : color_);
  billboard_shader_.SetUniform("uOpacity", opacity_);

  // Bind font atlas texture
  if (font_renderer_ && font_renderer_->IsInitialized()) {
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, font_renderer_->GetAtlasTexture());
    billboard_shader_.SetUniform("uFontAtlas", 0);
  }

  // Enable blending for text transparency
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Disable face culling so text is visible from both sides
  glDisable(GL_CULL_FACE);

  glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()),
                 GL_UNSIGNED_INT, nullptr);

  glEnable(GL_CULL_FACE);
  glDisable(GL_BLEND);

  glBindVertexArray(0);
}

void Billboard::DrawIdBuffer(const glm::mat4& mvp) {
  if (vertices_.empty() || indices_.empty()) return;

  id_shader_.Use();
  id_shader_.SetUniform("uMVP", mvp);
  id_shader_.SetUniform("uIdColor", id_color_);

  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()),
                 GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}

void Billboard::GenerateGeometry() {
  if (text_.empty() || !font_renderer_ || !font_renderer_->IsInitialized()) {
    vertices_.clear();
    tex_coords_.clear();
    indices_.clear();
    UpdateGpuBuffers();
    return;
  }

  // Generate text vertices using FontRenderer with proper scaling for 3D world
  // Scale based on font_size_ to make it meaningful: font_size_ represents
  // desired world height
  float world_scale =
      pixels_to_world_scale_ *
      (font_size_ / 24.0f);  // Scale relative to 24px atlas baseline

  std::vector<FontRenderer::TextVertex> text_vertices =
      font_renderer_->GenerateTextVertices(text_, glm::vec3(0.0f, 0.0f, 0.0f),
                                           world_scale);

  // Apply text alignment offset
  glm::vec2 offset = CalculateTextOffset();
  offset *= world_scale;  // Scale the offset to world units

  // Convert FontRenderer vertices to our format with alignment offset
  vertices_.clear();
  tex_coords_.clear();
  indices_.clear();

  vertices_.reserve(text_vertices.size());
  tex_coords_.reserve(text_vertices.size());

  for (const auto& vertex : text_vertices) {
    // Apply alignment offset to position
    glm::vec3 aligned_pos =
        vertex.position + glm::vec3(offset.x, offset.y, 0.0f);
    vertices_.push_back(aligned_pos);
    tex_coords_.push_back(vertex.tex_coord);
  }

  // FontRenderer already generates triangulated vertices, so create sequential
  // indices
  indices_.reserve(text_vertices.size());
  for (size_t i = 0; i < text_vertices.size(); ++i) {
    indices_.push_back(static_cast<uint32_t>(i));
  }

  UpdateGpuBuffers();
}

void Billboard::UpdateGpuBuffers() {
  if (vao_ == 0) return;

  glBindVertexArray(vao_);

  // Update vertex buffer
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  size_t vertex_data_size = vertices_.size() * sizeof(glm::vec3) +
                            tex_coords_.size() * sizeof(glm::vec2);
  glBufferData(GL_ARRAY_BUFFER, vertex_data_size, nullptr, GL_DYNAMIC_DRAW);

  // Upload vertex positions
  glBufferSubData(GL_ARRAY_BUFFER, 0, vertices_.size() * sizeof(glm::vec3),
                  vertices_.data());

  // Upload texture coordinates
  glBufferSubData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3),
                  tex_coords_.size() * sizeof(glm::vec2), tex_coords_.data());

  // Setup vertex attributes
  glEnableVertexAttribArray(0);  // Position
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);

  glEnableVertexAttribArray(1);  // Texture coordinates
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2),
                        (void*)(vertices_.size() * sizeof(glm::vec3)));

  // Update element buffer
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(uint32_t),
               indices_.data(), GL_STATIC_DRAW);

  glBindVertexArray(0);
}

void Billboard::SetupShaders() {
  // Billboard vertex shader
  std::string vertex_shader_source = R"(
    #version 330 core
    layout (location = 0) in vec3 aPosition;
    layout (location = 1) in vec2 aTexCoord;

    uniform mat4 uMVP;

    out vec2 vTexCoord;
    out vec3 vWorldPos;

    void main() {
      gl_Position = uMVP * vec4(aPosition, 1.0);
      vTexCoord = aTexCoord;
      vWorldPos = aPosition; // Simplified for now
    }
  )";

  // Simplified Billboard fragment shader - minimal uniforms
  std::string fragment_shader_source = R"(
    #version 330 core
    in vec2 vTexCoord;
    in vec3 vWorldPos;

    out vec4 FragColor;

    uniform vec3 uColor;
    uniform float uOpacity;
    uniform sampler2D uFontAtlas;

    void main() {
      // Sample from font atlas texture (red channel contains glyph alpha)
      float glyph_alpha = texture(uFontAtlas, vTexCoord).r;

      if (glyph_alpha < 0.01) {
        discard; // Transparent areas
      }

      // Apply text color with font atlas alpha
      FragColor = vec4(uColor, glyph_alpha * uOpacity);
    }
  )";

  // Create and compile shaders (use c_str() to ensure we pass source code, not
  // file path)
  Shader vertex_shader(vertex_shader_source.c_str(), Shader::Type::kVertex);
  Shader fragment_shader(fragment_shader_source.c_str(),
                         Shader::Type::kFragment);

  if (!vertex_shader.Compile()) {
    std::cerr << "Billboard vertex shader compilation failed" << std::endl;
    throw std::runtime_error("Billboard vertex shader compilation failed");
  }

  if (!fragment_shader.Compile()) {
    std::cerr << "Billboard fragment shader compilation failed" << std::endl;
    throw std::runtime_error("Billboard fragment shader compilation failed");
  }

  billboard_shader_.AttachShader(vertex_shader);
  billboard_shader_.AttachShader(fragment_shader);
  if (!billboard_shader_.LinkProgram()) {
    std::cerr << "Billboard shader program linking failed" << std::endl;
    throw std::runtime_error("Billboard shader linking failed");
  }
}

glm::mat4 Billboard::CalculateBillboardMatrix(
    const glm::mat4& view_matrix) const {
  glm::mat4 billboard_transform =
      glm::mat4(1.0f);  // Start with identity, not position

  switch (billboard_mode_) {
    case Mode::kSphere: {
      // Full spherical billboarding - always face camera
      // Extract camera's rotation axes from view matrix
      glm::vec3 right =
          glm::vec3(view_matrix[0][0], view_matrix[1][0], view_matrix[2][0]);
      glm::vec3 up =
          glm::vec3(view_matrix[0][1], view_matrix[1][1], view_matrix[2][1]);
      glm::vec3 forward =
          -glm::vec3(view_matrix[0][2], view_matrix[1][2], view_matrix[2][2]);

      // Apply to billboard transform (rotation only)
      billboard_transform[0] = glm::vec4(right, 0.0f);
      billboard_transform[1] = glm::vec4(up, 0.0f);
      billboard_transform[2] = glm::vec4(forward, 0.0f);
      break;
    }
    case Mode::kCylinder: {
      // Cylindrical billboarding - only rotate around Y axis
      glm::vec3 camera_pos = glm::vec3(glm::inverse(view_matrix)[3]);
      glm::vec3 to_camera = glm::normalize(camera_pos - position_);
      to_camera.y = 0.0f;  // Lock Y rotation

      if (glm::length(to_camera) > 0.001f) {
        to_camera = glm::normalize(to_camera);

        glm::vec3 right =
            glm::normalize(glm::cross(glm::vec3(0, 1, 0), to_camera));
        glm::vec3 up = glm::vec3(0, 1, 0);
        glm::vec3 forward = glm::normalize(glm::cross(right, up));

        billboard_transform[0] = glm::vec4(right, 0.0f);
        billboard_transform[1] = glm::vec4(up, 0.0f);
        billboard_transform[2] = glm::vec4(forward, 0.0f);
      }
      break;
    }
    case Mode::kFixed:
      // No billboarding - fixed orientation (identity matrix)
      break;
  }

  return billboard_transform;
}

glm::vec2 Billboard::CalculateTextOffset() const {
  glm::vec2 text_size = GetTextDimensions();
  glm::vec2 offset(0.0f);

  // Horizontal alignment - offset to center the text properly
  switch (alignment_) {
    case Alignment::kLeft:
      offset.x = 0.0f;  // Text starts at origin
      break;
    case Alignment::kCenter:
      offset.x = -text_size.x * 0.5f;  // Center horizontally
      break;
    case Alignment::kRight:
      offset.x = -text_size.x;  // Right-align
      break;
  }

  // Vertical alignment - adjust for baseline
  switch (vertical_alignment_) {
    case VerticalAlignment::kTop:
      offset.y = -text_size.y;  // Top of text at origin
      break;
    case VerticalAlignment::kMiddle:
      offset.y = -text_size.y * 0.5f;  // Center vertically
      break;
    case VerticalAlignment::kBottom:
      offset.y = 0.0f;  // Bottom/baseline at origin
      break;
  }

  return offset;
}

}  // namespace quickviz