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

#include "glad/glad.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


#include "gldraw/shader.hpp"

namespace quickviz {

// Static members for shared font renderer
std::shared_ptr<FontRenderer> Billboard::font_renderer_;
bool Billboard::font_renderer_initialized_ = false;

Billboard::Billboard() {
  // Set default material properties
  SetColor(glm::vec3(1.0f, 1.0f, 1.0f)); // White text by default
  SetOpacity(1.0f);
  InitializeFontRenderer();
  AllocateGpuResources();
}

Billboard::Billboard(const std::string& text) : Billboard() {
  SetText(text);
}

Billboard::~Billboard() {
  ReleaseGpuResources();
}

void Billboard::SetText(const std::string& text) {
  if (text_ != text) {
    text_ = text;
    text_dimensions_dirty_ = true;
    GenerateGeometry();
  }
}

void Billboard::SetAlignment(Alignment align, VerticalAlignment vertical_align) {
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
    GenerateGeometry();
  }
}


void Billboard::SetBillboardMode(Mode mode) {
  billboard_mode_ = mode;
}

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

void Billboard::SetOutlineEnabled(bool enabled) {
  outline_enabled_ = enabled;
}

void Billboard::SetOutlineColor(const glm::vec3& color) {
  outline_color_ = color;
}

void Billboard::SetOutlineWidth(float width) {
  outline_width_ = std::max(0.0f, width);
}

void Billboard::SetPosition(const glm::vec3& position) {
  position_ = position;
}

void Billboard::SetTransform(const glm::mat4& transform) {
  // Extract position from transform matrix
  position_ = glm::vec3(transform[3]);
}

glm::mat4 Billboard::GetTransform() const {
  return glm::translate(glm::mat4(1.0f), position_);
}

float Billboard::GetSurfaceArea() const {
  glm::vec2 dims = GetTextDimensions();
  return dims.x * dims.y; // Pixel area
}

std::pair<glm::vec3, glm::vec3> Billboard::GetBoundingBox() const {
  // Billboard bounding box is approximate since it's screen-aligned
  // Use a small world-space approximation around the position
  float world_scale = font_size_ * 0.01f; // Rough pixel-to-world conversion
  glm::vec3 half_size(world_scale, world_scale, 0.01f);
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
  
  SetupShaders();
  
  // Generate OpenGL objects
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &ebo_);
  
  GenerateGeometry();
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

void Billboard::PrepareShaders(const glm::mat4& mvp_matrix, const glm::mat4& model_matrix) {
  // Store matrices for use in rendering methods
  stored_mvp_matrix_ = mvp_matrix;
  stored_model_matrix_ = model_matrix;
  
  // Calculate billboard transformation matrix
  glm::mat4 view_matrix = mvp_matrix * glm::inverse(model_matrix);
  glm::mat4 billboard_matrix = CalculateBillboardMatrix(view_matrix);
  glm::mat4 final_mvp = mvp_matrix * billboard_matrix;
  
  // Setup billboard shader
  billboard_shader_.Use();
  billboard_shader_.SetUniform("uMVP", final_mvp);
  // Skip uModel uniform temporarily to avoid error
  // billboard_shader_.SetUniform("uModel", billboard_matrix);
  
  // Set material properties
  const auto& material = GetMaterial();
  billboard_shader_.SetUniform("uColor", material.diffuse_color);
  billboard_shader_.SetUniform("uOpacity", material.opacity);
  
  // Bind font atlas if available
  if (font_renderer_ && font_renderer_->IsInitialized()) {
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, font_renderer_->GetAtlasTexture());
    billboard_shader_.SetUniform("uFontAtlas", 0);
  }
}

void Billboard::RenderSolid() {
  if (text_.empty() || vao_ == 0 || !font_renderer_ || !font_renderer_->IsInitialized()) return;
  
  // Shader and uniforms are already set in PrepareShaders
  billboard_shader_.Use();
  
  // Enable blending for text transparency
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  // Render text geometry
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()), GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
  
  glDisable(GL_BLEND);
}

void Billboard::RenderWireframe() {
  // For wireframe mode, just render the bounding box outline
  if (text_.empty() || vao_ == 0) return;
  
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  billboard_shader_.SetUniform("uWireframeMode", true);
  
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()), GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
  
  billboard_shader_.SetUniform("uWireframeMode", false);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Billboard::RenderPoints() {
  // For points mode, render corner points of the text bounds
  if (vertices_.empty()) return;
  
  glPointSize(2.0f); // Fixed point size for billboard corners
  glBindVertexArray(vao_);
  glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(vertices_.size()));
  glBindVertexArray(0);
}

void Billboard::RenderIdBuffer(const glm::mat4& mvp_matrix) {
  if (text_.empty() || vao_ == 0) return;
  
  // Use the shared GeometricPrimitive ID shader with billboard transformation
  static std::unique_ptr<ShaderProgram> id_shader = nullptr;
  static bool shader_initialized = false;
  
  if (!shader_initialized) {
    try {
      const char* id_vertex_shader = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        
        uniform mat4 mvp;
        
        void main() {
          gl_Position = mvp * vec4(aPos, 1.0);
        }
      )";
      
      const char* id_fragment_shader = R"(
        #version 330 core
        uniform vec3 id_color;
        out vec4 FragColor;
        
        void main() {
          FragColor = vec4(id_color, 1.0);
        }
      )";
      
      id_shader = std::make_unique<ShaderProgram>();
      Shader vs(id_vertex_shader, Shader::Type::kVertex);  // const char* is source code
      Shader fs(id_fragment_shader, Shader::Type::kFragment);  // const char* is source code
      
      if (!vs.Compile() || !fs.Compile()) {
        throw std::runtime_error("Billboard ID shader compilation failed");
      }
      
      id_shader->AttachShader(vs);
      id_shader->AttachShader(fs);
      
      if (!id_shader->LinkProgram()) {
        throw std::runtime_error("Billboard ID shader linking failed");
      }
      
      shader_initialized = true;
    } catch (const std::exception& e) {
      std::cerr << "Billboard ID shader setup failed: " << e.what() << std::endl;
      return;
    }
  }
  
  // Calculate billboard transformation for ID rendering
  glm::mat4 view_matrix = mvp_matrix * glm::inverse(GetTransform());
  glm::mat4 billboard_matrix = CalculateBillboardMatrix(view_matrix);
  glm::mat4 final_mvp = mvp_matrix * billboard_matrix;
  
  id_shader->Use();
  id_shader->SetUniform("mvp", final_mvp);
  id_shader->SetUniform("id_color", id_color_);
  
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()), GL_UNSIGNED_INT, nullptr);
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
  float world_scale = 0.01f; // 1 pixel = 0.01 world units
  std::vector<FontRenderer::TextVertex> text_vertices = 
      font_renderer_->GenerateTextVertices(text_, glm::vec3(0.0f, 0.0f, 0.0f), world_scale);
  
  // Convert FontRenderer vertices to our format
  vertices_.clear();
  tex_coords_.clear();
  indices_.clear();
  
  vertices_.reserve(text_vertices.size());
  tex_coords_.reserve(text_vertices.size());
  
  for (const auto& vertex : text_vertices) {
    vertices_.push_back(vertex.position);
    tex_coords_.push_back(vertex.tex_coord);
  }
  
  // FontRenderer already generates triangulated vertices, so create sequential indices
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
  size_t vertex_data_size = vertices_.size() * sizeof(glm::vec3) + tex_coords_.size() * sizeof(glm::vec2);
  glBufferData(GL_ARRAY_BUFFER, vertex_data_size, nullptr, GL_DYNAMIC_DRAW);
  
  // Upload vertex positions
  glBufferSubData(GL_ARRAY_BUFFER, 0, vertices_.size() * sizeof(glm::vec3), vertices_.data());
  
  // Upload texture coordinates
  glBufferSubData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glm::vec3), 
                  tex_coords_.size() * sizeof(glm::vec2), tex_coords_.data());
  
  // Setup vertex attributes
  glEnableVertexAttribArray(0); // Position
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  
  glEnableVertexAttribArray(1); // Texture coordinates
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
  
  // Create and compile shaders (use c_str() to ensure we pass source code, not file path)
  Shader vertex_shader(vertex_shader_source.c_str(), Shader::Type::kVertex);
  Shader fragment_shader(fragment_shader_source.c_str(), Shader::Type::kFragment);
  
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

glm::mat4 Billboard::CalculateBillboardMatrix(const glm::mat4& view_matrix) const {
  glm::mat4 billboard_transform = glm::translate(glm::mat4(1.0f), position_);
  
  switch (billboard_mode_) {
    case Mode::kSphere: {
      // Full spherical billboarding - always face camera
      glm::mat4 inv_view = glm::inverse(view_matrix);
      glm::vec3 right = glm::vec3(inv_view[0]);
      glm::vec3 up = glm::vec3(inv_view[1]);
      glm::vec3 forward = glm::vec3(inv_view[2]);
      
      billboard_transform[0] = glm::vec4(right, 0.0f);
      billboard_transform[1] = glm::vec4(up, 0.0f);
      billboard_transform[2] = glm::vec4(forward, 0.0f);
      break;
    }
    case Mode::kCylinder: {
      // Cylindrical billboarding - only rotate around Y axis
      glm::vec3 camera_pos = glm::vec3(glm::inverse(view_matrix)[3]);
      glm::vec3 to_camera = glm::normalize(camera_pos - position_);
      to_camera.y = 0.0f; // Lock Y rotation
      to_camera = glm::normalize(to_camera);
      
      glm::vec3 right = glm::normalize(glm::cross(glm::vec3(0, 1, 0), to_camera));
      glm::vec3 up = glm::vec3(0, 1, 0);
      glm::vec3 forward = glm::normalize(glm::cross(right, up));
      
      billboard_transform[0] = glm::vec4(right, 0.0f);
      billboard_transform[1] = glm::vec4(up, 0.0f);
      billboard_transform[2] = glm::vec4(forward, 0.0f);
      break;
    }
    case Mode::kFixed:
      // No billboarding - fixed orientation
      break;
  }
  
  return billboard_transform;
}

glm::vec2 Billboard::CalculateTextOffset() const {
  glm::vec2 text_size = GetTextDimensions();
  glm::vec2 offset(0.0f);
  
  // Horizontal alignment
  switch (alignment_) {
    case Alignment::kLeft:
      offset.x = -text_size.x * 0.5f;
      break;
    case Alignment::kCenter:
      offset.x = 0.0f;
      break;
    case Alignment::kRight:
      offset.x = text_size.x * 0.5f;
      break;
  }
  
  // Vertical alignment
  switch (vertical_alignment_) {
    case VerticalAlignment::kTop:
      offset.y = text_size.y * 0.5f;
      break;
    case VerticalAlignment::kMiddle:
      offset.y = 0.0f;
      break;
    case VerticalAlignment::kBottom:
      offset.y = -text_size.y * 0.5f;
      break;
  }
  
  return offset;
}

void Billboard::InitializeFontRenderer() {
  if (!font_renderer_initialized_) {
    font_renderer_ = std::make_shared<FontRenderer>();
    if (font_renderer_->InitializeWithOpenSans(font_size_)) {
      font_renderer_initialized_ = true;
    } else {
      std::cerr << "Billboard: Failed to initialize FontRenderer with OpenSans" << std::endl;
    }
  }
}

} // namespace quickviz