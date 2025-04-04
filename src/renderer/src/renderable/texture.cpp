/*
 * texture.cpp
 *
 * Created on 4/3/25 9:33 PM
 * Description:
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "renderer/renderable/texture.hpp"

#include <iostream>
#include <stdexcept>
#include <cstring>
#include "glad/glad.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

namespace quickviz {
namespace {
std::string texture_vertex_shader_source = R"(
#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aTexCoord;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform mat4 coordSystemTransform;

out vec2 TexCoord;
out vec3 FragPos;

void main() {
    gl_Position = projection * view * model * coordSystemTransform * vec4(aPos, 1.0);
    TexCoord = aTexCoord;
    FragPos = aPos;
}
)";

std::string texture_fragment_shader_source = R"(
#version 330 core

in vec2 TexCoord;
in vec3 FragPos;
out vec4 FragColor;

uniform sampler2D backgroundTexture;

void main() {
    // Simply sample the texture with proper alpha handling
    vec4 texColor = texture(backgroundTexture, TexCoord);
    
    // If alpha is very small, discard the fragment to avoid artifacts
    // if (texColor.a < 0.01) {
    //     discard;
    // }
    
    // Use the texture color directly with its original alpha
    FragColor = texColor;
    
    // Add a subtle border for visual feedback on the texture boundaries
    float border = 0.005; // Increased border width
    if (TexCoord.x < border || TexCoord.x > 1.0 - border || 
        TexCoord.y < border || TexCoord.y > 1.0 - border) {
        FragColor = mix(vec4(1.0, 0.0, 0.0, 1.0), texColor, 0.1); // More prominent red border
    }
}
)";

struct TextureFormatInfo {
  GLenum format;
  GLenum internal_format;
};

TextureFormatInfo GetGlFormats(Texture::PixelFormat format) {
  TextureFormatInfo info{GL_RGB, GL_RGB8};  // Default values

  switch (format) {
    case Texture::PixelFormat::kGray:
      info.format = GL_RED;
      info.internal_format = GL_R8;
      break;
    case Texture::PixelFormat::kRgb:
      info.format = GL_RGB;
      info.internal_format = GL_SRGB8;  // Use SRGB for better color accuracy
      break;
    case Texture::PixelFormat::kRgba:
      info.format = GL_RGBA;
      info.internal_format = GL_SRGB8_ALPHA8;  // Use SRGB with alpha
      break;
    case Texture::PixelFormat::kBgr:
      info.format = GL_BGR;
      info.internal_format = GL_SRGB8;
      break;
    case Texture::PixelFormat::kBgra:
      info.format = GL_BGRA;
      info.internal_format = GL_SRGB8_ALPHA8;
      break;
  }

  return info;
}
}  // namespace

Texture::Texture() {
  // Initialize OpenGL resources
  AllocateGpuResources();
}

Texture::~Texture() { ReleaseGpuResources(); }

void Texture::AllocateGpuResources() {
  // First make sure any existing resources are released
  ReleaseGpuResources();

  try {
    // Generate texture and PBO
    glGenTextures(1, &texture_id_);
    glGenBuffers(1, &pbo_id_);
    
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      throw std::runtime_error("Failed to generate texture/PBO: OpenGL error " +
                             std::to_string(err));
    }

    std::cout << "Compiling texture vertex shader..." << std::endl;
    Shader vertex_shader(texture_vertex_shader_source.c_str(),
                         Shader::Type::kVertex);
    if (!vertex_shader.Compile()) {
      std::cerr << "ERROR::TEXTURE::VERTEX_SHADER_COMPILATION_FAILED"
                << std::endl;
      throw std::runtime_error("Vertex shader compilation failed");
    }

    std::cout << "Compiling texture fragment shader..." << std::endl;
    Shader fragment_shader(texture_fragment_shader_source.c_str(),
                           Shader::Type::kFragment);
    if (!fragment_shader.Compile()) {
      std::cerr << "ERROR::TEXTURE::FRAGMENT_SHADER_COMPILATION_FAILED"
                << std::endl;
      throw std::runtime_error("Fragment shader compilation failed");
    }

    std::cout << "Attaching shaders to program and linking..." << std::endl;
    shader_.AttachShader(vertex_shader);
    shader_.AttachShader(fragment_shader);

    if (!shader_.LinkProgram()) {
      std::cerr << "ERROR::TEXTURE::SHADER_PROGRAM_LINKING_FAILED" << std::endl;
      throw std::runtime_error("Shader program linking failed");
    }

    std::cout << "Texture shader program compiled and linked successfully."
              << std::endl;

    // Create VAO and VBO for the quad
    glGenVertexArrays(1, &vao_);
    if (vao_ == 0) {
      std::cerr << "ERROR::TEXTURE::VAO_GENERATION_FAILED" << std::endl;
      throw std::runtime_error("VAO generation failed");
    }

    glGenBuffers(1, &vbo_);
    if (vbo_ == 0) {
      std::cerr << "ERROR::TEXTURE::VBO_GENERATION_FAILED" << std::endl;
      throw std::runtime_error("VBO generation failed");
    }

    // Default setup of a simple quad - actual coordinates will be updated in
    // OnDraw
    float vertices[] = {
        // Positions (x, y, z)       // Texture coords
        -0.5f, -0.5f, 0.0f, 0.0f, 0.0f,  // Bottom left
        0.5f,  -0.5f, 0.0f, 1.0f, 0.0f,  // Bottom right
        0.5f,  0.5f,  0.0f, 1.0f, 1.0f,  // Top right
        -0.5f, 0.5f,  0.0f, 0.0f, 1.0f   // Top left
    };

    // Setup the VBO and VAO
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                          (void*)0);
    glEnableVertexAttribArray(0);

    // Texture coordinate attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    std::cout << "GPU resources allocated successfully." << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "ERROR: Failed to allocate GPU resources: " << e.what()
              << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void Texture::ReleaseGpuResources() {
  if (texture_id_ != 0) {
    glDeleteTextures(1, &texture_id_);
    texture_id_ = 0;
  }

  if (pbo_id_ != 0) {
    glDeleteBuffers(1, &pbo_id_);
    pbo_id_ = 0;
  }

  if (vao_ != 0) {
    glDeleteVertexArrays(1, &vao_);
    vao_ = 0;
  }

  if (vbo_ != 0) {
    glDeleteBuffers(1, &vbo_);
    vbo_ = 0;
  }
}

bool Texture::LoadFromFile(const std::string& image_path) {
  int width = 0, height = 0, channels = 0;
  stbi_set_flip_vertically_on_load(true);

  std::cout << "Loading image from path: " << image_path << std::endl;

  unsigned char* data = stbi_load(image_path.c_str(), &width, &height, &channels, 0);

  if (!data) {
    std::cerr << "Failed to load image: " << image_path << std::endl;
    std::cerr << "STB Error: " << stbi_failure_reason() << std::endl;
    return false;
  }

  // Determine format based on channels
  PixelFormat format;
  switch (channels) {
    case 1: format = PixelFormat::kGray; break;
    case 3: format = PixelFormat::kRgb; break;
    case 4: format = PixelFormat::kRgba; break;
    default:
      stbi_image_free(data);
      std::cerr << "Unsupported number of channels: " << channels << std::endl;
      return false;
  }

  // Update texture with loaded data
  bool success = UpdateData(width, height, format, data);
  stbi_image_free(data);
  return success;
}

void Texture::PreallocateBuffer(int width, int height, PixelFormat format) {
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("Invalid texture dimensions");
  }

  image_width_ = width;
  image_height_ = height;
  pixel_format_ = format;

  auto format_info = GetGlFormats(format);

  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glTexImage2D(GL_TEXTURE_2D, 0, format_info.internal_format, width, height,
               0, format_info.format, GL_UNSIGNED_BYTE, nullptr);
  
  // Set texture parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // Initialize PBO
  size_t buffer_size = width * height * (format == PixelFormat::kGray ? 1 : 4);
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo_id_);
  glBufferData(GL_PIXEL_UNPACK_BUFFER, buffer_size, nullptr, GL_STREAM_DRAW);
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

  buffer_preallocated_ = true;
}

bool Texture::ShouldUsePBO(size_t data_size) const {
  switch (buffer_update_strategy_) {
    case BufferUpdateStrategy::kAuto:
      return data_size > buffer_update_threshold_;
    case BufferUpdateStrategy::kBufferSubData:
      return false;
    case BufferUpdateStrategy::kMapBuffer:
      return true;
    default:
      return false;
  }
}

void Texture::UpdateTextureWithSubData(const void* data, size_t size_bytes) {
  auto format_info = GetGlFormats(pixel_format_);
  
  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_width_, image_height_,
                  format_info.format, GL_UNSIGNED_BYTE, data);
  glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::UpdateTextureWithPBO(const void* data, size_t size_bytes) {
  auto format_info = GetGlFormats(pixel_format_);

  glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo_id_);
  
  // Map the PBO and update its contents
  void* ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
  if (ptr) {
    std::memcpy(ptr, data, size_bytes);
    glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
    
    // Update texture from PBO
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_width_, image_height_,
                    format_info.format, GL_UNSIGNED_BYTE, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
  }
  
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
}

bool Texture::UpdateData(int width, int height, PixelFormat format, const unsigned char* data) {
  if (!data) return false;

  try {
    // If dimensions changed or not preallocated, allocate new buffers
    if (!buffer_preallocated_ || width != image_width_ || height != image_height_ || format != pixel_format_) {
      PreallocateBuffer(width, height, format);
    }

    size_t pixel_size = (format == PixelFormat::kGray ? 1 : 4);
    size_t data_size = width * height * pixel_size;

    // Use appropriate update strategy
    if (ShouldUsePBO(data_size)) {
      UpdateTextureWithPBO(data, data_size);
    } else {
      UpdateTextureWithSubData(data, data_size);
    }

    needs_update_ = false;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Error updating texture data: " << e.what() << std::endl;
    return false;
  }
}

bool Texture::UpdateData(int width, int height, PixelFormat format, std::vector<unsigned char>&& data) {
  size_t pixel_size = (format == PixelFormat::kGray ? 1 : 4);
  size_t expected_size = width * height * pixel_size;
  
  if (data.size() != expected_size) {
    std::cerr << "Data size mismatch for texture update" << std::endl;
    return false;
  }

  return UpdateData(width, height, format, data.data());
}

void Texture::SetOrigin(const glm::vec3& origin, float resolution) {
  assert(resolution > 0.0f);
  origin_ = origin;
  resolution_ = resolution;
}

void Texture::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                     const glm::mat4& coord_transform) {
  if (texture_id_ == 0) return;

  // Bind shader and set uniforms
  shader_.Use();
  shader_.TrySetUniform("projection", projection);
  shader_.TrySetUniform("view", view);
  shader_.TrySetUniform("model", glm::mat4(1.0f)); // Identity matrix, transformations done in vertices
  shader_.TrySetUniform("coordSystemTransform", coord_transform);

  // Bind texture
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture_id_);
  shader_.TrySetUniform("backgroundTexture", 0);

  // Setup blending for proper transparency
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Calculate the real-world dimensions of the image based on resolution
  float world_width = image_width_ * resolution_;
  float world_height = image_height_ * resolution_;

  // Extract origin parameters
  float origin_x = origin_.x;  // x-coordinate of lower-left pixel
  float origin_y = origin_.y;  // y-coordinate of lower-left pixel
  float yaw = origin_.z;       // Rotation angle in radians (counterclockwise)

  // Calculate the coordinates of the four corners of the image in real-world coordinates
  // Start with the corners relative to origin (before rotation)
  glm::vec2 bottom_left(0.0f, 0.0f);             // Origin point
  glm::vec2 bottom_right(world_width, 0.0f);     // Right from origin
  glm::vec2 top_right(world_width, world_height); // Top-right corner
  glm::vec2 top_left(0.0f, world_height);        // Top from origin

  // Create rotation matrix for yaw
  float cos_yaw = cos(yaw);
  float sin_yaw = sin(yaw);

  // Apply rotation and translation to each corner
  // Rotation around origin, then translation
  auto rotatePoint = [&](const glm::vec2& p) -> glm::vec2 {
    float rotated_x = p.x * cos_yaw - p.y * sin_yaw;
    float rotated_y = p.x * sin_yaw + p.y * cos_yaw;
    return glm::vec2(rotated_x + origin_x, rotated_y + origin_y);
  };

  glm::vec2 bl = rotatePoint(bottom_left);
  glm::vec2 br = rotatePoint(bottom_right);
  glm::vec2 tr = rotatePoint(top_right);
  glm::vec2 tl = rotatePoint(top_left);

  // Vertices for the transformed quad (all on z=0 plane)
  float vertices[] = {
      // Positions (x, y, z)              // Texture coords
      bl.x, bl.y, 0.0f, 0.0f, 0.0f,  // Bottom left
      br.x, br.y, 0.0f, 1.0f, 0.0f,  // Bottom right
      tr.x, tr.y, 0.0f, 1.0f, 1.0f,  // Top right
      tl.x, tl.y, 0.0f, 0.0f, 1.0f   // Top left
  };

  // Update vertex buffer
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);

  // Draw quad
  glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

  // Cleanup
  glBindVertexArray(0);
  glBindTexture(GL_TEXTURE_2D, 0);
  glUseProgram(0);
}
}  // namespace quickviz