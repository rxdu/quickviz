/**
 * @file canvas.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/component/opengl/renderer/canvas.hpp"

#include <iostream>
#include <cmath>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "glad/glad.h"
#include <glm/gtc/type_ptr.hpp>

#include "component/opengl/renderer/details/canvas_data.hpp"

namespace quickviz {
namespace {
std::string vertex_shader_source = R"(
#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
layout(location = 2) in float aSize;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform mat4 coordSystemTransform;

out vec4 vertexColor;

void main() {
    gl_Position = projection * view * model * coordSystemTransform * vec4(aPos, 1.0);
    gl_PointSize = aSize;
    vertexColor = aColor;
}
)";

std::string fragment_shader_source = R"(
#version 330 core

in vec4 vertexColor;
out vec4 FragColor;

void main() {
    // Create a circular point by discarding fragments outside the circle
    vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
    if (dot(circCoord, circCoord) > 1.0) {
        discard;
    }
    
    FragColor = vertexColor;
}
)";

// Shader sources for background image rendering
std::string background_vertex_shader_source = R"(
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

std::string background_fragment_shader_source = R"(
#version 330 core

in vec2 TexCoord;
in vec3 FragPos;
out vec4 FragColor;

uniform sampler2D backgroundTexture;

void main() {
    // Simply sample the texture with proper alpha handling
    vec4 texColor = texture(backgroundTexture, TexCoord);
    
    // If alpha is very small, discard the fragment to avoid artifacts
    if (texColor.a < 0.01) {
        discard;
    }
    
    // Use the texture color directly with its original alpha
    FragColor = texColor;
    
    // Add a subtle border for visual feedback on the texture boundaries
    float border = 0.005;
    if (TexCoord.x < border || TexCoord.x > 1.0 - border || 
        TexCoord.y < border || TexCoord.y > 1.0 - border) {
        FragColor = mix(vec4(1.0, 0.0, 0.0, 1.0), texColor, 0.3); // Subtle red border
    }
}
)";
}  // namespace

Canvas::Canvas() {
  data_ = std::make_unique<CanvasData>();
  AllocateGpuResources();
}

Canvas::~Canvas() { ReleaseGpuResources(); }

void Canvas::AddBackgroundImage(const std::string& image_path,
                                const glm::vec3& origin, float resolution) {
  int background_image_width = 0;
  int background_image_height = 0;
  int background_image_channels = 0;
  unsigned char* background_image_data = nullptr;

  // Delete previous texture if it exists
  {
    std::lock_guard<std::mutex> lock(background_mutex_);
    uint32_t texture_id = background_texture_.load();
    if (texture_id != 0) {
      glDeleteTextures(1, &texture_id);
      background_texture_ = 0;
    }
  }

  std::cout << "Loading background image: " << image_path << std::endl;
  std::cout << "Origin: (" << origin.x << ", " << origin.y << ", " << origin.z
            << ") - (x, y, yaw)" << std::endl;
  std::cout << "Resolution: " << resolution << " meters/pixel" << std::endl;

  // Load the image using stb_image
  stbi_set_flip_vertically_on_load(
      true);  // OpenGL expects texture coordinates with origin at bottom

  std::cout << "Attempting to load image from path: " << image_path
            << std::endl;

  background_image_data =
      stbi_load(image_path.c_str(), &background_image_width,
                &background_image_height, &background_image_channels,
                0  // Desired channels, 0 means use original
      );

  if (!background_image_data) {
    std::cerr << "ERROR::CANVAS::BACKGROUND_IMAGE_LOAD_FAILED: " << image_path
              << std::endl;
    std::cerr << "STB Error: " << stbi_failure_reason() << std::endl;
    return;
  }

  std::cout << "Background image loaded successfully: "
            << background_image_width << "x" << background_image_height
            << " with " << background_image_channels << " channels"
            << std::endl;

  // Setup the background image resources
  SetupBackgroundImage(background_image_width, background_image_height,
                       background_image_channels, background_image_data);

  // Calculate the real-world dimensions of the image based on resolution
  // Resolution is in meters/pixel, so we multiply by pixel dimensions to get
  // real-world size
  float real_width = background_image_width * resolution;
  float real_height = background_image_height * resolution;

  std::cout << "Real-world dimensions: " << real_width << "m x " << real_height
            << "m" << std::endl;

  // Extract origin parameters
  float origin_x = origin.x;  // x-coordinate of lower-left pixel
  float origin_y = origin.y;  // y-coordinate of lower-left pixel
  float yaw = origin.z;       // Rotation angle in radians (counterclockwise)

  std::cout << "Applying transform: origin_x=" << origin_x
            << ", origin_y=" << origin_y << ", yaw=" << yaw << " radians ("
            << (yaw * 180.0f / M_PI) << " degrees)" << std::endl;

  // Calculate the coordinates of the four corners of the image in real-world
  // coordinates Start with the corners relative to origin (before rotation)
  glm::vec2 bottom_left(0.0f, 0.0f);             // Origin point
  glm::vec2 bottom_right(real_width, 0.0f);      // Right from origin
  glm::vec2 top_right(real_width, real_height);  // Top-right corner
  glm::vec2 top_left(0.0f, real_height);         // Top from origin

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

  std::cout << "Transformed corners: " << std::endl;
  std::cout << "  Bottom-left: (" << bl.x << ", " << bl.y << ")" << std::endl;
  std::cout << "  Bottom-right: (" << br.x << ", " << br.y << ")" << std::endl;
  std::cout << "  Top-right: (" << tr.x << ", " << tr.y << ")" << std::endl;
  std::cout << "  Top-left: (" << tl.x << ", " << tl.y << ")" << std::endl;

  // Vertices for the transformed quad (all on z=0 plane)
  float vertices[] = {
      // Positions (x, y, z)              // Texture coords
      bl.x, bl.y, 0.0f, 0.0f, 0.0f,  // Bottom left
      br.x, br.y, 0.0f, 1.0f, 0.0f,  // Bottom right
      tr.x, tr.y, 0.0f, 1.0f, 1.0f,  // Top right
      tl.x, tl.y, 0.0f, 0.0f, 1.0f   // Top left
  };

  // Update the vertex buffer with the new positions
  glBindVertexArray(background_vao_);
  glBindBuffer(GL_ARRAY_BUFFER, background_vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  glBindVertexArray(0);

  // Free the background image data
  if (background_image_data) {
    stbi_image_free(background_image_data);
    background_image_data = nullptr;
  }
}

void Canvas::SetupBackgroundImage(int width, int height, int channels,
                                  unsigned char* data) {
  if (!data) {
    std::cerr << "ERROR::CANVAS::BACKGROUND_IMAGE_DATA_NULL" << std::endl;
    return;
  }

  // Setup the background shader if it hasn't been compiled yet
  try {
    Shader background_vertex_shader(background_vertex_shader_source.c_str(),
                                    Shader::Type::kVertex);
    Shader background_fragment_shader(background_fragment_shader_source.c_str(),
                                      Shader::Type::kFragment);
    background_shader_.AttachShader(background_vertex_shader);
    background_shader_.AttachShader(background_fragment_shader);

    if (!background_shader_.LinkProgram()) {
      std::cerr << "ERROR::CANVAS::BACKGROUND_SHADER_PROGRAM_LINKING_FAILED"
                << std::endl;
      return;
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception during shader compilation: " << e.what()
              << std::endl;
    return;
  }

  // Create texture for the background image
  {
    std::lock_guard<std::mutex> lock(background_mutex_);
    uint32_t texture_id = background_texture_.load();
    if (texture_id == 0) {
      GLuint new_texture_id = 0;
      glGenTextures(1, &new_texture_id);
      background_texture_ = new_texture_id;
      texture_id = new_texture_id;
    }

    glBindTexture(GL_TEXTURE_2D, texture_id);

    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                    GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  }

  // Determine the format based on the number of channels
  GLenum format;
  GLenum internal_format;

  if (channels == 1) {
    format = GL_RED;
    internal_format = GL_RED;
  } else if (channels == 3) {
    format = GL_RGB;
    internal_format = GL_SRGB;  // Use SRGB for better color accuracy
  } else if (channels == 4) {
    format = GL_RGBA;
    internal_format =
        GL_SRGB_ALPHA;  // Use SRGB_ALPHA for better color accuracy with alpha
  } else {
    std::cerr << "ERROR::CANVAS::UNSUPPORTED_IMAGE_FORMAT: " << channels
              << " channels" << std::endl;
    return;
  }

  std::cout << "Using texture format: "
            << (format == GL_RGBA  ? "RGBA"
                : format == GL_RGB ? "RGB"
                                   : "RED")
            << " with " << channels << " channels" << std::endl;
  std::cout << "Internal format: "
            << (internal_format == GL_SRGB_ALPHA ? "SRGB_ALPHA"
                : internal_format == GL_SRGB     ? "SRGB"
                                                 : "RED")
            << std::endl;

  // Print first few pixels to debug
  if (channels == 4) {
    std::cout << "First few pixels RGBA values:" << std::endl;
    for (int i = 0; i < 10; i++) {
      int idx = i * 4;
      std::cout << "Pixel " << i << ": R=" << (int)data[idx]
                << " G=" << (int)data[idx + 1] << " B=" << (int)data[idx + 2]
                << " A=" << (int)data[idx + 3]
                << " (alpha: " << (int)data[idx + 3] / 255.0f * 100.0f << "%)"
                << std::endl;
    }

    // Check some sample pixels in the middle of the image
    std::cout << "Middle area pixels:" << std::endl;
    int middle_row = height / 2;
    int middle_col = width / 2;
    for (int r = middle_row; r < middle_row + 5; r++) {
      for (int c = middle_col; c < middle_col + 5; c++) {
        int pixel_idx = (r * width + c) * 4;
        std::cout << "Pixel at (" << r << "," << c << "): RGBA=("
                  << (int)data[pixel_idx] << "," << (int)data[pixel_idx + 1]
                  << "," << (int)data[pixel_idx + 2] << ","
                  << (int)data[pixel_idx + 3] << ")" << std::endl;
      }
    }
  }

  // Upload the image data to the texture
  {
    std::lock_guard<std::mutex> lock(background_mutex_);
    uint32_t texture_id = background_texture_.load();
    glBindTexture(GL_TEXTURE_2D, texture_id);

    glTexImage2D(GL_TEXTURE_2D,
                 0,                // Mipmap level
                 internal_format,  // Internal format
                 width, height,
                 0,                 // Border (always 0)
                 format,            // Format
                 GL_UNSIGNED_BYTE,  // Type
                 data               // Data
    );

    // Generate mipmaps for better quality when scaled
    glGenerateMipmap(GL_TEXTURE_2D);

    // Check for GL errors
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
      std::cerr << "OpenGL error after texture setup: " << error << std::endl;
    }

    // Unbind the texture
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  // Create VAO and VBO for the background quad if they don't exist
  if (background_vao_ == 0) {
    glGenVertexArrays(1, &background_vao_);
    if (background_vao_ == 0) {
      std::cerr << "ERROR::CANVAS::BACKGROUND_VAO_GENERATION_FAILED"
                << std::endl;
      return;
    }
  }

  if (background_vbo_ == 0) {
    glGenBuffers(1, &background_vbo_);
    if (background_vbo_ == 0) {
      std::cerr << "ERROR::CANVAS::BACKGROUND_VBO_GENERATION_FAILED"
                << std::endl;
      return;
    }
  }

  // Default setup of a simple quad - actual coordinates will be updated in
  // AddBackgroundImage We're creating a 1x1 quad centered at the origin
  float vertices[] = {
      // Positions (x, y, z)       // Texture coords
      -0.5f, -0.5f, 0.0f, 0.0f, 0.0f,  // Bottom left
      0.5f,  -0.5f, 0.0f, 1.0f, 0.0f,  // Bottom right
      0.5f,  0.5f,  0.0f, 1.0f, 1.0f,  // Top right
      -0.5f, 0.5f,  0.0f, 0.0f, 1.0f   // Top left
  };

  // Setup the VBO and VAO
  glBindVertexArray(background_vao_);
  glBindBuffer(GL_ARRAY_BUFFER, background_vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  // Position attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  // Texture coordinate attribute
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                        (void*)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  glBindVertexArray(0);

  std::cout << "Background image setup completed successfully" << std::endl;
}

void Canvas::AddPoint(float x, float y, const glm::vec4& color,
                      float thickness) {
  std::vector<Point> points;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    points = data_->points;
    Point point;
    point.position = glm::vec3(x, y, 0.0f);  // Use X-Y plane for 2D drawing
    point.color = color;
    point.size = thickness;

    points.push_back(point);
    data_->points = points;
  }

  // Update the VBO with the new data
  if (primitive_vao_ != 0 && primitive_vbo_ != 0) {
    glBindVertexArray(primitive_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, primitive_vbo_);
    glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(Point), points.data(),
                 GL_STATIC_DRAW);
    glBindVertexArray(0);
  }
}

void Canvas::AddLine(float x1, float y1, float x2, float y2,
                     const glm::vec4& color, float thickness,
                     LineType line_type) {
  // For simplicity, we'll just add two points for now
  // In a more complete implementation, we would use line primitives
  AddPoint(x1, y1, color, thickness);
  AddPoint(x2, y2, color, thickness);
}

void Canvas::AddRectangle(float x, float y, float width, float height,
                          const glm::vec4& color, bool filled, float thickness,
                          LineType line_type) {
  // For simplicity, we'll just add four points for the corners
  // In a more complete implementation, we would use line or triangle primitives
  AddPoint(x, y, color, thickness);
  AddPoint(x + width, y, color, thickness);
  AddPoint(x + width, y + height, color, thickness);
  AddPoint(x, y + height, color, thickness);
}

void Canvas::AddCircle(float x, float y, float radius, const glm::vec4& color,
                       bool filled, float thickness, LineType line_type) {
  // For simplicity, we'll just add a single point at the center
  // In a more complete implementation, we would use triangle primitives
  AddPoint(x, y, color, radius * 2.0f);
}

void Canvas::AddEllipse(float x, float y, float rx, float ry, float angle,
                        float start_angle, float end_angle,
                        const glm::vec4& color, bool filled, float thickness,
                        LineType line_type) {
  // For simplicity, we'll just add a single point at the center
  // In a more complete implementation, we would use triangle primitives
  AddPoint(x, y, color, std::max(rx, ry) * 2.0f);
}

void Canvas::AddPolygon(const std::vector<glm::vec2>& points,
                        const glm::vec4& color, bool filled, float thickness,
                        LineType line_type) {
  // For simplicity, we'll just add points for each vertex
  // In a more complete implementation, we would use line or triangle primitives
  for (const auto& point : points) {
    AddPoint(point.x, point.y, color, thickness);
  }
}

void Canvas::Clear() {
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_->Clear();
  }

  // Update the VBO with the new data
  if (primitive_vao_ != 0 && primitive_vbo_ != 0) {
    glBindVertexArray(primitive_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, primitive_vbo_);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
    glBindVertexArray(0);
  }
}

void Canvas::AllocateGpuResources() {
  // First make sure any existing resources are released
  ReleaseGpuResources();

  // Compile and link shaders
  Shader vertex_shader(vertex_shader_source.c_str(), Shader::Type::kVertex);
  Shader fragment_shader(fragment_shader_source.c_str(),
                         Shader::Type::kFragment);
  primitive_shader_.AttachShader(vertex_shader);
  primitive_shader_.AttachShader(fragment_shader);

  if (!primitive_shader_.LinkProgram()) {
    std::cerr << "ERROR::CANVAS::SHADER_PROGRAM_LINKING_FAILED" << std::endl;
    throw std::runtime_error("Shader program linking failed");
  }

  // Create and set up VAO and VBO
  glGenVertexArrays(1, &primitive_vao_);
  glGenBuffers(1, &primitive_vbo_);

  glBindVertexArray(primitive_vao_);

  // Set up VBO for points
  glBindBuffer(GL_ARRAY_BUFFER, primitive_vbo_);

  // Set up vertex attributes
  // Position attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point),
                        (void*)offsetof(Point, position));
  glEnableVertexAttribArray(0);

  // Color attribute
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Point),
                        (void*)offsetof(Point, color));
  glEnableVertexAttribArray(1);

  // Size attribute
  glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(Point),
                        (void*)offsetof(Point, size));
  glEnableVertexAttribArray(2);

  // Unbind
  glBindVertexArray(0);
}

void Canvas::ReleaseGpuResources() {
  // Delete background resources
  {
    std::lock_guard<std::mutex> lock(background_mutex_);
    uint32_t texture_id = background_texture_.load();
    if (texture_id != 0) {
      glDeleteTextures(1, &texture_id);
      background_texture_ = 0;
    }
  }

  if (background_vao_ != 0) {
    glDeleteVertexArrays(1, &background_vao_);
    background_vao_ = 0;
  }

  if (background_vbo_ != 0) {
    glDeleteBuffers(1, &background_vbo_);
    background_vbo_ = 0;
  }

  // Delete point resources
  if (primitive_vao_ != 0) {
    glDeleteVertexArrays(1, &primitive_vao_);
    primitive_vao_ = 0;
  }

  if (primitive_vbo_ != 0) {
    glDeleteBuffers(1, &primitive_vbo_);
    primitive_vbo_ = 0;
  }
}

void Canvas::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                    const glm::mat4& coord_transform) {
  // Draw background if available
  {
    std::lock_guard<std::mutex> lock(background_mutex_);
    uint32_t texture_id = background_texture_.load();
    if (texture_id != 0) {
      // std::cout << "Drawing background image (texture ID: " <<
      // background_texture_ << ")" << std::endl;

      // Bind background shader and set uniforms
      background_shader_.Use();
      background_shader_.SetUniform("projection", projection);
      background_shader_.SetUniform("view", view);
      background_shader_.SetUniform("model", glm::mat4(1.0f));
      background_shader_.SetUniform("coordSystemTransform", coord_transform);

      // Bind texture
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, texture_id);
      background_shader_.SetUniform("backgroundTexture", 0);

      // Check for errors after binding texture
      GLenum error = glGetError();
      if (error != GL_NO_ERROR) {
        std::cerr << "OpenGL error after binding texture: " << error
                  << std::endl;
      }

      // Save current OpenGL state
      GLboolean blendEnabled = glIsEnabled(GL_BLEND);
      GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
      GLint blendSrcRGB, blendDstRGB, blendSrcAlpha, blendDstAlpha;
      glGetIntegerv(GL_BLEND_SRC_RGB, &blendSrcRGB);
      glGetIntegerv(GL_BLEND_DST_RGB, &blendDstRGB);
      glGetIntegerv(GL_BLEND_SRC_ALPHA, &blendSrcAlpha);
      glGetIntegerv(GL_BLEND_DST_ALPHA, &blendDstAlpha);
      GLint depthFunc;
      glGetIntegerv(GL_DEPTH_FUNC, &depthFunc);

      // Setup blending for proper transparency
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      // Setup depth testing
      glEnable(GL_DEPTH_TEST);
      glDepthFunc(GL_LEQUAL);

      // Draw quad
      glBindVertexArray(background_vao_);
      glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

      // Check for errors after drawing
      error = glGetError();
      if (error != GL_NO_ERROR) {
        std::cerr << "OpenGL error after drawing background: " << error
                  << std::endl;
      }

      // Restore OpenGL state
      if (!blendEnabled) {
        glDisable(GL_BLEND);
      } else {
        glBlendFuncSeparate(blendSrcRGB, blendDstRGB, blendSrcAlpha,
                            blendDstAlpha);
      }

      if (!depthTestEnabled) {
        glDisable(GL_DEPTH_TEST);
      } else {
        glDepthFunc(depthFunc);
      }

      glBindVertexArray(0);
      glBindTexture(GL_TEXTURE_2D, 0);
    }
  }

  CanvasData data;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data = *data_;
  }

  // Draw points if available
  if (!data.points.empty()) {
    primitive_shader_.Use();
    primitive_shader_.SetUniform("projection", projection);
    primitive_shader_.SetUniform("view", view);
    primitive_shader_.SetUniform("model", glm::mat4(1.0f));
    primitive_shader_.SetUniform("coordSystemTransform", coord_transform);

    glBindVertexArray(primitive_vao_);

    // Enable depth test and blending
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Enable point size
    glEnable(GL_PROGRAM_POINT_SIZE);

    // Draw the points
    glDrawArrays(GL_POINTS, 0, data.points.size());

    // Disable point size
    glDisable(GL_PROGRAM_POINT_SIZE);

    glBindVertexArray(0);
  }
}
}  // namespace quickviz
