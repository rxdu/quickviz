/**
 * @file canvas.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "renderer/renderable/canvas.hpp"

#include <iostream>
#include <cmath>
#include <chrono>
#include <map>
#include <algorithm>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "glad/glad.h"
#include <glm/gtc/type_ptr.hpp>

#include "renderable/details/canvas_data.hpp"
#include "renderer/renderable/details/canvas_batching.hpp"
#include "renderer/renderable/details/canvas_performance.hpp"
#include "renderable/details/render_strategy.hpp"
#include "renderable/details/batched_render_strategy.hpp"
#include "renderable/details/individual_render_strategy.hpp"
#include "renderable/details/shape_renderer.hpp"
#include "renderable/details/shape_generators.hpp"

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

out vec4 fragColor;
out vec2 fragPos;
out float fragSize;

void main() {
    gl_Position = projection * view * model * coordSystemTransform * vec4(aPos, 1.0);
    gl_PointSize = aSize;
    fragColor = aColor;
    fragPos = gl_Position.xy;
    fragSize = aSize;
}
)";

std::string fragment_shader_source = R"(
#version 330 core

in vec4 fragColor;
in vec2 fragPos;
in float fragSize;
out vec4 FragColor;

uniform int renderMode; // 0 = points, 1 = lines, 2 = filled shapes, 3 = outlined shapes
uniform int lineType;   // 0 = solid, 1 = dashed, 2 = dotted
uniform vec4 uColor;    // Fallback uniform color (used when fragColor is not available)

void main() {
    vec4 finalColor = fragColor;
    
    // Use uniform color as fallback if fragColor alpha is near zero (indicates no per-vertex color)
    if (fragColor.a < 0.01) {
        finalColor = uColor;
    }
    
    // Handle different rendering modes
    if (renderMode == 0) { // Points
        vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
        if (dot(circCoord, circCoord) > 1.0) {
            discard;
        }
        FragColor = finalColor;
        return;
    }
    
    // For lines and outlines, handle different line types
    if (renderMode == 1 || renderMode == 3) {
        if (lineType > 0) {
            float pattern;
            float scale = 20.0; // Adjust this to change the dash/dot pattern size
            
            if (lineType == 1) { // Dashed
                pattern = mod(fragPos.x * scale, 10.0);
                if (pattern > 5.0) discard;
            } else if (lineType == 2) { // Dotted
                pattern = mod(fragPos.x * scale, 4.0);
                if (pattern > 2.0) discard;
            }
        }
    }
    
    FragColor = finalColor;
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
   // float border = 0.005;
   // if (TexCoord.x < border || TexCoord.x > 1.0 - border || 
   //     TexCoord.y < border || TexCoord.y > 1.0 - border) {
   //     FragColor = mix(vec4(1.0, 0.0, 0.0, 1.0), texColor, 0.3); // Subtle red border
   // }
}
)";
}  // namespace

Canvas::Canvas() {
  // Initialize core data storage (original working system)
  data_ = std::make_unique<CanvasData>();
  
  // Ensure data is not null
  if (!data_) {
    std::cerr << "ERROR::CANVAS::FAILED_TO_ALLOCATE_CANVAS_DATA" << std::endl;
    throw std::runtime_error("Failed to allocate CanvasData");
  }
  
  // Initialize the data object (just to be extra safe)
  data_->Clear();
  
  // Initialize render strategies (Phase 1 improvement - defer shape_renderer_ until after GPU resources)
  batched_strategy_ = std::make_unique<BatchedRenderStrategy>(
    line_batches_, filled_shape_batch_, outline_shape_batches_);
  individual_strategy_ = std::make_unique<IndividualRenderStrategy>();
  
  // Set default strategy based on batching preference
  current_render_strategy_ = batching_enabled_ ? 
    static_cast<RenderStrategy*>(batched_strategy_.get()) : 
    static_cast<RenderStrategy*>(individual_strategy_.get());
  
  // Re-enable batching now that ellipse/polygon renderMode is fixed
  batching_enabled_ = true;
  
  AllocateGpuResources();
  InitializeBatches();
  
  // Create shape renderer after GPU resources are allocated
  shape_renderer_ = std::make_unique<ShapeRenderer>(&primitive_shader_);
  
  // Update render strategies with shape renderer
  batched_strategy_ = std::make_unique<BatchedRenderStrategy>(
    line_batches_, filled_shape_batch_, outline_shape_batches_, shape_renderer_.get());
  individual_strategy_ = std::make_unique<IndividualRenderStrategy>(shape_renderer_.get());
  
  // Update current strategy pointer
  current_render_strategy_ = batching_enabled_ ? 
    static_cast<RenderStrategy*>(batched_strategy_.get()) : 
    static_cast<RenderStrategy*>(individual_strategy_.get());
}

Canvas::~Canvas() { 
  ReleaseGpuResources(); 
}

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

  // Vertices for the transformed quad (placed behind primitives)
  float vertices[] = {
      // Positions (x, y, z)              // Texture coords
      bl.x, bl.y, -0.01f, 0.0f, 0.0f,  // Bottom left
      br.x, br.y, -0.01f, 1.0f, 0.0f,  // Bottom right
      tr.x, tr.y, -0.01f, 1.0f, 1.0f,  // Top right
      tl.x, tl.y, -0.01f, 0.0f, 1.0f   // Top left
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

glm::vec2 Canvas::GetBackgroundImageSize() const {
  return background_image_size_;
}

void Canvas::SetupBackgroundImage(int width, int height, int channels,
                                  unsigned char* data) {
  if (!data) {
    std::cerr << "ERROR::CANVAS::BACKGROUND_IMAGE_DATA_NULL" << std::endl;
    return;
  }

  background_image_size_.x = static_cast<float>(width);
  background_image_size_.y = static_cast<float>(height);

  // Setup the background shader if it hasn't been compiled yet
  try {
    Shader background_vertex_shader(background_vertex_shader_source.c_str(),
                                    Shader::Type::kVertex);
    Shader background_fragment_shader(background_fragment_shader_source.c_str(),
                                      Shader::Type::kFragment);
    
    // IMPORTANT: Compile shaders before linking
    if (!background_vertex_shader.Compile()) {
      std::cerr << "ERROR::CANVAS::BACKGROUND_VERTEX_SHADER_COMPILATION_FAILED" << std::endl;
      throw std::runtime_error("Background vertex shader compilation failed");
    }
    
    if (!background_fragment_shader.Compile()) {
      std::cerr << "ERROR::CANVAS::BACKGROUND_FRAGMENT_SHADER_COMPILATION_FAILED" << std::endl;
      throw std::runtime_error("Background fragment shader compilation failed");
    }
    
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

    // Check for OpenGL row alignment issues
    // OpenGL expects texture data to be aligned to 4-byte boundaries by default
    int bytes_per_pixel = (format == GL_RGBA) ? 4 : (format == GL_RGB) ? 3 : 1;
    int bytes_per_row = width * bytes_per_pixel;
    bool alignment_issue = (bytes_per_row % 4) != 0;
    
    if (alignment_issue) {
      std::cout << "Canvas: Adjusting alignment for " << width << "x" << height 
                << " texture (row size: " << bytes_per_row << " bytes)" << std::endl;
      // Set pixel alignment to 1 byte to handle unaligned row data
      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    }

    // Clear any previous OpenGL errors
    while (glGetError() != GL_NO_ERROR);

    glTexImage2D(GL_TEXTURE_2D,
                 0,                // Mipmap level
                 internal_format,  // Internal format
                 width, height,
                 0,                 // Border (always 0)
                 format,            // Format
                 GL_UNSIGNED_BYTE,  // Type
                 data               // Data
    );
    
    // Restore default alignment if we changed it
    if (alignment_issue) {
      glPixelStorei(GL_UNPACK_ALIGNMENT, 4);  // Restore default
    }

    // Check for GL errors after texture upload
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
      std::cerr << "OpenGL error after glTexImage2D: " << error;
      switch (error) {
        case GL_INVALID_ENUM:
          std::cerr << " (GL_INVALID_ENUM - format/internal format invalid)";
          break;
        case GL_INVALID_VALUE:
          std::cerr << " (GL_INVALID_VALUE - width/height invalid)";
          break;
        case GL_INVALID_OPERATION:
          std::cerr << " (GL_INVALID_OPERATION - format/internal format incompatible)";
          break;
        case GL_OUT_OF_MEMORY:
          std::cerr << " (GL_OUT_OF_MEMORY - insufficient memory)";
          break;
        default:
          std::cerr << " (Unknown error)";
          break;
      }
      std::cerr << std::endl;
      glBindTexture(GL_TEXTURE_2D, 0);
      return;
    }

    // Generate mipmaps for better quality when scaled
    glGenerateMipmap(GL_TEXTURE_2D);

    // Check for GL errors after mipmap generation
    error = glGetError();
    if (error != GL_NO_ERROR) {
      std::cerr << "OpenGL error after glGenerateMipmap: " << error;
      switch (error) {
        case GL_INVALID_ENUM:
          std::cerr << " (GL_INVALID_ENUM - target invalid)";
          break;
        case GL_INVALID_OPERATION:
          std::cerr << " (GL_INVALID_OPERATION - texture not complete)";
          break;
        default:
          std::cerr << " (Unknown error)";
          break;
      }
      std::cerr << std::endl;
      glBindTexture(GL_TEXTURE_2D, 0);
      return;
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
  // Using very small negative Z to ensure background renders behind primitives while minimizing alignment issues
  float vertices[] = {
      // Positions (x, y, z)       // Texture coords
      -0.5f, -0.5f, -0.001f, 0.0f, 0.0f,  // Bottom left
      0.5f,  -0.5f, -0.001f, 1.0f, 0.0f,  // Bottom right
      0.5f,  0.5f,  -0.001f, 1.0f, 1.0f,  // Top right
      -0.5f, 0.5f,  -0.001f, 0.0f, 1.0f   // Top left
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
  // Always store the primitive - the difference between modes is in how they're rendered
  // Non-batching mode: render individually in sequence order during OnDraw
  // Batching mode: batch similar primitives for efficient rendering during OnDraw

  // Batching mode - store for later rendering
  PendingUpdate update;
  update.type = PendingUpdate::Type::kPoint;
  update.color = color;
  update.thickness = thickness;
  update.point.x = x;
  update.point.y = y;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(update);
  has_pending_updates_ = true;
}

void Canvas::AddLine(float x1, float y1, float x2, float y2,
                     const glm::vec4& color, float thickness,
                     LineType line_type) {
  // Always store the primitive for proper rendering during OnDraw

  // Batching mode - store for later rendering
  PendingUpdate update;
  update.type = PendingUpdate::Type::kLine;
  update.color = color;
  update.thickness = thickness;
  update.line_type = line_type;
  update.line.x1 = x1;
  update.line.y1 = y1;
  update.line.x2 = x2;
  update.line.y2 = y2;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(update);
  has_pending_updates_ = true;
}

void Canvas::AddRectangle(float x, float y, float width, float height,
                          const glm::vec4& color, bool filled, float thickness,
                          LineType line_type) {
  // Always store the primitive for proper rendering during OnDraw

  // Batching mode - store for later rendering
  PendingUpdate update;
  update.type = PendingUpdate::Type::kRectangle;
  update.color = color;
  update.thickness = thickness;
  update.line_type = line_type;
  update.filled = filled;
  update.rect.x = x;
  update.rect.y = y;
  update.rect.width = width;
  update.rect.height = height;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(update);
  has_pending_updates_ = true;
}

void Canvas::AddCircle(float x, float y, float radius, const glm::vec4& color,
                       bool filled, float thickness, LineType line_type) {
  // Always store the primitive for proper rendering during OnDraw

  // Batching mode - store for later rendering
  PendingUpdate update;
  update.type = PendingUpdate::Type::kCircle;
  update.color = color;
  update.thickness = thickness;
  update.line_type = line_type;
  update.filled = filled;
  update.circle.x = x;
  update.circle.y = y;
  update.circle.radius = radius;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(update);
  has_pending_updates_ = true;
}

void Canvas::AddEllipse(float x, float y, float rx, float ry, float angle,
                        float start_angle, float end_angle, const glm::vec4& color,
                        bool filled, float thickness, LineType line_type) {
  // Always store the primitive for proper rendering during OnDraw

  // Batching mode - store for later rendering
  PendingUpdate update;
  update.type = PendingUpdate::Type::kEllipse;
  update.color = color;
  update.thickness = thickness;
  update.line_type = line_type;
  update.filled = filled;
  update.ellipse.x = x;
  update.ellipse.y = y;
  update.ellipse.rx = rx;
  update.ellipse.ry = ry;
  update.ellipse.angle = angle;
  update.ellipse.start_angle = start_angle;
  update.ellipse.end_angle = end_angle;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(update);
  has_pending_updates_ = true;
}

void Canvas::AddPolygon(const std::vector<glm::vec2>& points,
                        const glm::vec4& color, bool filled, float thickness,
                        LineType line_type) {
  // Always store the primitive for proper rendering during OnDraw

  // Batching mode - store for later rendering
  PendingUpdate update;
  update.type = PendingUpdate::Type::kPolygon;
  update.color = color;
  update.thickness = thickness;
  update.line_type = line_type;
  update.filled = filled;
  update.polygon_vertices = points;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(update);
  has_pending_updates_ = true;
}

void Canvas::Clear() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  data_->Clear();
  ClearBatches();
  
  // Clear pending updates
  std::queue<PendingUpdate> empty_queue;
  pending_updates_.swap(empty_queue);
  has_pending_updates_ = false;
}

void Canvas::ProcessPendingUpdates() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  while (!pending_updates_.empty()) {
    const auto& update = pending_updates_.front();
    
    if (batching_enabled_) {
      // Use batching system with proper sequence tracking
      switch (update.type) {
        case PendingUpdate::Type::kPoint:
          // Points still use the original system since they're already efficient
          data_->AddPoint(update.point.x, update.point.y, update.color, update.thickness);
          break;
          
        case PendingUpdate::Type::kLine: {
          // Add line to batch with sequence tracking
          auto& line_batch = line_batches_[update.line_type];
          uint32_t sequence = batch_order_tracker_.GetNextSequence();
          uint32_t line_index = line_batch.vertices.size() / 2; // 2 vertices per line
          
          line_batch.vertices.emplace_back(update.line.x1, update.line.y1, 0.0f);
          line_batch.vertices.emplace_back(update.line.x2, update.line.y2, 0.0f);
          line_batch.colors.push_back(update.color);
          line_batch.colors.push_back(update.color);
          line_batch.thicknesses.push_back(update.thickness);
          line_batch.line_types.push_back(update.line_type);
          line_batch.sequence_numbers.push_back(sequence);
          line_batch.needs_update = true;
          
          // Record in order tracker
          batch_order_tracker_.render_order.push_back({
            BatchOrderTracker::OrderedPrimitive::Type::kLine,
            update.line_type,
            sequence,
            line_index
          });
          break;
        }
        
        case PendingUpdate::Type::kRectangle: {
          uint32_t sequence = batch_order_tracker_.GetNextSequence();
          
          if (update.filled) {
            uint32_t shape_index = filled_shape_batch_.sequence_numbers.size();
            uint32_t base_index = filled_shape_batch_.vertices.size() / 3;
            uint32_t start_index_pos = filled_shape_batch_.indices.size();
            
            GenerateRectangleVertices(update.rect.x, update.rect.y,
                                      update.rect.width, update.rect.height,
                                      filled_shape_batch_.vertices,
                                      filled_shape_batch_.indices,
                                      true, base_index, sequence);
            
            // Track index range for this rectangle (6 indices for 2 triangles)
            uint32_t index_count = filled_shape_batch_.indices.size() - start_index_pos;
            filled_shape_batch_.index_ranges.push_back({start_index_pos, index_count});
            
            
            // Add color for each vertex (4 vertices for rectangle)
            for (int i = 0; i < 4; i++) {
              filled_shape_batch_.colors.push_back(update.color);
            }
            filled_shape_batch_.sequence_numbers.push_back(sequence);
            filled_shape_batch_.needs_update = true;
            
            // Record in order tracker
            batch_order_tracker_.render_order.push_back({
              BatchOrderTracker::OrderedPrimitive::Type::kFilledShape,
              LineType::kSolid, // Not used for filled shapes
              sequence,
              shape_index
            });
          } else {
            auto& outline_batch = outline_shape_batches_[update.line_type];
            uint32_t shape_index = outline_batch.sequence_numbers.size();
            uint32_t base_index = outline_batch.vertices.size() / 3;
            uint32_t start_index_pos = outline_batch.indices.size();
            
            GenerateRectangleVertices(update.rect.x, update.rect.y,
                                      update.rect.width, update.rect.height,
                                      outline_batch.vertices,
                                      outline_batch.indices,
                                      false, base_index, sequence);
            
            // Track index range for this rectangle outline (8 indices for line loop)
            uint32_t index_count = outline_batch.indices.size() - start_index_pos;
            outline_batch.index_ranges.push_back({start_index_pos, index_count});
            
            // Add color for each vertex (4 vertices for rectangle)
            for (int i = 0; i < 4; i++) {
              outline_batch.colors.push_back(update.color);
            }
            outline_batch.sequence_numbers.push_back(sequence);
            outline_batch.needs_update = true;
            
            // Record in order tracker
            batch_order_tracker_.render_order.push_back({
              BatchOrderTracker::OrderedPrimitive::Type::kOutlineShape,
              update.line_type,
              sequence,
              shape_index
            });
          }
          break;
        }
        
        case PendingUpdate::Type::kCircle: {
          const int segments = 32; // Could be made adaptive based on radius
          uint32_t sequence = batch_order_tracker_.GetNextSequence();
          
          if (update.filled) {
            uint32_t shape_index = filled_shape_batch_.sequence_numbers.size();
            uint32_t base_index = filled_shape_batch_.vertices.size() / 3;
            uint32_t start_index_pos = filled_shape_batch_.indices.size();
            
            GenerateCircleVertices(update.circle.x, update.circle.y,
                                   update.circle.radius, segments,
                                   filled_shape_batch_.vertices,
                                   filled_shape_batch_.indices,
                                   true, base_index, sequence);
            
            // Track index range for this circle (3 * segments indices for triangle fan)
            uint32_t index_count = filled_shape_batch_.indices.size() - start_index_pos;
            filled_shape_batch_.index_ranges.push_back({start_index_pos, index_count});
            
            
            // Add color for center + perimeter vertices
            for (int i = 0; i < segments + 2; i++) {
              filled_shape_batch_.colors.push_back(update.color);
            }
            filled_shape_batch_.sequence_numbers.push_back(sequence);
            filled_shape_batch_.needs_update = true;
            
            // Record in order tracker
            batch_order_tracker_.render_order.push_back({
              BatchOrderTracker::OrderedPrimitive::Type::kFilledShape,
              LineType::kSolid,
              sequence,
              shape_index
            });
          } else {
            auto& outline_batch = outline_shape_batches_[update.line_type];
            uint32_t shape_index = outline_batch.sequence_numbers.size();
            uint32_t base_index = outline_batch.vertices.size() / 3;
            uint32_t start_index_pos = outline_batch.indices.size();
            
            GenerateCircleVertices(update.circle.x, update.circle.y,
                                   update.circle.radius, segments,
                                   outline_batch.vertices,
                                   outline_batch.indices,
                                   false, base_index, sequence);
            
            // Track index range for this circle outline (segments * 2 indices for line loop)
            uint32_t index_count = outline_batch.indices.size() - start_index_pos;
            outline_batch.index_ranges.push_back({start_index_pos, index_count});
            
            // Add color for perimeter vertices
            for (int i = 0; i < segments + 1; i++) {
              outline_batch.colors.push_back(update.color);
            }
            outline_batch.sequence_numbers.push_back(sequence);
            outline_batch.needs_update = true;
            
            // Record in order tracker
            batch_order_tracker_.render_order.push_back({
              BatchOrderTracker::OrderedPrimitive::Type::kOutlineShape,
              update.line_type,
              sequence,
              shape_index
            });
          }
          break;
        }
        
        case PendingUpdate::Type::kEllipse:
        case PendingUpdate::Type::kPolygon:
          // These still use the original system as they're not batched
          if (update.type == PendingUpdate::Type::kEllipse) {
            data_->AddEllipse(update.ellipse.x, update.ellipse.y, update.ellipse.rx, update.ellipse.ry,
                             update.ellipse.angle, update.ellipse.start_angle, update.ellipse.end_angle,
                             update.color, update.filled, update.thickness, update.line_type);
          } else {
            data_->AddPolygon(update.polygon_vertices, update.color, update.filled, 
                             update.thickness, update.line_type);
          }
          break;
          
        case PendingUpdate::Type::kClear:
          data_->Clear();
          ClearBatches();
          batch_order_tracker_.Clear();
          break;
      }
    } else {
      // Non-batching mode - use original system
      switch (update.type) {
        case PendingUpdate::Type::kPoint:
          data_->AddPoint(update.point.x, update.point.y, update.color, update.thickness);
          break;
        case PendingUpdate::Type::kLine:
          data_->AddLine(update.line.x1, update.line.y1, update.line.x2, update.line.y2,
                        update.color, update.thickness, update.line_type);
          break;
        case PendingUpdate::Type::kRectangle:
          data_->AddRectangle(update.rect.x, update.rect.y, update.rect.width, update.rect.height,
                             update.color, update.filled, update.thickness, update.line_type);
          break;
        case PendingUpdate::Type::kCircle:
          data_->AddCircle(update.circle.x, update.circle.y, update.circle.radius,
                          update.color, update.filled, update.thickness, update.line_type);
          break;
        case PendingUpdate::Type::kEllipse:
          data_->AddEllipse(update.ellipse.x, update.ellipse.y, update.ellipse.rx, update.ellipse.ry,
                           update.ellipse.angle, update.ellipse.start_angle, update.ellipse.end_angle,
                           update.color, update.filled, update.thickness, update.line_type);
          break;
        case PendingUpdate::Type::kPolygon:
          data_->AddPolygon(update.polygon_vertices, update.color, update.filled, 
                           update.thickness, update.line_type);
          break;
        case PendingUpdate::Type::kClear:
          data_->Clear();
          ClearBatches();
          break;
      }
    }
    
    pending_updates_.pop();
  }
  
  has_pending_updates_ = false;
  
  // Update batches after processing updates
  UpdateBatches();
}

void Canvas::SetBatchingEnabled(bool enabled) {
  batching_enabled_ = enabled;
  current_render_strategy_ = batching_enabled_ ? 
    static_cast<RenderStrategy*>(batched_strategy_.get()) : 
    static_cast<RenderStrategy*>(individual_strategy_.get());
}

bool Canvas::IsBatchingEnabled() const {
  return batching_enabled_;
}

void Canvas::AllocateGpuResources() {
  // First make sure any existing resources are released
  ReleaseGpuResources();

  try {
    std::cout << "Compiling primitive vertex shader..." << std::endl;
    quickviz::Shader vertex_shader(vertex_shader_source.c_str(), quickviz::Shader::Type::kVertex);
    if (!vertex_shader.Compile()) {
      std::cerr << "ERROR::CANVAS::VERTEX_SHADER_COMPILATION_FAILED" << std::endl;
      throw std::runtime_error("Vertex shader compilation failed");
    }
    
    std::cout << "Compiling primitive fragment shader..." << std::endl;
    quickviz::Shader fragment_shader(fragment_shader_source.c_str(), quickviz::Shader::Type::kFragment);
    if (!fragment_shader.Compile()) {
      std::cerr << "ERROR::CANVAS::FRAGMENT_SHADER_COMPILATION_FAILED" << std::endl;
      throw std::runtime_error("Fragment shader compilation failed");
    }
    
    std::cout << "Attaching shaders to program and linking..." << std::endl;
    primitive_shader_.AttachShader(vertex_shader);
    primitive_shader_.AttachShader(fragment_shader);
    if (!primitive_shader_.LinkProgram()) {
      std::cerr << "ERROR::CANVAS::SHADER_PROGRAM_LINKING_FAILED" << std::endl;
      throw std::runtime_error("Shader program linking failed");
    }
    std::cout << "Primitive shader program compiled and linked successfully." << std::endl;
    
    // Generate vertex array and vertex buffer for primitives
    glGenVertexArrays(1, &primitive_vao_);
    if (primitive_vao_ == 0) {
      throw std::runtime_error("Failed to generate vertex array object");
    }
    
    glGenBuffers(1, &primitive_vbo_);
    if (primitive_vbo_ == 0) {
      throw std::runtime_error("Failed to generate vertex buffer object");
    }
    
    // Bind and configure vertex array object
    glBindVertexArray(primitive_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, primitive_vbo_);
    
    // Setup vertex attributes for Point structure
    const size_t initial_buffer_size = 10 * sizeof(quickviz::Point);
    glBufferData(GL_ARRAY_BUFFER, initial_buffer_size, nullptr, GL_DYNAMIC_DRAW);
    
    // Position attribute (location = 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(quickviz::Point), 
                          (void*)offsetof(quickviz::Point, position));
    glEnableVertexAttribArray(0);
    
    // Color attribute (location = 1)
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(quickviz::Point), 
                          (void*)offsetof(quickviz::Point, color));
    glEnableVertexAttribArray(1);
    
    // Size attribute (location = 2)
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(quickviz::Point), 
                          (void*)offsetof(quickviz::Point, size));
    glEnableVertexAttribArray(2);
    
    // Unbind for safety
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    std::cout << "GPU resources allocated successfully." << std::endl;
    
  } catch (const std::exception& e) {
    std::cerr << "ERROR::CANVAS::GPU_RESOURCE_ALLOCATION_FAILED: " << e.what() << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void Canvas::ReleaseGpuResources() noexcept {
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

  // Delete primitive rendering resources
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
  // Process any pending updates using original system
  if (has_pending_updates_) {
    ProcessPendingUpdates();
  }

  // Get a copy of the data to avoid locking during rendering
  CanvasData data;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data = *data_;
  }

  // Draw background if available (render first, so primitives appear on top)
  {
    std::lock_guard<std::mutex> lock(background_mutex_);
    uint32_t texture_id = background_texture_.load();
    if (texture_id != 0) {
      // Bind background shader and set uniforms
      background_shader_.Use();
      background_shader_.TrySetUniform("projection", projection);
      background_shader_.TrySetUniform("view", view);
      background_shader_.TrySetUniform("model", glm::mat4(1.0f));
      background_shader_.TrySetUniform("coordSystemTransform", coord_transform);

      // Bind texture
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, texture_id);
      background_shader_.TrySetUniform("backgroundTexture", 0);

      // Setup blending for proper transparency
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      
      // Enable depth testing to ensure background renders behind primitives
      glEnable(GL_DEPTH_TEST);

      // Draw quad
      glBindVertexArray(background_vao_);
      
      // Make sure the right attributes are enabled for background rendering
      glEnableVertexAttribArray(0);  // Position
      glEnableVertexAttribArray(1);  // Texture coordinates
      // Disable any other attributes that might be enabled
      glDisableVertexAttribArray(2);
      
      glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

      // Cleanup background rendering state
      glDisableVertexAttribArray(0);
      glDisableVertexAttribArray(1);
      glBindVertexArray(0);
      glBindTexture(GL_TEXTURE_2D, 0);
      glActiveTexture(GL_TEXTURE0);
      glUseProgram(0);
    }
  }
  
  // Skip if there's no data to render
  bool has_line_data = false;
  for (const auto& [line_type, line_batch] : line_batches_) {
    if (!line_batch.vertices.empty()) {
      has_line_data = true;
      break;
    }
  }
  
  bool has_outline_data = false;
  for (const auto& [line_type, outline_batch] : outline_shape_batches_) {
    if (!outline_batch.vertices.empty()) {
      has_outline_data = true;
      break;
    }
  }
  
  if (data.points.empty() && data.lines.empty() && data.rectangles.empty() &&
      data.circles.empty() && data.ellipses.empty() &&
      data.polygons.empty() && !has_line_data &&
      filled_shape_batch_.vertices.empty() && !has_outline_data) {
    return;
  }

  // Render individual shapes
  if (!data.ellipses.empty() || !data.polygons.empty()) {
    RenderIndividualShapes(data, projection, view, coord_transform);
  }

  // Render batched shapes
  if (has_line_data || !filled_shape_batch_.vertices.empty() || has_outline_data) {
    RenderBatches(projection, view, coord_transform);
  }
}

void Canvas::FlushBatches() {
  // Force processing of any pending updates
  if (has_pending_updates_) {
    ProcessPendingUpdates();
  }
}

const RenderStats& Canvas::GetRenderStats() const {
  return render_stats_;
}

void Canvas::ResetRenderStats() {
  render_stats_.Reset();
}

void Canvas::SetPerformanceConfig(const PerformanceConfig& config) {
  perf_config_ = config;
}

const PerformanceConfig& Canvas::GetPerformanceConfig() const {
  return perf_config_;
}

size_t Canvas::GetMemoryUsage() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  size_t total = sizeof(Canvas);
  
  // Add data structure sizes
  if (data_) {
    total += data_->points.capacity() * sizeof(Point);
    total += data_->lines.capacity() * sizeof(Line);
    total += data_->rectangles.capacity() * sizeof(Rectangle);
    total += data_->circles.capacity() * sizeof(Circle);
    total += data_->ellipses.capacity() * sizeof(Ellipse);
    total += data_->polygons.capacity() * sizeof(Polygon);
  }
  
  // Add batch sizes
  for (const auto& [type, batch] : line_batches_) {
    total += batch.vertices.capacity() * sizeof(float);
    total += batch.colors.capacity() * sizeof(float);
  }
  
  total += filled_shape_batch_.vertices.capacity() * sizeof(float);
  total += filled_shape_batch_.indices.capacity() * sizeof(uint32_t);
  
  return total;
}

void Canvas::OptimizeMemory() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Shrink data vectors to fit
  if (data_) {
    data_->points.shrink_to_fit();
    data_->lines.shrink_to_fit();
    data_->rectangles.shrink_to_fit();
    data_->circles.shrink_to_fit();
    data_->ellipses.shrink_to_fit();
    data_->polygons.shrink_to_fit();
  }
  
  // Clear empty pending updates queue
  if (pending_updates_.empty()) {
    std::queue<PendingUpdate> empty_queue;
    pending_updates_.swap(empty_queue);
  }
}

void Canvas::PreallocateMemory(size_t estimated_objects) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  if (data_) {
    // Reserve based on typical distribution
    data_->points.reserve(estimated_objects / 10);
    data_->lines.reserve(estimated_objects / 4);
    data_->rectangles.reserve(estimated_objects / 8);
    data_->circles.reserve(estimated_objects / 8);
    data_->ellipses.reserve(estimated_objects / 20);
    data_->polygons.reserve(estimated_objects / 20);
  }
}

void Canvas::ShrinkToFit() {
  OptimizeMemory(); // Same operation
}

void Canvas::InitializeBatches() {
  // Initialize line batches for each line type
  line_batches_[LineType::kSolid] = LineBatch{};
  line_batches_[LineType::kDashed] = LineBatch{};
  line_batches_[LineType::kDotted] = LineBatch{};
  
  // Initialize shape batches
  filled_shape_batch_ = ShapeBatch{};
  outline_shape_batches_[LineType::kSolid] = ShapeBatch{};
  outline_shape_batches_[LineType::kDashed] = ShapeBatch{};
  outline_shape_batches_[LineType::kDotted] = ShapeBatch{};
}

void Canvas::ClearBatches() {
  // Clean up line batches
  for (auto& [line_type, line_batch] : line_batches_) {
    if (line_batch.vao != 0) {
      glDeleteVertexArrays(1, &line_batch.vao);
      glDeleteBuffers(1, &line_batch.position_vbo);
      glDeleteBuffers(1, &line_batch.color_vbo);
      line_batch.vao = 0;
      line_batch.position_vbo = 0;
      line_batch.color_vbo = 0;
    }
    line_batch.vertices.clear();
    line_batch.colors.clear();
    line_batch.thicknesses.clear();
    line_batch.line_types.clear();
    line_batch.sequence_numbers.clear();
  }
  
  // Clean up filled shape batch
  if (filled_shape_batch_.vao != 0) {
    glDeleteVertexArrays(1, &filled_shape_batch_.vao);
    glDeleteBuffers(1, &filled_shape_batch_.vertex_vbo);
    glDeleteBuffers(1, &filled_shape_batch_.color_vbo);
    glDeleteBuffers(1, &filled_shape_batch_.ebo);
    filled_shape_batch_.vao = 0;
    filled_shape_batch_.vertex_vbo = 0;
    filled_shape_batch_.color_vbo = 0;
    filled_shape_batch_.ebo = 0;
  }
  filled_shape_batch_.vertices.clear();
  filled_shape_batch_.indices.clear();
  filled_shape_batch_.colors.clear();
  filled_shape_batch_.sequence_numbers.clear();
  
  // Clean up outline shape batches
  for (auto& [line_type, outline_batch] : outline_shape_batches_) {
    if (outline_batch.vao != 0) {
      glDeleteVertexArrays(1, &outline_batch.vao);
      glDeleteBuffers(1, &outline_batch.vertex_vbo);
      glDeleteBuffers(1, &outline_batch.color_vbo);
      glDeleteBuffers(1, &outline_batch.ebo);
      outline_batch.vao = 0;
      outline_batch.vertex_vbo = 0;
      outline_batch.color_vbo = 0;
      outline_batch.ebo = 0;
    }
    outline_batch.vertices.clear();
    outline_batch.indices.clear();
    outline_batch.colors.clear();
    outline_batch.sequence_numbers.clear();
  }
  
  // Clear the batch order tracker
  batch_order_tracker_.Clear();
}

void Canvas::UpdateBatches() {
  // The batching system is managed by the render strategies
  // This method primarily exists for API compatibility
  // Actual batching happens in BatchedRenderStrategy
  
  // Mark batches as needing update
  for (auto& [type, batch] : line_batches_) {
    batch.needs_update = true;
  }
  
  filled_shape_batch_.needs_update = true;
  
  for (auto& [type, batch] : outline_shape_batches_) {
    batch.needs_update = true;
  }
}

void Canvas::RenderBatches(const glm::mat4& projection, const glm::mat4& view,
                          const glm::mat4& coord_transform) {
  // Both batching and non-batching modes need to respect sequence order
  if (batching_enabled_) {
    // Batching mode: render batched primitives first, then individual primitives
    if (!batch_order_tracker_.render_order.empty()) {
      RenderBatchesInOrder(projection, view, coord_transform);
    }
    
    // Also render non-batched shapes (polygons, ellipses) that are stored in data_
    if (current_render_strategy_) {
      CanvasData data;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        data = *data_;
      }
      
      RenderContext context(projection, view, coord_transform,
                           &primitive_shader_, primitive_vao_, primitive_vbo_,
                           &render_stats_, &perf_config_);
      current_render_strategy_->Render(data, context);
    }
  } else if (current_render_strategy_) {
    // Non-batching mode: use individual rendering with sequence order
    CanvasData data;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      data = *data_;
    }
    
    RenderContext context(projection, view, coord_transform,
                         &primitive_shader_, primitive_vao_, primitive_vbo_,
                         &render_stats_, &perf_config_);
    current_render_strategy_->Render(data, context);
  }
}

void Canvas::RenderBatchesInOrder(const glm::mat4& projection, const glm::mat4& view,
                                  const glm::mat4& coord_transform) {
  // Sort primitives by sequence number to maintain draw order
  std::vector<BatchOrderTracker::OrderedPrimitive> sorted_order = batch_order_tracker_.render_order;
  std::sort(sorted_order.begin(), sorted_order.end(),
            [](const auto& a, const auto& b) { return a.sequence_number < b.sequence_number; });
  
  // Setup shader
  primitive_shader_.Use();
  primitive_shader_.TrySetUniform("projection", projection);
  primitive_shader_.TrySetUniform("view", view);
  primitive_shader_.TrySetUniform("model", glm::mat4(1.0f));
  primitive_shader_.TrySetUniform("coordSystemTransform", coord_transform);
  
  // Enable OpenGL states
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);  // Ensure later primitives draw on top
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  // Render each primitive in sequence order
  for (const auto& primitive : sorted_order) {
    switch (primitive.type) {
      case BatchOrderTracker::OrderedPrimitive::Type::kLine: {
        auto& line_batch = line_batches_[primitive.line_type];
        if (line_batch.vertices.empty()) continue;
        
        // Setup VAO/VBO if needed
        if (line_batch.vao == 0) {
          glGenVertexArrays(1, &line_batch.vao);
          glGenBuffers(1, &line_batch.position_vbo);
          glGenBuffers(1, &line_batch.color_vbo);
        }
        
        glBindVertexArray(line_batch.vao);
        
        // Upload vertex data
        glBindBuffer(GL_ARRAY_BUFFER, line_batch.position_vbo);
        glBufferData(GL_ARRAY_BUFFER, line_batch.vertices.size() * sizeof(glm::vec3),
                     line_batch.vertices.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);
        
        // Upload color data
        glBindBuffer(GL_ARRAY_BUFFER, line_batch.color_vbo);
        glBufferData(GL_ARRAY_BUFFER, line_batch.colors.size() * sizeof(glm::vec4),
                     line_batch.colors.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);
        
        // Set uniforms
        primitive_shader_.TrySetUniform("renderMode", 1); // Lines mode
        primitive_shader_.TrySetUniform("lineType", static_cast<int>(primitive.line_type));
        
        // Draw this specific line (2 vertices per line)
        uint32_t start_vertex = primitive.batch_index * 2;
        glLineWidth(line_batch.thicknesses[primitive.batch_index]);
        glDrawArrays(GL_LINES, start_vertex, 2);
        
        glBindVertexArray(0);
        break;
      }
      
      case BatchOrderTracker::OrderedPrimitive::Type::kFilledShape: {
        if (filled_shape_batch_.vertices.empty()) continue;
        
        // Setup VAO/VBO/EBO if needed
        if (filled_shape_batch_.vao == 0) {
          glGenVertexArrays(1, &filled_shape_batch_.vao);
          glGenBuffers(1, &filled_shape_batch_.vertex_vbo);
          glGenBuffers(1, &filled_shape_batch_.color_vbo);
          glGenBuffers(1, &filled_shape_batch_.ebo);
        }
        
        glBindVertexArray(filled_shape_batch_.vao);
        
        // Upload vertex data
        glBindBuffer(GL_ARRAY_BUFFER, filled_shape_batch_.vertex_vbo);
        glBufferData(GL_ARRAY_BUFFER, filled_shape_batch_.vertices.size() * sizeof(float),
                     filled_shape_batch_.vertices.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
        glEnableVertexAttribArray(0);
        
        // Upload color data
        glBindBuffer(GL_ARRAY_BUFFER, filled_shape_batch_.color_vbo);
        glBufferData(GL_ARRAY_BUFFER, filled_shape_batch_.colors.size() * sizeof(glm::vec4),
                     filled_shape_batch_.colors.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);
        
        // Upload index data
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, filled_shape_batch_.ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, filled_shape_batch_.indices.size() * sizeof(uint32_t),
                     filled_shape_batch_.indices.data(), GL_DYNAMIC_DRAW);
        
        // Set uniforms
        primitive_shader_.TrySetUniform("renderMode", 2); // Filled shapes mode
        
        // Draw only the specific shape using its index range
        if (primitive.batch_index < filled_shape_batch_.index_ranges.size()) {
          const auto& range = filled_shape_batch_.index_ranges[primitive.batch_index];
          glDrawElements(GL_TRIANGLES, range.count, GL_UNSIGNED_INT, 
                        (void*)(range.start * sizeof(uint32_t)));
        }
        
        glBindVertexArray(0);
        break;
      }
      
      case BatchOrderTracker::OrderedPrimitive::Type::kOutlineShape: {
        auto& outline_batch = outline_shape_batches_[primitive.line_type];
        if (outline_batch.vertices.empty()) continue;
        
        // Setup VAO/VBO/EBO if needed
        if (outline_batch.vao == 0) {
          glGenVertexArrays(1, &outline_batch.vao);
          glGenBuffers(1, &outline_batch.vertex_vbo);
          glGenBuffers(1, &outline_batch.color_vbo);
          glGenBuffers(1, &outline_batch.ebo);
        }
        
        glBindVertexArray(outline_batch.vao);
        
        // Upload vertex data
        glBindBuffer(GL_ARRAY_BUFFER, outline_batch.vertex_vbo);
        glBufferData(GL_ARRAY_BUFFER, outline_batch.vertices.size() * sizeof(float),
                     outline_batch.vertices.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
        glEnableVertexAttribArray(0);
        
        // Upload color data
        glBindBuffer(GL_ARRAY_BUFFER, outline_batch.color_vbo);
        glBufferData(GL_ARRAY_BUFFER, outline_batch.colors.size() * sizeof(glm::vec4),
                     outline_batch.colors.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);
        
        // Upload index data
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, outline_batch.ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, outline_batch.indices.size() * sizeof(uint32_t),
                     outline_batch.indices.data(), GL_DYNAMIC_DRAW);
        
        // Set uniforms
        primitive_shader_.TrySetUniform("renderMode", 3); // Outlined shapes mode
        primitive_shader_.TrySetUniform("lineType", static_cast<int>(primitive.line_type));
        
        // Draw only the specific shape using its index range
        if (primitive.batch_index < outline_batch.index_ranges.size()) {
          const auto& range = outline_batch.index_ranges[primitive.batch_index];
          glLineWidth(2.0f); // Default line width for outlines
          glDrawElements(GL_LINE_LOOP, range.count, GL_UNSIGNED_INT, 
                        (void*)(range.start * sizeof(uint32_t)));
        }
        
        glBindVertexArray(0);
        break;
      }
    }
  }
}

void Canvas::RenderIndividualShapes(const CanvasData& data,
                                   const glm::mat4& projection,
                                   const glm::mat4& view,
                                   const glm::mat4& coord_transform) {
  if (individual_strategy_) {
    RenderContext context(projection, view, coord_transform,
                         &primitive_shader_, primitive_vao_, primitive_vbo_,
                         &render_stats_, &perf_config_);
    individual_strategy_->Render(data, context);
  }
}

void Canvas::GenerateRectangleVertices(float x, float y, float width, float height,
                                       std::vector<float>& vertices,
                                       std::vector<uint32_t>& indices, bool filled,
                                       uint32_t base_index, uint32_t sequence_number) {
  // Use sequence number to determine Z depth for proper layering
  float z_depth = sequence_number * 0.001f;
  
  // Generate 4 corner vertices for rectangle
  float vertices_data[] = {
    x, y, z_depth,                    // Bottom-left
    x + width, y, z_depth,            // Bottom-right
    x + width, y + height, z_depth,   // Top-right
    x, y + height, z_depth            // Top-left
  };
  
  // Add vertices to the vector
  for (int i = 0; i < 12; i += 3) {
    vertices.push_back(vertices_data[i]);
    vertices.push_back(vertices_data[i + 1]);
    vertices.push_back(vertices_data[i + 2]);
  }
  
  if (filled) {
    // Two triangles for filled rectangle
    indices.insert(indices.end(), {
      base_index, base_index + 1, base_index + 2,  // Triangle 1
      base_index, base_index + 2, base_index + 3   // Triangle 2
    });
  } else {
    // Line loop for outline
    indices.insert(indices.end(), {
      base_index, base_index + 1,
      base_index + 1, base_index + 2,
      base_index + 2, base_index + 3,
      base_index + 3, base_index
    });
  }
}

void Canvas::GenerateCircleVertices(float cx, float cy, float radius, int segments,
                                    std::vector<float>& vertices,
                                    std::vector<uint32_t>& indices, bool filled,
                                    uint32_t base_index, uint32_t sequence_number) {
  // Use sequence number to determine Z depth for proper layering
  float z_depth = sequence_number * 0.001f;
  
  if (filled) {
    // Add center vertex
    vertices.insert(vertices.end(), {cx, cy, z_depth});
    
    // Add perimeter vertices
    for (int i = 0; i <= segments; ++i) {
      float angle = 2.0f * M_PI * i / segments;
      float x = cx + radius * std::cos(angle);
      float y = cy + radius * std::sin(angle);
      vertices.insert(vertices.end(), {x, y, z_depth});
    }
    
    // Generate triangle fan indices
    for (int i = 0; i < segments; ++i) {
      indices.insert(indices.end(), {
        base_index,           // Center
        base_index + 1 + i,   // Current perimeter vertex
        base_index + 1 + ((i + 1) % segments)  // Next perimeter vertex
      });
    }
  } else {
    // Add perimeter vertices only
    for (int i = 0; i <= segments; ++i) {
      float angle = 2.0f * M_PI * i / segments;
      float x = cx + radius * std::cos(angle);
      float y = cy + radius * std::sin(angle);
      vertices.insert(vertices.end(), {x, y, z_depth});
    }
    
    // Generate line loop indices
    for (int i = 0; i < segments; ++i) {
      indices.insert(indices.end(), {
        base_index + i,
        base_index + ((i + 1) % segments)
      });
    }
  }
}

void Canvas::GenerateEllipseVertices(float x, float y, float rx, float ry, float angle,
                                     float start_angle, float end_angle,
                                     std::vector<float>& vertices,
                                     std::vector<uint32_t>& indices, bool filled,
                                     uint32_t base_index) {
  const int segments = 32;
  float cos_rot = std::cos(angle);
  float sin_rot = std::sin(angle);
  
  if (filled) {
    // Add center vertex
    vertices.insert(vertices.end(), {x, y, 0.0f});
    
    // Add perimeter vertices
    for (int i = 0; i <= segments; ++i) {
      float t = start_angle + (end_angle - start_angle) * i / segments;
      float local_x = rx * std::cos(t);
      float local_y = ry * std::sin(t);
      
      // Apply rotation
      float rotated_x = local_x * cos_rot - local_y * sin_rot;
      float rotated_y = local_x * sin_rot + local_y * cos_rot;
      
      vertices.insert(vertices.end(), {x + rotated_x, y + rotated_y, 0.0f});
    }
    
    // Generate triangle fan indices
    for (int i = 0; i < segments; ++i) {
      indices.insert(indices.end(), {
        base_index,           // Center
        base_index + 1 + i,   // Current perimeter vertex
        base_index + 1 + ((i + 1) % segments)  // Next perimeter vertex
      });
    }
  } else {
    // Add perimeter vertices only
    for (int i = 0; i <= segments; ++i) {
      float t = start_angle + (end_angle - start_angle) * i / segments;
      float local_x = rx * std::cos(t);
      float local_y = ry * std::sin(t);
      
      // Apply rotation
      float rotated_x = local_x * cos_rot - local_y * sin_rot;
      float rotated_y = local_x * sin_rot + local_y * cos_rot;
      
      vertices.insert(vertices.end(), {x + rotated_x, y + rotated_y, 0.0f});
    }
    
    // Generate line strip indices
    for (int i = 0; i < segments; ++i) {
      indices.insert(indices.end(), {
        base_index + i,
        base_index + i + 1
      });
    }
  }
}

void Canvas::GeneratePolygonVertices(const std::vector<glm::vec2>& points,
                                     std::vector<float>& vertices,
                                     std::vector<uint32_t>& indices, bool filled,
                                     uint32_t base_index) {
  // Add vertices
  for (const auto& point : points) {
    vertices.insert(vertices.end(), {point.x, point.y, 0.0f});
  }
  
  if (filled) {
    // Simple triangulation (works for convex polygons)
    for (size_t i = 1; i < points.size() - 1; ++i) {
      indices.insert(indices.end(), {
        base_index,         // First vertex
        base_index + i,     // Current vertex
        base_index + i + 1  // Next vertex
      });
    }
  } else {
    // Line loop for outline
    for (size_t i = 0; i < points.size(); ++i) {
      indices.insert(indices.end(), {
        base_index + i,
        base_index + ((i + 1) % points.size())
      });
    }
  }
}

// Immediate rendering methods for non-batching mode
void Canvas::RenderPointImmediate(float x, float y, const glm::vec4& color, float thickness) {
  glm::mat4 projection, view, coord_transform;
  GetCurrentMatrices(projection, view, coord_transform);
  
  // Setup shader
  primitive_shader_.Use();
  primitive_shader_.TrySetUniform("projection", projection);
  primitive_shader_.TrySetUniform("view", view);
  primitive_shader_.TrySetUniform("model", glm::mat4(1.0f));
  primitive_shader_.TrySetUniform("coordSystemTransform", coord_transform);
  primitive_shader_.TrySetUniform("renderMode", 0); // Points mode
  
  // Enable OpenGL states
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  glBindVertexArray(primitive_vao_);
  glEnable(GL_PROGRAM_POINT_SIZE);
  
  Point point = {{x, y, 0.0f}, color, thickness};
  glBindBuffer(GL_ARRAY_BUFFER, primitive_vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(Point), &point, GL_DYNAMIC_DRAW);
  
  glPointSize(thickness);
  glDrawArrays(GL_POINTS, 0, 1);
  glPointSize(1.0f); // Reset
  
  glBindVertexArray(0);
}

void Canvas::RenderLineImmediate(float x1, float y1, float x2, float y2, const glm::vec4& color, 
                                float thickness, LineType line_type) {
  glm::mat4 projection, view, coord_transform;
  GetCurrentMatrices(projection, view, coord_transform);
  
  // Setup shader
  primitive_shader_.Use();
  primitive_shader_.TrySetUniform("projection", projection);
  primitive_shader_.TrySetUniform("view", view);
  primitive_shader_.TrySetUniform("model", glm::mat4(1.0f));
  primitive_shader_.TrySetUniform("coordSystemTransform", coord_transform);
  primitive_shader_.TrySetUniform("renderMode", 1); // Lines mode
  primitive_shader_.TrySetUniform("lineType", static_cast<int>(line_type));
  
  // Enable OpenGL states
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  glBindVertexArray(primitive_vao_);
  
  Point vertices[] = {
    {{x1, y1, 0.0f}, color, thickness},
    {{x2, y2, 0.0f}, color, thickness}
  };
  
  glBindBuffer(GL_ARRAY_BUFFER, primitive_vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
  
  glLineWidth(thickness);
  glDrawArrays(GL_LINES, 0, 2);
  glLineWidth(1.0f); // Reset
  
  glBindVertexArray(0);
}

void Canvas::RenderCircleImmediate(float x, float y, float radius, const glm::vec4& color,
                                  bool filled, float thickness, LineType line_type) {
  glm::mat4 projection, view, coord_transform;
  GetCurrentMatrices(projection, view, coord_transform);
  
  // Setup shader
  primitive_shader_.Use();
  primitive_shader_.TrySetUniform("projection", projection);
  primitive_shader_.TrySetUniform("view", view);
  primitive_shader_.TrySetUniform("model", glm::mat4(1.0f));
  primitive_shader_.TrySetUniform("coordSystemTransform", coord_transform);
  
  // Enable OpenGL states
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  glBindVertexArray(primitive_vao_);
  
  const int segments = 32;
  std::vector<Point> vertices;
  
  if (filled) {
    primitive_shader_.TrySetUniform("renderMode", 2); // Filled shapes
    // Add center point for triangle fan
    vertices.push_back({{x, y, 0.0f}, color, thickness});
  } else {
    primitive_shader_.TrySetUniform("renderMode", 3); // Outlined shapes
    primitive_shader_.TrySetUniform("lineType", static_cast<int>(line_type));
  }
  
  // Generate circle points
  for (int i = 0; i <= segments; i++) {
    float angle = 2.0f * M_PI * i / segments;
    float px = x + radius * std::cos(angle);
    float py = y + radius * std::sin(angle);
    vertices.push_back({{px, py, 0.0f}, color, thickness});
  }
  
  glBindBuffer(GL_ARRAY_BUFFER, primitive_vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Point), vertices.data(), GL_DYNAMIC_DRAW);
  
  if (filled) {
    glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.size());
  } else {
    glLineWidth(thickness);
    glDrawArrays(GL_LINE_LOOP, 1, segments); // Skip center point for outline
    glLineWidth(1.0f);
  }
  
  glBindVertexArray(0);
}

void Canvas::RenderRectangleImmediate(float x, float y, float width, float height,
                                     const glm::vec4& color, bool filled, float thickness, 
                                     LineType line_type) {
  glm::mat4 projection, view, coord_transform;
  GetCurrentMatrices(projection, view, coord_transform);
  
  // Setup shader
  primitive_shader_.Use();
  primitive_shader_.TrySetUniform("projection", projection);
  primitive_shader_.TrySetUniform("view", view);
  primitive_shader_.TrySetUniform("model", glm::mat4(1.0f));
  primitive_shader_.TrySetUniform("coordSystemTransform", coord_transform);
  
  // Enable OpenGL states
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  glBindVertexArray(primitive_vao_);
  
  // Generate rectangle vertices
  Point vertices[4] = {
    {{x, y, 0.0f}, color, thickness},                    // bottom-left
    {{x + width, y, 0.0f}, color, thickness},            // bottom-right  
    {{x + width, y + height, 0.0f}, color, thickness},   // top-right
    {{x, y + height, 0.0f}, color, thickness}            // top-left
  };
  
  glBindBuffer(GL_ARRAY_BUFFER, primitive_vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
  
  if (filled) {
    primitive_shader_.TrySetUniform("renderMode", 2); // Filled shapes
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
  } else {
    primitive_shader_.TrySetUniform("renderMode", 3); // Outlined shapes
    primitive_shader_.TrySetUniform("lineType", static_cast<int>(line_type));
    glLineWidth(thickness);
    glDrawArrays(GL_LINE_LOOP, 0, 4);
    glLineWidth(1.0f);
  }
  
  glBindVertexArray(0);
}

void Canvas::RenderEllipseImmediate(float x, float y, float rx, float ry, float angle,
                                   float start_angle, float end_angle, const glm::vec4& color,
                                   bool filled, float thickness, LineType line_type) {
  // For ellipses, use the shape renderer if available, or fall back to circle approximation
  if (shape_renderer_) {
    // Create ellipse data structure
    Ellipse ellipse;
    ellipse.center = glm::vec3(x, y, 0.0f);
    ellipse.rx = rx;
    ellipse.ry = ry; 
    ellipse.angle = angle;
    ellipse.start_angle = start_angle;
    ellipse.end_angle = end_angle;
    ellipse.color = color;
    ellipse.filled = filled;
    ellipse.thickness = thickness;
    ellipse.line_type = line_type;
    ellipse.num_segments = 32;
    
    auto vertices = ShapeGenerators::GenerateEllipseVertices(ellipse);
    VertexLayout layout = VertexLayout::PositionOnly();
    
    RenderParams params;
    params.color = color;
    params.thickness = thickness;
    params.filled = filled;
    params.primitive_type = filled ? GL_TRIANGLE_FAN : GL_LINE_LOOP;
    params.vertex_count = vertices.size() / 3;
    
    shape_renderer_->RenderShape(vertices, layout, params);
  } else {
    // Simple fallback: render as circle using average radius
    float avg_radius = (rx + ry) / 2.0f;
    RenderCircleImmediate(x, y, avg_radius, color, filled, thickness, line_type);
  }
}

void Canvas::RenderPolygonImmediate(const std::vector<glm::vec2>& points, const glm::vec4& color,
                                   bool filled, float thickness, LineType line_type) {
  if (shape_renderer_) {
    // Create polygon data structure
    Polygon polygon;
    polygon.vertices.reserve(points.size());
    for (const auto& p : points) {
      polygon.vertices.push_back(glm::vec3(p, 0.0f));
    }
    polygon.color = color;
    polygon.filled = filled;
    polygon.thickness = thickness;
    polygon.line_type = line_type;
    
    auto vertices = ShapeGenerators::GeneratePolygonVertices(polygon);
    VertexLayout layout = VertexLayout::PositionOnly();
    
    RenderParams params;
    params.color = color;
    params.thickness = thickness;
    params.filled = filled;
    params.primitive_type = filled ? GL_TRIANGLE_FAN : GL_LINE_LOOP;
    params.vertex_count = vertices.size() / 3;
    
    shape_renderer_->RenderShape(vertices, layout, params);
  } else {
    // Simple fallback: render as line loop
    glm::mat4 projection, view, coord_transform;
    GetCurrentMatrices(projection, view, coord_transform);
    
    primitive_shader_.Use();
    primitive_shader_.TrySetUniform("projection", projection);
    primitive_shader_.TrySetUniform("view", view);
    primitive_shader_.TrySetUniform("model", glm::mat4(1.0f));
    primitive_shader_.TrySetUniform("coordSystemTransform", coord_transform);
    primitive_shader_.TrySetUniform("renderMode", 3); // Outlined shapes
    primitive_shader_.TrySetUniform("lineType", static_cast<int>(line_type));
    
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glBindVertexArray(primitive_vao_);
    
    std::vector<Point> vertices;
    for (const auto& p : points) {
      vertices.push_back({{p.x, p.y, 0.0f}, color, thickness});
    }
    
    glBindBuffer(GL_ARRAY_BUFFER, primitive_vbo_);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Point), vertices.data(), GL_DYNAMIC_DRAW);
    
    glLineWidth(thickness);
    glDrawArrays(GL_LINE_LOOP, 0, vertices.size());
    glLineWidth(1.0f);
    
    glBindVertexArray(0);
  }
}

void Canvas::GetCurrentMatrices(glm::mat4& projection, glm::mat4& view, glm::mat4& coord_transform) {
  // For immediate rendering, we need to get the current matrices
  // This is a simplified implementation - in a real application, you'd get these from the current render context
  // For now, we'll use identity matrices as a fallback
  projection = glm::mat4(1.0f);
  view = glm::mat4(1.0f);
  coord_transform = glm::mat4(1.0f);
  
  // TODO: In a complete implementation, this should retrieve the current view/projection matrices
  // from the active rendering context or scene manager
}

}  // namespace quickviz
