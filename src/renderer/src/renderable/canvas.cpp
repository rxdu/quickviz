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

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "glad/glad.h"
#include <glm/gtc/type_ptr.hpp>

#include "renderable/details/canvas_data.hpp"

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
  
  // Ensure data is not null
  if (!data_) {
    std::cerr << "ERROR::CANVAS::FAILED_TO_ALLOCATE_CANVAS_DATA" << std::endl;
    throw std::runtime_error("Failed to allocate CanvasData");
  }
  
  // Initialize the data object (just to be extra safe)
  data_->Clear();
  
  // Re-enable batching now that ellipse/polygon renderMode is fixed
  batching_enabled_ = true;
  
  AllocateGpuResources();
  InitializeBatches();
}

Canvas::~Canvas() { 
  ClearBatches();
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
  PendingUpdate update;
  update.type = PendingUpdate::Type::kPoint;
  update.color = color;
  update.thickness = thickness;
  update.point.x = x;
  update.point.y = y;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(std::move(update));
  has_pending_updates_ = true;
}

void Canvas::AddLine(float x1, float y1, float x2, float y2,
                     const glm::vec4& color, float thickness,
                     LineType line_type) {
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
  pending_updates_.push(std::move(update));
  has_pending_updates_ = true;
}

void Canvas::AddRectangle(float x, float y, float width, float height,
                          const glm::vec4& color, bool filled, float thickness,
                          LineType line_type) {
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
  pending_updates_.push(std::move(update));
  has_pending_updates_ = true;
}

void Canvas::AddCircle(float x, float y, float radius, const glm::vec4& color,
                       bool filled, float thickness, LineType line_type) {
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
  pending_updates_.push(std::move(update));
  has_pending_updates_ = true;
}

void Canvas::AddEllipse(float x, float y, float rx, float ry, float angle,
                        float start_angle, float end_angle, const glm::vec4& color,
                        bool filled, float thickness, LineType line_type) {
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
  pending_updates_.push(std::move(update));
  has_pending_updates_ = true;
}

void Canvas::AddPolygon(const std::vector<glm::vec2>& points,
                        const glm::vec4& color, bool filled, float thickness,
                        LineType line_type) {
  PendingUpdate update;
  update.type = PendingUpdate::Type::kPolygon;
  update.color = color;
  update.thickness = thickness;
  update.line_type = line_type;
  update.filled = filled;
  update.polygon_vertices = points;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(std::move(update));
  has_pending_updates_ = true;
}

void Canvas::Clear() {
  PendingUpdate update;
  update.type = PendingUpdate::Type::kClear;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(std::move(update));
  has_pending_updates_ = true;
}

void Canvas::ProcessPendingUpdates() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  while (!pending_updates_.empty()) {
    const auto& update = pending_updates_.front();
    
    if (batching_enabled_) {
      // Use batching system for better performance
      switch (update.type) {
        case PendingUpdate::Type::kPoint:
          // Points still use the original system since they're already efficient
          data_->AddPoint(update.point.x, update.point.y, update.color, update.thickness);
          break;
          
        case PendingUpdate::Type::kLine:
          // Add line to batch
          line_batch_.vertices.emplace_back(update.line.x1, update.line.y1, 0.0f);
          line_batch_.vertices.emplace_back(update.line.x2, update.line.y2, 0.0f);
          line_batch_.colors.push_back(update.color);
          line_batch_.colors.push_back(update.color);
          line_batch_.thicknesses.push_back(update.thickness);
          line_batch_.line_types.push_back(update.line_type);
          line_batch_.needs_update = true;
          break;
          
        case PendingUpdate::Type::kRectangle: {
          uint32_t base_index = update.filled ? 
            filled_shape_batch_.vertices.size() / 3 : 
            outline_shape_batch_.vertices.size() / 3;
          
          if (update.filled) {
            GenerateRectangleVertices(update.rect.x, update.rect.y, 
                                    update.rect.width, update.rect.height,
                                    filled_shape_batch_.vertices, 
                                    filled_shape_batch_.indices,
                                    true, base_index);
            // Add color for each vertex (4 vertices for rectangle)
            for (int i = 0; i < 4; i++) {
              filled_shape_batch_.colors.push_back(update.color);
            }
            filled_shape_batch_.needs_update = true;
          } else {
            GenerateRectangleVertices(update.rect.x, update.rect.y,
                                    update.rect.width, update.rect.height,
                                    outline_shape_batch_.vertices,
                                    outline_shape_batch_.indices,
                                    false, base_index);
            // Add color for each vertex (4 vertices for rectangle)
            for (int i = 0; i < 4; i++) {
              outline_shape_batch_.colors.push_back(update.color);
            }
            outline_shape_batch_.needs_update = true;
          }
          break;
        }
        
        case PendingUpdate::Type::kCircle: {
          const int segments = 32; // Could be made adaptive based on radius
          uint32_t base_index = update.filled ?
            filled_shape_batch_.vertices.size() / 3 :
            outline_shape_batch_.vertices.size() / 3;
          
          if (update.filled) {
            GenerateCircleVertices(update.circle.x, update.circle.y, 
                                 update.circle.radius, segments,
                                 filled_shape_batch_.vertices,
                                 filled_shape_batch_.indices,
                                 true, base_index);
            // Add color for center + perimeter vertices
            for (int i = 0; i <= segments + 1; i++) {
              filled_shape_batch_.colors.push_back(update.color);
            }
            filled_shape_batch_.needs_update = true;
          } else {
            GenerateCircleVertices(update.circle.x, update.circle.y,
                                 update.circle.radius, segments,
                                 outline_shape_batch_.vertices,
                                 outline_shape_batch_.indices,
                                 false, base_index);
            // Add color for perimeter vertices
            for (int i = 0; i <= segments; i++) {
              outline_shape_batch_.colors.push_back(update.color);
            }
            outline_shape_batch_.needs_update = true;
          }
          break;
        }
        
        case PendingUpdate::Type::kEllipse:
          // For now, fall back to individual rendering for ellipses
          // TODO: Implement ellipse batching
          data_->AddEllipse(update.ellipse.x, update.ellipse.y, update.ellipse.rx,
                           update.ellipse.ry, update.ellipse.angle,
                           update.ellipse.start_angle, update.ellipse.end_angle,
                           update.color, update.filled, update.thickness,
                           update.line_type);
          break;
          
        case PendingUpdate::Type::kPolygon:
          // For now, fall back to individual rendering for polygons
          // TODO: Implement polygon batching
          data_->AddPolygon(update.polygon_vertices, update.color, update.filled,
                           update.thickness, update.line_type);
          break;
          
        case PendingUpdate::Type::kClear:
          // Clear both traditional data and batches
          data_->Clear();
          line_batch_.vertices.clear();
          line_batch_.colors.clear();
          line_batch_.thicknesses.clear();
          line_batch_.line_types.clear();
          filled_shape_batch_.vertices.clear();
          filled_shape_batch_.indices.clear();
          filled_shape_batch_.colors.clear();
          outline_shape_batch_.vertices.clear();
          outline_shape_batch_.indices.clear();
          outline_shape_batch_.colors.clear();
          line_batch_.needs_update = true;
          filled_shape_batch_.needs_update = true;
          outline_shape_batch_.needs_update = true;
          break;
      }
    } else {
      // Use original individual rendering system
      switch (update.type) {
        case PendingUpdate::Type::kPoint:
          data_->AddPoint(update.point.x, update.point.y, update.color, update.thickness);
          break;
          
        case PendingUpdate::Type::kLine:
          data_->AddLine(update.line.x1, update.line.y1, update.line.x2,
                        update.line.y2, update.color, update.thickness,
                        update.line_type);
          break;
          
        case PendingUpdate::Type::kRectangle:
          data_->AddRectangle(update.rect.x, update.rect.y, update.rect.width,
                            update.rect.height, update.color, update.filled,
                            update.thickness, update.line_type);
          break;
          
        case PendingUpdate::Type::kCircle:
          data_->AddCircle(update.circle.x, update.circle.y, update.circle.radius,
                          update.color, update.filled, update.thickness,
                          update.line_type);
          break;
          
        case PendingUpdate::Type::kEllipse:
          data_->AddEllipse(update.ellipse.x, update.ellipse.y, update.ellipse.rx,
                           update.ellipse.ry, update.ellipse.angle,
                           update.ellipse.start_angle, update.ellipse.end_angle,
                           update.color, update.filled, update.thickness,
                           update.line_type);
          break;
          
        case PendingUpdate::Type::kPolygon:
          data_->AddPolygon(update.polygon_vertices, update.color, update.filled,
                           update.thickness, update.line_type);
          break;
          
        case PendingUpdate::Type::kClear:
          data_->Clear();
          break;
      }
    }
    
    pending_updates_.pop();
  }
  has_pending_updates_ = false;
}

void Canvas::AllocateGpuResources() {
  // First make sure any existing resources are released
  ReleaseGpuResources();

  try {
    std::cout << "Compiling primitive vertex shader..." << std::endl;
    Shader vertex_shader(vertex_shader_source.c_str(), Shader::Type::kVertex);
    if (!vertex_shader.Compile()) {
      std::cerr << "ERROR::CANVAS::VERTEX_SHADER_COMPILATION_FAILED" << std::endl;
      throw std::runtime_error("Vertex shader compilation failed");
    }
    
    std::cout << "Compiling primitive fragment shader..." << std::endl;
    Shader fragment_shader(fragment_shader_source.c_str(), Shader::Type::kFragment);
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

    // Create and set up VAO and VBO
    glGenVertexArrays(1, &primitive_vao_);
    if (primitive_vao_ == 0) {
      std::cerr << "ERROR::CANVAS::VAO_GENERATION_FAILED" << std::endl;
      throw std::runtime_error("VAO generation failed");
    }
    
    glGenBuffers(1, &primitive_vbo_);
    if (primitive_vbo_ == 0) {
      std::cerr << "ERROR::CANVAS::VBO_GENERATION_FAILED" << std::endl;
      throw std::runtime_error("VBO generation failed");
    }

    glBindVertexArray(primitive_vao_);

    // Set up VBO for points - initialize with a small amount of memory
    glBindBuffer(GL_ARRAY_BUFFER, primitive_vbo_);
    
    // Allocate a small buffer initially (for 10 points) to prevent segmentation faults
    // when drawing before data is added
    const size_t initial_buffer_size = 10 * sizeof(Point);
    glBufferData(GL_ARRAY_BUFFER, initial_buffer_size, nullptr, GL_DYNAMIC_DRAW);
    
    // Check for errors
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
      std::cerr << "OpenGL error during VBO allocation: " << error << std::endl;
      throw std::runtime_error("VBO allocation failed");
    }

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
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    std::cout << "GPU resources allocated successfully." << std::endl;
  }
  catch (const std::exception& e) {
    std::cerr << "ERROR: Failed to allocate GPU resources: " << e.what() << std::endl;
    ReleaseGpuResources(); // Clean up any resources that were created
    throw; // Rethrow the exception
  }
}

void Canvas::ReleaseGpuResources() noexcept{
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
  if (has_pending_updates_) {
    ProcessPendingUpdates();
  }

  // Draw background if available
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

  // Get a copy of the data to avoid locking during rendering
  CanvasData data;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data = *data_;
  }

  // Skip if there's no data to render
  if (data.points.empty() && data.lines.empty() && 
      data.rectangles.empty() && data.circles.empty() &&
      data.ellipses.empty() && data.polygons.empty()) {
    return;
  }

  // Choose rendering path based on batching setting
  if (batching_enabled_) {
    // Use efficient batched rendering for better performance
    RenderBatches(projection, view, coord_transform);
    
    // Still render points individually (they're already efficient)
    if (!data.points.empty()) {
      // Setup for point rendering
      primitive_shader_.Use();
      primitive_shader_.TrySetUniform("projection", projection);
      primitive_shader_.TrySetUniform("view", view);
      primitive_shader_.TrySetUniform("model", glm::mat4(1.0f));
      primitive_shader_.TrySetUniform("coordSystemTransform", coord_transform);
      primitive_shader_.TrySetUniform("renderMode", 0); // Points mode
      
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      
      glBindVertexArray(primitive_vao_);
      glEnable(GL_PROGRAM_POINT_SIZE);
      
      // Update buffer with point data
      glBindBuffer(GL_ARRAY_BUFFER, primitive_vbo_);
      glBufferData(GL_ARRAY_BUFFER, sizeof(Point) * data.points.size(), 
                   data.points.data(), GL_DYNAMIC_DRAW);
      
      // Enable point attributes
      glEnableVertexAttribArray(0); // Position
      glEnableVertexAttribArray(1); // Color
      glEnableVertexAttribArray(2); // Size
      
      // Draw points
      glDrawArrays(GL_POINTS, 0, data.points.size());
      
      // Cleanup point rendering
      glDisable(GL_PROGRAM_POINT_SIZE);
      glDisableVertexAttribArray(0);
      glDisableVertexAttribArray(1);
      glDisableVertexAttribArray(2);
      glBindVertexArray(0);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      
      // Update stats
      render_stats_.points_rendered = data.points.size();
      render_stats_.draw_calls++;
    }
    
    // Handle non-batched shapes (ellipses, polygons) with individual rendering
    if (!data.ellipses.empty() || !data.polygons.empty()) {
      RenderIndividualShapes(data, projection, view, coord_transform);
    }
    
    return; // Early return to skip the old rendering path
  }

  // Original individual rendering system (fallback when batching is disabled)
  // Setup common rendering state for primitives
  primitive_shader_.Use();
  primitive_shader_.TrySetUniform("projection", projection);
  primitive_shader_.TrySetUniform("view", view);
  primitive_shader_.TrySetUniform("model", glm::mat4(1.0f));
  primitive_shader_.TrySetUniform("coordSystemTransform", coord_transform);
  primitive_shader_.TrySetUniform("lineType", 0); // Default to solid line
  primitive_shader_.TrySetUniform("thickness", 1.0f); // Default thickness
  primitive_shader_.TrySetUniform("uColor", glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)); // Default to white

  // Enable depth test and blending
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // 1. Draw Points
  if (!data.points.empty()) {
    glBindVertexArray(primitive_vao_);
    glEnable(GL_PROGRAM_POINT_SIZE);
    
    // Set drawing mode to points
    primitive_shader_.TrySetUniform("renderMode", 0);
    
    // Update the buffer with current point data
    glBindBuffer(GL_ARRAY_BUFFER, primitive_vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Point) * data.points.size(), data.points.data(), GL_DYNAMIC_DRAW);
    
    // For points, we use the per-vertex color from the buffer
    // This color is passed to the shader via the 'aColor' attribute and becomes 'fragColor'
    
    // Make sure all attributes are correctly enabled for points
    glEnableVertexAttribArray(0); // Position
    glEnableVertexAttribArray(1); // Color
    glEnableVertexAttribArray(2); // Size
    
    // Draw the points
    glDrawArrays(GL_POINTS, 0, data.points.size());
    
    glDisable(GL_PROGRAM_POINT_SIZE);
    
    // Clean up point rendering state
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }

  // Set drawing mode to lines/shapes for all subsequent primitives
  primitive_shader_.TrySetUniform("renderMode", 1);

  // 2. Draw Lines - Use modern OpenGL approach
  if (!data.lines.empty()) {
    for (const auto& line : data.lines) {
      // Set line width
      glLineWidth(line.thickness);
      
      // Create temporary VBO/VAO for the line
      GLuint tempVAO, tempVBO;
      glGenVertexArrays(1, &tempVAO);
      glGenBuffers(1, &tempVBO);
      
      glBindVertexArray(tempVAO);
      glBindBuffer(GL_ARRAY_BUFFER, tempVBO);
      
      // Line vertices
      float vertices[] = {
          line.start.x, line.start.y, line.start.z,
          line.end.x, line.end.y, line.end.z
      };
      
      glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
      
      // Position attribute
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
      glEnableVertexAttribArray(0);
      
      // Make sure other attributes are disabled
      glDisableVertexAttribArray(1);
      glDisableVertexAttribArray(2);
      
      // Set line color in the shader
      primitive_shader_.TrySetUniform("uColor", line.color);
      
      // Handle different line types with shader-based approach
      primitive_shader_.TrySetUniform("lineType", static_cast<int>(line.line_type));
      primitive_shader_.TrySetUniform("thickness", line.thickness);
      
      // Draw the line
      glDrawArrays(GL_LINES, 0, 2);
      
      // Clean up
      glDisableVertexAttribArray(0);
      glBindVertexArray(0);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      glDeleteVertexArrays(1, &tempVAO);
      glDeleteBuffers(1, &tempVBO);
    }
    
    // Reset line width
    glLineWidth(1.0f);
  }

  // 3. Draw Rectangles - Use modern OpenGL approach
  if (!data.rectangles.empty()) {
    for (const auto& rect : data.rectangles) {
      // Create temporary VBO/VAO for the rectangle
      GLuint tempVAO, tempVBO;
      glGenVertexArrays(1, &tempVAO);
      glGenBuffers(1, &tempVBO);
      
      glBindVertexArray(tempVAO);
      glBindBuffer(GL_ARRAY_BUFFER, tempVBO);
      
      // Rectangle vertices (2 triangles making a quad)
      float x = rect.position.x;
      float y = rect.position.y;
      float z = rect.position.z;
      float w = rect.width;
      float h = rect.height;
      
      float vertices[] = {
          x, y, z,           // Bottom left
          x + w, y, z,       // Bottom right
          x + w, y + h, z,   // Top right
          x, y + h, z        // Top left
      };
      
      glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
      
      // Position attribute
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
      glEnableVertexAttribArray(0);
      
      // Make sure other attributes are disabled
      glDisableVertexAttribArray(1);
      glDisableVertexAttribArray(2);
      
      // Set rectangle color
      primitive_shader_.TrySetUniform("uColor", rect.color);
      
      if (rect.filled) {
        // Draw filled rectangle
        primitive_shader_.TrySetUniform("lineType", 0); // Solid fill
        glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
      } else {
        // Draw outline
        glLineWidth(rect.thickness);
        
        // Handle different line types with shader-based approach
        primitive_shader_.TrySetUniform("lineType", static_cast<int>(rect.line_type));
        primitive_shader_.TrySetUniform("thickness", rect.thickness);
        
        // Draw rectangle outline
        glDrawArrays(GL_LINE_LOOP, 0, 4);
        
        glLineWidth(1.0f);
      }
      
      // Clean up
      glDisableVertexAttribArray(0);
      glBindVertexArray(0);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      glDeleteVertexArrays(1, &tempVAO);
      glDeleteBuffers(1, &tempVBO);
    }
  }

  // 4. Draw Circles - Use modern OpenGL approach
  if (!data.circles.empty()) {
    for (const auto& circle : data.circles) {
      // Create temporary VBO/VAO for the circle
      GLuint tempVAO, tempVBO;
      glGenVertexArrays(1, &tempVAO);
      glGenBuffers(1, &tempVBO);
      
      glBindVertexArray(tempVAO);
      glBindBuffer(GL_ARRAY_BUFFER, tempVBO);
      
      // Generate circle vertices
      const int segments = circle.num_segments;
      std::vector<float> vertices;
      vertices.reserve((segments + 2) * 3); // +2 for center and closing point
      
      // Center point (for filled circles)
      if (circle.filled) {
        vertices.push_back(circle.center.x);
        vertices.push_back(circle.center.y);
        vertices.push_back(circle.center.z);
      }
      
      // Circle perimeter points
      for (int i = 0; i <= segments; i++) {
        float angle = 2.0f * M_PI * i / segments;
        float x = circle.center.x + circle.radius * std::cos(angle);
        float y = circle.center.y + circle.radius * std::sin(angle);
        float z = circle.center.z;
        
        vertices.push_back(x);
        vertices.push_back(y);
        vertices.push_back(z);
      }
      
      glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
      
      // Position attribute
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
      glEnableVertexAttribArray(0);
      
      // Make sure other attributes are disabled
      glDisableVertexAttribArray(1);
      glDisableVertexAttribArray(2);
      
      // Set circle color
      primitive_shader_.TrySetUniform("uColor", circle.color);
      
      if (circle.filled) {
        // Draw filled circle
        primitive_shader_.TrySetUniform("lineType", 0); // Solid fill
        glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.size() / 3);
      } else {
        // Draw outline
        glLineWidth(circle.thickness);
        
        // Handle different line types with shader-based approach
        primitive_shader_.TrySetUniform("lineType", static_cast<int>(circle.line_type));
        primitive_shader_.TrySetUniform("thickness", circle.thickness);
        
        // Draw circle outline
        glDrawArrays(GL_LINE_STRIP, circle.filled ? 1 : 0, segments + 1);
        
        glLineWidth(1.0f);
      }
      
      // Clean up
      glDisableVertexAttribArray(0);
      glBindVertexArray(0);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      glDeleteVertexArrays(1, &tempVAO);
      glDeleteBuffers(1, &tempVBO);
    }
  }

  // 5. Draw Ellipses - Use modern OpenGL approach
  if (!data.ellipses.empty()) {
    for (const auto& ellipse : data.ellipses) {
      // Create temporary VBO/VAO for the ellipse
      GLuint tempVAO, tempVBO;
      glGenVertexArrays(1, &tempVAO);
      glGenBuffers(1, &tempVBO);
      
      glBindVertexArray(tempVAO);
      glBindBuffer(GL_ARRAY_BUFFER, tempVBO);
      
      // Generate ellipse vertices
      const int segments = ellipse.num_segments;
      std::vector<float> vertices;
      vertices.reserve((segments + 2) * 3); // +2 for center and closing point
      
      // Center point (for filled ellipses)
      if (ellipse.filled) {
        vertices.insert(vertices.end(), {ellipse.center.x, ellipse.center.y, ellipse.center.z});
      }
      
      for (int i = 0; i <= segments; i++) {
        float t = ellipse.start_angle + (ellipse.end_angle - ellipse.start_angle) * i / segments;
        float x_local = ellipse.rx * std::cos(t);
        float y_local = ellipse.ry * std::sin(t);
        float x_rotated = x_local * std::cos(ellipse.angle) - y_local * std::sin(ellipse.angle);
        float y_rotated = x_local * std::sin(ellipse.angle) + y_local * std::cos(ellipse.angle);
        float x = ellipse.center.x + x_rotated;
        float y = ellipse.center.y + y_rotated;
        float z = ellipse.center.z;
        vertices.insert(vertices.end(), {x, y, z});
      }
      
      glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
      
      // Set up vertex attributes correctly
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
      glEnableVertexAttribArray(0);
      
      // Explicitly set default values for disabled attributes to ensure proper shader behavior
      glDisableVertexAttribArray(1);  // aColor - will default to (0,0,0,0)
      glDisableVertexAttribArray(2);  // aSize - will default to 0
      
      // Set explicit default values for vertex attributes that aren't used
      glVertexAttrib4f(1, 0.0f, 0.0f, 0.0f, 0.0f);  // Ensure fragColor.a = 0 to trigger uniform fallback
      glVertexAttrib1f(2, 1.0f);  // Set default size
      
      primitive_shader_.TrySetUniform("uColor", ellipse.color);
      
      if (ellipse.filled) {
        primitive_shader_.TrySetUniform("renderMode", 2); // Filled shapes mode
        primitive_shader_.TrySetUniform("lineType", 0); // Solid fill
        glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.size() / 3);
      } else {
        primitive_shader_.TrySetUniform("renderMode", 3); // Outline shapes mode
        primitive_shader_.TrySetUniform("lineType", static_cast<int>(ellipse.line_type));
        glLineWidth(ellipse.thickness);
        glDrawArrays(GL_LINE_STRIP, ellipse.filled ? 1 : 0, segments + 1);
        glLineWidth(1.0f);
      }
      
      glDisableVertexAttribArray(0);
      glBindVertexArray(0);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      glDeleteVertexArrays(1, &tempVAO);
      glDeleteBuffers(1, &tempVBO);
      
      render_stats_.shapes_rendered++;
      render_stats_.draw_calls++;
    }
  }

  // 6. Draw Polygons - Use modern OpenGL approach
  if (!data.polygons.empty()) {
    for (const auto& polygon : data.polygons) {
      // Create temporary VBO/VAO for the polygon
      GLuint tempVAO, tempVBO;
      glGenVertexArrays(1, &tempVAO);
      glGenBuffers(1, &tempVBO);
      
      glBindVertexArray(tempVAO);
      glBindBuffer(GL_ARRAY_BUFFER, tempVBO);
      
      // Convert vertices to flat array
      std::vector<float> vertices;
      vertices.reserve(polygon.vertices.size() * 3);
      
      for (const auto& vertex : polygon.vertices) {
        vertices.push_back(vertex.x);
        vertices.push_back(vertex.y);
        vertices.push_back(vertex.z);
      }
      
      glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
      
      // Position attribute
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
      glEnableVertexAttribArray(0);
      
      // Make sure other attributes are disabled
      glDisableVertexAttribArray(1);
      glDisableVertexAttribArray(2);
      
      // Set polygon color using the uniform (not per-vertex colors)
      primitive_shader_.TrySetUniform("uColor", polygon.color);
      
      if (polygon.filled) {
        // Draw filled polygon
        primitive_shader_.TrySetUniform("renderMode", 2); // Filled shapes mode
        primitive_shader_.TrySetUniform("lineType", 0); // Solid fill
        glDrawArrays(GL_TRIANGLE_FAN, 0, polygon.vertices.size());
      } else {
        // Draw outline
        primitive_shader_.TrySetUniform("renderMode", 3); // Outline shapes mode
        primitive_shader_.TrySetUniform("lineType", static_cast<int>(polygon.line_type));
        glLineWidth(polygon.thickness);
        glDrawArrays(GL_LINE_LOOP, 0, polygon.vertices.size());
        glLineWidth(1.0f);
      }
      
      // Clean up
      glDisableVertexAttribArray(0);
      glBindVertexArray(0);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      glDeleteVertexArrays(1, &tempVAO);
      glDeleteBuffers(1, &tempVBO);
      
      render_stats_.shapes_rendered++;
      render_stats_.draw_calls++;
    }
  }

  // Reset OpenGL state
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glBindVertexArray(0);
  glUseProgram(0);
}

void Canvas::RenderBatches(const glm::mat4& projection, const glm::mat4& view,
                          const glm::mat4& coord_transform) {
  if (perf_config_.detailed_timing_enabled) {
    frame_timer_.Start();
  }
  
  render_stats_.Reset();
  
  // Track memory usage
  if (perf_config_.memory_tracking_enabled) {
    render_stats_.vertex_memory_used = 
      line_batch_.vertices.size() * sizeof(glm::vec3) +
      filled_shape_batch_.vertices.size() * sizeof(float) +
      outline_shape_batch_.vertices.size() * sizeof(float);
    
    render_stats_.index_memory_used = 
      filled_shape_batch_.indices.size() * sizeof(uint32_t) +
      outline_shape_batch_.indices.size() * sizeof(uint32_t);
    
    render_stats_.total_memory_used = 
      render_stats_.vertex_memory_used + 
      render_stats_.index_memory_used +
      memory_tracker_.current_usage.load();
  }
  
  // Update batches if needed
  UpdateBatches();
  
  // Setup common rendering state
  primitive_shader_.Use();
  primitive_shader_.TrySetUniform("projection", projection);
  primitive_shader_.TrySetUniform("view", view);  
  primitive_shader_.TrySetUniform("model", glm::mat4(1.0f));
  primitive_shader_.TrySetUniform("coordSystemTransform", coord_transform);
  
  // Enable depth test and blending
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  render_stats_.state_changes += 3; // Track state changes
  
  // Render lines batch
  if (!line_batch_.vertices.empty()) {
    primitive_shader_.TrySetUniform("renderMode", 1); // Lines mode
    primitive_shader_.TrySetUniform("lineType", 0);   // Solid lines for now
    
    glBindVertexArray(line_batch_.vao);
    glDrawArrays(GL_LINES, 0, line_batch_.vertices.size());
    glBindVertexArray(0);
    
    render_stats_.lines_rendered = line_batch_.vertices.size() / 2;
    render_stats_.batched_objects += line_batch_.vertices.size() / 2;
    render_stats_.draw_calls++;
    render_stats_.state_changes += 2; // VAO bind/unbind
  }
  
  // Render filled shapes batch
  if (!filled_shape_batch_.vertices.empty() && !filled_shape_batch_.indices.empty()) {
    primitive_shader_.TrySetUniform("renderMode", 2); // Filled shapes mode
    
    glBindVertexArray(filled_shape_batch_.vao);
    glDrawElements(GL_TRIANGLES, filled_shape_batch_.indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
    
    render_stats_.shapes_rendered += filled_shape_batch_.indices.size() / 3;
    render_stats_.batched_objects += filled_shape_batch_.indices.size() / 3;
    render_stats_.draw_calls++;
    render_stats_.state_changes += 2; // VAO bind/unbind
  }
  
  // Render outline shapes batch
  if (!outline_shape_batch_.vertices.empty() && !outline_shape_batch_.indices.empty()) {
    primitive_shader_.TrySetUniform("renderMode", 3); // Outline shapes mode
    primitive_shader_.TrySetUniform("lineType", 0);   // Solid lines for now
    
    glBindVertexArray(outline_shape_batch_.vao);
    glDrawElements(GL_LINES, outline_shape_batch_.indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
    
    render_stats_.shapes_rendered += outline_shape_batch_.indices.size() / 8; // Approximate shapes
    render_stats_.batched_objects += outline_shape_batch_.indices.size() / 8;
    render_stats_.draw_calls++;
    render_stats_.state_changes += 2; // VAO bind/unbind
  }
  
  // Reset OpenGL state
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glBindVertexArray(0);
  glUseProgram(0);
  render_stats_.state_changes += 4; // State reset
  
  // Update performance statistics
  if (perf_config_.detailed_timing_enabled) {
    float frame_time = frame_timer_.ElapsedMs();
    render_stats_.UpdateFrameStats(frame_time);
  }
}

void Canvas::RenderIndividualShapes(const CanvasData& data, const glm::mat4& projection,
                                   const glm::mat4& view, const glm::mat4& coord_transform) {
  if (data.ellipses.empty() && data.polygons.empty()) {
    return;
  }
  
  if (perf_config_.detailed_timing_enabled) {
    operation_timer_.Start();
  }
  
  // Setup rendering state - make sure we have the right shader program active
  primitive_shader_.Use();
  primitive_shader_.TrySetUniform("projection", projection);
  primitive_shader_.TrySetUniform("view", view);
  primitive_shader_.TrySetUniform("model", glm::mat4(1.0f));
  primitive_shader_.TrySetUniform("coordSystemTransform", coord_transform);
  
  // Enable necessary OpenGL states
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  render_stats_.state_changes += 3;
  
  // Render ellipses individually (not yet batched)
  for (const auto& ellipse : data.ellipses) {
    GLuint tempVAO, tempVBO;
    glGenVertexArrays(1, &tempVAO);
    glGenBuffers(1, &tempVBO);
    
    glBindVertexArray(tempVAO);
    glBindBuffer(GL_ARRAY_BUFFER, tempVBO);
    
    // Generate ellipse vertices
    const int segments = ellipse.num_segments;
    std::vector<float> vertices;
    vertices.reserve((segments + 2) * 3);
    
    if (ellipse.filled) {
      vertices.insert(vertices.end(), {ellipse.center.x, ellipse.center.y, ellipse.center.z});
    }
    
    for (int i = 0; i <= segments; i++) {
      float t = ellipse.start_angle + (ellipse.end_angle - ellipse.start_angle) * i / segments;
      float x_local = ellipse.rx * std::cos(t);
      float y_local = ellipse.ry * std::sin(t);
      float x_rotated = x_local * std::cos(ellipse.angle) - y_local * std::sin(ellipse.angle);
      float y_rotated = x_local * std::sin(ellipse.angle) + y_local * std::cos(ellipse.angle);
      float x = ellipse.center.x + x_rotated;
      float y = ellipse.center.y + y_rotated;
      float z = ellipse.center.z;
      vertices.insert(vertices.end(), {x, y, z});
    }
    
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    
    // Set up vertex attributes correctly
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    // Explicitly set default values for disabled attributes to ensure proper shader behavior
    glDisableVertexAttribArray(1);  // aColor - will default to (0,0,0,0)
    glDisableVertexAttribArray(2);  // aSize - will default to 0
    
    // Set explicit default values for vertex attributes that aren't used
    glVertexAttrib4f(1, 0.0f, 0.0f, 0.0f, 0.0f);  // Ensure fragColor.a = 0 to trigger uniform fallback
    glVertexAttrib1f(2, 1.0f);  // Set default size
    
    primitive_shader_.TrySetUniform("uColor", ellipse.color);
    
    if (ellipse.filled) {
      primitive_shader_.TrySetUniform("renderMode", 2); // Filled shapes mode
      primitive_shader_.TrySetUniform("lineType", 0); // Solid fill
      glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.size() / 3);
    } else {
      primitive_shader_.TrySetUniform("renderMode", 3); // Outline shapes mode
      primitive_shader_.TrySetUniform("lineType", static_cast<int>(ellipse.line_type));
      glLineWidth(ellipse.thickness);
      glDrawArrays(GL_LINE_STRIP, ellipse.filled ? 1 : 0, segments + 1);
      glLineWidth(1.0f);
    }
    
    glDisableVertexAttribArray(0);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDeleteVertexArrays(1, &tempVAO);
    glDeleteBuffers(1, &tempVBO);
    
    render_stats_.shapes_rendered++;
    render_stats_.draw_calls++;
  }
  
  // Render polygons individually (not yet batched)
  for (const auto& polygon : data.polygons) {
    GLuint tempVAO, tempVBO;
    glGenVertexArrays(1, &tempVAO);
    glGenBuffers(1, &tempVBO);
    
    glBindVertexArray(tempVAO);
    glBindBuffer(GL_ARRAY_BUFFER, tempVBO);
    
    std::vector<float> vertices;
    vertices.reserve(polygon.vertices.size() * 3);
    for (size_t i = 0; i < polygon.vertices.size(); ++i) {
      const auto& vertex = polygon.vertices[i];
      vertices.insert(vertices.end(), {vertex.x, vertex.y, vertex.z});
    }
    
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    
    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    // Make sure other attributes are disabled
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    
    // Set polygon color using the uniform (not per-vertex colors)
    primitive_shader_.TrySetUniform("uColor", polygon.color);
    
    if (polygon.filled) {
      // Draw filled polygon
      primitive_shader_.TrySetUniform("renderMode", 2); // Filled shapes mode
      primitive_shader_.TrySetUniform("lineType", 0); // Solid fill
      glDrawArrays(GL_TRIANGLE_FAN, 0, polygon.vertices.size());
    } else {
      // Draw outline
      primitive_shader_.TrySetUniform("renderMode", 3); // Outline shapes mode
      primitive_shader_.TrySetUniform("lineType", static_cast<int>(polygon.line_type));
      glLineWidth(polygon.thickness);
      glDrawArrays(GL_LINE_LOOP, 0, polygon.vertices.size());
      glLineWidth(1.0f);
    }
    
    // Clean up
    glDisableVertexAttribArray(0);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDeleteVertexArrays(1, &tempVAO);
    glDeleteBuffers(1, &tempVBO);
    
    render_stats_.shapes_rendered++;
    render_stats_.draw_calls++;
  }
  
  // Clean up OpenGL state
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glBindVertexArray(0);
  glUseProgram(0);
  
  if (perf_config_.detailed_timing_enabled) {
    float operation_time = operation_timer_.ElapsedMs();
    render_stats_.UpdateOperationStats(operation_time);
  }
}

void Canvas::InitializeBatches() {
  // Initialize line batch
  glGenVertexArrays(1, &line_batch_.vao);
  glGenBuffers(1, &line_batch_.position_vbo);
  glGenBuffers(1, &line_batch_.color_vbo);
  
  glBindVertexArray(line_batch_.vao);
  
  // Position buffer
  glBindBuffer(GL_ARRAY_BUFFER, line_batch_.position_vbo);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(0);
  
  // Color buffer  
  glBindBuffer(GL_ARRAY_BUFFER, line_batch_.color_vbo);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(1);
  
  glBindVertexArray(0);
  
  // Initialize filled shape batch
  glGenVertexArrays(1, &filled_shape_batch_.vao);
  glGenBuffers(1, &filled_shape_batch_.vertex_vbo);
  glGenBuffers(1, &filled_shape_batch_.color_vbo);
  glGenBuffers(1, &filled_shape_batch_.ebo);
  
  glBindVertexArray(filled_shape_batch_.vao);
  
  // Vertex buffer
  glBindBuffer(GL_ARRAY_BUFFER, filled_shape_batch_.vertex_vbo);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(0);
  
  // Color buffer
  glBindBuffer(GL_ARRAY_BUFFER, filled_shape_batch_.color_vbo);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(1);
  
  // Element buffer
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, filled_shape_batch_.ebo);
  
  glBindVertexArray(0);
  
  // Initialize outline shape batch
  glGenVertexArrays(1, &outline_shape_batch_.vao);
  glGenBuffers(1, &outline_shape_batch_.vertex_vbo);
  glGenBuffers(1, &outline_shape_batch_.color_vbo);
  glGenBuffers(1, &outline_shape_batch_.ebo);
  
  glBindVertexArray(outline_shape_batch_.vao);
  
  // Vertex buffer
  glBindBuffer(GL_ARRAY_BUFFER, outline_shape_batch_.vertex_vbo);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(0);
  
  // Color buffer
  glBindBuffer(GL_ARRAY_BUFFER, outline_shape_batch_.color_vbo);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(1);
  
  // Element buffer
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, outline_shape_batch_.ebo);
  
  glBindVertexArray(0);
  
  std::cout << "Batching system initialized successfully." << std::endl;
}

void Canvas::ClearBatches() {
  // Clean up line batch
  if (line_batch_.vao != 0) {
    glDeleteVertexArrays(1, &line_batch_.vao);
    glDeleteBuffers(1, &line_batch_.position_vbo);
    glDeleteBuffers(1, &line_batch_.color_vbo);
    line_batch_.vao = 0;
    line_batch_.position_vbo = 0;
    line_batch_.color_vbo = 0;
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
  
  // Clean up outline shape batch
  if (outline_shape_batch_.vao != 0) {
    glDeleteVertexArrays(1, &outline_shape_batch_.vao);
    glDeleteBuffers(1, &outline_shape_batch_.vertex_vbo);
    glDeleteBuffers(1, &outline_shape_batch_.color_vbo);
    glDeleteBuffers(1, &outline_shape_batch_.ebo);
    outline_shape_batch_.vao = 0;
    outline_shape_batch_.vertex_vbo = 0;
    outline_shape_batch_.color_vbo = 0;
    outline_shape_batch_.ebo = 0;
  }
  
  // Clear CPU data
  line_batch_.vertices.clear();
  line_batch_.colors.clear();
  line_batch_.thicknesses.clear();
  line_batch_.line_types.clear();
  
  filled_shape_batch_.vertices.clear();
  filled_shape_batch_.indices.clear();
  filled_shape_batch_.colors.clear();
  
  outline_shape_batch_.vertices.clear();
  outline_shape_batch_.indices.clear();
  outline_shape_batch_.colors.clear();
}

void Canvas::FlushBatches() {
  // Force update of all batches
  line_batch_.needs_update = true;
  filled_shape_batch_.needs_update = true;
  outline_shape_batch_.needs_update = true;
  UpdateBatches();
}

void Canvas::UpdateBatches() {
  // Update line batch
  if (line_batch_.needs_update && !line_batch_.vertices.empty()) {
    glBindVertexArray(line_batch_.vao);
    
    // Update position buffer
    glBindBuffer(GL_ARRAY_BUFFER, line_batch_.position_vbo);
    glBufferData(GL_ARRAY_BUFFER, 
                 line_batch_.vertices.size() * sizeof(glm::vec3),
                 line_batch_.vertices.data(), GL_DYNAMIC_DRAW);
    
    // Update color buffer
    glBindBuffer(GL_ARRAY_BUFFER, line_batch_.color_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 line_batch_.colors.size() * sizeof(glm::vec4),
                 line_batch_.colors.data(), GL_DYNAMIC_DRAW);
    
    glBindVertexArray(0);
    line_batch_.needs_update = false;
  }
  
  // Update filled shape batch
  if (filled_shape_batch_.needs_update && !filled_shape_batch_.vertices.empty()) {
    glBindVertexArray(filled_shape_batch_.vao);
    
    // Update vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, filled_shape_batch_.vertex_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 filled_shape_batch_.vertices.size() * sizeof(float),
                 filled_shape_batch_.vertices.data(), GL_DYNAMIC_DRAW);
    
    // Update color buffer (expand to match vertices)
    std::vector<glm::vec4> expanded_colors;
    expanded_colors.reserve(filled_shape_batch_.vertices.size() / 3);
    for (size_t i = 0; i < filled_shape_batch_.colors.size(); ++i) {
      // Each color applies to one vertex (vertices.size() / 3 vertices total)
      expanded_colors.push_back(filled_shape_batch_.colors[i]);
    }
    
    glBindBuffer(GL_ARRAY_BUFFER, filled_shape_batch_.color_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 expanded_colors.size() * sizeof(glm::vec4),
                 expanded_colors.data(), GL_DYNAMIC_DRAW);
    
    // Update index buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, filled_shape_batch_.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 filled_shape_batch_.indices.size() * sizeof(uint32_t),
                 filled_shape_batch_.indices.data(), GL_DYNAMIC_DRAW);
    
    glBindVertexArray(0);
    filled_shape_batch_.needs_update = false;
  }
  
  // Update outline shape batch
  if (outline_shape_batch_.needs_update && !outline_shape_batch_.vertices.empty()) {
    glBindVertexArray(outline_shape_batch_.vao);
    
    // Update vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, outline_shape_batch_.vertex_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 outline_shape_batch_.vertices.size() * sizeof(float),
                 outline_shape_batch_.vertices.data(), GL_DYNAMIC_DRAW);
    
    // Update color buffer
    std::vector<glm::vec4> expanded_colors;
    expanded_colors.reserve(outline_shape_batch_.vertices.size() / 3);
    for (size_t i = 0; i < outline_shape_batch_.colors.size(); ++i) {
      expanded_colors.push_back(outline_shape_batch_.colors[i]);
    }
    
    glBindBuffer(GL_ARRAY_BUFFER, outline_shape_batch_.color_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 expanded_colors.size() * sizeof(glm::vec4),
                 expanded_colors.data(), GL_DYNAMIC_DRAW);
    
    // Update index buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, outline_shape_batch_.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 outline_shape_batch_.indices.size() * sizeof(uint32_t),
                 outline_shape_batch_.indices.data(), GL_DYNAMIC_DRAW);
    
    glBindVertexArray(0);
    outline_shape_batch_.needs_update = false;
  }
}

void Canvas::GenerateCircleVertices(float cx, float cy, float radius, int segments,
                                   std::vector<float>& vertices, std::vector<uint32_t>& indices,
                                   bool filled, uint32_t base_index) {
  if (filled) {
    // Add center vertex for filled circle
    vertices.insert(vertices.end(), {cx, cy, 0.0f});
    
    // Add perimeter vertices
    for (int i = 0; i <= segments; i++) {
      float angle = 2.0f * M_PI * i / segments;
      float x = cx + radius * std::cos(angle);
      float y = cy + radius * std::sin(angle);
      vertices.insert(vertices.end(), {x, y, 0.0f});
      
      // Create triangle indices (center, current, next)
      if (i < segments) {
        indices.insert(indices.end(), {
          base_index,           // Center
          base_index + 1 + i,   // Current perimeter vertex
          base_index + 1 + ((i + 1) % segments)  // Next perimeter vertex
        });
      }
    }
  } else {
    // Add perimeter vertices for outline
    for (int i = 0; i <= segments; i++) {
      float angle = 2.0f * M_PI * i / segments;
      float x = cx + radius * std::cos(angle);
      float y = cy + radius * std::sin(angle);
      vertices.insert(vertices.end(), {x, y, 0.0f});
      
      // Create line indices (current, next)
      if (i < segments) {
        indices.insert(indices.end(), {
          base_index + i,           // Current vertex
          base_index + (i + 1)      // Next vertex
        });
      }
    }
  }
}

void Canvas::GenerateRectangleVertices(float x, float y, float width, float height,
                                      std::vector<float>& vertices, std::vector<uint32_t>& indices,
                                      bool filled, uint32_t base_index) {
  // Add the four corner vertices
  vertices.insert(vertices.end(), {
    x, y, 0.0f,                    // Bottom left
    x + width, y, 0.0f,            // Bottom right  
    x + width, y + height, 0.0f,   // Top right
    x, y + height, 0.0f            // Top left
  });
  
  if (filled) {
    // Two triangles for filled rectangle
    indices.insert(indices.end(), {
      base_index, base_index + 1, base_index + 2,  // First triangle
      base_index, base_index + 2, base_index + 3   // Second triangle
    });
  } else {
    // Four lines for rectangle outline
    indices.insert(indices.end(), {
      base_index, base_index + 1,     // Bottom edge
      base_index + 1, base_index + 2, // Right edge
      base_index + 2, base_index + 3, // Top edge
      base_index + 3, base_index      // Left edge
    });
  }
}

void Canvas::OptimizeMemory() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  if (perf_config_.aggressive_memory_cleanup) {
    // Shrink batch vectors to fit current usage
    line_batch_.vertices.shrink_to_fit();
    line_batch_.colors.shrink_to_fit();
    line_batch_.thicknesses.shrink_to_fit();
    
    filled_shape_batch_.vertices.shrink_to_fit();
    filled_shape_batch_.indices.shrink_to_fit();
    filled_shape_batch_.colors.shrink_to_fit();
    
    outline_shape_batch_.vertices.shrink_to_fit();
    outline_shape_batch_.indices.shrink_to_fit();
    outline_shape_batch_.colors.shrink_to_fit();
    
    // Clear pending updates queue if it's grown too large
    if (pending_updates_.size() == 0) {
      std::queue<PendingUpdate> empty_queue;
      pending_updates_.swap(empty_queue);
    }
  }
  
  // Update memory tracking
  if (perf_config_.memory_tracking_enabled) {
    size_t current_usage = GetMemoryUsage();
    memory_tracker_.current_usage = current_usage;
  }
}

void Canvas::PreallocateMemory(size_t estimated_objects) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Pre-allocate batch vectors based on estimated usage
  size_t reserve_size = std::min(estimated_objects, perf_config_.max_batch_size);
  
  line_batch_.vertices.reserve(reserve_size * 2); // 2 vertices per line
  line_batch_.colors.reserve(reserve_size * 2);
  line_batch_.thicknesses.reserve(reserve_size);
  
  filled_shape_batch_.vertices.reserve(reserve_size * 12); // ~4 vertices per shape * 3 components
  filled_shape_batch_.indices.reserve(reserve_size * 6);   // ~2 triangles per shape
  filled_shape_batch_.colors.reserve(reserve_size * 4);    // 4 vertices per shape
  
  outline_shape_batch_.vertices.reserve(reserve_size * 12);
  outline_shape_batch_.indices.reserve(reserve_size * 8);  // ~4 edges per shape
  outline_shape_batch_.colors.reserve(reserve_size * 4);
  
  // Record the pre-allocation
  if (perf_config_.memory_tracking_enabled) {
    size_t allocated_size = GetMemoryUsage();
    memory_tracker_.RecordAllocation(allocated_size);
  }
}

void Canvas::ShrinkToFit() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Shrink all batch vectors to minimum required size
  line_batch_.vertices.shrink_to_fit();
  line_batch_.colors.shrink_to_fit();
  line_batch_.thicknesses.shrink_to_fit();
  
  filled_shape_batch_.vertices.shrink_to_fit();
  filled_shape_batch_.indices.shrink_to_fit();
  filled_shape_batch_.colors.shrink_to_fit();
  
  outline_shape_batch_.vertices.shrink_to_fit();
  outline_shape_batch_.indices.shrink_to_fit();
  outline_shape_batch_.colors.shrink_to_fit();
  
  // Clear and shrink pending updates queue
  std::queue<PendingUpdate> empty_queue;
  pending_updates_.swap(empty_queue);
}

size_t Canvas::GetMemoryUsage() const {
  size_t total_usage = 0;
  
  // Calculate batch memory usage
  total_usage += line_batch_.vertices.capacity() * sizeof(glm::vec3);
  total_usage += line_batch_.colors.capacity() * sizeof(glm::vec4);
  total_usage += line_batch_.thicknesses.capacity() * sizeof(float);
  
  total_usage += filled_shape_batch_.vertices.capacity() * sizeof(float);
  total_usage += filled_shape_batch_.indices.capacity() * sizeof(uint32_t);
  total_usage += filled_shape_batch_.colors.capacity() * sizeof(glm::vec4);
  
  total_usage += outline_shape_batch_.vertices.capacity() * sizeof(float);
  total_usage += outline_shape_batch_.indices.capacity() * sizeof(uint32_t);
  total_usage += outline_shape_batch_.colors.capacity() * sizeof(glm::vec4);
  
  // Add estimated size of pending updates queue
  total_usage += pending_updates_.size() * sizeof(PendingUpdate);
  
  // Add estimated CanvasData memory usage
  if (data_) {
    // Rough estimation - in a real implementation you'd calculate exact sizes
    total_usage += 1024; // Estimated base CanvasData size
  }
  
  return total_usage;
}

}  // namespace quickviz
