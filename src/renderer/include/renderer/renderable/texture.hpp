/*
 * texture.hpp
 *
 * Created on 4/3/25 9:33 PM
 * Description:
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef COMPONENT_OPENGL_TEXTURE_HPP
#define COMPONENT_OPENGL_TEXTURE_HPP

#include <string>
#include <vector>
#include <memory>

#include <glm/glm.hpp>

#include "renderer/interface/opengl_object.hpp"
#include "renderer/shader_program.hpp"

namespace quickviz {
class Texture : public OpenGlObject {
 public:
  Texture();
  ~Texture();

  // Pixel format enumeration
  enum class PixelFormat { kGray, kRgb, kRgba, kBgr, kBgra };

  // Buffer update strategy (similar to PointCloud)
  enum class BufferUpdateStrategy {
    kAuto,           // Automatically choose based on image size
    kBufferSubData,  // Always use glTexSubImage2D
    kMapBuffer       // Always use PBO for updates
  };

  // Efficient update methods with move semantics
  bool UpdateData(int width, int height, PixelFormat format,
                  const unsigned char* data);
  bool UpdateData(int width, int height, PixelFormat format,
                  std::vector<unsigned char>&& data);

  // Buffer management
  void PreallocateBuffer(int width, int height, PixelFormat format);
  void SetBufferUpdateStrategy(BufferUpdateStrategy strategy) {
    buffer_update_strategy_ = strategy;
  }
  void SetBufferUpdateThreshold(size_t threshold) {
    buffer_update_threshold_ = threshold;
  }

  // Position and orientation
  void SetOrigin(const glm::vec3& origin, float resolution = 1.0f);

  // Getters
  int GetWidth() const { return image_width_; }
  int GetHeight() const { return image_height_; }
  PixelFormat GetFormat() const { return pixel_format_; }

 protected:
  void AllocateGpuResources() override;
  void ReleaseGpuResources() override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;

 private:
  // Helper methods for buffer updates
  void UpdateTextureWithSubData(const void* data, size_t size_bytes);
  void UpdateTextureWithPBO(const void* data, size_t size_bytes);
  bool ShouldUsePBO(size_t data_size) const;

  // OpenGL resources
  uint32_t texture_id_ = 0;
  uint32_t pbo_id_ = 0;  // Pixel Buffer Object for efficient updates
  uint32_t vao_ = 0;
  uint32_t vbo_ = 0;
  ShaderProgram shader_;

  // Texture properties
  int image_width_ = 0;
  int image_height_ = 0;
  PixelFormat pixel_format_ = PixelFormat::kRgba;

  // Position and orientation
  glm::vec3 origin_ = glm::vec3(0.0f);
  float resolution_ = 1.0f;

  // Buffer management
  bool buffer_preallocated_ = false;
  BufferUpdateStrategy buffer_update_strategy_ = BufferUpdateStrategy::kAuto;
  size_t buffer_update_threshold_ = 1024 * 1024;  // Default: 1MB
  bool needs_update_ = false;
};
}  // namespace quickviz

#endif  // COMPONENT_OPENGL_TEXTURE_HPP
