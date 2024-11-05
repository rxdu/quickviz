/*
 * @file frame_buffer.hpp
 * @date 10/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_FRAME_BUFFER_HPP
#define XMOTION_FRAME_BUFFER_HPP

#include <cstdint>

namespace quickviz {
class FrameBuffer {
 public:
  FrameBuffer(uint32_t width, uint32_t height);
  ~FrameBuffer();

  // do not allow copy or move
  FrameBuffer(const FrameBuffer&) = delete;
  FrameBuffer& operator=(const FrameBuffer&) = delete;
  FrameBuffer(FrameBuffer&&) = delete;
  FrameBuffer& operator=(FrameBuffer&&) = delete;

  // public methods
  void Bind(bool lock_aspect_ratio = true) const;
  void Unbind() const;
  void Clear(float r = 0.0, float g = 0.0, float b = 0.0, float a = 1.0) const;
  uint32_t GetTextureId() const { return texture_buffer_; }

  uint32_t GetWidth() const { return width_; }
  uint32_t GetHeight() const { return height_; }
  void Resize(uint32_t width, uint32_t height);

 private:
  void CreateBuffers();
  void DestroyBuffers();

  float aspect_ratio_ = 1.0;
  bool lock_aspect_ratio_ = true;
  uint32_t width_;
  uint32_t height_;
  uint32_t frame_buffer_;
  uint32_t texture_buffer_;
  uint32_t render_buffer_;
};
}  // namespace quickviz

#endif  // XMOTION_FRAME_BUFFER_HPP