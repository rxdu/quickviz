/*
 * @file gl_frame_buffer.hpp
 * @date 10/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_GL_FRAME_BUFFER_HPP
#define XMOTION_GL_FRAME_BUFFER_HPP

#include <cstdint>

namespace quickviz {
class GlFrameBuffer {
 public:
  GlFrameBuffer(uint32_t width, uint32_t height);
  ~GlFrameBuffer();

  // do not allow copy or move
  GlFrameBuffer(const GlFrameBuffer&) = delete;
  GlFrameBuffer& operator=(const GlFrameBuffer&) = delete;
  GlFrameBuffer(GlFrameBuffer&&) = delete;
  GlFrameBuffer& operator=(GlFrameBuffer&&) = delete;

  // public methods
  void Bind() const;
  void Unbind() const;
  void Clear() const;
  uint32_t GetTextureId() const { return texture_id_; }

  uint32_t GetWidth() const { return width_; }
  uint32_t GetHeight() const { return height_; }
  void Resize(uint32_t width, uint32_t height);

 private:
  void CreateBuffers();
  void DestroyBuffers();

  uint32_t width_;
  uint32_t height_;
  uint32_t texture_id_;
  uint32_t frame_buffer_;
  uint32_t texture_buffer_;
  uint32_t render_buffer_;
};
}  // namespace quickviz

#endif  // XMOTION_GL_FRAME_BUFFER_HPP