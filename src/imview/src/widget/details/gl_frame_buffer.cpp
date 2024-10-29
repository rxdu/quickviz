/*
 * @file gl_frame_buffer.cpp
 * @date 10/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/widget/details/gl_frame_buffer.hpp"

#include <iostream>

#include "glad/glad.h"

namespace quickviz {
GlFrameBuffer::GlFrameBuffer(uint32_t width, uint32_t height)
    : width_(width),
      height_(height),
      frame_buffer_(0),
      texture_buffer_(0),
      render_buffer_(0) {
  CreateBuffers();
}

GlFrameBuffer::~GlFrameBuffer() { DestroyBuffers(); }

void GlFrameBuffer::CreateBuffers() {
  // generate and bind the framebuffer
  glGenFramebuffers(1, &frame_buffer_);
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_);

  // create a color attachment texture
  glGenTextures(1, &texture_buffer_);
  glBindTexture(GL_TEXTURE_2D, texture_buffer_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB,
               GL_UNSIGNED_BYTE, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         texture_buffer_, 0);

  // create a renderbuffer object for depth and stencil attachment
  glGenRenderbuffers(1, &render_buffer_);
  glBindRenderbuffer(GL_RENDERBUFFER, render_buffer_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width_, height_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                            GL_RENDERBUFFER, render_buffer_);

  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    throw std::runtime_error("Framebuffer initialization unsuccessful!");
  }

  // Unbind framebuffer
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void GlFrameBuffer::DestroyBuffers() {
  glDeleteFramebuffers(1, &frame_buffer_);
  glDeleteTextures(1, &texture_buffer_);
  glDeleteRenderbuffers(1, &render_buffer_);
  frame_buffer_ = 0;
  texture_buffer_ = 0;
  render_buffer_ = 0;
}

void GlFrameBuffer::Bind() const {
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_);
  glViewport(0, 0, width_, height_);
  glEnable(GL_DEPTH_TEST);
}

void GlFrameBuffer::Unbind() const { glBindFramebuffer(GL_FRAMEBUFFER, 0); }

void GlFrameBuffer::Clear() const {
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void GlFrameBuffer::Resize(uint32_t width, uint32_t height) {
  if (width == width_ && height == height_) return;

  width_ = width;
  height_ = height;

  // delete old buffers and create new ones
  DestroyBuffers();
  CreateBuffers();

  std::cout << "new frame buffer id: " << frame_buffer_ << std::endl;
}
}  // namespace quickviz