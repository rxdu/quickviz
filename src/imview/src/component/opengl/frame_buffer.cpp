/*
 * @file frame_buffer.cpp
 * @date 10/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/component/opengl/frame_buffer.hpp"

#include <iostream>
#include <algorithm>

// Include GLAD for OpenGL function loading
#include "glad/glad.h"

namespace quickviz {
FrameBuffer::FrameBuffer(uint32_t width, uint32_t height, uint32_t samples)
    : width_(width),
      height_(height),
      samples_(samples),
      frame_buffer_(0),
      texture_buffer_(0),
      render_buffer_(0) {
  aspect_ratio_ = static_cast<float>(width) / static_cast<float>(height);
  CreateBuffers();
}

FrameBuffer::~FrameBuffer() { DestroyBuffers(); }

void FrameBuffer::CreateBuffers() {
  // Check if multisampling is supported
  GLint max_samples = 0;
  glGetIntegerv(GL_MAX_SAMPLES, &max_samples);
  samples_ = std::min(samples_, static_cast<uint32_t>(max_samples));

  // Generate and bind the framebuffer
  glGenFramebuffers(1, &frame_buffer_);
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_);

  // Create a color attachment texture
  glGenTextures(1, &texture_buffer_);
  
  if (samples_ > 0) {
    // Use multisampling
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, texture_buffer_);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, samples_, GL_RGB, width_, height_, GL_TRUE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, texture_buffer_, 0);
  } else {
    glBindTexture(GL_TEXTURE_2D, texture_buffer_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture_buffer_, 0);
  }

  // Create a renderbuffer object for depth and stencil attachment
  glGenRenderbuffers(1, &render_buffer_);
  glBindRenderbuffer(GL_RENDERBUFFER, render_buffer_);
  
  if (samples_ > 0) {
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples_, GL_DEPTH24_STENCIL8, width_, height_);
  } else {
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width_, height_);
  }
  
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, render_buffer_);

  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    throw std::runtime_error("Framebuffer initialization unsuccessful!");
  }

  // If using multisampling, create an intermediate framebuffer for resolving
  if (samples_ > 0) {
    glGenFramebuffers(1, &intermediate_fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, intermediate_fbo_);
    
    // Create a color attachment texture for the intermediate framebuffer
    glGenTextures(1, &screen_texture_);
    glBindTexture(GL_TEXTURE_2D, screen_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, screen_texture_, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      throw std::runtime_error("Intermediate framebuffer initialization unsuccessful!");
    }
  }

  // Unbind framebuffer
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void FrameBuffer::DestroyBuffers() {
  glDeleteFramebuffers(1, &frame_buffer_);
  glDeleteTextures(1, &texture_buffer_);
  glDeleteRenderbuffers(1, &render_buffer_);
  
  if (samples_ > 0) {
    glDeleteFramebuffers(1, &intermediate_fbo_);
    glDeleteTextures(1, &screen_texture_);
  }
  
  frame_buffer_ = 0;
  texture_buffer_ = 0;
  render_buffer_ = 0;
  intermediate_fbo_ = 0;
  screen_texture_ = 0;
}

void FrameBuffer::Bind(bool lock_aspect_ratio) const {
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_);
  if (lock_aspect_ratio) {
    float expected_width = height_ * aspect_ratio_;
    if (expected_width > width_) {
      float actual_height = width_ / aspect_ratio_;
      glViewport(0, (height_ - actual_height) / 2, width_, actual_height);
    } else {
      float actual_width = expected_width;
      glViewport((width_ - actual_width) / 2, 0, actual_width, height_);
    }
  } else {
    glViewport(0, 0, width_, height_);
  }
  glEnable(GL_DEPTH_TEST);
}

void FrameBuffer::Unbind() const { 
  if (samples_ > 0) {
    // Resolve multisampled framebuffer to intermediate framebuffer
    glBindFramebuffer(GL_READ_FRAMEBUFFER, frame_buffer_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, intermediate_fbo_);
    glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT, GL_NEAREST);
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0); 
}

void FrameBuffer::Clear(float r, float g, float b, float a) const {
  glClearColor(r, g, b, a);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

uint32_t FrameBuffer::GetTextureId() const { 
  // Return the resolved texture if using multisampling
  return (samples_ > 0) ? screen_texture_ : texture_buffer_; 
}

void FrameBuffer::Resize(uint32_t width, uint32_t height) {
  if ((width == width_ && height == height_) || (width == 0 || height == 0))
    return;

  width_ = width;
  height_ = height;

  // delete old buffers and create new ones
  DestroyBuffers();
  CreateBuffers();
}
}  // namespace quickviz