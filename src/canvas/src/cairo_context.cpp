/*
 * cairo_context.cpp
 *
 * Created on: Mar 04, 2021 17:01
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "canvas/cairo_context.hpp"

#include <iostream>

namespace rdu {
CairoContext::CairoContext(uint32_t width, uint32_t height)
    : width_(width), height_(height) {
  // create cairo context
  surface_ = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, width_, height_);
  if (cairo_surface_status(surface_) != CAIRO_STATUS_SUCCESS) {
    std::cerr << "[ERROR] create_cairo_context() - Couldn't create surface\n";
    return;
  }

  cr_ = cairo_create(surface_);
  if (cairo_status(cr_) != CAIRO_STATUS_SUCCESS) {
    std::cerr << "[ERROR] create_cairo_context() - Couldn't create context\n";
    return;
  }

  initialized_ = true;
}

CairoContext::~CairoContext() {
  // destroy resource
  if (gl_texture_created_) glDeleteTextures(1, &image_texture_);
  cairo_surface_destroy(surface_);
  cairo_destroy(cr_);
}

void CairoContext::BindGlTexture() {
  // create and setup OpenGL texture
  glGenTextures(1, &image_texture_);
  glBindTexture(GL_TEXTURE_2D, image_texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  // the following two are required on WebGL for non power-of-two textures
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  gl_texture_created_ = true;
}

GLuint CairoContext::RenderToGlTexture() {
  // convert surface to OpenGL texture
  int tex_w = cairo_image_surface_get_width(surface_);
  int tex_h = cairo_image_surface_get_height(surface_);
  unsigned char* data = cairo_image_surface_get_data(surface_);
  glTexImage2D(GL_TEXTURE_2D, 0, 4, tex_w, tex_h, 0, GL_BGRA, GL_UNSIGNED_BYTE,
               data);
  return image_texture_;
}
}  // namespace rdu