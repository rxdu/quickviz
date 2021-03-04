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
CairoContext::CairoContext(uint32_t width, uint32_t height) {
  // create cairo context
  surface_ = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, width, height);
  if (cairo_surface_status(surface_) != CAIRO_STATUS_SUCCESS) {
    std::cerr << "create_cairo_context() - Couldn't create surface\n";
    return;
  }

  cr_ = cairo_create(surface_);
  if (cairo_status(cr_) != CAIRO_STATUS_SUCCESS) {
    std::cerr << "create_cairo_context() - Couldn't create context\n";
    return;
  }

  //   // create and setup OpenGL texture
  //   glGenTextures(1, &image_texture_);
  //   glBindTexture(GL_TEXTURE_2D, image_texture_);
  //   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  //   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  //   // The following two are required on WebGL for non power-of-two textures
  //   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  //   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  initialized_ = true;
}

CairoContext::~CairoContext() {
  // destroy resource
  //   glDeleteTextures(1, &image_texture_);
  cairo_surface_destroy(surface_);
  cairo_destroy(cr_);
}

void CairoContext::Clear(Color bg_color) {
  cairo_set_source_rgba(cr_, bg_color.r, bg_color.g, bg_color.b, bg_color.a);
  cairo_paint(cr_);
}
}  // namespace rdu