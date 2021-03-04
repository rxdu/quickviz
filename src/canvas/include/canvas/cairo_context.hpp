/*
 * cairo_context.hpp
 *
 * Created on: Mar 04, 2021 16:44
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef CAIRO_CONTEXT_HPP
#define CAIRO_CONTEXT_HPP

#include <cstdint>

#include <cairo.h>
#include <GL/gl3w.h>

namespace rdu {
class CairoContext {
 public:
  struct Color {
    Color(double _r, double _g, double _b, double _a)
        : r(_r), g(_g), b(_b), a(_a) {}

    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    double a = 1.0;
  };

 public:
  CairoContext(uint32_t width, uint32_t height);
  ~CairoContext();

  // do not allow copy or move
  CairoContext(const CairoContext&) = delete;
  CairoContext& operator=(const CairoContext&) = delete;
  CairoContext(const CairoContext&&) = delete;
  CairoContext& operator=(const CairoContext&&) = delete;

  bool Initialized() const { return initialized_; };
  void Clear(Color bg_color);

  cairo_t* GetObject() { return cr_; }
  cairo_surface_t* GetSurface() { return surface_; }

 private:
  bool initialized_ = false;

  cairo_surface_t* surface_ = nullptr;
  cairo_t* cr_ = nullptr;
  GLuint image_texture_;
};
}  // namespace rdu

#endif /* CAIRO_CONTEXT_HPP */
