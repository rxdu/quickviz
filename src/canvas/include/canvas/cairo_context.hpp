/*
 * cairo_context.hpp
 *
 * Created on: Mar 04, 2021 16:44
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef CAIRO_CONTEXT_HPP
#define CAIRO_CONTEXT_HPP

#include <cstdint>
#include <stack>

#include <cairo.h>
#include <GL/gl3w.h>

namespace rdu {
class CairoContext {
  struct Scaler {
    double x;
    double y;
  };

 public:
  // normalize coordinate so that drawing area will be x,y in [0, 1]
  CairoContext(uint32_t width = 640, uint32_t height = 320,
               bool normalize_coordinate = false);
  ~CairoContext();

  // do not allow copy or move
  CairoContext(const CairoContext&) = delete;
  CairoContext& operator=(const CairoContext&) = delete;
  CairoContext(const CairoContext&&) = delete;
  CairoContext& operator=(const CairoContext&&) = delete;

  /* public functions */
  // check whether Cairo context has been initialized correctly
  bool Initialized() const { return initialized_; }

  float GetWidth() const { return static_cast<float>(width_); }
  float GetHeight() const { return static_cast<float>(height_); }

  // set/reset surface coordinate scale (relative), sx, sy in [0, 1.0]
  void PushScale(double sx = 0, double sy = 0);
  void PopScale();

  // GL-related functions can only be called after a GL context has been created
  GLuint RenderToGlTexture();

  // get object of Cairo context and surface
  cairo_t* GetCairoObject() { return cr_; }
  cairo_surface_t* GetSurface() { return surface_; }

 private:
  bool initialized_ = false;
  bool gl_texture_created_ = false;

  uint32_t width_;
  uint32_t height_;
  std::stack<Scaler> scaler_stack_;

  cairo_surface_t* surface_ = nullptr;
  cairo_t* cr_ = nullptr;
  GLuint image_texture_;

  void GenGlTexture();
};
}  // namespace rdu

#endif /* CAIRO_CONTEXT_HPP */
