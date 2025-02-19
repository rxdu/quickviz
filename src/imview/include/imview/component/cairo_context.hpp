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

#include <cairo.h>

#include <cstdint>
#include <stack>

namespace quickviz {
class CairoContext {
  struct Scaler {
    double x;
    double y;
  };

 public:
  // default constructor: empty cairo surface
  CairoContext() = default;
  // unify coordinate so that drawing area will be defined as:
  // x (width): [0, 1.0 * aspect_ratio]
  // y (height): [0, 1.0]
  CairoContext(uint32_t width, uint32_t height, bool unify_coordinate = false);

  ~CairoContext();

  // do not allow copy or move
  CairoContext(const CairoContext&) = delete;
  CairoContext& operator=(const CairoContext&) = delete;
  CairoContext(const CairoContext&&) = delete;
  CairoContext& operator=(const CairoContext&&) = delete;

  /* public functions */
  void Resize(uint32_t width, uint32_t height);
  float GetAspectRatio() const;

  float GetWidth() const { return static_cast<float>(width_); }
  float GetHeight() const { return static_cast<float>(height_); }

  // set/reset surface coordinate scale (relative), sx, sy in [0, 1.0]
  void PushScale(double sx = 0, double sy = 0);
  void PopScale();

  // GL-related functions can only be called after a GL context has been created
  uint32_t RenderToGlTexture();

  // get object of Cairo context and surface
  cairo_t* GetCairoObject() { return cr_; }
  cairo_surface_t* GetSurface() { return surface_; }

 private:
  bool gl_texture_created_ = false;

  uint32_t width_;
  uint32_t height_;
  bool unified_coordinate_;
  std::stack<Scaler> scaler_stack_;

  cairo_surface_t* surface_ = nullptr;
  cairo_t* cr_ = nullptr;
  uint32_t image_texture_;

  float aspect_ratio_ = 1.0;

  void CreateSurface();
  void GenGlTexture();
};
}  // namespace quickviz

#endif /* CAIRO_CONTEXT_HPP */
