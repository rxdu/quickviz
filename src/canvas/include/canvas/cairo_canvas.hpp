/*
 * cairo_canvas.hpp
 *
 * Created on: Dec 03, 2020 21:24
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef CAIRO_CANVAS_HPP
#define CAIRO_CANVAS_HPP

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <cairo.h>

#include "canvas/im_canvas.hpp"

namespace rdu {
class CairoCanvas : public ImCanvas {
 public:
  CairoCanvas(uint32_t width = 640, uint32_t height = 480,
              std::string title = "Canvas");
  ~CairoCanvas();

  // do not allow copy
  CairoCanvas(const CairoCanvas& other) = delete;
  CairoCanvas& operator=(const CairoCanvas& other) = delete;
  CairoCanvas(const CairoCanvas&& other) = delete;
  CairoCanvas& operator=(const CairoCanvas&& other) = delete;

  virtual void Paint(cairo_t* cr) {}

 private:
  cairo_surface_t* surface = nullptr;
  cairo_t* cr = nullptr;
  GLuint image_texture_;

  void Draw() override;
};
}  // namespace rdu

#endif /* CAIRO_CANVAS_HPP */
