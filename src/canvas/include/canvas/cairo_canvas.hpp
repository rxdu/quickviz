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
  struct Color {
    Color(double _r, double _g, double _b, double _a)
        : r(_r), g(_g), b(_b), a(_a) {}

    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    double a = 1.0;
  };

 public:
  CairoCanvas(uint32_t width = 640, uint32_t height = 480,
              std::string title = "Cairo");
  virtual ~CairoCanvas();

  // do not allow copy
  CairoCanvas(const CairoCanvas& other) = delete;
  CairoCanvas& operator=(const CairoCanvas& other) = delete;
  CairoCanvas(const CairoCanvas&& other) = delete;
  CairoCanvas& operator=(const CairoCanvas&& other) = delete;

  void SetBackgroundColor(Color color);

  void EraseAll();
  virtual void Paint(cairo_t* cr) {}

 private:
  Color background_color_ = {1.0, 1.0, 1.0, 1.0};
  cairo_surface_t* surface = nullptr;
  cairo_t* cr = nullptr;
  GLuint image_texture_;

  void Draw() override;
};
}  // namespace rdu

#endif /* CAIRO_CANVAS_HPP */
