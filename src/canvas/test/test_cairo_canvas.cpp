/*
 * test_cairo_canvas.cpp
 *
 * Created on: Dec 03, 2020 21:27
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "canvas/cairo_canvas.hpp"

using namespace rdu;

struct DrawArc : public CairoCanvas {
  void Paint(cairo_t* cr) override {
    double xc = 128.0;
    double yc = 128.0;
    double radius = 100.0;
    double angle1 = 45.0 * (M_PI / 180.0);  /* angles are specified */
    double angle2 = 180.0 * (M_PI / 180.0); /* in radians           */

    cairo_set_line_width(cr, 10.0);
    cairo_arc(cr, xc, yc, radius, angle1, angle2);
    cairo_stroke(cr);

    /* draw helping lines */
    cairo_set_source_rgba(cr, 1, 0.2, 0.2, 0.6);
    cairo_set_line_width(cr, 6.0);

    cairo_arc(cr, xc, yc, 10.0, 0, 2 * M_PI);
    cairo_fill(cr);

    cairo_arc(cr, xc, yc, radius, angle1, angle1);
    cairo_line_to(cr, xc, yc);
    cairo_arc(cr, xc, yc, radius, angle2, angle2);
    cairo_line_to(cr, xc, yc);
    cairo_stroke(cr);
  }
};

int main(int argc, const char* argv[]) {
  DrawArc cc;
  cc.Show();
  return 0;
}