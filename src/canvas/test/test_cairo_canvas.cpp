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
    // double xc = 128.0;
    // double yc = 128.0;
    // double radius = 100.0;
    // double angle1 = 45.0 * (M_PI / 180.0);  /* angles are specified */
    // double angle2 = 180.0 * (M_PI / 180.0); /* in radians           */

    // cairo_set_line_width(cr, 10.0);
    // cairo_arc(cr, xc, yc, radius, angle1, angle2);
    // cairo_stroke(cr);

    // /* draw helping lines */
    // cairo_set_source_rgba(cr, 1, 0.2, 0.2, 0.6);
    // cairo_set_line_width(cr, 6.0);

    // cairo_arc(cr, xc, yc, 10.0, 0, 2 * M_PI);
    // cairo_fill(cr);

    // cairo_arc(cr, xc, yc, radius, angle1, angle1);
    // cairo_line_to(cr, xc, yc);
    // cairo_arc(cr, xc, yc, radius, angle2, angle2);
    // cairo_line_to(cr, xc, yc);
    // cairo_stroke(cr);

    ///////////////////////////////////////////////////

    // double xc = 128.0;
    // double yc = 128.0;
    // double radius = 100.0;
    // double angle1 = 45.0 * (M_PI / 180.0);  /* angles are specified */
    // double angle2 = 180.0 * (M_PI / 180.0); /* in radians           */

    // cairo_set_source_rgba(cr, 0, 0.2, 0.2, 0.6);
    // cairo_set_line_width(cr, 10.0);
    // cairo_arc_negative(cr, xc, yc, radius, angle1, angle2);
    // cairo_stroke(cr);

    // /* draw helping lines */
    // cairo_set_source_rgba(cr, 1, 0.2, 0.2, 0.6);
    // cairo_set_line_width(cr, 6.0);

    // cairo_arc(cr, xc, yc, 10.0, 0, 2 * M_PI);
    // cairo_fill(cr);

    // cairo_arc(cr, xc, yc, radius, angle1, angle1);
    // cairo_line_to(cr, xc, yc);
    // cairo_arc(cr, xc, yc, radius, angle2, angle2);
    // cairo_line_to(cr, xc, yc);
    // cairo_stroke(cr);

    ///////////////////////////////////////////////////

    EraseAll();

    double x = 25.6, y = 128.0;
    double x1 = 102.4, y1 = 230.4, x2 = 153.6, y2 = 25.6, x3 = 230.4,
           y3 = 128.0;

    cairo_set_source_rgba(cr, 0.2, 0.2, 0.2, 0.6);

    cairo_move_to(cr, x, y);
    cairo_curve_to(cr, x1, y1, x2, y2, x3, y3);

    cairo_set_line_width(cr, 10.0);
    cairo_stroke(cr);

    cairo_set_source_rgba(cr, 1, 0.2, 0.2, 0.6);
    cairo_set_line_width(cr, 6.0);
    cairo_move_to(cr, x, y);
    cairo_line_to(cr, x1, y1);
    cairo_move_to(cr, x2, y2);
    cairo_line_to(cr, x3, y3);
    cairo_stroke(cr);
  }
};

int main(int argc, const char* argv[]) {
  DrawArc cc;
  //   cc.SetBackgroundColor({0.5, 0.5, 0.5, 0.5});
  cc.Show();
  return 0;
}