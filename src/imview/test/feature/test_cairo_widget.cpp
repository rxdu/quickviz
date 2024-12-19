/*
 * test_cairo_widget.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>
#include <thread>

#include <opencv2/opencv.hpp>

#include "imview/viewer.hpp"
#include "imview/widget/cairo_widget.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace quickviz;

void PaintUnifiedCoordinate(cairo_t* cr, float aspect_ratio) {
  float pos_x = 0.5 * aspect_ratio;
  float pos_y = 0.5;

  DrawPoint(cr, {pos_x, pos_y}, 0.01, {1, 0.2, 0.2, 0.6});
  DrawCircle(cr, {pos_x, pos_y}, 0.3, 0.002, {1, 0.2, 0.2, 0.6});
  DrawRectangle(cr, {pos_x - 0.2f, pos_y - 0.2f}, {pos_x + 0.2f, pos_y + 0.2f},
                0.002);
}

void Paint(cairo_t* cr, float aspect_ratio) {
  DrawPoint(cr, {860, 200});
  DrawPoint(cr, {1060, 200}, 10, {1, 0.2, 0.2, 0.6});

  DrawLine(cr, {860, 150}, {1060, 150});
  DrawLine(cr, {860, 250}, {1060, 250}, 2, {1, 0.2, 0.2, 0.6});

  DrawCircle(cr, {960, 540}, 30);
  DrawCircle(cr, {960, 540}, 50, 5, {1, 0.2, 0.2, 0.6});

  DrawRing(cr, {960, 540}, 100, 130, 0, M_PI / 4.0);
  DrawRing(cr, {960, 540}, 140, 180, 0, M_PI / 4.0, 5, colors[GREEN]);

  DrawRing(cr, {960, 540}, 100, 200, M_PI, M_PI * 1.5f, 5, colors[YELLOW],
           true);
  DrawRing(cr, {960, 540}, 140, 220, M_PI / 2.0, 2 * M_PI / 3.0, 5,
           colors[GREEN], true);
  DrawRing(cr, {960, 540}, 140, 220, M_PI / 2.0, 5 * M_PI / 6.0, 5,
           colors[PURPLE], false);

  DrawArc(cr, {600, 800}, 60, M_PI, 2 * M_PI / 4.0);
  DrawArc(cr, {600, 800}, 70, M_PI / 4.0, M_PI, 5, {1, 0.2, 0.2, 0.6});

  DrawArcSector(cr, {1060, 800}, 60, 0, -M_PI / 4.0);
  DrawArcSector(cr, {1060, 800}, 80, M_PI / 4.0, M_PI, 5, {1, 0.2, 0.2, 0.6});
  DrawArcSector(cr, {1060, 800}, 85, 5.0 * M_PI / 4.0, 6.0 * M_PI / 4.0, 5,
                {1, 0.2, 0.2, 0.3}, true);

  DrawRectangle(cr, {400, 400}, {500, 600});
  DrawRectangle(cr, {550, 400}, {600, 600}, 5, colors[MAGENTA]);
  DrawRectangle(cr, {650, 400}, {700, 600}, 5, colors[CYAN], true);

  for (int i = 0; i < COLOR_LAST; ++i) {
    DrawLine(cr, {200.0f, 300.0f + 20 * i}, {250.0f, 300.0f + 20 * i}, 10,
             colors[i]);
  }
}

int main(int argc, char* argv[]) {
  Viewer viewer;

  auto cairo_widget = std::make_shared<CairoWidget>("cairo_unified", true);
  cairo_widget->OnResize(300, 200);
  cairo_widget->AttachDrawFunction(PaintUnifiedCoordinate);
  viewer.AddSceneObject(cairo_widget);

  auto cairo_widget2 = std::make_shared<CairoWidget>("cairo");
  cairo_widget2->OnResize(300, 200);
  cairo_widget2->AttachDrawFunction(Paint);
  viewer.AddSceneObject(cairo_widget2);

  viewer.Show();

  return 0;
}