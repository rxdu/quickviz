/*
 * test_canvas.cpp
 *
 * Created on: Dec 02, 2020 21:23
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "canvas/im_canvas.hpp"

using namespace rdu;

struct ImDraw : public ImCanvas {
  void Draw() override {
    // do nothing
  }
};

int main(int argc, char *argv[]) {
  ImDraw canvas;
  canvas.Show();
  return 0;
}