/*
 * test_viewer.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/viewer.hpp"
#include "font_panel.hpp"

using namespace quickviz;

class FirstLayer : public Layer {
 public:
  FirstLayer() : Layer("FirstLayer") {}
};

class SecondLayer : public Layer {
 public:
  SecondLayer() : Layer("SecondLayer") {}
};

int main(int argc, char* argv[]) {
  Viewer viewer;

  auto layer1 = std::make_shared<FirstLayer>();
  viewer.AddRenderable(1, layer1);

  auto layer2 = std::make_shared<SecondLayer>();
  viewer.AddRenderable(2, layer2);

  viewer.Show();
  return 0;
}