/*
 * test_viewer.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>

#include "imview/viewer.hpp"
#include "imview/ui_panel.hpp"
#include "font_panel.hpp"

using namespace quickviz;

class FirstLayer : public Layer {
 public:
  FirstLayer() : Layer("FirstLayer") {
    auto panel = std::make_shared<UiPanel>("FirstPanel");
    panel->AddRenderable(std::make_shared<FontPanel>());
    this->AddResizableUiNode(panel);
  }
};

class SecondLayer : public Layer {
 public:
  SecondLayer() : Layer("SecondLayer") {}
};

int main(int argc, char* argv[]) {
  int opt = 0;

  if (argc > 1) {
    opt = std::stoi(argv[1]);
  } else {
    std::cout << "Usage: " << argv[0] << " <option>" << std::endl;
    std::cout << "Option: 0 - add renderable directly" << std::endl;
    std::cout << "        1 - add renderable through layers" << std::endl;
    return 0;
  }

  Viewer viewer;

  bool add_renderable_directly = false;

  if (opt == 0) {
    // to test adding multiple renderables
    viewer.AddRenderable(std::make_shared<FontPanel>());
  } else if (opt == 1) {
    auto layer1 = std::make_shared<FirstLayer>();
    viewer.AddRenderable(layer1);

    auto layer2 = std::make_shared<SecondLayer>();
    viewer.AddRenderable(layer2);
  } else {
    std::cout << "Invalid option" << std::endl;
    return -1;
  }

  viewer.Show();
  return 0;
}