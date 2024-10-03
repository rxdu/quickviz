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
#include "opengl_panel.hpp"
#include "gl_triangle_panel.hpp"
#include "imview/gl_layer.hpp"

using namespace quickviz;

class FirstLayer : public GlLayer {
 public:
  FirstLayer() : GlLayer("FirstLayer") {
    this->SetFlexDirection(Styling::FlexDirection::kColumn);
    //    this->SetJustifyContent(Styling::JustifyContent::kSpaceBetween);

    //    auto panel0 = std::make_shared<OpenGLPanel>(1.0, 0, 0);
    //    panel0->SetHeight(100);
    //    this->AddResizableUiNode(panel0);

    auto panel1 = std::make_shared<OpenGLPanel>(1.0, 0, 0);
    panel1->SetHeight(50);
    this->AddResizableUiNode(panel1);

    auto panel2 = std::make_shared<OpenGLPanel>(0, 1.0, 0);
    panel2->SetHeight(60);
    this->AddResizableUiNode(panel2);

    auto panel4 = std::make_shared<OpenGLPanel>(0, 0, 1.0);
    panel4->SetHeight(70);
    this->AddResizableUiNode(panel4);

    //    auto panel5 = std::make_shared<GLTrianglePanel>();
    //    panel5->SetHeight(400);
    //    this->AddResizableUiNode(panel5);
  }
};

class SecondLayer : public Layer {
 public:
  SecondLayer() : Layer("SecondLayer") {
    this->SetFlexDirection(Styling::FlexDirection::kColumn);

    auto panel1 = std::make_shared<FontPanel>();
    panel1->SetHeight(100);
    this->AddResizableUiNode(panel1);
  }
};

int main(int argc, char* argv[]) {
  int opt = 0;

  if (argc > 1) {
    opt = std::stoi(argv[1]);
  } else {
    // clang-format off
    std::cout << "Usage: " << argv[0] << " <option>" << std::endl;
    std::cout << "Option: 0 - add imgui renderable directly" << std::endl;
    std::cout << "        1 - add opengl + imgui renderable directly" << std::endl;
    std::cout << "        2 - add renderable through layers" << std::endl;
    // clang-format on
    return 0;
  }

  Viewer viewer;

  bool add_renderable_directly = false;

  if (opt == 0) {
    // to test adding multiple renderables
    viewer.AddRenderable(std::make_shared<FontPanel>());
  } else if (opt == 1) {
    viewer.AddRenderable(std::make_shared<OpenGLPanel>());
    viewer.AddRenderable(std::make_shared<FontPanel>());
  } else if (opt == 2) {
    auto layer1 = std::make_shared<FirstLayer>();
    viewer.AddRenderable(layer1);

    //    auto layer2 = std::make_shared<SecondLayer>();
    //    viewer.AddRenderable(layer2);
  } else {
    std::cout << "Invalid option" << std::endl;
    return -1;
  }

  viewer.Show();
  return 0;
}