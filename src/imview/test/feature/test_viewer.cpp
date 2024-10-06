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
#include "imview/box.hpp"

#include "scene_objects/imtext_scene_object.hpp"
#include "scene_objects/imgui_fixed_scene_object.hpp"
#include "scene_objects/opengl_scene_object.hpp"
#include "scene_objects/gl_triangle_scene_object.hpp"

using namespace quickviz;

class FirstLayer : public Box {
 public:
  FirstLayer() : Box("PrimaryGlBox") {
    this->SetFlexDirection(Styling::FlexDirection::kColumn);
    this->SetJustifyContent(Styling::JustifyContent::kFlexStart);

    auto panel1 = std::make_shared<OpenGLSceneObject>("text1", 1.0, 0, 0);
    panel1->SetHeight(50);
    this->AddChild(panel1);

    auto panel2 = std::make_shared<OpenGLSceneObject>("text2", 0, 1.0, 0);
    panel2->SetHeight(60);
    this->AddChild(panel2);

    auto panel3 = std::make_shared<OpenGLSceneObject>("text3", 0, 0, 1.0);
    panel3->SetHeight(70);
    this->AddChild(panel3);

    auto panel4 = std::make_shared<GLTriangleSceneObject>();
    panel4->SetMinHeight(400);
    this->AddChild(panel4);
  }
};

class SecondLayer : public Box {
 public:
  SecondLayer() : Box("PrimaryUiBox") {
    this->SetFlexDirection(Styling::FlexDirection::kRow);

    auto left = std::make_shared<Box>("LeftBox");
    left->SetWidth(500);
    {
      left->SetFlexDirection(Styling::FlexDirection::kColumn);

      auto panel1 = std::make_shared<ImGuiFixedSceneObject>("left-imgui1");
      panel1->SetHeight(100);
      left->AddChild(panel1);

      auto panel2 = std::make_shared<ImGuiFixedSceneObject>("left-imgui2");
      panel2->SetHeight(200);
      left->AddChild(panel2);

      auto panel3 = std::make_shared<ImGuiFixedSceneObject>("left-imgui3");
      panel3->SetHeight(250);
      left->AddChild(panel3);
    }
    this->AddChild(left);

    auto right = std::make_shared<Box>("RightBox");
    right->SetFlexGrow(1);
    {
      right->SetFlexDirection(Styling::FlexDirection::kColumn);

      auto panel4 = std::make_shared<ImGuiFixedSceneObject>("right-imgui1");
      panel4->SetHeight(500);
      right->AddChild(panel4);

      auto panel5 = std::make_shared<ImGuiFixedSceneObject>("right-imgui2");
      panel5->SetHeight(300);
      right->AddChild(panel5);
    }
    this->AddChild(right);
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

  if (opt == 0) {
    auto obj1 = std::make_shared<ImTextSceneObject>("Panel1");
    viewer.AddSceneObject(obj1);

    auto obj2 = std::make_shared<ImTextSceneObject>("Panel2");
    viewer.AddSceneObject(obj2);
  } else if (opt == 1) {
    auto obj1 = std::make_shared<ImTextSceneObject>("UiObject");
    viewer.AddSceneObject(obj1);

    auto obj2 = std::make_shared<OpenGLSceneObject>("obj2", 0, 0.6, 0.6);
    obj2->SetPosition(0, 0);
    obj2->OnResize(viewer.GetWidth(), viewer.GetHeight() / 2.0);
    viewer.AddSceneObject(obj2);

    auto obj3 = std::make_shared<GLTriangleSceneObject>();
    obj3->SetPosition(0, viewer.GetHeight() / 2.0);
    obj3->OnResize(viewer.GetWidth(), viewer.GetHeight() / 2.0);
    viewer.AddSceneObject(obj3);
  } else if (opt == 2) {
    //    auto layer1 = std::make_shared<FirstLayer>();
    //    viewer.AddSceneObject(layer1);

    auto layer2 = std::make_shared<SecondLayer>();
    viewer.AddSceneObject(layer2);
  } else {
    std::cout << "Invalid option" << std::endl;
    return -1;
  }

  viewer.Show();
  return 0;
}