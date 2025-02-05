/*
 * test_auto_layout.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>

#include "imview/viewer.hpp"
#include "imview/box.hpp"

#include "scene_objects/imtext_panel.hpp"
#include "scene_objects/imgui_fixed_panel.hpp"
#include "scene_objects/opengl_scene_object.hpp"
#include "scene_objects/gl_triangle_scene_object.hpp"

using namespace quickviz;

class GuiLayer : public Box {
 public:
  GuiLayer() : Box("PrimaryUiBox") {
    this->SetFlexDirection(Styling::FlexDirection::kRow);
    this->SetAlignItems(Styling::AlignItems::kStretch);

    auto left = std::make_shared<Box>("LeftBox");
    left->SetFlexDirection(Styling::FlexDirection::kColumn);
    left->SetAlignItems(Styling::AlignItems::kStretch);
    left->SetFlexGrow(0.5);
    left->SetFlexShrink(0.5);
    {
      auto panel1 = std::make_shared<ImGuiFixedPanel>("left-imgui1");
      panel1->SetFlexGrow(0.25);
      panel1->SetFlexShrink(0.25);
      left->AddChild(panel1);

      auto panel2 = std::make_shared<ImGuiFixedPanel>("left-imgui2");
      panel2->SetFlexGrow(0.25);
      panel2->SetFlexShrink(0.25);
      left->AddChild(panel2);

      auto panel3 = std::make_shared<ImGuiFixedPanel>("left-imgui3");
      panel3->SetFlexGrow(0.25);
      panel3->SetFlexShrink(0.25);
      left->AddChild(panel3);
    }
    this->AddChild(left);

    auto right = std::make_shared<Box>("RightBox");
    right->SetFlexDirection(Styling::FlexDirection::kColumn);
    right->SetAlignItems(Styling::AlignItems::kStretch);
    right->SetFlexGrow(0.5);
    right->SetFlexShrink(0.5);
    {
      auto panel4 = std::make_shared<ImGuiFixedPanel>("right-imgui1");
      panel4->SetFlexGrow(0.25);
      panel4->SetFlexShrink(0.25);
      right->AddChild(panel4);

      auto inner_box = std::make_shared<Box>("InnerBox");
      inner_box->SetFlexGrow(0.5);
      inner_box->SetFlexShrink(0.5);
      {
        inner_box->SetFlexDirection(Styling::FlexDirection::kRow);

        auto panel4 = std::make_shared<ImGuiFixedPanel>("right-inner-imgui1");
        panel4->SetFlexGrow(0.5);
        inner_box->AddChild(panel4);

        auto panel5 = std::make_shared<ImGuiFixedPanel>("right-inner-imgui2");
        panel5->SetFlexGrow(0.5);
        inner_box->AddChild(panel5);
      }
      right->AddChild(inner_box);
    }
    this->AddChild(right);
  }
};

int main(int argc, char* argv[]) {
  Viewer viewer;

  auto gui_layer = std::make_shared<GuiLayer>();
  viewer.AddSceneObject(gui_layer);

  viewer.Show();
  return 0;
}