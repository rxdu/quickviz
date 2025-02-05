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

#include "feature/scene_objects/imtext_panel.hpp"
#include "feature/scene_objects/imgui_fixed_panel.hpp"
#include "feature/scene_objects/opengl_scene_object.hpp"
#include "feature/scene_objects/gl_triangle_scene_object.hpp"

using namespace quickviz;

class GuiLayer : public Box {
 public:
  GuiLayer() : Box("PrimaryUiBox") {
    this->SetFlexDirection(Styling::FlexDirection::kRow);
    // this->SetFlexDirection(Styling::FlexDirection::kColumn);
    this->SetAlignItems(Styling::AlignItems::kStretch);

    {
      auto left = std::make_shared<Box>("LeftBox");
      left->SetFlexDirection(Styling::FlexDirection::kColumn);
      // left->SetFlexDirection(Styling::FlexDirection::kRow);
      left->SetAlignItems(Styling::AlignItems::kStretch);
      left->SetFlexGrow(0.5);
      left->SetFlexShrink(0.5);
      {
        auto panel1 = std::make_shared<ImGuiFixedPanel>("left-imgui1");
        panel1->SetFlexGrow(0.5);
        panel1->SetFlexShrink(0.5);
        left->AddChild(panel1);

        auto panel2 = std::make_shared<ImGuiFixedPanel>("left-imgui2");
        panel2->SetFlexGrow(0.25);
        panel2->SetFlexShrink(0.25);
        left->AddChild(panel2);
      }
      this->AddChild(left);

      auto right = std::make_shared<Box>("RightBox");
      right->SetFlexDirection(Styling::FlexDirection::kRowReverse);
      right->SetAlignItems(Styling::AlignItems::kStretch);
      right->SetFlexGrow(0.5);
      right->SetFlexShrink(0.5);
      {
        auto panel1 = std::make_shared<ImGuiFixedPanel>("right-imgui1");
        panel1->SetFlexGrow(0.5);
        panel1->SetFlexShrink(0.5);
        right->AddChild(panel1);

        auto panel2 = std::make_shared<ImGuiFixedPanel>("right-imgui2");
        panel2->SetFlexGrow(0.25);
        panel2->SetFlexShrink(0.25);
        right->AddChild(panel2);
      }
      this->AddChild(right);
    }
  }
};

int main(int argc, char* argv[]) {
  Viewer viewer;

  auto gui_layer = std::make_shared<GuiLayer>();
  viewer.AddSceneObject(gui_layer);

  viewer.Show();
  return 0;
}