/*
 * @file test_box.cpp
 * @date 10/6/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>

#include "imview/viewer.hpp"
#include "imview/box.hpp"

#include "feature/scene_objects/imgui_fixed_panel.hpp"

using namespace quickviz;

class PrimaryBox : public Box {
 public:
  PrimaryBox() : Box("PrimaryBox") {
    this->SetFlexDirection(Styling::FlexDirection::kRow);

    auto left = std::make_shared<Box>("LeftBox");
    left->SetWidth(500);
    {
      left->SetFlexDirection(Styling::FlexDirection::kColumn);

      auto panel1 = std::make_shared<ImGuiFixedPanel>("left-imgui1");
      panel1->SetHeight(100);
      left->AddChild(panel1);

      auto panel2 = std::make_shared<ImGuiFixedPanel>("left-imgui2");
      panel2->SetHeight(200);
      left->AddChild(panel2);

      auto panel3 = std::make_shared<ImGuiFixedPanel>("left-imgui3");
      panel3->SetHeight(250);
      left->AddChild(panel3);
    }
    this->AddChild(left);

    auto right = std::make_shared<Box>("RightBox");
    right->SetFlexGrow(1);
    {
      right->SetFlexDirection(Styling::FlexDirection::kColumn);

      auto panel4 = std::make_shared<ImGuiFixedPanel>("right-imgui1");
      panel4->SetHeight(500);
      right->AddChild(panel4);

      auto panel5 = std::make_shared<ImGuiFixedPanel>("right-imgui2");
      panel5->SetHeight(300);
      right->AddChild(panel5);
    }
    this->AddChild(right);
  }
};

int main(int argc, char* argv[]) {
  PrimaryBox primary_box;
  primary_box.OnResize(1920, 1080);
  std::cout << "------------------------------------------------" << std::endl;
  primary_box.PrintLayout();
  return 0;
}