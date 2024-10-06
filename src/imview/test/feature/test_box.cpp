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

#include "scene_objects/imgui_fixed_scene_object.hpp"

using namespace quickviz;

class PrimaryBox : public Box {
 public:
  PrimaryBox() : Box("PrimaryBox") {
    this->SetFlexDirection(Styling::FlexDirection::kRow);

    auto left = std::make_shared<Box>("LeftBox");
    left->SetWidth(350);
    //    {
    //      left->SetFlexDirection(Styling::FlexDirection::kColumn);
    //
    //      auto panel1 =
    //      std::make_shared<ImGuiFixedSceneObject>("left-imgui1");
    //      panel1->SetHeight(100);
    //      left->AddChild(panel1);
    //
    //      auto panel2 =
    //      std::make_shared<ImGuiFixedSceneObject>("left-imgui2");
    //      panel2->SetHeight(200);
    //      left->AddChild(panel2);
    //
    //      auto panel3 =
    //      std::make_shared<ImGuiFixedSceneObject>("left-imgui3");
    //      //      panel3->SetHeight(250);
    //      panel3->SetFlexGrow(1);
    //      left->AddChild(panel3);
    //    }
    this->AddChild(left);

    auto right = std::make_shared<Box>("RightBox");
    //    right->SetFlexGrow(1);
    right->SetWidth(450);
    //    {
    //      right->SetFlexDirection(Styling::FlexDirection::kColumn);
    //
    //      auto panel4 =
    //      std::make_shared<ImGuiFixedSceneObject>("right-imgui1");
    //      panel4->SetHeight(500);
    //      right->AddChild(panel4);
    //
    //      auto panel5 =
    //      std::make_shared<ImGuiFixedSceneObject>("right-imgui2");
    //      //      panel5->SetHeight(300);
    //      panel5->SetFlexGrow(1);
    //      right->AddChild(panel5);
    //    }
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