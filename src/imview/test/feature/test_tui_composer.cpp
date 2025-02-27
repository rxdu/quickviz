/**
 * @file test_tui_composer.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-02-27
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/box.hpp"
#include "imview/terminal/tui_composer.hpp"

using namespace quickviz;

class TestPanel : public TuiPanel {
 public:
  TestPanel(std::string panel) : TuiPanel(panel) {}

  void Draw() override { mvprintw(0, 0, GetName().c_str()); }
};

int main(int argc, char **argv) {
  TuiComposer tui;

  auto box = std::make_shared<Box>("Box");
  box->SetFlexDirection(Styling::FlexDirection::kColumn);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  auto test_panel = std::make_shared<TestPanel>("MyPanel1");
  test_panel->SetFlexGrow(0.5f);
  test_panel->SetFlexShrink(0.5f);
  box->AddChild(test_panel);

  //   auto test_panel2 = std::make_shared<TestPanel>("MyPanel2");
  //   test_panel2->SetFlexGrow(0.3f);
  //   test_panel2->SetFlexShrink(0.3f);
  //   box->AddChild(test_panel2);

  tui.AddSceneObject(box);
  tui.Show();
  return 0;
}
