/**
 * @file test_tui_composer.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-02-27
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/box.hpp"
#include "imview/terminal/tui_viewer.hpp"

using namespace quickviz;

class TestPanel : public TuiPanel {
 public:
  TestPanel(std::string panel) : TuiPanel(panel) {}

  void Draw() override {
    TuiText::Printw(window_, 2, 1, TuiText::Color::kBlue,
                    "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz",
                    NULL);

    TuiText::SetAttribute(window_, TuiText::ATTR_UNDERLINE |
                                       TuiText::ATTR_BOLD |
                                       TuiText::ATTR_BLINK);
    TuiText::Printw(window_, 2 + 3, 1, TuiText::Color::kRed,
                    "012345678901234567890", NULL);
    TuiText::ResetAttribute(window_);

    TuiText::Printw(window_, 2 + 5, 1, TuiText::Color::kGreen,
                    "012345678901234567890", NULL);
  }
};

int main(int argc, char **argv) {
  TuiViewer tui;

  auto box = std::make_shared<Box>("Box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  auto test_panel = std::make_shared<TestPanel>("MyPanel1");
  test_panel->SetFlexGrow(0.5f);
  test_panel->SetFlexShrink(0.5f);
  box->AddChild(test_panel);

  auto vbox = std::make_shared<Box>("VBox");
  vbox->SetFlexGrow(0.5f);
  vbox->SetFlexShrink(0.5f);
  vbox->SetFlexDirection(Styling::FlexDirection::kColumn);
  vbox->SetAlignItems(Styling::AlignItems::kStretch);

  auto test_panel2 = std::make_shared<TestPanel>("MyPanel2");
  test_panel2->SetFlexGrow(0.4f);
  test_panel2->SetFlexShrink(0.4f);
  vbox->AddChild(test_panel2);

  auto test_panel3 = std::make_shared<TestPanel>("MyPanel3");
  test_panel3->SetFlexGrow(0.6f);
  test_panel3->SetFlexShrink(0.6f);
  vbox->AddChild(test_panel3);

  box->AddChild(vbox);

  tui.AddSceneObject(box);
  tui.Show();
  return 0;
}
