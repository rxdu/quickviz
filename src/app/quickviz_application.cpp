/*
 * @file quickviz_application.cpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quickviz_application.hpp"

namespace quickviz {
QuickvizApplication::QuickvizApplication() {
  viewer_ = std::make_unique<Viewer>("QuickViz");
  data_reader_ = std::make_shared<DataReader>();
}

bool QuickvizApplication::Initialize() {
  // setup ui layer
  ui_layer_ = std::make_shared<Box>("ui_layer");
  ui_layer_->SetFlexDirection(Styling::FlexDirection::kColumn);
  ui_layer_->SetAlignItems(Styling::AlignItems::kStretch);
  {
    menu_bar_ = std::make_shared<MenuBar>("menu_bar");
    menu_bar_->SetHeight(24);
    menu_bar_->SetFlexGrow(0);
    menu_bar_->SetFlexShrink(0);
    ui_layer_->AddChild(menu_bar_);

    main_docking_panel_ = std::make_shared<MainDockingPanel>();
    main_docking_panel_->SetFlexGrow(1);
    main_docking_panel_->SetFlexShrink(1);
    ui_layer_->AddChild(main_docking_panel_);
  }

  // add ui layer to viewer
  viewer_->AddSceneObject(ui_layer_);

  // set up callbacks
  menu_bar_->SetChangeDebugPanelVisibilityCallback(
      std::bind(&QuickvizApplication::OnChangeDebugPanelVisibility, this,
                std::placeholders::_1));
  menu_bar_->SetWindowShouldCloseCallback(
      std::bind(&QuickvizApplication::OnWindowShouldClose, this));

  // initialize data reader

  return true;
}

void QuickvizApplication::Run() { viewer_->Show(); }

void QuickvizApplication::OnChangeDebugPanelVisibility(bool visible) {
  main_docking_panel_->ChangeDebugPanelVisibility(visible);
}

void QuickvizApplication::OnWindowShouldClose() {
  viewer_->SetWindowShouldClose();
}
}  // namespace quickviz