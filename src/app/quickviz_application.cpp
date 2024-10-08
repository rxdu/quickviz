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
}

bool QuickvizApplication::Initialize() {
  // setup ui layer
  ui_layer_ = std::make_shared<Layer>("ui_layer");
  main_docking_panel_ = std::make_shared<MainDockingPanel>();
  ui_layer_->AddChild(main_docking_panel_);

  // add ui layer to viewer
  viewer_->AddSceneObject(ui_layer_);

  return true;
}

void QuickvizApplication::Run() { viewer_->Show(); }
}  // namespace quickviz