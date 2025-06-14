/*
 * @file quickviz_application.cpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quickviz_application.hpp"

#include <stdexcept>
#include <iostream>

namespace quickviz {
QuickvizApplication::QuickvizApplication() {
  try {
    viewer_ = std::make_unique<Viewer>("QuickViz");
    data_reader_ = std::make_shared<DataReader>();
  } catch (const std::exception& e) {
    std::cerr << "Failed to create application components: " << e.what() << std::endl;
    throw;
  }
}

bool QuickvizApplication::Initialize() {
  try {
    // setup ui layer
    ui_layer_ = std::make_shared<Box>("ui_layer");
    if (!ui_layer_) {
      std::cerr << "Failed to create UI layer" << std::endl;
      return false;
    }
    
    ui_layer_->SetFlexDirection(Styling::FlexDirection::kColumn);
    ui_layer_->SetAlignItems(Styling::AlignItems::kStretch);
    
    {
      menu_bar_ = std::make_shared<MenuBar>("menu_bar");
      if (!menu_bar_) {
        std::cerr << "Failed to create menu bar" << std::endl;
        return false;
      }
      
      menu_bar_->SetHeight(24);
      menu_bar_->SetFlexGrow(0);
      menu_bar_->SetFlexShrink(0);
      ui_layer_->AddChild(menu_bar_);

      main_docking_panel_ = std::make_shared<MainDockingPanel>();
      if (!main_docking_panel_) {
        std::cerr << "Failed to create main docking panel" << std::endl;
        return false;
      }
      
      main_docking_panel_->SetFlexGrow(1);
      main_docking_panel_->SetFlexShrink(1);
      ui_layer_->AddChild(main_docking_panel_);
    }

    // add ui layer to viewer
    if (!viewer_->AddSceneObject(ui_layer_)) {
      std::cerr << "Failed to add UI layer to viewer" << std::endl;
      return false;
    }

    // set up callbacks
    menu_bar_->SetChangeDebugPanelVisibilityCallback(
        std::bind(&QuickvizApplication::OnChangeDebugPanelVisibility, this,
                  std::placeholders::_1));
    menu_bar_->SetWindowShouldCloseCallback(
        std::bind(&QuickvizApplication::OnWindowShouldClose, this));

    // initialize data reader
    // TODO: Add data reader initialization if needed

    return true;
  } catch (const std::exception& e) {
    std::cerr << "Failed to initialize application: " << e.what() << std::endl;
    return false;
  }
}

void QuickvizApplication::Run() { 
  if (viewer_) {
    viewer_->Show(); 
  } else {
    std::cerr << "Cannot run application: viewer not initialized" << std::endl;
  }
}

void QuickvizApplication::OnChangeDebugPanelVisibility(bool visible) {
  if (main_docking_panel_) {
    main_docking_panel_->ChangeDebugPanelVisibility(visible);
  }
}

void QuickvizApplication::OnWindowShouldClose() {
  if (viewer_) {
    viewer_->SetWindowShouldClose();
  }
}
}  // namespace quickviz