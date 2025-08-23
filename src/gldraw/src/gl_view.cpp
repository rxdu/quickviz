/*
 * @file gl_view.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Implementation of reusable OpenGL view class for testing renderable
 * objects
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/gl_view.hpp"

#include <iostream>
#include <stdexcept>

#include "imview/styling.hpp"

namespace quickviz {

GlView::GlView(const Config& config) : config_(config) { SetupViewer(); }

void GlView::SetupViewer() {
  // Create box container for layout
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // Create scene manager with proper layout settings
  scene_manager_ = std::make_shared<GlSceneManager>(config_.window_title,
                                                    config_.scene_mode);
  scene_manager_->SetAutoLayout(true);
  scene_manager_->SetNoTitleBar(true);
  scene_manager_->SetFlexGrow(1.0f);
  scene_manager_->SetFlexShrink(0.0f);

  box->AddChild(scene_manager_);
  viewer_.AddSceneObject(box);
}

void GlView::SetupBasicScene() {
  // Add grid if requested
  if (config_.show_grid) {
    auto grid = std::make_unique<Grid>(config_.grid_size, config_.grid_step,
                                       config_.grid_color);
    scene_manager_->AddOpenGLObject("grid", std::move(grid));
  }

  // Add coordinate frame if requested
  if (config_.show_coordinate_frame) {
    auto frame = std::make_unique<CoordinateFrame>(
        config_.coordinate_frame_size,
        config_.scene_mode == GlSceneManager::Mode::k2D);
    scene_manager_->AddOpenGLObject("coordinate_frame", std::move(frame));
  }
}

void GlView::SetSceneSetup(SceneSetupCallback callback) {
  scene_setup_callback_ = std::move(callback);
}

void GlView::AddHelpSection(const std::string& section_title,
                            const std::vector<std::string>& help_lines) {
  help_sections_.emplace_back(section_title, help_lines);
}

void GlView::SetDescription(const std::string& description) {
  description_ = description;
}

GlSceneManager* GlView::GetSceneManager() const { return scene_manager_.get(); }

void GlView::DisplayHelp() const {
  std::cout << "\n=== " << config_.window_title << " ===" << std::endl;

  if (!description_.empty()) {
    std::cout << description_ << std::endl;
  }

  std::cout << "\n=== Camera Controls ===" << std::endl;
  std::cout << "Left Mouse: Rotate camera (orbit mode)" << std::endl;
  std::cout << "Middle Mouse: Translate/Pan in 3D space" << std::endl;
  std::cout << "Scroll Wheel: Zoom in/out" << std::endl;
  std::cout << "R: Reset camera to default position" << std::endl;
  std::cout << "ESC: Exit application" << std::endl;

  // Display additional help sections
  for (const auto& section : help_sections_) {
    std::cout << "\n=== " << section.first << " ===" << std::endl;
    for (const auto& line : section.second) {
      std::cout << line << std::endl;
    }
  }

  std::cout << std::endl;
}

void GlView::Run() {
  try {
    // Set up basic scene elements
    SetupBasicScene();

    // Call user-provided scene setup if available
    if (scene_setup_callback_) {
      scene_setup_callback_(scene_manager_.get());
    }

    // Display help information
    DisplayHelp();

    // Run the viewer (blocks until window is closed)
    viewer_.Show();

  } catch (const std::exception& e) {
    std::cerr << "Error in GlView::Run(): " << e.what() << std::endl;
    throw;
  }
}

}  // namespace quickviz