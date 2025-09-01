/*
 * panel.hpp
 *
 * Created on 4/3/22 11:07 PM
 * Description: a panel represents a GUI window that can be rendered by ImGui
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_VISUALIZATION_IMVIEW_INCLUDE_IMVIEW_PANEL_HPP
#define ROBOSW_SRC_VISUALIZATION_IMVIEW_INCLUDE_IMVIEW_PANEL_HPP

#include <string>
#include <vector>
#include <memory>

#include "imgui.h"
#include "imview/scene_object.hpp"
#include "imview/input/imgui_input_utils.hpp"
#include "imview/input/input_manager.hpp"

namespace quickviz {
class Panel : public SceneObject {
 public:
  Panel(std::string name);
  virtual ~Panel() = default;

  // public API
  void SetAutoLayout(bool value);
  bool IsAutoLayout() const;
  void OnRender() override;
  void OnJoystickUpdate(const JoystickInput& input) override;

  void SetNoTitleBar(bool value);
  void SetNoResize(bool value);
  void SetNoMove(bool value);
  void SetNoScrollbar(bool value);
  void SetNoScrollWithMouse(bool value);
  void SetNoCollapse(bool value);
  void SetAlwaysAutoResize(bool value);
  void SetNoBackground(bool value);
  void SetNoSavedSettings(bool value);
  void SetNoMouseInputs(bool value);
  void SetMenuBar(bool value);
  void SetHorizontalScrollbar(bool value);
  void SetNoFocusOnAppearing(bool value);
  void SetNoBringToFrontOnFocus(bool value);
  void SetAlwaysVerticalScrollbar(bool value);
  void SetAlwaysHorizontalScrollbar(bool value);
  void SetNoNavInputs(bool value);
  void SetNoNavFocus(bool value);
  void SetUnsavedDocument(bool value);
  void SetNoDocking(bool value);
  void SetNoNav();
  void SetNoDecoration();
  void SetNoInputs();

  void SetWindowNoMenuButton();
  void SetWindowNoTabBar();
  void SetWindowHiddenTabBar();
  void SetWindowNoCloseButton();
  
  // Window position and size access
  ImVec2 GetWindowPos() const { return ImGui::GetWindowPos(); }
  ImVec2 GetWindowSize() const { return ImGui::GetWindowSize(); }

  virtual void Draw() = 0;

  // Input management
  void SetInputManager(std::shared_ptr<InputManager> input_manager) { 
    input_manager_ = input_manager; 
  }
  InputManager* GetInputManager() const { return input_manager_.get(); }

 protected:
  // for derived classes
  void Begin(bool* p_open = NULL);
  void End();

  // Input processing (called during panel rendering)
  virtual void ProcessPanelInput();

  // Input event handling - override in derived classes
  virtual bool OnInputEvent(const InputEvent& event) { return false; }
  virtual void OnMouseClick(const glm::vec2& position, int button) {}
  virtual void OnMouseMove(const glm::vec2& position, const glm::vec2& delta) {}
  virtual void OnKeyPress(int key, const ModifierKeys& modifiers) {}

  // Input utilities for derived classes  
  glm::vec2 GetContentRelativeMousePos() const;
  bool IsMouseOverContent() const;

 private:
  bool auto_layout_ = false;
  ImGuiWindowFlags flags_ = ImGuiWindowFlags_None;
  ImGuiWindowClass window_class_;
  std::shared_ptr<InputManager> input_manager_;
};
}  // namespace quickviz

#endif  // ROBOSW_SRC_VISUALIZATION_IMVIEW_INCLUDE_IMVIEW_PANEL_HPP
