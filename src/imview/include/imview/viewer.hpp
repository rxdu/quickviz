/*
 * viewer.hpp
 *
 * Created on: Jul 27, 2021 08:56
 * Description: Viewer manages rendering of imgui/implot and other OpenGL
 * renderables and eventually writes the content to the frame buffer for
 * display in the GLFW window (class Window).
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef IMVIEW_VIEWER_HPP
#define IMVIEW_VIEWER_HPP

#include <memory>
#include <vector>
#include <unordered_map>

#include "imgui.h"

#include "imview/fonts.hpp"
#include "imview/window.hpp"
#include "imview/scene_object.hpp"

namespace quickviz {
class Viewer : public Window {
 public:
  Viewer(std::string title = "Viewer", uint32_t width = 1920,
         uint32_t height = 1080,
         uint32_t window_hints = WIN_RESIZABLE | WIN_DECORATED);
  virtual ~Viewer();

  /********************* public API *********************/
  // window style
  void ApplyDarkColorScheme();
  void ApplyLightColorScheme();
  void SetBackgroundColor(float r, float g, float b, float a);

  void SetWindowSizeLimits(int min_x, int min_y, int max_x = -1,
                           int max_y = -1);
  void SetWindowShouldClose();

  // optional features
  void EnableDocking(bool enable);
  void EnableKeyboardNav(bool enable);
  void EnableGamepadNav(bool enable);

  // window content rendering
  bool AddSceneObject(std::shared_ptr<SceneObject> obj);
  bool RemoveSceneObject(std::shared_ptr<SceneObject> obj);
  void ClearSceneObjects();

  // start the rendering loop (blocking)
  void Show();

 protected:
  void SetupOpenGL();
  void ClearBackground();
  void CreateNewImGuiFrame();
  void RenderImGuiFrame();
  void RenderSceneObjects();

 private:
  void LoadDefaultStyle();
  void OnResize(GLFWwindow* window, int width, int height);
  void CheckOpenGLCapabilities();

  std::vector<std::shared_ptr<SceneObject>> scene_objects_;
  float bg_color_[4];
};
}  // namespace quickviz

#endif /* IMVIEW_VIEWER_HPP */
