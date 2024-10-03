/*
 * viewer.hpp
 *
 * Created on: Jul 27, 2021 08:56
 * Description: Viewer is built on top of Window,
 *  with base setup for imgui and implot
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef IMVIEW_VIEWER_HPP
#define IMVIEW_VIEWER_HPP

#include <memory>
#include <map>

#include "imgui.h"

#include "imview/fonts.hpp"
#include "imview/window.hpp"
#include "implot/implot.h"
#include "imview/layer.hpp"

namespace quickviz {
class Viewer : public Window {
 public:
  Viewer(std::string title = "Viewer", uint32_t width = 1920,
         uint32_t height = 1080,
         uint32_t window_hints = WIN_RESIZABLE | WIN_DECORATED);
  virtual ~Viewer();

  // public API
  void ApplyDarkColorScheme();
  void ApplyLightColorScheme();
  void SetBackgroundColor(float r, float g, float b, float a);

  void EnableDocking(bool enable);
  void EnableKeyboardNav(bool enable);
  void EnableGamepadNav(bool enable);
  void DockSpaceOverMainViewport();

  // add renderable layers and start viewer loop
  bool AddRenderable(std::shared_ptr<Renderable> renderable);
  void Show();

 protected:
  void ClearBackground();
  void CreateNewImGuiFrame();
  void RenderImGuiFrame();
  void RenderRenderables();

 private:
  void LoadDefaultStyle();
  void OnResize(GLFWwindow* window, int width, int height);

  std::vector<std::shared_ptr<Layer>> layers_;
  float bg_color_[4];
};
}  // namespace quickviz

#endif /* IMVIEW_VIEWER_HPP */
