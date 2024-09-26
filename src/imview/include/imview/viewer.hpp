/*
 * viewer.hpp
 *
 * Created on: Jul 27, 2021 08:56
 * Description: base setup for imgui and implot on top of glfw
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef IMVIEW_VIEWER_HPP
#define IMVIEW_VIEWER_HPP

#include <memory>

#include "imgui.h"
#include "implot/implot.h"
#include "imview/window.hpp"

namespace quickviz {
enum class FontSize { Tiny, Small, Normal, Big, Large, ExtraLarge };

struct DisplayRegion {
  ImVec2 pos;
  ImVec2 size;
};

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

  ImFont *GetFont(FontSize size);

  // start viewer loop
  void Show();

 protected:
  void StartNewFrame();
  void RenderFrame();

  // draw function (to be implemented in derived classes)
  virtual void Draw() {}

 private:
  void LoadDefaultStyle();
  void LoadFonts();
  void Render();

  ImFont *font_tiny_;
  ImFont *font_small_;
  ImFont *font_normal_;
  ImFont *font_big_;
  ImFont *font_large_;
  ImFont *font_extra_large_;
};
}  // namespace quickviz

#endif /* IMVIEW_VIEWER_HPP */
