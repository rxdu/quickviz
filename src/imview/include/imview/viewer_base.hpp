/*
 * viewer_base.hpp
 *
 * Created on: Jul 27, 2021 08:56
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef VIEWER_BASE_HPP
#define VIEWER_BASE_HPP

#include <memory>

#include "imview/window.hpp"

namespace rdu {
namespace wgui {
class ViewerBase {
 public:
  ViewerBase(uint32_t width = 640, uint32_t height = 480,
             std::string title = "wgui Viewer",
             uint32_t window_hints = Window::WIN_RESIZABLE |
                                     Window::WIN_DECORATED);
  ~ViewerBase();

  // start viewer loop
  void Show();

 protected:
  std::unique_ptr<wgui::Window> window_;

  // draw function (to be implemented in derived classes)
  virtual void Draw() {}
};
}  // namespace wgui
}  // namespace rdu

#endif /* VIEWER_BASE_HPP */
