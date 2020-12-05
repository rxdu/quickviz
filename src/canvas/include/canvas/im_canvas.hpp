/*
 * im_canvas.hpp
 *
 * Created on: Dec 02, 2020 21:20
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef IM_CANVAS_HPP
#define IM_CANVAS_HPP

#include <stdio.h>

#include <cstdint>
#include <string>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

namespace rdu {
class ImCanvas {
 public:
  ImCanvas(uint32_t width = 640, uint32_t height = 480,
           std::string title = "Canvas");
  ~ImCanvas();

  // do not allow copy
  ImCanvas(const ImCanvas& other) = delete;
  ImCanvas& operator=(const ImCanvas& other) = delete;
  ImCanvas(const ImCanvas&& other) = delete;
  ImCanvas& operator=(const ImCanvas&& other) = delete;

  uint32_t GetWidth() const { return width_; }
  uint32_t GetHeight() const { return height_; }

  // customization
  void SetBackgroundColor(ImVec4 bk);

  // draw on canvas
  virtual void Draw() = 0;
  void Show();

 protected:
  bool initialized_ = false;
  uint32_t width_ = 640;
  uint32_t height_ = 480;

  GLFWwindow* window_;
  ImVec4 background_color_;
};
}  // namespace rdu

#endif /* IM_CANVAS_HPP */
