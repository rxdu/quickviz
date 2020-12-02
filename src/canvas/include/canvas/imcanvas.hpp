/*
 * imcanvas.hpp
 *
 * Created on: Dec 02, 2020 21:20
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef IMCANVAS_HPP
#define IMCANVAS_HPP

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

  // customization
  void SetBackgroundColor(ImVec4 bk);

  // draw on canvas
  virtual void Draw(){};
  void Show();

 protected:
  bool initialized_ = false;
  GLFWwindow* window_;
  ImVec4 background_color_;
};
}  // namespace rdu

#endif /* IMCANVAS_HPP */
