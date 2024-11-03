/*
 * @file gl_widget.hpp
 * @date 10/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_GL_WIDGET_HPP
#define XMOTION_GL_WIDGET_HPP

#include <memory>
#include <functional>

#include "imview/panel.hpp"
#include "imview/component/opengl/frame_buffer.hpp"

namespace quickviz {
class GlWidget : public Panel {
 public:
  GlWidget(const std::string& widget_name);
  ~GlWidget() = default;

  // public methods
  using GlRenderFunction = std::function<void(const FrameBuffer&)>;
  void SetGlRenderFunction(GlRenderFunction func);
  void Draw() override;

 private:
  GlRenderFunction render_function_;
  std::unique_ptr<FrameBuffer> frame_buffer_;
};
}  // namespace quickviz

#endif  // XMOTION_GL_WIDGET_HPP