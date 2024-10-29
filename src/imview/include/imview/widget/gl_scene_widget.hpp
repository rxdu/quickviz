/*
 * @file gl_scene_widget.hpp
 * @date 10/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_GL_SCENE_WIDGET_HPP
#define XMOTION_GL_SCENE_WIDGET_HPP

#include <memory>
#include <functional>

#include "imview/panel.hpp"
#include "imview/widget/details/gl_frame_buffer.hpp"

namespace quickviz {
class GlSceneWidget : public Panel {
 public:
  GlSceneWidget(const std::string& widget_name);
  ~GlSceneWidget() = default;

  // public methods
  using GlRenderFunction = std::function<void(const GlFrameBuffer&)>;
  void SetGlRenderFunction(GlRenderFunction func);
  void Draw() override;

 private:
  GlRenderFunction render_function_;
  std::unique_ptr<GlFrameBuffer> frame_buffer_;
};
}  // namespace quickviz

#endif  // XMOTION_GL_SCENE_WIDGET_HPP