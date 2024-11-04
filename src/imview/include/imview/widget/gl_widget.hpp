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
#include <unordered_map>

#include "imview/panel.hpp"
#include "imview/component/opengl/frame_buffer.hpp"
#include "imview/interface/opengl_drawable.hpp"

namespace quickviz {
class GlWidget : public Panel {
 public:
  GlWidget(const std::string& widget_name);
  ~GlWidget() = default;

  // public methods
  void AddOpenGLObject(const std::string& name,
                       std::unique_ptr<OpenGLDrawable> object);
  void RemoveOpenGLObject(const std::string& name);
  void ClearOpenGLObjects();
  void UpdateView(const glm::mat4& projection, const glm::mat4& view);

  void Draw() override;

 protected:
  std::unique_ptr<FrameBuffer> frame_buffer_;
  glm::mat4 projection_ = glm::mat4(1.0f);
  glm::mat4 view_ = glm::mat4(1.0f);
  std::unordered_map<std::string, std::unique_ptr<OpenGLDrawable>>
      drawable_objects_;
};
}  // namespace quickviz

#endif  // XMOTION_GL_WIDGET_HPP