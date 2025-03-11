/*
 * gl_scene_manager.hpp
 *
 * Created on 3/6/25 9:09 PM
 * Description:
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef GL_SCENE_MANAGER_HPP
#define GL_SCENE_MANAGER_HPP

#include <memory>
#include <string>
#include <unordered_map>

#include "imview/panel.hpp"
#include "imview/interface/opengl_object.hpp"
#include "imview/component/opengl/frame_buffer.hpp"
#include "imview/component/opengl/camera.hpp"
#include "imview/component/opengl/camera_controller.hpp"

namespace quickviz {
class GlSceneManager : public Panel {
  enum MouseButton {
    kLeft = 0,
    kRight = 1,
    kMiddle = 2,
  };

 public:
  GlSceneManager(const std::string& name);
  ~GlSceneManager() = default;

  // public methods
  void SetShowRenderingInfo(bool show);

  void AddOpenGLObject(const std::string& name,
                       std::unique_ptr<OpenGlObject> object);
  void RemoveOpenGLObject(const std::string& name);
  OpenGlObject* GetOpenGLObject(const std::string& name);
  void ClearOpenGLObjects();
  void UpdateView(const glm::mat4& projection, const glm::mat4& view);

  void Draw() override;

 protected:
  void DrawOpenGLObject();

  std::unique_ptr<FrameBuffer> frame_buffer_;
  glm::mat4 projection_ = glm::mat4(1.0f);
  glm::mat4 view_ = glm::mat4(1.0f);
  std::unordered_map<std::string, std::unique_ptr<OpenGlObject>>
      drawable_objects_;

  std::unique_ptr<Camera> camera_;
  std::unique_ptr<CameraController> camera_controller_;
  bool show_rendering_info_ = true;
};
}  // namespace quickviz

#endif  // GL_SCENE_MANAGER_HPP
