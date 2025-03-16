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
#include "imview/input/mouse.hpp"
#include "imview/interface/opengl_object.hpp"
#include "imview/component/opengl/frame_buffer.hpp"
#include "imview/component/opengl/camera.hpp"
#include "imview/component/opengl/camera_controller.hpp"

namespace quickviz {
class GlSceneManager : public Panel {
 public:
  enum class Mode { k2D, k3D };

  GlSceneManager(const std::string& name, Mode mode = Mode::k3D);
  ~GlSceneManager() = default;

  // public methods
  Mode GetMode() const { return mode_; }

  void SetShowRenderingInfo(bool show);
  void SetBackgroundColor(float r, float g, float b, float a);

  void AddOpenGLObject(const std::string& name,
                       std::unique_ptr<OpenGlObject> object);
  void RemoveOpenGLObject(const std::string& name);
  OpenGlObject* GetOpenGLObject(const std::string& name);
  void ClearOpenGLObjects();

  void Draw() override;

 protected:
  void UpdateView(const glm::mat4& projection, const glm::mat4& view);
  void DrawOpenGLObject();

  glm::vec4 background_color_ = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

  Mode mode_ = Mode::k3D;
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
