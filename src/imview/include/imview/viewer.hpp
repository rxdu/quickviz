/*
 * viewer.hpp
 *
 * Created on: Jul 27, 2021 08:56
 * Description: Viewer manages rendering of imgui/implot and other OpenGL
 * renderables and eventually writes the content to the frame buffer for
 * display in the GLFW window (class Window).
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef IMVIEW_VIEWER_HPP
#define IMVIEW_VIEWER_HPP

#include <memory>
#include <vector>
#include <functional>
#include <unordered_map>

#include "imgui.h"

#include "imview/fonts.hpp"
#include "imview/window.hpp"
#include "imview/scene_object.hpp"

namespace quickviz {
struct JoystickDevice {
  int id;
  std::string name;
};

struct JoystickInput {
  JoystickDevice device;
  std::vector<float> axes;
  std::vector<unsigned char> buttons;
  std::vector<unsigned char> hats;

  bool operator!=(const JoystickInput& rhs) const {
    bool axes_not_equal = false;
    if (axes.size() != rhs.axes.size()) {
      axes_not_equal = true;
    } else {
      for (int i = 0; i < axes.size(); ++i) {
        if (std::abs(axes[i] - rhs.axes[i]) > 0.0001) {
          axes_not_equal = true;
          break;
        }
      }
    }
    return device.id != rhs.device.id || axes_not_equal || buttons != rhs.buttons ||
           hats != rhs.hats;
  }
};

class Viewer : public Window {
 public:
  Viewer(std::string title = "Viewer", uint32_t width = 1920,
         uint32_t height = 1080,
         uint32_t window_hints = WIN_RESIZABLE | WIN_DECORATED);
  virtual ~Viewer();

  /********************* public API *********************/
  // window style
  void ApplyDarkColorScheme();
  void ApplyLightColorScheme();
  void SetBackgroundColor(float r, float g, float b, float a);

  void SetWindowSizeLimits(int min_x, int min_y, int max_x = -1,
                           int max_y = -1);
  void SetWindowShouldClose();

  // optional features
  void EnableDocking(bool enable);
  void EnableKeyboardNav(bool enable);
  void EnableGamepadNav(bool enable);

  // user input handling
  void EnableJoystickInput(bool enable);
  std::vector<JoystickDevice> GetListOfJoysticks();
  using JoystickDeviceChangeCallback =
      std::function<void(const std::vector<JoystickDevice>&)>;
  void SetJoystickDeviceChangeCallback(JoystickDeviceChangeCallback callback);
  using JoystickInputUpdateCallback = std::function<void(const JoystickInput&)>;
  bool MonitorJoystickInputUpdate(int id, JoystickInputUpdateCallback callback);

  // window content rendering
  bool AddSceneObject(std::shared_ptr<SceneObject> obj);

  // start the rendering loop (blocking)
  void Show();

 protected:
  void SetupOpenGL();
  void ClearBackground();
  void CreateNewImGuiFrame();
  void RenderImGuiFrame();
  void RenderSceneObjects();

 private:
  void LoadDefaultStyle();
  void OnResize(GLFWwindow* window, int width, int height);

  void EnumerateJoysticks();
  void OnJoystickEvent(int id, int event);
  bool GetJoystickInput(int id, JoystickInput& input);

  bool handle_joystick_input_ = false;
  std::unordered_map<int, JoystickDevice> joysticks_;
  JoystickDeviceChangeCallback joystick_device_change_callback_;
  JoystickInput current_joystick_input_;
  JoystickInputUpdateCallback joystick_input_update_callback_;

  std::vector<std::shared_ptr<SceneObject>> scene_objects_;
  float bg_color_[4];
};
}  // namespace quickviz

#endif /* IMVIEW_VIEWER_HPP */
