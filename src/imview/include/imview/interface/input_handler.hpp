/*
 * @file input_handler.hpp
 * @date 2/14/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef INPUT_HANDLER_HPP
#define INPUT_HANDLER_HPP

#include "imview/user_input/joystick.hpp"

namespace quickviz {
class InputHandler {
 public:
  enum class Type {
    kKeyboard,
    kMouse,
    kJoystick,
  };

  enum class Strategy {
    kNone,
    kProcessOnly,
    kPropagateOnly,
    kProcessAndPropagate,
  };

 public:
  virtual ~InputHandler() = default;

  /****** public methods ******/
  // common methods
  virtual void SetInputHandlingStrategy(Type type, Strategy strategy) = 0;

  // TODO (rdu): keyboard and mouse input handling is not implemented yet
  virtual void OnKeyPress(int key, int scancode, int action, int mods) {};
  virtual void OnMouseMove(double xpos, double ypos) {};
  virtual void OnMouseButton(int button, int action, int mods) {};
  virtual void OnMouseScroll(double xoffset, double yoffset) {};

  virtual void OnJoystickDeviceChange(
      const std::vector<JoystickDevice>& devices) = 0;
  virtual void OnJoystickUpdate(const JoystickInput& input) = 0;
};
}  // namespace quickviz

#endif  // INPUT_HANDLER_HPP
