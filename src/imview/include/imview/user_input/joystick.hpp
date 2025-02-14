/*
 * @file joystick.hpp
 * @date 2/13/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */
#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <string>
#include <vector>
#include <functional>

namespace quickviz {
struct JoystickDevice {
  int id = -1;
  std::string name;
};

struct JoystickInput {
  JoystickDevice device;
  std::vector<float> axes;
  std::vector<unsigned char> buttons;
  std::vector<unsigned char> hats;

  // comparison operator
  static constexpr float kEpsilon = 0.0001;
  bool operator!=(const JoystickInput& rhs) const {
    bool axes_not_equal = false;
    if (axes.size() != rhs.axes.size()) {
      axes_not_equal = true;
    } else {
      for (int i = 0; i < axes.size(); ++i) {
        if (std::abs(axes[i] - rhs.axes[i]) > kEpsilon) {
          axes_not_equal = true;
          break;
        }
      }
    }
    return device.id != rhs.device.id || axes_not_equal ||
           buttons != rhs.buttons || hats != rhs.hats;
  }
};

using JoystickDeviceChangeCallback =
    std::function<void(const std::vector<JoystickDevice>&)>;
using JoystickInputUpdateCallback = std::function<void(const JoystickInput&)>;
using JoystickInputMonitoringRegistrator =
    std::function<bool(int, JoystickInputUpdateCallback)>;
}  // namespace quickviz

#endif  // JOYSTICK_HPP
