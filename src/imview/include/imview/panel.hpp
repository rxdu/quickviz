/*
 * @file panel.hpp
 * @date 9/29/24
 * @brief a panel is defined as a region that contains one or more renderable
 * objects within a window
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_PANEL_HPP
#define QUICKVIZ_PANEL_HPP

#include <string>

#include <yoga/Yoga.h>

#include "imview/interface/resizable.hpp"

namespace quickviz {
class Panel : public Resizable {
 public:
  Panel(std::string name = "Panel");
  virtual ~Panel();

  // do not allow copy or move
  Panel(const Panel &other) = delete;
  Panel(Panel &&other) = delete;
  Panel &operator=(const Panel &other) = delete;
  Panel &operator=(Panel &&other) = delete;

  // public methods
  void OnResize(int width, int height) override;

 private:
  std::string name_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_PANEL_HPP