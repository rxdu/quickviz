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
#include <memory>
#include <vector>

#include "imview/details/resizable_ui_node.hpp"

namespace quickviz {
class Panel : public ResizableUiNode {
 public:
  Panel(std::string name = "Panel");
  virtual ~Panel();

  // do not allow copy or move
  Panel(const Panel &other) = delete;
  Panel(Panel &&other) = delete;
  Panel &operator=(const Panel &other) = delete;
  Panel &operator=(Panel &&other) = delete;

  // public methods
  void AddRenderable(std::shared_ptr<Renderable> renderable);

  void SetPosition(float x, float y) override;
  void OnResize(float width, float height) override;
  void OnRender() override;

 protected:
  std::string name_;
  float x_ = 0;
  float y_ = 0;
  float width_ = 0;
  float height_ = 0;
  std::vector<std::shared_ptr<Renderable>> renderables_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_PANEL_HPP