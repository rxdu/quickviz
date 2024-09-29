/*
 * @file layer.hpp
 * @date 9/29/24
 * @brief a layer contains one or more panel objects and is responsible for
 * automatic layout update and rendering of the panels
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_LAYER_HPP
#define QUICKVIZ_LAYER_HPP

#include <string>
#include <vector>

#include <yoga/Yoga.h>

#include "imview/interface/renderable.hpp"
#include "imview/interface/resizable.hpp"
#include "imview/panel.hpp"

namespace quickviz {
class Layer : public Renderable, public Resizable {
 public:
  Layer(std::string name = "Layer");
  virtual ~Layer();

  // do not allow copy or move
  Layer(const Layer &other) = delete;
  Layer(Layer &&other) = delete;
  Layer &operator=(const Layer &other) = delete;
  Layer &operator=(Layer &&other) = delete;

  // public methods
  void AddPanel(Panel *panel);
  void OnResize(int width, int height) override;
  void OnRender() override;

 private:
  std::string name_;
  YGNodeRef root_node_;
  std::vector<Panel *> panels_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_LAYER_HPP