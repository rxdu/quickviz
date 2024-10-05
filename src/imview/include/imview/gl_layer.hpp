/*
 * @file layer.hpp
 * @date 9/29/24
 * @brief a layer contains one or more panel objects and is responsible for
 * automatic layout and rendering of the panels
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_GL_LAYER_HPP
#define QUICKVIZ_GL_LAYER_HPP

#include "imview/layer.hpp"

namespace quickviz {
class GlLayer : public Layer {
 public:
  GlLayer(std::string name = "GlLayer");
  virtual ~GlLayer() = default;

  // public methods
  void OnResize(float width, float height) override;
};
}  // namespace quickviz

#endif  // QUICKVIZ_GL_LAYER_HPP