/*
 * @file layer.hpp
 * @date 9/29/24
 * @brief a layer contains one or more panel objects and is responsible for
 * automatic layout and rendering of the panels
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_LAYER_HPP
#define QUICKVIZ_LAYER_HPP

#include <string>
#include <unordered_map>
#include <memory>

#include "imview/scene_object.hpp"
#include "imview/panel.hpp"

namespace quickviz {
class Layer : public SceneObject {
 public:
  explicit Layer(std::string name);
  virtual ~Layer() = default;

  // do not allow copy or move
  Layer(const Layer &other) = delete;
  Layer(Layer &&other) = delete;
  Layer &operator=(const Layer &other) = delete;
  Layer &operator=(Layer &&other) = delete;

  // public methods
  void PrintLayout() const;

  void AddResizableUiNode(std::shared_ptr<SceneObject> resizable);
  void AddRenderable(std::shared_ptr<Renderable> renderable);

  void OnResize(float width, float height) override;
  void OnRender() override;

 protected:
  std::unordered_map<size_t, std::shared_ptr<SceneObject>> resizables_;
  std::vector<std::shared_ptr<Renderable>> renderables_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_LAYER_HPP