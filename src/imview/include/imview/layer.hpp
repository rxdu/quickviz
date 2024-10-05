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

#include "imview/interface/container.hpp"
#include "imview/scene_object.hpp"

namespace quickviz {
class Layer : public SceneObject, public Container {
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

  void AddChild(std::shared_ptr<SceneObject> obj);
  void RemoveChild(const std::string &name);

  void OnResize(float width, float height) override;
  void OnRender() override;

 protected:
  std::unordered_map<std::string, std::shared_ptr<SceneObject>> children_;
  std::unordered_map<size_t, std::string> child_name_by_index_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_LAYER_HPP