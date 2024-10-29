/*
 * @file layer.hpp
 * @date 9/29/24
 * @brief a layer groups multiple scene objects into a logical unit and is
 *  used for event handling and rendering management
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

  // public methods
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