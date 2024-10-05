/*
 * @file container.hpp
 * @date 10/5/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_CONTAINER_HPP
#define QUICKVIZ_CONTAINER_HPP

#include <memory>

#include "imview/scene_object.hpp"

namespace quickviz {
class Container {
 public:
  virtual ~Container() = default;

  /****** public methods ******/
  virtual void AddChild(std::shared_ptr<SceneObject> obj) = 0;
  virtual void RemoveChild(const std::string& name) = 0;
};
}  // namespace quickviz

#endif  // QUICKVIZ_CONTAINER_HPP
