/*
 * @file box.hpp
 * @date 10/6/24
 * @brief a layer contains one or more scene objects and is responsible for
 * automatic layout and rendering of the objects
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_BOX_HPP
#define QUICKVIZ_BOX_HPP

#include <string>
#include <unordered_map>
#include <memory>

#include "imview/interface/container.hpp"
#include "imview/scene_object.hpp"

namespace quickviz {
class Box : public SceneObject, public Container {
 public:
  explicit Box(std::string name);
  virtual ~Box() = default;

  // do not allow copy or move
  Box(const Box &other) = delete;
  Box(Box &&other) = delete;
  Box &operator=(const Box &other) = delete;
  Box &operator=(Box &&other) = delete;

  // public methods
  void PrintLayout() const;

  void AddChild(std::shared_ptr<SceneObject> obj) override;
  void RemoveChild(const std::string &name) override;

  void OnResize(float width, float height) override;
  void OnRender() override;
  void OnJoystickUpdate(const JoystickInput &input) override;

  virtual void ProcessJoystickInput(const JoystickInput &input) {};

 protected:
  std::unordered_map<std::string, std::shared_ptr<SceneObject>> children_;
  std::unordered_map<size_t, std::string> child_name_by_index_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_BOX_HPP