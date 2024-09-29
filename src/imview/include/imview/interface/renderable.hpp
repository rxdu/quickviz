/*
 * @file renderable.hpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */
#ifndef QUICKVIZ_RENDERABLE_HPP
#define QUICKVIZ_RENDERABLE_HPP

namespace quickviz {
class Renderable {
 public:
  virtual ~Renderable() = default;

  // public methods
  void SetVisible(bool visible) { visible_ = visible; }
  bool IsVisible() const { return visible_; }
  bool IsContainer() const { return is_container_; }

  virtual void OnRender() = 0;

 protected:
  bool visible_ = true;
  bool is_container_ = false;
};
}  // namespace quickviz

#endif  // QUICKVIZ_RENDERABLE_HPP
