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

  /****** public methods ******/
  virtual bool IsVisible() const = 0;
  virtual void OnRender() = 0;
};
}  // namespace quickviz

#endif  // QUICKVIZ_RENDERABLE_HPP
