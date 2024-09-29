/*
 * @file resizable.hpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_RESIZABLE_HPP
#define QUICKVIZ_RESIZABLE_HPP

namespace quickviz {
class Resizable {
  struct Size {
    int x;
    int y;
    int width;
    int height;
  };

 public:
  virtual void OnResize(int width, int height) = 0;
};
}  // namespace quickviz

#endif  // QUICKVIZ_RESIZABLE_HPP
