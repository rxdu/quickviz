/*
 * @file image_widget.hpp
 * @date 10/10/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_IMAGE_WIDGET_HPP
#define QUICKVIZ_IMAGE_WIDGET_HPP

#include "imview/panel.hpp"
//#include "imview/buffer/buffer_registry.hpp"

#include "glad/glad.h"

namespace quickviz {
//template<typename T>
class ImageWidget : public Panel {
 public:
  ImageWidget(std::string name = "ImageWidget");
  ~ImageWidget();

  void Draw() override;

 private:
  GLuint image_texture_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_IMAGE_WIDGET_HPP