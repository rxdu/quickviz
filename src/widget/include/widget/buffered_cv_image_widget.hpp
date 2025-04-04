/*
 * @file image_widget.hpp
 * @date 10/10/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_CV_IMAGE_WIDGET_HPP
#define QUICKVIZ_IMAGE_WIDGET_HPP

#include <functional>

#include "core/buffer/buffer_registry.hpp"

#include <opencv2/opencv.hpp>

#include "glad/glad.h"

#include "imview/panel.hpp"

namespace quickviz {
class BufferedCvImageWidget : public Panel {
 public:
  BufferedCvImageWidget(const std::string& widget_name,
                        const std::string& buffer_name);
  ~BufferedCvImageWidget();

  // public methods
  void SetKeepAspectRatio(bool keep);
  void Draw() override;

 private:
  GLuint image_texture_;
  std::shared_ptr<BufferInterface<cv::Mat>> buffer_;
  bool keep_aspect_ratio_ = false;
};
}  // namespace quickviz

#endif  // QUICKVIZ_CV_IMAGE_WIDGET_HPP