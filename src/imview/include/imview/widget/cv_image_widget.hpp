/*
 * @file image_widget.hpp
 * @date 10/10/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_CV_IMAGE_WIDGET_HPP
#define QUICKVIZ_CV_IMAGE_WIDGET_HPP

#include "glad/glad.h"

#include <mutex>
#include <functional>

#include <opencv2/opencv.hpp>

#include "imview/panel.hpp"

namespace quickviz {
class CvImageWidget : public Panel {
 public:
  CvImageWidget(const std::string& widget_name);
  ~CvImageWidget();

  // public methods
  void SetKeepAspectRatio(bool keep);
  void UpdateImage(const cv::Mat& image);
  void Draw() override;

 private:
  std::mutex image_mutex_;
  cv::Mat image_mat_;
  GLuint image_texture_;
  bool keep_aspect_ratio_ = false;
};
}  // namespace quickviz

#endif  // QUICKVIZ_CV_IMAGE_WIDGET_HPP