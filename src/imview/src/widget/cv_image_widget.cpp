/*
 * @file cv_image_widget.cpp
 * @date 10/25/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/widget/cv_image_widget.hpp"

#include "imview/utils/image_utils.hpp"

namespace quickviz {
CvImageWidget::CvImageWidget(const std::string& widget_name)
    : Panel(widget_name) {
  this->SetAutoLayout(false);
  //  this->SetNoResize(true);
  //  this->SetNoMove(true);
  this->SetWindowNoMenuButton();
  this->SetNoBackground(true);

  glGenTextures(1, &image_texture_);
}

CvImageWidget::~CvImageWidget() { glDeleteTextures(1, &image_texture_); }

void CvImageWidget::SetKeepAspectRatio(bool keep) { keep_aspect_ratio_ = keep; }

void CvImageWidget::UpdateImage(const cv::Mat& image) {
  std::lock_guard<std::mutex> lock(image_mutex_);
  image_mat_ = image.clone();
}

void CvImageWidget::Draw() {
  Begin();
  {
    ImVec2 contentSize = ImGui::GetContentRegionAvail();
    float width = contentSize.x;
    float height = contentSize.y;

    cv::Mat mat;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      mat = image_mat_.clone();
    }

    if (!mat.empty()) {
      if (keep_aspect_ratio_) {
        cv::Mat proc = mat;
        float aspect_ratio = mat.cols / (float)mat.rows;
        float img_width = width;
        float img_height = height;
        if (width / height > aspect_ratio) {
          img_width = height * aspect_ratio;
        } else {
          img_height = width / aspect_ratio;
        }
        if (img_width > width) {
          // need to scale down
          img_width = width;
          img_height = width / aspect_ratio;
        } else {
          // need to scale down
          img_height = height;
          img_width = height * aspect_ratio;
        }
        cv::resize(mat, proc, cv::Size(img_width, img_height), 0, 0,
                   cv::INTER_CUBIC);
        // copy display to center of image
        cv::Mat display = cv::Mat::zeros(height, width, mat.type());
        cv::Rect roi((width - proc.cols) / 2, (height - proc.rows) / 2,
                     proc.cols, proc.rows);
        proc.copyTo(display(roi));
        CopyTextureFromCvMat(display, image_texture_);
      } else {
        CopyTextureFromCvMat(mat, image_texture_);
      }
    }
    ImGui::Image((void*)(intptr_t)image_texture_, ImVec2(width, height));
  }
  End();
}
}  // namespace quickviz