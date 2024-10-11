/*
 * @file image_widget.cpp
 * @date 10/10/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/widget/image_widget.hpp"

#include <opencv2/opencv.hpp>

#include "imview/buffer/buffer_registry.hpp"

namespace quickviz {
namespace {
cv::Mat GenerateRandomMat(int width, int height) {
  cv::Mat randomImage(height, width, CV_8UC3);

  // Fill the image with random values between 0 and 255
  cv::randu(randomImage, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

  return randomImage;
}

void CopyTextureFromCvMat(cv::Mat& mat, GLuint& texture) {
  glBindTexture(GL_TEXTURE_2D, texture);
  {
    cv::Mat rgbMat;
    if (mat.channels() == 3) {
      cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);
    } else {
      rgbMat = mat;  // Assuming single-channel grayscale
    }
    GLenum format = (mat.channels() == 1) ? GL_RED : GL_RGB;

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rgbMat.cols, rgbMat.rows, 0, format,
                 GL_UNSIGNED_BYTE, rgbMat.ptr());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  }
  glBindTexture(GL_TEXTURE_2D, 0);
}
}  // namespace

ImageWidget::ImageWidget(std::string name) : Panel(name) {
  this->SetAutoLayout(false);
  //  this->SetNoResize(true);
  //  this->SetNoMove(true);
  this->SetWindowNoMenuButton();
  this->SetNoBackground(true);

  glGenTextures(1, &image_texture_);
}

ImageWidget::~ImageWidget() { glDeleteTextures(1, &image_texture_); }

void ImageWidget::Draw() {
  Begin();
  {
    ImVec2 contentSize = ImGui::GetContentRegionAvail();
    float width = contentSize.x;
    float height = contentSize.y;

    //    cv::Mat mat = GenerateRandomMat(width, height);
    auto& buffer_registry = BufferRegistry::GetInstance();
    auto cv_buffer = buffer_registry.GetBuffer<cv::Mat>("video_buffer");

    cv::Mat mat;
    cv_buffer->Read(mat);

    if (!mat.empty()) {
      //      cv::Mat display;
      //      cv::resize(mat, display, cv::Size(width, height), 0, 0,
      //      cv::INTER_LINEAR);
      CopyTextureFromCvMat(mat, image_texture_);
    }
    ImGui::Image((void*)(intptr_t)image_texture_, ImVec2(width, height));

    //    ImGui::Image((void*)(intptr_t)image_texture_, ImVec2(width, height));
  }
  End();
}
}  // namespace quickviz