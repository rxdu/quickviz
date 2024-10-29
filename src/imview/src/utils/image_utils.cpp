/*
 * @file image_utils.cpp
 * @date 10/11/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/utils/image_utils.hpp"

namespace quickviz {
cv::Mat GenerateRandomMat(int width, int height) {
  cv::Mat random_image(height, width, CV_8UC3);

  // fill the image with random values between 0 and 255
  cv::randu(random_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

  return random_image;
}

void CopyTextureFromCvMat(const cv::Mat& mat, GLuint& texture) {
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
}  // namespace quickviz