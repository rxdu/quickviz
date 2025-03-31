/*
 * @file image_utils.hpp
 * @date 10/11/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_IMAGE_UTILS_HPP
#define QUICKVIZ_IMAGE_UTILS_HPP

#include <glad/glad.h>
#include <opencv2/opencv.hpp>

namespace quickviz {
cv::Mat GenerateRandomMat(int width, int height);
void CopyTextureFromCvMat(const cv::Mat& mat, GLuint& texture);
}  // namespace quickviz

#endif  // QUICKVIZ_IMAGE_UTILS_HPP