/*
 * @file image_widget.hpp
 * @date 10/10/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_IMAGE_WIDGET_HPP
#define QUICKVIZ_IMAGE_WIDGET_HPP

#include <functional>

#include "imview/panel.hpp"
#include "imview/buffer/buffer_registry.hpp"

#include "glad/glad.h"

namespace quickviz {
template <typename T>
class ImageWidget : public Panel {
 public:
  using CopyTextureFunc = std::function<void(const T&, GLuint&)>;
  ImageWidget(std::string widget_name, const std::string& buffer_name,
              CopyTextureFunc func)
      : Panel(widget_name), copy_texture_func_(func) {
    this->SetAutoLayout(false);
    //  this->SetNoResize(true);
    //  this->SetNoMove(true);
    this->SetWindowNoMenuButton();
    this->SetNoBackground(true);

    auto& buffer_registry = BufferRegistry::GetInstance();
    buffer_ = buffer_registry.GetBuffer<T>(buffer_name);

    glGenTextures(1, &image_texture_);
  }

  ~ImageWidget() { glDeleteTextures(1, &image_texture_); }

  void Draw() override {
    Begin();
    {
      ImVec2 contentSize = ImGui::GetContentRegionAvail();
      float width = contentSize.x;
      float height = contentSize.y;

      T mat;
      buffer_->Read(mat);

      if (!mat.empty()) {
        //      cv::Mat display;
        //      cv::resize(mat, display, cv::Size(width, height), 0, 0,
        //      cv::INTER_LINEAR);
        copy_texture_func_(mat, image_texture_);
      }
      ImGui::Image((void*)(intptr_t)image_texture_, ImVec2(width, height));
    }
    End();
  }

 private:
  GLuint image_texture_;
  std::shared_ptr<BufferInterface<T>> buffer_;
  CopyTextureFunc copy_texture_func_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_IMAGE_WIDGET_HPP