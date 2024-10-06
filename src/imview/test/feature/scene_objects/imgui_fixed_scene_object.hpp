/*
 * @file imgui_fixed_scene_object.hpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_IMGUI_FIXED_SCENE_OBJECT_HPP
#define QUICKVIZ_IMGUI_FIXED_SCENE_OBJECT_HPP

#include "imview/viewer.hpp"
#include "imview/scene_object.hpp"

namespace quickviz {
class ImGuiFixedSceneObject : public SceneObject {
 public:
  ImGuiFixedSceneObject(std::string name = "ImGuiFixedPanel")
      : SceneObject(name) {}

  void OnRender() override {
    // clang-format off
//    std::cout << "Window " << name_ <<": " << x_ << ", " << y_ << ", " << width_ << ", "
//              << height_ << std::endl;
    // clang-format on
    ImGui::SetNextWindowPos(ImVec2(x_, y_));
    ImGui::SetNextWindowSize(ImVec2(width_, height_));
    ImGui::Begin(name_.c_str(), NULL,
                 ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoBringToFrontOnFocus |
                     ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoBackground);

    // ui elements of the panel
    {
      ImGui::PushFont(Fonts::GetFont(FontSize::kFont18));
      ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 153, 153, 200));
      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                  1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGui::PopStyleColor();
      ImGui::PopFont();
    }

    ImGui::End();
  }
};
}  // namespace quickviz

#endif  // QUICKVIZ_IMGUI_FIXED_SCENE_OBJECT_HPP