/*
 * viewer_base.cpp
 *
 * Created on: Jul 27, 2021 08:59
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/viewer.hpp"

#include "fonts/opensans_regular.hpp"
#include "fonts/opensans_semibold.hpp"
#include "fonts/opensans_bold.hpp"

namespace quickviz {
Viewer::Viewer(std::string title, uint32_t width, uint32_t height,
               uint32_t window_hints)
    : Window(title, width, height, window_hints) {
  LoadFonts();
}

Viewer::~Viewer() {}

void Viewer::LoadFonts() {
  ImGuiIO &io = ImGui::GetIO();

  // default font (first loaded font)
  font_normal_ = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 20.f);

  // additional fonts
  font_tiny_ = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 16.f);
  font_small_ = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 18.f);
  font_big_ = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 28.f);
  font_large_ = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 32.f);
  font_extra_large_ = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 40.f);
}

ImFont *Viewer::GetFont(FontSize size) {
  if (size == FontSize::Tiny) {
    return font_tiny_;
  } else if (size == FontSize::Small) {
    return font_small_;
  } else if (size == FontSize::Big) {
    return font_big_;
  } else if (size == FontSize::Large) {
    return font_large_;
  } else if (size == FontSize::ExtraLarge) {
    return font_extra_large_;
  } else {
    return font_normal_;
  }
}

void Viewer::DockSpaceOverMainViewport() {
  ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
}

void Viewer::Show() {
  while (!ShouldClose()) {
    // handle events
    PollEvents();
    if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape))) {
      CloseWindow();
    }

    // draw stuff
    StartNewFrame();

    Draw();

    RenderFrame();
  }
}
}  // namespace quickviz