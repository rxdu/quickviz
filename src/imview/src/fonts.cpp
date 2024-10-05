/*
 * @file fonts.cpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/fonts.hpp"

#include <unordered_map>

#include "fonts/opensans_regular.hpp"
#include "fonts/opensans_semibold.hpp"
#include "fonts/opensans_bold.hpp"

namespace quickviz {
namespace {
std::unordered_map<FontSize, ImFont *> custom_fonts_;
}  // namespace

void Fonts::LoadFonts() {
  ImGuiIO &io = ImGui::GetIO();

  custom_fonts_[FontSize::kFont16] = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 16.f);
  custom_fonts_[FontSize::kFont18] = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 18.f);
  custom_fonts_[FontSize::kFont20] = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 20.f);
  custom_fonts_[FontSize::kFont28] = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 28.f);
  custom_fonts_[FontSize::kFont32] = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 32.f);
  custom_fonts_[FontSize::kFont40] = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 40.f);
}

ImFont *Fonts::GetFont(FontSize size) {
  if (custom_fonts_.find(size) != custom_fonts_.end()) {
    return custom_fonts_[size];
  }
  return custom_fonts_[FontSize::kFont20];
}
}  // namespace quickviz