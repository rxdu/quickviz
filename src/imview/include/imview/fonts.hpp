/*
 * @file fonts.hpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_FONTS_HPP
#define QUICKVIZ_FONTS_HPP

#include "imgui.h"

namespace quickviz {
// enum class FontSize { Tiny, Small, Normal, Big, Large, ExtraLarge };
enum class FontSize {
  kDefault = 0,
  kFont16 = 16,
  kFont18 = 18,
  kFont20 = 20,
  kFont24 = 24,
  kFont28 = 28,
  kFont32 = 32,
  kFont40 = 40
};

class Fonts {
 public:
  static void LoadFonts();
  static void UnloadFonts();
  static ImFont *GetFont(FontSize size);
};
}  // namespace quickviz

#endif  // QUICKVIZ_FONTS_HPP