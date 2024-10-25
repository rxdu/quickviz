/*
 * cairo_widget.hpp
 *
 * Created on: Mar 05, 2021 10:42
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef CAIRO_WIDGET_HPP
#define CAIRO_WIDGET_HPP

#include <cmath>
#include <string>
#include <cstdint>
#include <memory>
#include <functional>
#include <unordered_map>

#include "imgui.h"

#include "imview/panel.hpp"
#include "imview/widget/cairo/cairo_context.hpp"

namespace quickviz {
class CairoWidget : public Panel {
 public:
  CairoWidget(const std::string& widget_name, uint32_t width, uint32_t height,
              bool normalize_coordinate = false);
  ~CairoWidget() override;

  void Draw() override;
  void OnResize(float width, float height) override;

  float GetAspectRatio() const;

  // resize/fill cairo surface
  void Resize(uint32_t width, uint32_t height);
  void Fill(ImVec4 color = {1, 1, 1, 0.6});
  void Clear();

  // draw vector graphics with user function
  using CairoDrawFunc = std::function<void(cairo_t*)>;
  void Draw(CairoDrawFunc DrawFunc);

  // draw text to cairo surface
  void DrawText(std::string text, double pos_x, double pos_y,
                double angle = 0.0, ImVec4 color = {0, 0, 0, 1},
                double size = 14.0, double line_width = 3.0,
                const char* font = "Sans");

  // render cairo content to OpenGL context to display with ImGUI
  void Render(const ImVec2& uv0 = ImVec2(0, 0),
              const ImVec2& uv1 = ImVec2(1, 1),
              const ImVec4& tint_col = ImVec4(1, 1, 1, 1),
              const ImVec4& border_col = ImVec4(0, 0, 0, 0));

 private:
  std::unique_ptr<CairoContext> ctx_;
};
}  // namespace quickviz

#endif /* CAIRO_WIDGET_HPP */
