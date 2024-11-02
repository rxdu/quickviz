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
#include <mutex>
#include <functional>
#include <unordered_map>

#include "imgui.h"

#include "imview/panel.hpp"
#include "imview/component/cairo_context.hpp"
#include "imview/component/cairo_draw.hpp"

namespace quickviz {
class CairoWidget : public Panel {
 public:
  CairoWidget(const std::string& widget_name,
              bool normalize_coordinate = false);
  ~CairoWidget() override = default;

  void Draw() override;
  void OnResize(float width, float height) override;

  // draw vector graphics with user function
  using CairoDrawFunc = std::function<void(cairo_t*, float aspect_ratio)>;
  void AttachDrawFunction(CairoDrawFunc DrawFunc);

 private:
  void Fill(ImVec4 color = {1, 1, 1, 0.6});
  void Clear();

  void DrawText(std::string text, double pos_x, double pos_y,
                double angle = 0.0, ImVec4 color = {0, 0, 0, 1},
                double size = 14.0, double line_width = 3.0,
                const char* font = "Sans");

  void Render(const ImVec2& uv0 = ImVec2(0, 0),
              const ImVec2& uv1 = ImVec2(1, 1),
              const ImVec4& tint_col = ImVec4(1, 1, 1, 1),
              const ImVec4& border_col = ImVec4(0, 0, 0, 0));

  std::unique_ptr<CairoContext> ctx_;
  std::mutex draw_func_mutex_;
  CairoDrawFunc draw_func_;
};
}  // namespace quickviz

#endif /* CAIRO_WIDGET_HPP */
