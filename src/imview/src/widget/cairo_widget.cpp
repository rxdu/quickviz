/*
 * cairo_widget.cpp
 *
 * Created on: Mar 05, 2021 10:46
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/widget/cairo_widget.hpp"

#include <cmath>
#include <iostream>

#include <fontconfig/fontconfig.h>

namespace quickviz {
CairoWidget::CairoWidget(const std::string& widget_name,
                         bool normalize_coordinate)
    : Panel(widget_name), ctx_(new CairoContext(0, 0, normalize_coordinate)) {}

void CairoWidget::Draw() {
  Begin();
  Render();
  End();
}

void CairoWidget::OnResize(float width, float height) {
  Panel::OnResize(width, height);
  ctx_->Resize(width, height);
}

void CairoWidget::Fill(ImVec4 color) {
  auto cr = ctx_->GetCairoObject();
  cairo_set_source_rgba(cr, color.x, color.y, color.z, color.w);
  cairo_paint(cr);
}

void CairoWidget::Clear() {
  // Reference:
  // [1] https://www.cairographics.org/FAQ/#clear_a_surface
  auto cr = ctx_->GetCairoObject();
  cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
  cairo_paint(cr);
}

void CairoWidget::AttachDrawFunction(CairoDrawFunc DrawFunc) {
  std::lock_guard<std::mutex> lock(draw_func_mutex_);
  draw_func_ = DrawFunc;
}

void CairoWidget::DrawText(std::string text, double pos_x, double pos_y,
                           double angle, ImVec4 color, double size,
                           double line_width, const char* font) {
  auto cr = ctx_->GetCairoObject();

  cairo_save(cr);

  cairo_select_font_face(cr, font, CAIRO_FONT_SLANT_NORMAL,
                         CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size(cr, size);

  //   cairo_text_extents_t extents;
  //   cairo_text_extents(cr, text.c_str(), &extents);

  cairo_set_source_rgba(cr, color.x, color.y, color.z, color.w);
  cairo_set_line_width(cr, line_width);

  cairo_move_to(cr, pos_x, pos_y);
  cairo_show_text(cr, text.c_str());

  cairo_restore(cr);
}

void CairoWidget::Render(const ImVec2& uv0, const ImVec2& uv1,
                         const ImVec4& tint_col, const ImVec4& border_col) {
  assert(ctx_ != nullptr && ctx_->GetCairoObject() != NULL);

  // check if window size has changed
  if (width_ == 0 || height_ == 0) return;
  auto current_size = ImGui::GetWindowSize();
  if (current_size.x != width_ || current_size.y != height_) {
    OnResize(current_size.x, current_size.y);
  }

  // draw user-defined graphics
  CairoDrawFunc draw_func;
  {
    std::lock_guard<std::mutex> lock(draw_func_mutex_);
    draw_func = draw_func_;
  }
  draw_func(ctx_->GetCairoObject(), ctx_->GetAspectRatio());

  // render cairo content to OpenGL texture
  GLuint image = ctx_->RenderToGlTexture();
  ImGui::Image((void*)(intptr_t)image, ImGui::GetContentRegionAvail(), uv0, uv1,
               tint_col, border_col);
}
}  // namespace quickviz