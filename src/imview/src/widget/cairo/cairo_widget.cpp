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
CairoWidget::CairoWidget(const std::string& widget_name, uint32_t width,
                         uint32_t height, bool normalize_coordinate)
    : Panel(widget_name),
      ctx_(new CairoContext(width, height, normalize_coordinate)) {}

CairoWidget::~CairoWidget() {
  // font-related memory cleanup
  // Reference:
  // [1]
  // https://stackoverflow.com/questions/51174295/cairo-show-text-memory-leak
  // [2] https://gitlab.freedesktop.org/cairo/cairo/-/issues/393
  cairo_debug_reset_static_data();
  FcFini();
}

void CairoWidget::Draw() {}

void CairoWidget::OnResize(float width, float height) {
  Panel::OnResize(width, height);
}

float CairoWidget::GetAspectRatio() const { return ctx_->GetAspectRatio(); }

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

void CairoWidget::Draw(CairoDrawFunc DrawFunc) {
  assert(ctx_ != nullptr && ctx_->GetCairoObject() != NULL);

  // do actual paint with cairo
  DrawFunc(ctx_->GetCairoObject());
}

void CairoWidget::Resize(uint32_t width, uint32_t height) {
  ctx_->Resize(width, height);
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
  GLuint image = ctx_->RenderToGlTexture();
  ImGui::Image((void*)(intptr_t)image, ImGui::GetContentRegionAvail(), uv0, uv1,
               tint_col, border_col);
}
}  // namespace quickviz