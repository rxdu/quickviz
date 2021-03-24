/*
 * test_cairo_canvas.cpp
 *
 * Created on: Dec 03, 2020 21:27
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include <memory>

#include "canvas/im_canvas.hpp"
#include "canvas/cairo_context.hpp"

using namespace rdu;

struct DrawArc : public ImCanvas {
  DrawArc() {
    ctx1_ = std::make_shared<CairoContext>(320, 240);
    ctx2_ = std::make_shared<CairoContext>(320, 240, true);
  }

  std::shared_ptr<CairoContext> ctx1_;
  std::shared_ptr<CairoContext> ctx2_;

  void Draw() override {
    if (!ctx1_->Initialized()) return;
    if (!ctx2_->Initialized()) return;

    ImVec2 panel_size = {ImGui::GetIO().DisplaySize.x / 2.0f,
                         ImGui::GetIO().DisplaySize.y / 2.0f};

    // show on imgui
    {
      ImGui::SetNextWindowPos(ImVec2(0, 0));
      ImGui::SetNextWindowSize(panel_size);

      ImGui::Begin("Cairo Canvas 1", NULL,
                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
                       ImGuiWindowFlags_NoBringToFrontOnFocus |
                       ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoCollapse |
                       ImGuiWindowFlags_NoResize |
                       ImGuiWindowFlags_NoScrollbar);

      // do paint with cairo
      Paint(ctx1_->GetCairoObject());

      GLuint image = ctx1_->RenderToGlTexture();
      ImGui::Image((void*)(intptr_t)image, ImGui::GetContentRegionAvail());

      ImGui::End();
    }

    {
      ImGui::SetNextWindowPos(ImVec2(panel_size.x, 0));
      ImGui::SetNextWindowSize(panel_size);

      ImGui::Begin("Cairo Canvas 2", NULL,
                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
                       ImGuiWindowFlags_NoBringToFrontOnFocus |
                       ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoCollapse |
                       ImGuiWindowFlags_NoResize |
                       ImGuiWindowFlags_NoScrollbar);

      // do paint with cairo
      ctx2_->PushScale(0.5, 0.5);
      ScaledPaint(ctx2_->GetCairoObject());
      ctx2_->PushScale(0.5, 0.5);
      ScaledPaint(ctx2_->GetCairoObject());
      ctx2_->PopScale();
      ctx2_->PopScale();

      GLuint image = ctx2_->RenderToGlTexture();
      ImGui::Image((void*)(intptr_t)image, ImGui::GetContentRegionAvail());

      ImGui::End();
    }
  }

  void Paint(cairo_t* cr) {
    cairo_set_source_rgba(cr, background_color_.x, background_color_.y,
                          background_color_.z, background_color_.w);
    cairo_paint(cr);

    double x = 25.6, y = 128.0;
    double x1 = 102.4, y1 = 230.4, x2 = 153.6, y2 = 25.6, x3 = 230.4,
           y3 = 128.0;

    cairo_set_source_rgba(cr, 0.2, 0.2, 0.2, 0.6);

    cairo_move_to(cr, x, y);
    cairo_curve_to(cr, x1, y1, x2, y2, x3, y3);

    cairo_set_line_width(cr, 10.0);
    cairo_stroke(cr);

    cairo_set_source_rgba(cr, 1, 0.2, 0.2, 0.6);
    cairo_set_line_width(cr, 6.0);
    cairo_move_to(cr, x, y);
    cairo_line_to(cr, x1, y1);
    cairo_move_to(cr, x2, y2);
    cairo_line_to(cr, x3, y3);
    cairo_stroke(cr);
  }

  void ScaledPaint(cairo_t* cr) {
    cairo_set_source_rgba(cr, background_color_.x, background_color_.y,
                          background_color_.z, background_color_.w);
    cairo_paint(cr);

    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_move_to(cr, 0, 0);
    cairo_line_to(cr, 1, 1);
    cairo_move_to(cr, 1, 0);
    cairo_line_to(cr, 0, 1);
    cairo_set_line_width(cr, 0.2);
    cairo_stroke(cr);

    cairo_rectangle(cr, 0, 0, 0.5, 0.5);
    cairo_set_source_rgba(cr, 1, 0, 0, 0.80);
    cairo_fill(cr);

    cairo_rectangle(cr, 0, 0.5, 0.5, 0.5);
    cairo_set_source_rgba(cr, 0, 1, 0, 0.60);
    cairo_fill(cr);

    cairo_rectangle(cr, 0.5, 0, 0.5, 0.5);
    cairo_set_source_rgba(cr, 0, 0, 1, 0.40);
    cairo_fill(cr);
  }
};

int main(int argc, const char* argv[]) {
  DrawArc cc;
  cc.SetBackgroundColor({0.1, 0.3, 0.5, 0.2});
  cc.Show();

  return 0;
}