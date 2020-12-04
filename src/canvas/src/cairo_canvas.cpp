/*
 * cairo_canvas.cpp
 *
 * Created on: Dec 03, 2020 21:27
 * Description:
 *
 * Reference:
 * [1]
 * https://stackoverflow.com/questions/6108094/render-cairo-surface-directly-to-opengl-texture
 * [2]
 * https://stackoverflow.com/questions/36275507/how-to-speed-up-drawing-with-cairo-on-opengl-windows
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "canvas/cairo_canvas.hpp"

namespace rdu {
CairoCanvas::CairoCanvas(uint32_t width, uint32_t height, std::string title)
    : ImCanvas(width, height, title) {
  // create cairo context
  surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 640, 480);
  if (cairo_surface_status(surface) != CAIRO_STATUS_SUCCESS) {
    printf("create_cairo_context() - Couldn't create surface\n");
    return;
  }

  cr = cairo_create(surface);
  if (cairo_status(cr) != CAIRO_STATUS_SUCCESS) {
    printf("create_cairo_context() - Couldn't create context\n");
    return;
  }

  // create and setup OpenGL texture
  glGenTextures(1, &image_texture_);
  glBindTexture(GL_TEXTURE_2D, image_texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  // The following two are required on WebGL for non power-of-two textures
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

CairoCanvas::~CairoCanvas() {
  // destroy resource
  glDeleteTextures(1, &image_texture_);
  cairo_surface_destroy(surface);
  cairo_destroy(cr);
}

void CairoCanvas::SetBackgroundColor(Color color) { background_color_ = color; }

void CairoCanvas::EraseAll() {
  cairo_set_source_rgba(cr, background_color_.r, background_color_.g,
                        background_color_.b, background_color_.a);
  cairo_paint(cr);
}

void CairoCanvas::Draw() {
  // paint with Cairo
  Paint(cr);

  // convert surface to OpenGL texture
  int tex_w = cairo_image_surface_get_width(surface);
  int tex_h = cairo_image_surface_get_height(surface);
  unsigned char* data = cairo_image_surface_get_data(surface);
  glTexImage2D(GL_TEXTURE_2D, 0, 4, tex_w, tex_h, 0, GL_BGRA, GL_UNSIGNED_BYTE,
               data);

  // show on imgui
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);

  ImGui::Begin("Cairo Canvas", NULL,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
                   ImGuiWindowFlags_NoBringToFrontOnFocus |
                   ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoCollapse |
                   ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar);
  ImGui::Image((void*)(intptr_t)image_texture_, ImGui::GetContentRegionAvail());

  ImGui::End();
}
}  // namespace rdu