/**
 * https://stackoverflow.com/questions/6108094/render-cairo-surface-directly-to-opengl-texture
 * https://stackoverflow.com/questions/36275507/how-to-speed-up-drawing-with-cairo-on-opengl-windows
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <cairo.h>
#include <cairo-svg.h>

#include "canvas/im_canvas.hpp"

using namespace rdu;

struct CairoCanvas : public ImCanvas {
  GLuint image_texture;
  CairoCanvas() { glGenTextures(1, &image_texture); }

  ~CairoCanvas() { glDeleteTextures(1, &image_texture); }

  void Draw() override {
    // cairo_surface_t* surface = 0L;
    // cairo_t* cr = 0L;

    // // surface = cairo_svg_surface_create("sample.svg", 612, 792);
    // surface = cairo_svg_surface_create("sample.svg", 612, 792);
    // cairo_svg_surface_restrict_to_version(surface, CAIRO_SVG_VERSION_1_2);

    // cr = cairo_create(surface);

    // cairo_set_source_rgb(cr, 0, 0, 0);
    // cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL,
    //                        CAIRO_FONT_WEIGHT_NORMAL);
    // cairo_set_font_size(cr, 40.0);

    // //----------------------//
    // cairo_move_to(cr, 10.0, 50.0);
    // cairo_show_text(cr, "Sample Text");

    // cairo_set_line_width(cr, 1.0);

    // cairo_rectangle(cr, 20.0, 60.0, 430.0, 430.0);
    // cairo_stroke(cr);

    // cairo_show_page(cr);
    // //----------------------//

    // cairo_surface_destroy(surface);
    // cairo_destroy(cr);

    // create context
    cairo_surface_t* surface = 0L;
    cairo_t* cr = 0L;

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

    // more setup
    // cairo_set_source_rgb(cr, 0, 0, 0);
    // cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL,
    //                        CAIRO_FONT_WEIGHT_NORMAL);
    // cairo_set_font_size(cr, 40.0);

    // //----------------------//
    // cairo_move_to(cr, 10.0, 50.0);
    // cairo_show_text(cr, "Sample Text");

    // cairo_set_line_width(cr, 1.0);

    // cairo_rectangle(cr, 20.0, 60.0, 430.0, 430.0);
    // cairo_stroke(cr);

    // cairo_show_page(cr);
    //----------------------//

    /**/
    double xc = 128.0;
    double yc = 128.0;
    double radius = 100.0;
    double angle1 = 45.0 * (M_PI / 180.0);  /* angles are specified */
    double angle2 = 180.0 * (M_PI / 180.0); /* in radians           */

    cairo_set_line_width(cr, 10.0);
    cairo_arc(cr, xc, yc, radius, angle1, angle2);
    cairo_stroke(cr);

    /* draw helping lines */
    cairo_set_source_rgba(cr, 1, 0.2, 0.2, 0.6);
    cairo_set_line_width(cr, 6.0);

    cairo_arc(cr, xc, yc, 10.0, 0, 2 * M_PI);
    cairo_fill(cr);

    cairo_arc(cr, xc, yc, radius, angle1, angle1);
    cairo_line_to(cr, xc, yc);
    cairo_arc(cr, xc, yc, radius, angle2, angle2);
    cairo_line_to(cr, xc, yc);
    cairo_stroke(cr);
    /**/

    glBindTexture(GL_TEXTURE_2D, image_texture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,
                    GL_CLAMP_TO_EDGE);  // This is required on WebGL for non
                                        // power-of-two textures
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,
                    GL_CLAMP_TO_EDGE);  // Same

    int tex_w = cairo_image_surface_get_width(surface);
    int tex_h = cairo_image_surface_get_height(surface);
    unsigned char* data = cairo_image_surface_get_data(surface);
    glTexImage2D(GL_TEXTURE_2D, 0, 4, tex_w, tex_h, 0, GL_BGRA,
                 GL_UNSIGNED_BYTE, data);

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);

    // ImGui::Begin("OpenGL Texture Text", NULL,
    //              ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
    //                  ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
    //                  ImGuiWindowFlags_NoScrollWithMouse);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);

    ImGui::Begin("Canvas", NULL,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
                     ImGuiWindowFlags_NoBringToFrontOnFocus |
                     ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar);
    // ImGui::Image((void*)(intptr_t)image_texture, ImVec2(640, 480));
    ImGui::Image((void*)(intptr_t)image_texture,
                 ImGui::GetContentRegionAvail());

    ImGui::End();

    cairo_surface_destroy(surface);
    cairo_destroy(cr);
  }
};

int main(int argc, const char* argv[]) {
  CairoCanvas canvas;
  canvas.Show();

  return 0;
}