#include <stdio.h>
#include <string.h>
#include <cairo.h>
#include <cairo-svg.h>

//#define SVG1_2		1

void gen_page_1(cairo_t* cr) {
  cairo_move_to(cr, 10.0, 50.0);
  cairo_show_text(cr, "Sample Text");

  cairo_set_line_width(cr, 1.0);

  cairo_rectangle(cr, 20.0, 60.0, 430.0, 430.0);
  cairo_stroke(cr);

  cairo_show_page(cr);

  return;
}

int main(int argc, const char* argv[]) {
  cairo_surface_t* surface = 0L;
  cairo_t* cr = 0L;
  cairo_svg_version_t ver = CAIRO_SVG_VERSION_1_1;

  //   ver = CAIRO_SVG_VERSION_1_1;
  ver = CAIRO_SVG_VERSION_1_2;

  surface = cairo_svg_surface_create("sample.svg", 612, 792);
  cairo_svg_surface_restrict_to_version(surface, ver);

  cr = cairo_create(surface);

  cairo_set_source_rgb(cr, 0, 0, 0);
  cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL,
                         CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size(cr, 40.0);

  gen_page_1(cr);

  cairo_surface_destroy(surface);
  cairo_destroy(cr);

  return 0;
}