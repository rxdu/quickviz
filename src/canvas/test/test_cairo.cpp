#include <stdio.h>
#include <string.h>
#include <cairo.h>
#include <cairo-svg.h>

//#define SVG1_2		1

void gen_page_1(cairo_t* cr) {
  cairo_move_to(cr, 10.0, 50.0);
  cairo_show_text(cr, "Michael A. Uman");

  cairo_set_line_width(cr, 1.0);

  cairo_rectangle(cr, 20.0, 60.0, 430.0, 430.0);
  cairo_stroke(cr);

  cairo_show_page(cr);

  return;
}

void gen_page_2(cairo_t* cr) {
  cairo_move_to(cr, 10.0, 50.0);
  cairo_show_text(cr, "Page #2");

  cairo_show_page(cr);

  return;
}

int main(int argc, const char* argv[]) {
  cairo_surface_t* surface = 0L;
  cairo_t* cr = 0L;
  cairo_svg_version_t ver = CAIRO_SVG_VERSION_1_1;

  if (argc != 3) {
    fprintf(stderr, "%s [svg_version] [output file]\n", argv[0]);
    fprintf(stderr, "\tsvg version = 1_1 or 1_2\n");
    return -1;
  }

  /* Check the SVG version # passed in arg 1 */
  if (strcmp("1_1", argv[1]) == 0) {
    ver = CAIRO_SVG_VERSION_1_1;
  } else if (strcmp("1_2", argv[1]) == 0) {
    ver = CAIRO_SVG_VERSION_1_2;
  } else {
    fprintf(stderr, "ERROR: SVG version must be 1_1 or 1_2!\n");
    return -1;
  }

  surface = cairo_svg_surface_create(argv[2], 612, 792);
  cairo_svg_surface_restrict_to_version(surface, ver);

  cr = cairo_create(surface);

  cairo_set_source_rgb(cr, 0, 0, 0);
  cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL,
                         CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size(cr, 40.0);

  gen_page_1(cr);

  if (ver == CAIRO_SVG_VERSION_1_2) gen_page_2(cr);

  cairo_surface_destroy(surface);
  cairo_destroy(cr);

  return 0;
}