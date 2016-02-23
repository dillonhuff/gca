#include "catch.hpp"
#include "synthesis/circular_arc.h"

namespace gca {

  TEST_CASE("Orientation tests") {
    arena_allocator a;
    set_system_allocator(&a);
    
    double x_init = 0;
    double y_init = 0;
    double letter_width = 0.5;
    
    double xi = x_init;
    double xm = x_init + (letter_width / 2.0);
    double xl = x_init + letter_width;

    double yi = y_init;
    double ym = y_init - (letter_width / 2.0);
    double yl = y_init - letter_width;

    point p0 = point(xi, yi, 0);
    point p1 = point(xm, yi, 0);
    point p2 = point(xl, yi, 0);
  
    point p3 = point(xi, ym, 0);
    point p4 = point(xm, ym, 0);
    point p5 = point(xl, ym, 0);
  
    point p6 = point(xi, yl, 0);
    point p7 = point(xm, yl, 0);
    point p8 = point(xl, yl, 0);

    circular_arc* arc;
    point actual, correct;
    
    SECTION("Clockwise") {
      arc = circular_arc::make(p1, p7, p4 - p1, CLOCKWISE, XY);
      actual = arc->initial_orient();
      correct = (p2 - p1).normalize();
      REQUIRE(within_eps(actual, correct));
    }

    SECTION("Counterclockwise") {
      arc = circular_arc::make(p1, p7, p4 - p1, COUNTERCLOCKWISE, XY);
      actual = arc->initial_orient();
      correct = (p0 - p1).normalize();
      REQUIRE(within_eps(actual, correct));
    }
  }
}
