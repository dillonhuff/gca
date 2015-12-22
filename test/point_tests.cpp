#include "catch.hpp"
#include "point.h"

namespace gca {

  TEST_CASE("Rotate XY 90 degrees counterclockwise") {
    point p(0, 1, 0);
    point r = p.rotate_z(90);
    point c(-1, 0, 0);
    REQUIRE(within_eps(r, c));
  }

  TEST_CASE("Rotate XY 90 degrees clockwise") {
    point p(1, 0, 0);
    point r = p.rotate_z(-90);
    point c(0, -1, 0);
    REQUIRE(within_eps(r, c));
  }
  
}
