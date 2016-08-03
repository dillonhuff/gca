#include "catch.hpp"
#include "geometry/plane.h"

namespace gca {

  TEST_CASE("Plane line intersection") {
    plane p(point(0, 0, 1), point(0, 0, 0));

    SECTION("Line that does not intersect") {
      line l(point(10, 0, 0), point(11, 0, 0));
      REQUIRE(!intersection(p, l));
    }
  }
}
