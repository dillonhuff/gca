#include "catch.hpp"
#include "geometry/rotation.h"

namespace gca {

  TEST_CASE("Random rotation") {
    point from(0.90, -0.32, 0);
    point to(0, 0, 1);

    const rotation r = rotate_from_to(from, to);

    REQUIRE(within_eps(determinant(r), 1.0, 0.001));
  }
  
  TEST_CASE("Irregular 90 degree rotation") {
    point from(0.953399, -0.301714, -0);
    point to(0, 0, 1);

    const rotation r = rotate_from_to(from, to);

    REQUIRE(within_eps(determinant(r), 1.0, 0.001));
  }
  
}
