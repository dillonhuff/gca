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

  TEST_CASE("Extend back horizontal vector") {
    point s(1, 0, 0);
    point e(2, 0, 0);
    point sp = extend_back(s, e, 1);
    point c = point(0, 0, 0);
    REQUIRE(within_eps(sp, c));
  }

  TEST_CASE("Extend back (1, 1)") {
    point s(0, 0, 0);
    point e(1, 1, 0);
    point sp = extend_back(s, e, 1);
    double v = sqrt(1.0/2.0);
    point c = point(-v, -v, 0);
    REQUIRE(within_eps(sp,c ));
  }

  TEST_CASE("Extend back (-1, 1)") {
    point s(0, 0, 0);
    point e(-1, 1, 0);
    point sp = extend_back(s, e, 1);
    double v = sqrt(1.0/2.0);
    point c = point(v, -v, 0);
    REQUIRE(within_eps(sp,c ));
  }

  TEST_CASE("Extend back (-1, -1)") {
    point s(0, 0, 0);
    point e(-1, -1, 0);
    point sp = extend_back(s, e, 1);
    double v = sqrt(1.0/2.0);
    point c = point(v, v, 0);
    REQUIRE(within_eps(sp,c ));
  }

  TEST_CASE("Normalize (-5, 0, 0)") {
    point s(-5, 0, 0);
    point r = s.normalize();
    point c = point(-1, 0, 0);
    REQUIRE(within_eps(c, r));
  }

}
