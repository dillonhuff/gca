#include "core/arena_allocator.h"
#include "catch.hpp"
#include "geometry/b_spline.h"

namespace gca {

  TEST_CASE("Degree 1 spline test") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<point> control_points;
    control_points.push_back(point(0, 0, 0));
    control_points.push_back(point(1, 0, 0));
      
    vector<double> knots;
    knots.push_back(0.0);
    knots.push_back(0.33);
    knots.push_back(0.66);
    knots.push_back(1.0);

    int degree = 1;

    b_spline s(degree, control_points, knots);

    SECTION("Spline point 0.25") {
      REQUIRE(within_eps(s.eval(0.25), point(0, 0, 0)));
    }

    SECTION("Spline point 0.5") {
      REQUIRE(within_eps(s.eval(0.5), point(1, 0, 0)));
    }
  }

}
