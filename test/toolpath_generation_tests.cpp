#include "catch.hpp"
#include "synthesis/toolpath_generation.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Toolpath compression, nothing to compress") {
    vector<point> pts{point(0, 0, 0), point(1, 0, 0)};
    polyline p(pts);
    REQUIRE(compress_lines(p, 0.01).num_points() == 2);
  }

  TEST_CASE("Toolpath compression, redundant lines") {
    vector<point> pts{point(0, 0, 0),
	point(1, 1, 0),
	point(2, 2, 0),
	point(3, 3, 0)};
    polyline p(pts);
    REQUIRE(compress_lines(p, 0.001).num_points() == 2);
  }

  TEST_CASE("Box clipping") {
    arena_allocator a;
    set_system_allocator(&a);

    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);

    SECTION("Cut depth < box z") {
      box b(0, 1.2,
	    0, 1.4,
	    0, 0.5);

      vector<polyline> lines = rough_box(b, t1, 0.2);
      REQUIRE(lines.size() > 0);
    }

    SECTION("Cut depth > box z") {
      box b(0, 1.2,
	    0, 1.4,
	    0, 0.1);

      vector<polyline> lines = rough_box(b, t1, 0.2);
      REQUIRE(lines.size() > 0);
    }

  }

}
