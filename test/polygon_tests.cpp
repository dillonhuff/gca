#include "catch.hpp"
#include "geometry/polygon.h"
#include "geometry/triangle.h"
#include "system/arena_allocator.h"

namespace gca {

  TEST_CASE("Triangles to oriented polygons") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Empty triangle vector") {
      vector<triangle> ts;
      auto res = merge_triangles(ts);
      REQUIRE(res.size() == 0);
    }

    SECTION("Real stl file") {
      auto triangles = parse_stl("/Users/dillon/gca/test/stl-files/");
    }
  }
}
