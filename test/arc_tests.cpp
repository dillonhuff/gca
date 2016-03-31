#include "catch.hpp"
#include "geometry/arc.h"
#include "system/arena_allocator.h"

namespace gca {

  TEST_CASE("Arc locations") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Start location") {
      arc a = arc(point(1, 0, 0), point(1, 1, 0), point(0, 0.5, 0), CLOCKWISE);
      cout << "Start location: " << a.value(0.0) << endl;
      REQUIRE(within_eps(a.value(0.0), point(1, 0, 0)));
    }
  }
}
