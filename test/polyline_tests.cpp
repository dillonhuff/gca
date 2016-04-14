#include "catch.hpp"
#include "geometry/polyline.h"
#include "system/arena_allocator.h"

namespace gca {

  TEST_CASE("Offset line creation") {
    arena_allocator a;
    set_system_allocator(&a);
    
    SECTION("One segment") {
      polyline p({point(0, 0, 0), point(1, 0, 0)});
      auto off = offset(p, 90, 3.0);
      polyline correct({point(0, 3, 0), point(1, 3, 0)});
      REQUIRE(pointwise_within_eps(off, correct, 0.000001));
    }

    SECTION("Unconnected convex and concave segment") {
      polyline p({point(0, 0, 0),
	    point(0, 1, 0),
	    point(2, 1, 0),
	    point(3, 2, 0)});
      auto off = offset(p, -90, 0.5);
      polyline correct({});
      cout << "Offset " << endl;
      for (auto l : off.lines()) {
	cout << l << endl;
      }
      REQUIRE(pointwise_within_eps(off, correct, 0.00001));
    }
  }
}
