#include "catch.hpp"
#include "geometry/line.h"
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
      point p1(0, 0, 0);
      point p2(0, 1, 0);
      point p3(2, 1, 0);
      point p4(3, 2, 0);
      double deg = -90;
      double inc = 0.5;
      polyline p({p1, p2, p3, p4});
      auto off = offset(p, deg, inc);
      point c1(0.5, 0, 0);
      point c2(0.5, 0.5, 0);
      line l1(p2, p3);
      line l2(p3, p4);
      point i = inc*((p3 - p2).normalize().rotate_z(deg));      
      line k1(l1.start + i, l1.end + i);
      i = inc*((p4 - p3).normalize().rotate_z(deg));
      line k2(l2.start + i, l2.end + i);
      point c3 = trim_or_extend(k1, k2);
      point c4 = p4 + i;
      polyline correct({c1, c2, c3, c4});
      REQUIRE(pointwise_within_eps(off, correct, 0.00001));
    }
  }
}
