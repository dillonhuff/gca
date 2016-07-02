#include "catch.hpp"
#include "geometry/arc.h"
#include "synthesis/circular_arc.h"
#include "synthesis/linear_cut.h"
#include "synthesis/toolpath.h"
#include "utils/arena_allocator.h"

namespace gca {

  template<typename T>
  bool operator==(const vector<T> x, const vector<T>& y) {
    if (x.size() != y.size()) { return false; }
    return equal(x.begin(), x.end(), y.begin());
  }

  TEST_CASE("Cuts to toolpaths") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<cut*> cuts;
    vector<toolpath> correct;
    vector<toolpath> actual;

    SECTION("No cuts") {
      actual = cuts_to_toolpaths(cuts);
      REQUIRE(actual.size() == 0);
    }

    SECTION("One linear cut") {
      cuts.push_back(linear_cut::make(point(0, 0, 0), point(1, 4, -2)));
      actual = cuts_to_toolpaths(cuts);

      SECTION("Result contains one toolpath") {
	REQUIRE(actual.size() == 1);
      }

      SECTION("Result starts at cut sequence start") {
	REQUIRE(actual.front().start() == point(0, 0, 0));
      }

      SECTION("Result ends at cut sequence end") {
	REQUIRE(actual.back().end() == point(1, 4, -2));
      }
    }

    SECTION("One circular arc") {
      cuts.push_back(circular_arc::make(point(0, 0, 0), point(1, 0, 0),
					point(0.5, 0, 0), CLOCKWISE, XY));
      actual = cuts_to_toolpaths(cuts);
      toolpath t = actual.back();
      parametric_curve crv = t.c;
      arc& a = crv.get_obj<arc>();
      REQUIRE(within_eps(actual.back().c.value(0.5), point(0.5, 0.5, 0)));
    }
  }
}
