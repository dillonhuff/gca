#include "catch.hpp"
#include "system/arena_allocator.h"

#include "synthesis/linear_cut.h"
#include "synthesis/toolpath.h"

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
      REQUIRE(actual == correct);
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
  }
}
