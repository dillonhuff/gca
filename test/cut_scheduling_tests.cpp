#include <algorithm>
#include <vector>

#include "catch.hpp"
#include "synthesis/cut.h"
#include "synthesis/linear_cut.h"
#include "synthesis/schedule_cuts.h"

using namespace std;

namespace gca {

  bool cmp_cuts(const cut* l, const cut* r) {
    return (*l) == (*r);
  }

  TEST_CASE("Schedule lines") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<cut*> cuts;
    vector<cut*> actual;
    vector<cut*> correct;
    
    SECTION("No lines") {
      actual = schedule_cuts(cuts);
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("Several separated lines") {
      cuts.push_back(linear_cut::make(point(1, 0, -0.5), point(1, 1, -0.5)));
      cuts.push_back(linear_cut::make(point(2, 3, -0.75), point(2, 5, -0.75)));
      actual = schedule_cuts(cuts);
      correct.push_back(linear_cut::make(point(1, 0, -0.5), point(1, 1, -0.5)));
      correct.push_back(linear_cut::make(point(2, 3, -0.75), point(2, 5, -0.75)));
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("2 passes that need to be grouped") {
      cuts.push_back(linear_cut::make(point(0, 0, -0.1), point(1, 1, -0.1)));
      cuts.push_back(linear_cut::make(point(0, 0, -0.3), point(1, 1, -0.3)));
      cuts.push_back(linear_cut::make(point(1, 1, -0.1), point(1, 3, -0.1)));
      cuts.push_back(linear_cut::make(point(1, 1, -0.3), point(1, 3, -0.3)));
      actual = schedule_cuts(cuts);
      correct.push_back(linear_cut::make(point(0, 0, -0.1), point(1, 1, -0.1)));
      correct.push_back(linear_cut::make(point(1, 1, -0.1), point(1, 3, -0.1)));
      correct.push_back(linear_cut::make(point(0, 0, -0.3), point(1, 1, -0.3)));      
      correct.push_back(linear_cut::make(point(1, 1, -0.3), point(1, 3, -0.3)));
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }
  }
  
}
