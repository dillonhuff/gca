#include <algorithm>
#include <vector>

#include "catch.hpp"
#include "gcode/circular_arc.h"
#include "gcode/cut.h"
#include "gcode/hole_punch.h"
#include "gcode/linear_cut.h"
#include "synthesis/schedule_cuts.h"

using namespace std;

namespace gca {

  TEST_CASE("Schedule cuts") {
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
      cuts.push_back(linear_cut::make(point(1, 0, -0.5), point(1, 1, -0.5), DRILL));
      cuts.push_back(linear_cut::make(point(2, 3, -0.75), point(2, 5, -0.75), DRILL));
      actual = schedule_cuts(cuts);
      correct.push_back(linear_cut::make(point(1, 0, -0.5), point(1, 1, -0.5), DRILL));
      correct.push_back(linear_cut::make(point(2, 3, -0.75), point(2, 5, -0.75), DRILL));
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("2 passes that need to be grouped") {
      cuts.push_back(linear_cut::make(point(0, 0, -0.1), point(1, 1, -0.1), DRILL));
      cuts.push_back(linear_cut::make(point(0, 0, -0.3), point(1, 1, -0.3), DRILL));
      cuts.push_back(linear_cut::make(point(1, 1, -0.1), point(1, 3, -0.1), DRILL));
      cuts.push_back(linear_cut::make(point(1, 1, -0.3), point(1, 3, -0.3), DRILL));
      actual = schedule_cuts(cuts);
      correct.push_back(linear_cut::make(point(0, 0, -0.1), point(1, 1, -0.1), DRILL));
      correct.push_back(linear_cut::make(point(1, 1, -0.1), point(1, 3, -0.1), DRILL));
      correct.push_back(linear_cut::make(point(0, 0, -0.3), point(1, 1, -0.3), DRILL));
      correct.push_back(linear_cut::make(point(1, 1, -0.3), point(1, 3, -0.3), DRILL));
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("2 passes where the cuts are not adjacent") {
      cuts.push_back(linear_cut::make(point(0, 0, -0.1), point(1, 1, -0.1), DRILL));
      cuts.push_back(linear_cut::make(point(0, 0, -0.3), point(1, 1, -0.3), DRILL));
      cuts.push_back(linear_cut::make(point(2, 3, -0.1), point(3, 5, -0.1), DRILL));
      cuts.push_back(linear_cut::make(point(2, 3, -0.3), point(3, 5, -0.3), DRILL));
      actual = schedule_cuts(cuts);
      correct.push_back(linear_cut::make(point(0, 0, -0.1), point(1, 1, -0.1), DRILL));
      correct.push_back(linear_cut::make(point(0, 0, -0.3), point(1, 1, -0.3), DRILL));
      correct.push_back(linear_cut::make(point(2, 3, -0.1), point(3, 5, -0.1), DRILL));
      correct.push_back(linear_cut::make(point(2, 3, -0.3), point(3, 5, -0.3), DRILL));
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("Holes first") {
      cuts.push_back(linear_cut::make(point(1, 1, -0.1), point(2, 2, -0.2), DRILL));
      cuts.push_back(hole_punch::make(point(3, 3, -0.1), 0.125, DRILL));
      actual = schedule_cuts(cuts);
      correct.push_back(hole_punch::make(point(3, 3, -0.1), 0.125, DRILL));
      correct.push_back(linear_cut::make(point(1, 1, -0.1), point(2, 2, -0.2), DRILL));
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("Cuts with different tools are not adjacent") {
      cuts.push_back(linear_cut::make(point(0, 0, -0.1), point(1, 1, -0.1), DRILL));
      cuts.push_back(linear_cut::make(point(0, 0, -0.3), point(1, 1, -0.3), DRILL));
      cuts.push_back(linear_cut::make(point(1, 1, -0.1), point(1, 3, -0.1), DRAG_KNIFE));
      cuts.push_back(linear_cut::make(point(1, 1, -0.3), point(1, 3, -0.3), DRAG_KNIFE));
      actual = schedule_cuts(cuts);
      correct.push_back(linear_cut::make(point(0, 0, -0.1), point(1, 1, -0.1), DRILL));
      correct.push_back(linear_cut::make(point(0, 0, -0.3), point(1, 1, -0.3), DRILL));
      correct.push_back(linear_cut::make(point(1, 1, -0.1), point(1, 3, -0.1), DRAG_KNIFE));
      correct.push_back(linear_cut::make(point(1, 1, -0.3), point(1, 3, -0.3), DRAG_KNIFE));
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));      
    }

    SECTION("2 pass drag knife code that should not be grouped") {
      cuts.push_back(linear_cut::make(point(0, 0, -0.1), point(1, 1, -0.1), DRAG_KNIFE));
      cuts.push_back(linear_cut::make(point(0, 0, -0.3), point(1, 1, -0.3), DRAG_KNIFE));
      cuts.push_back(linear_cut::make(point(1, 1, -0.1), point(1, 3, -0.1), DRAG_KNIFE));
      cuts.push_back(linear_cut::make(point(1, 1, -0.3), point(1, 3, -0.3), DRAG_KNIFE));
      actual = schedule_cuts(cuts);
      correct.push_back(linear_cut::make(point(0, 0, -0.1), point(1, 1, -0.1), DRAG_KNIFE));
      correct.push_back(linear_cut::make(point(0, 0, -0.3), point(1, 1, -0.3), DRAG_KNIFE));
      correct.push_back(linear_cut::make(point(1, 1, -0.1), point(1, 3, -0.1), DRAG_KNIFE));
      correct.push_back(linear_cut::make(point(1, 1, -0.3), point(1, 3, -0.3), DRAG_KNIFE));
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("One arc and 2 cuts") {
      cuts.push_back(circular_arc::make(point(1, 1, -0.1),
					point(2, 1, -0.1),
					point(0.5, 0, 0),
					CLOCKWISE,
					XY,
					DRILL));
      cuts.push_back(linear_cut::make(point(0, 0, -0.1), point(1, 1, -0.1), DRILL));
      cuts.push_back(linear_cut::make(point(2, 1, -0.1), point(4, 5, -0.1), DRILL));
      actual = schedule_cuts(cuts);
      cuts.push_back(linear_cut::make(point(0, 0, -0.1), point(1, 1, -0.1), DRILL));
      cuts.push_back(circular_arc::make(point(1, 1, -0.1),
					point(2, 1, -0.1),
					point(0.5, 0, 0),
					CLOCKWISE,
					XY,
					DRILL));
      cuts.push_back(linear_cut::make(point(2, 1, -0.1), point(4, 5, -0.1), DRILL));
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("Concentric triangles") {
      cuts.push_back(linear_cut::make(point(0, 0, -1), point(10, 0, -1), DRILL));
      cuts.push_back(linear_cut::make(point(10, 0, -1), point(10, 10, -1), DRILL));
      cuts.push_back(linear_cut::make(point(10, 10, -1), point(0, 0, -1), DRILL));

      cuts.push_back(linear_cut::make(point(1, 1, -1), point(9, 1, -1), DRILL));
      cuts.push_back(linear_cut::make(point(9, 1, -1), point(9, 9, -1), DRILL));
      cuts.push_back(linear_cut::make(point(9, 9, -1), point(1, 1, -1), DRILL));

      actual = schedule_cuts(cuts);

      correct.push_back(linear_cut::make(point(1, 1, -1), point(9, 1, -1), DRILL));
      correct.push_back(linear_cut::make(point(9, 1, -1), point(9, 9, -1), DRILL));
      correct.push_back(linear_cut::make(point(9, 9, -1), point(1, 1, -1), DRILL));

      correct.push_back(linear_cut::make(point(0, 0, -1), point(10, 0, -1), DRILL));
      correct.push_back(linear_cut::make(point(10, 0, -1), point(10, 10, -1), DRILL));
      correct.push_back(linear_cut::make(point(10, 10, -1), point(0, 0, -1), DRILL));
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("Concentric circles") {
      cuts.push_back(circular_arc::make(point(1, 1, -0.1),
					point(7, 1, -0.1),
					point(3, 0, 0),
					CLOCKWISE,
					XY,
					DRILL));
      cuts.push_back(circular_arc::make(point(7, 1, -0.1),
					point(1, 1, -0.1),
					point(-3, 0, 0),
					CLOCKWISE,
					XY,
					DRILL));
      cuts.push_back(circular_arc::make(point(2, 1, -0.1),
					point(4, 1, -0.1),
					point(1, 0, 0),
					CLOCKWISE,
					XY,
					DRILL));
      cuts.push_back(circular_arc::make(point(4, 1, -0.1),
					point(2, 1, -0.1),
					point(-1, 0, 0),
					CLOCKWISE,
					XY,
					DRILL));
      actual = schedule_cuts(cuts);
      correct.push_back(circular_arc::make(point(2, 1, -0.1),
					   point(4, 1, -0.1),
					   point(1, 0, 0),
					   CLOCKWISE,
					   XY,
					   DRILL));
      correct.push_back(circular_arc::make(point(4, 1, -0.1),
					   point(2, 1, -0.1),
					   point(-1, 0, 0),
					   CLOCKWISE,
					   XY,
					   DRILL));
      correct.push_back(circular_arc::make(point(1, 1, -0.1),
					   point(7, 1, -0.1),
					   point(3, 0, 0),
					   CLOCKWISE,
					   XY,
					   DRILL));
      correct.push_back(circular_arc::make(point(7, 1, -0.1),
					   point(1, 1, -0.1),
					   point(-3, 0, 0),
					   CLOCKWISE,
					   XY,
					   DRILL));
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }
  }
}
