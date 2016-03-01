#include "analysis/gcode_to_cuts.h"
#include "catch.hpp"
#include "core/arena_allocator.h"
#include "core/parser.h"
#include "synthesis/safe_move.h"

namespace gca {

  TEST_CASE("GCODE to cuts PROBOTIX") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<cut*> actual;
    vector<cut*> correct;

    gcode_settings s;
    s.initial_coord_orient = GCA_ABSOLUTE;
    s.initial_pos = point(0, 0, 0);
    s.initial_orient = point(1, 0, 0);

    SECTION("Empty program") {
      gprog* p = parse_gprog("");
      actual = gcode_to_cuts(*p, s);
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("One safe move") {
      gprog* p = parse_gprog("G0 X1 Y1 Z1");
      actual = gcode_to_cuts(*p, s);
      correct.push_back(safe_move::make(point(0, 0, 0), point(1, 1, 1)));
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("Safe move and linear move") {
      gprog* p = parse_gprog("G90 G0 X2 Y1 Z-2 G1 F20 X3 Y3 Z2");
      actual = gcode_to_cuts(*p, s);
      correct.push_back(safe_move::make(point(0, 0, 0), point(2, 1, -2)));
      linear_cut* lc = linear_cut::make(point(2, 1, -2), point(3, 3, 2));
      lc->feedrate = lit::make(20);
      correct.push_back(lc);
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));

    }
  }
}
