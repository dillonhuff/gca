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

    SECTION("Empty program") {
      gprog* p = parse_gprog("");
      actual = gcode_to_cuts(*p, s);
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("One safe move") {
      // gprog* p = parse_gprog("G0 X1 Y1 Z1");
      // actual = gcode_to_cuts(*p, s);
      // correct.push_back(safe_move::make(point(0, 0, 0), point(1, 1, 1)));
      //REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }
  }
}
