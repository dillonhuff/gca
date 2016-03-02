#include "analysis/gcode_to_cuts.h"
#include "catch.hpp"
#include "core/arena_allocator.h"
#include "core/parser.h"
#include "synthesis/circular_arc.h"
#include "synthesis/linear_cut.h"
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

    SECTION("Safe move and circular arc clockwise in XY plane") {
      gprog* p = parse_gprog("G90 G0 X1 Y1 Z1 G2 F12.5 X3 Y4.5 Z1 I2.0 J3.0");
      actual = gcode_to_cuts(*p, s);
      correct.push_back(safe_move::make(point(0, 0, 0), point(1, 1, 1)));
      circular_arc* arc = circular_arc::make(point(1, 1, 1),
					     point(3, 4.5, 1),
					     point(2, 3, 0),
					     CLOCKWISE,
					     XY);
      arc->feedrate = lit::make(12.5);
      correct.push_back(arc);
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));      
    }

    SECTION("Safe move and circular arc counterclockwise in XY plane") {
      gprog* p = parse_gprog("G90 G0 X1 Y1 Z1 G3 F12.5 X3 Y4.5 Z1 I2.0 J3.0");
      actual = gcode_to_cuts(*p, s);
      correct.push_back(safe_move::make(point(0, 0, 0), point(1, 1, 1)));
      circular_arc* arc = circular_arc::make(point(1, 1, 1),
					     point(3, 4.5, 1),
					     point(2, 3, 0),
					     COUNTERCLOCKWISE,
					     XY);
      arc->feedrate = lit::make(12.5);
      correct.push_back(arc);
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));      
    }

  }
}
