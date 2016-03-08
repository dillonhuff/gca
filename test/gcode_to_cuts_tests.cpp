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
    s.initial_tool = DRILL;
    s.initial_pos = point(0, 0, 0);
    s.initial_orient = point(1, 0, 0);

    SECTION("Empty program") {
      gprog* p = parse_gprog("");
      actual = gcode_to_cuts(*p, s);
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("One safe move") {
      gprog* p = parse_gprog("S3000 M3 G0 X1 Y1 Z1");
      actual = gcode_to_cuts(*p, s);
      correct.push_back(safe_move::make(point(0, 0, 0), point(1, 1, 1)));
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("Safe move and linear move") {
      gprog* p = parse_gprog("G90 S3000 M3 G0 X2 Y1 Z-2 G1 F20 X3 Y3 Z2");
      actual = gcode_to_cuts(*p, s);
      correct.push_back(safe_move::make(point(0, 0, 0), point(2, 1, -2), DRILL));
      linear_cut* lc = linear_cut::make(point(2, 1, -2), point(3, 3, 2), DRILL);
      lc->feedrate = lit::make(20);
      lc->spindle_speed = lit::make(3000);
      correct.push_back(lc);
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("Safe move and linear move with spindle") {
      gprog* p = parse_gprog("G90 S2000 M3 G0 X2 Y1 Z-2 G1 F20 X3 Y3 Z2");
      actual = gcode_to_cuts(*p, s);
      safe_move* sm = safe_move::make(point(0, 0, 0), point(2, 1, -2), DRILL);
      sm->spindle_speed = lit::make(2000);
      correct.push_back(sm);
      linear_cut* lc = linear_cut::make(point(2, 1, -2), point(3, 3, 2), DRILL);
      lc->feedrate = lit::make(20);
      lc->spindle_speed = lit::make(2000);
      correct.push_back(lc);
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }
    
    SECTION("Safe move and circular arc clockwise in XY plane") {
      gprog* p = parse_gprog("G90 S2000 G0 X1 Y1 Z1 G2 F12.5 X3 Y4.5 Z1 I1.0 J1.75");
      actual = gcode_to_cuts(*p, s);
      safe_move* sm = safe_move::make(point(0, 0, 0), point(1, 1, 1));
      sm->spindle_speed = lit::make(2000);
      correct.push_back(sm);
      circular_arc* arc = circular_arc::make(point(1, 1, 1),
      					     point(3, 4.5, 1),
      					     point(1, 1.75, 0),
      					     CLOCKWISE,
      					     XY);
      arc->feedrate = lit::make(12.5);
      arc->spindle_speed = lit::make(2000);
      correct.push_back(arc);
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));      
    }

    SECTION("Safe move and circular arc counterclockwise in XY plane") {
      gprog* p = parse_gprog("G90 S2000 G0 X1 Y1 Z1 G3 F12.5 X3 Y4.5 Z1 I1.0 J1.75");
      actual = gcode_to_cuts(*p, s);
      safe_move* sm = safe_move::make(point(0, 0, 0), point(1, 1, 1));
      sm->spindle_speed = lit::make(2000);
      correct.push_back(sm);
      circular_arc* arc = circular_arc::make(point(1, 1, 1),
      					     point(3, 4.5, 1),
      					     point(1, 1.75, 0),
      					     COUNTERCLOCKWISE,
      					     XY);
      arc->feedrate = lit::make(12.5);
      arc->spindle_speed = lit::make(2000);
      correct.push_back(arc);
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }
  }

  TEST_CASE("GCODE to cuts CAMASTER") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<cut*> actual;
    vector<cut*> correct;

    gcode_settings s;
    s.initial_coord_orient = GCA_ABSOLUTE;
    s.initial_tool = DRILL;
    s.initial_pos = point(0, 0, 0);
    s.initial_orient = point(1, 0, 0);

    SECTION("Linear moves with spindle speed change") {
      gprog* p = parse_gprog("G90 S2000 G1 X1 Y1 Z1 S1000 G1 X2 Y2 Z2");
      actual = gcode_to_cuts(*p, s);
      linear_cut* lc1 = linear_cut::make(point(0, 0, 0), point(1, 1, 1), DRILL);
      lc1->spindle_speed = lit::make(2000);
      correct.push_back(lc1);
      linear_cut* lc2 = linear_cut::make(point(1, 1, 1), point(2, 2, 2), DRILL);
      lc2->spindle_speed = lit::make(1000);
      correct.push_back(lc2);
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("Linear moves with tool change") {
      gprog* p = parse_gprog("G90 S2000 G1 X1 Y1 Z1 S1000 T6 G1 X2 Y2 Z2");
      actual = gcode_to_cuts(*p, s);
      linear_cut* lc1 = linear_cut::make(point(0, 0, 0), point(1, 1, 1), DRILL);
      lc1->spindle_speed = lit::make(2000);
      correct.push_back(lc1);
      linear_cut* lc2 = linear_cut::make(point(1, 1, 1), point(2, 2, 2), DRAG_KNIFE);
      lc2->spindle_speed = lit::make(1000);
      correct.push_back(lc2);
      REQUIRE(equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }
  }

}
