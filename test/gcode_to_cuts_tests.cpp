#include "analysis/gcode_to_cuts.h"
#include "catch.hpp"
#include "system/arena_allocator.h"
#include "synthesis/circular_arc.h"
#include "synthesis/linear_cut.h"
#include "synthesis/safe_move.h"

namespace gca {

  bool operator==(const vector<cut*>& x, const vector<cut*>& y) {
    if (x.size() != y.size()) { return false; }
    return equal(x.begin(), x.end(), y.begin(),
		 [](const cut* l, const cut* r) { return (*l) == (*r); });
  }

  TEST_CASE("GCODE to cuts PROBOTIX") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<cut*> c;
    vector<vector<cut*>> actual;
    vector<vector<cut*>> correct;

    vector<block> p;
    gcode_to_cuts_result r;
    
    SECTION("Empty program") {
      p = lex_gprog("");
      r = gcode_to_cuts(p, actual);
      REQUIRE(correct == actual); //equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("One safe move") {
      p = lex_gprog("G90 S3000 M3 \n G0 X1 Y1 Z1");
      r = gcode_to_cuts(p, actual);
      REQUIRE(correct == actual); //equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("Safe move and linear move") {
      p = lex_gprog("G90 S3000 M3 \n G0 X2 Y1 Z-2 \n G1 F20 X3 Y3 Z2");
      r = gcode_to_cuts(p, actual);
      linear_cut* lc = linear_cut::make(point(2, 1, -2), point(3, 3, 2), DRILL);
      lc->set_feedrate(lit::make(20));
      lc->set_spindle_speed(lit::make(3000));
      correct.push_back({lc});
      REQUIRE(correct == actual); //equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("Safe move and linear move with spindle") {
      p = lex_gprog("G90 S2000 M3 \n G0 X2 Y1 Z-2 \n G1 F20 X3 Y3 Z2");
      r = gcode_to_cuts(p, actual);
      linear_cut* lc = linear_cut::make(point(2, 1, -2), point(3, 3, 2), DRILL);
      lc->set_feedrate(lit::make(20));
      lc->set_spindle_speed(lit::make(2000));
      correct.push_back({lc});
      REQUIRE(correct == actual); //equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }
    
    SECTION("Safe move and circular arc clockwise in XY plane") {
      p = lex_gprog("G17 G90 S2000 G0 X1 Y1 Z1 \n G2 F12.5 X3 Y4.5 Z1 I1.0 J1.75");
      r = gcode_to_cuts(p, actual);
      circular_arc* arc = circular_arc::make(point(1, 1, 1),
      					     point(3, 4.5, 1),
      					     point(1, 1.75, 0),
      					     CLOCKWISE,
      					     XY,
					     DRILL);
      arc->set_feedrate(lit::make(12.5));
      arc->set_spindle_speed(lit::make(2000));
      correct.push_back({arc});
      REQUIRE(correct == actual); //equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));      
    }

    SECTION("Safe move and circular arc counterclockwise in XY plane") {
      p = lex_gprog("G17 G90 S2000 \n G0 X1 Y1 Z1 \n G3 F12.5 X3 Y4.5 Z1 I1.0 J1.75");
      r = gcode_to_cuts(p, actual);
      circular_arc* arc = circular_arc::make(point(1, 1, 1),
      					     point(3, 4.5, 1),
      					     point(1, 1.75, 0),
      					     COUNTERCLOCKWISE,
      					     XY,
					     DRILL);
      arc->set_feedrate(lit::make(12.5));
      arc->set_spindle_speed(lit::make(2000));
      correct.push_back({arc});
      REQUIRE(correct == actual); //equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }
  }

  TEST_CASE("GCODE to cuts CAMASTER") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<vector<cut*>> actual;
    vector<vector<cut*>> correct;

    gcode_to_cuts_result r;
    vector<block> p;

    SECTION("Linear moves with spindle speed change") {
      p = lex_gprog("G90 S2000 M3 \n G0 X0 Y0 Z0 \n G1 X1 Y1 Z1 \n S1000 G1 X2 Y2 Z2");
      r = gcode_to_cuts(p, actual);
      linear_cut* lc1 = linear_cut::make(point(0, 0, 0), point(1, 1, 1), DRILL);
      lc1->set_spindle_speed(lit::make(2000));
      linear_cut* lc2 = linear_cut::make(point(1, 1, 1), point(2, 2, 2), DRILL);
      lc2->set_spindle_speed(lit::make(1000));
      correct.push_back({lc1, lc2});
      REQUIRE(correct == actual); //equal(correct.begin(), correct.end(), actual.begin(), cmp_cuts));
    }

    SECTION("Linear moves with tool change") {
      p = lex_gprog("G90 S2000 \n G0 X0 Y0 Z0 \n G1 X1 Y1 Z1 \n S1000 T6 M6 \n G0 X1 Y1 Z1 \n G1 X2 Y2 Z2");
      r = gcode_to_cuts(p, actual);
      linear_cut* lc1 = linear_cut::make(point(0, 0, 0), point(1, 1, 1));
      lc1->set_spindle_speed(lit::make(2000));
      correct.push_back({lc1});
      linear_cut* lc2 = linear_cut::make(point(1, 1, 1), point(2, 2, 2));
      lc2->set_spindle_speed(lit::make(1000));
      correct.push_back({lc2});
      REQUIRE(correct == actual);
    }

    SECTION("Linear moves in G54 with tool change") {
      p = lex_gprog("G54 G90 S2000 \n G0 X0 Y0 Z0 \n G1 X1 Y1 Z1 \n S1000 T6 M6 \n G0 X1 Y1 Z1 \n G1 X2 Y2 Z2");
      r = gcode_to_cuts(p, actual);
      linear_cut* lc1 = linear_cut::make(point(0, 0, 0), point(1, 1, 1));
      lc1->set_spindle_speed(lit::make(2000));
      correct.push_back({lc1});
      linear_cut* lc2 = linear_cut::make(point(1, 1, 1), point(2, 2, 2));
      lc2->set_spindle_speed(lit::make(1000));
      correct.push_back({lc2});
      REQUIRE(correct == actual);
    }

    SECTION("Fail on selected plane other than G17") {
      p = lex_gprog("G18");
      r = gcode_to_cuts(p, actual);
      REQUIRE(r == GCODE_TO_CUTS_UNSUPPORTED_SETTINGS);
    }

    SECTION("Fast move as part of toolpath") {
      p = lex_gprog("G54 G90 S2000 \n G0 X0 Y0 Z0 \n G0 X1 Y1 Z1 \n");
      r = gcode_to_cuts(p, actual);
      safe_move* lc1 = safe_move::make(point(0, 0, 0), point(1, 1, 1));
      lc1->set_spindle_speed(lit::make(2000));
      correct.push_back({lc1});
      REQUIRE(correct == actual);
    }
  }

}
