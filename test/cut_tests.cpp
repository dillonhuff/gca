#include <math.h>

#include "catch.hpp"
#include "context.h"
#include "output.h"

namespace gca {
  
  TEST_CASE("Cut to GCODE") {
    context c;

  //   SECTION("GCODE from one cut") {
  //     cut* s = c.mk_cut(point(0, 0, -1), point(0, 3, -1));
  //     vector<cut*> cuts;
  //     cuts.push_back(s);
  //     gprog* res = gcode_for_cuts(c, cuts);
  //     gprog* correct = c.mk_gprog();
  //     correct->push_back(c.mk_G0(0, 0, -1));
  //     correct->push_back(c.mk_G1(0, 3, -1));
  //     correct->push_back(c.mk_G0(0, 3, 0));
  //     correct->push_back(c.mk_G0(0, 0, 0));
  //     correct->push_back(c.mk_minstr(2));
  //     REQUIRE(*res == *correct);
  //   }

  //   SECTION("GCODE for adjacent cuts") {
  //     cut* s1 = c.mk_cut(point(0, 0, -1), point(0, 3, -1));
  //     cut* s2 = c.mk_cut(point(0, 3, -1.00000001), point(2, 5, -1));
  //     vector<cut*> cuts;
  //     cuts.push_back(s1);
  //     cuts.push_back(s2);
  //     gprog* res = gcode_for_cuts(c, cuts);
  //     gprog* correct = c.mk_gprog();
  //     correct->push_back(c.mk_G0(0, 0, -1));
  //     correct->push_back(c.mk_G1(0, 3, -1));
  //     correct->push_back(c.mk_G1(2, 5, -1));
  //     correct->push_back(c.mk_G0(2, 5, 0));
  //     correct->push_back(c.mk_G0(0, 0, 0));
  //     correct->push_back(c.mk_minstr(2));
  //     cout << "-- Correct" << endl;
  //     cout << *correct;
  //     cout << "-- Actual" << endl;
  //     cout << *res;
  //     REQUIRE(*res == *correct);      
  //   }
  }

  TEST_CASE("Compute sink cut") {
    context c;

    SECTION("X axis sink") {
      cut* s1 = c.mk_cut(point(0, 0, -1), point(1, 0, -1));
      cut* sink = sink_cut(c, s1, 1.0);
      cut* correct = c.mk_cut(point(-1, 0, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Y axis sink") {
      cut* s1 = c.mk_cut(point(0, 0, -1), point(0, 1, -1));
      cut* sink = sink_cut(c, s1, 1.0);
      cut* correct = c.mk_cut(point(0, -1, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Mixed axis sink q1") {
      cut* s1 = c.mk_cut(point(0, 0, -1), point(1, 1, -1));
      cut* sink = sink_cut(c, s1, 1.0);
      double v = sqrt(1.0/2.0);
      cut* correct = c.mk_cut(point(-v, -v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }
    
    SECTION("Mixed axis sink q2") {
      cut* s1 = c.mk_cut(point(0, 0, -1), point(-1, 1, -1));
      cut* sink = sink_cut(c, s1, 1.0);
      double v = sqrt(1.0/2.0);
      cut* correct = c.mk_cut(point(v, -v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Mixed axis sink q3") {
      cut* s1 = c.mk_cut(point(0, 0, -1), point(-1, -1, -1));
      cut* sink = sink_cut(c, s1, 1.0);
      double v = sqrt(1.0/2.0);
      cut* correct = c.mk_cut(point(v, v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Mixed axis sink q4") {
      cut* s1 = c.mk_cut(point(0, 0, -1), point(1, -1, -1));
      cut* sink = sink_cut(c, s1, 1.0);
      double v = sqrt(1.0/2.0);
      cut* correct = c.mk_cut(point(-v, v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    // SECTION("Square cuts") {
    //   double depth = 0.05;
    //   double len = 1.0;
    //   cut* e1 = c.mk_cut(point(0, 0, -depth), point(0, len, -depth));
    //   cut* e2 = c.mk_cut(point(0, len, -depth), point(len, len, -depth));
    //   cut* e3 = c.mk_cut(point(len, len, -depth), point(len, 0, -depth));
    //   cut* e4 = c.mk_cut(point(len, 0, -depth), point(0, 0, -depth));
    //   vector<cut*> cuts;
    //   cuts.push_back(e1);
    //   cuts.push_back(e2);
    //   cuts.push_back(e3);
    //   cuts.push_back(e4);
    //   gprog* p = gcode_for_cuts(c, cuts);
    //   cout << "-- Final square program" << endl;
    //   cout << *p;
    // }

  }

  TEST_CASE("Surface cuts") {
    double depth = -0.05;
    double cutter_width = 1.0;
    double x_s = 5;
    double x_e = 6;
    point start(x_s, 5, depth);
    point end(x_e, 5, depth);
    point shift(0, cutter_width*(2.0/3.0), 0);
    context c;
    vector<cut*> cuts = surface_cuts(c, start, end, shift, 2);
    gprog* p = gcode_for_cuts(c, cuts);
    cout << "-- Final surface program" << endl;
    cout << *p;
  }
  
}
