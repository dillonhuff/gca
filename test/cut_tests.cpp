#include <math.h>

#include "catch.hpp"
#include "context.h"
#include "output.h"

namespace gca {
  
  TEST_CASE("Cut to GCODE") {
    context c;

    SECTION("GCODE from one cut") {
      cut* s = c.mk_cut(point(0, 0, -1), point(0, 3, -1));
      vector<cut*> cuts;
      cuts.push_back(s);
      gprog* res = gcode_for_cuts(c, cuts);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G0(0, 0, -1));
      correct->push_back(c.mk_G1(0, 3, -1));
      correct->push_back(c.mk_G0(0, 3, 0));
      correct->push_back(c.mk_G0(0, 0, 0));
      correct->push_back(c.mk_minstr(2));
      REQUIRE(*res == *correct);
    }

    SECTION("GCODE for adjacent cuts") {
      cut* s1 = c.mk_cut(point(0, 0, -1), point(0, 3, -1));
      cut* s2 = c.mk_cut(point(0, 3, -1.00000001), point(2, 5, -1));
      vector<cut*> cuts;
      cuts.push_back(s1);
      cuts.push_back(s2);
      gprog* res = gcode_for_cuts(c, cuts);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G0(0, 0, -1));
      correct->push_back(c.mk_G1(0, 3, -1));
      correct->push_back(c.mk_G1(2, 5, -1));
      correct->push_back(c.mk_G0(2, 5, 0));
      correct->push_back(c.mk_G0(0, 0, 0));
      correct->push_back(c.mk_minstr(2));
      cout << "-- Correct" << endl;
      cout << *correct;
      cout << "-- Actual" << endl;
      cout << *res;
      REQUIRE(*res == *correct);      
    }
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

    SECTION("Square cuts") {
      cut* e1 = c.mk_cut(point(0, 0, -10), point(0, 10, -10));
      cut* e2 = c.mk_cut(point(0, 10, -10), point(10, 10, -10));
      cut* e3 = c.mk_cut(point(10, 10, -10), point(10, 0, -10));
      cout << "-- e3 sink" << endl;
      cut* e3_sink = sink_cut(c, e3, 3.0);
      cout << "-- END e3 sink" << endl;
      //      cout << *e3 << endl;
      //      cout << *e3_sink << endl;
      cut* e4 = c.mk_cut(point(10, 0, -10), point(0, 0, -10));
      vector<cut*> cuts;
      cuts.push_back(e1);
      cuts.push_back(e2);
      cuts.push_back(e3);
      cuts.push_back(e4);
      vector<cut*> cuts_with_sinks;
      insert_sink_cuts(c, 3.0, cuts, cuts_with_sinks);
      gprog* p = gcode_for_cuts(c, cuts_with_sinks);
      cout << "-- Final square program" << endl;
      cout << *p;
    }
    
  }
  
}
