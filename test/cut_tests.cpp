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
      correct->push_back(c.mk_G0(0, 0, 0));
      correct->push_back(c.mk_minstr(2));
      cout << "-- Correct" << endl;
      cout << *correct;
      cout << "-- Actual" << endl;
      cout << *res;
      REQUIRE(*res == *correct);      
    }
  }
  
}
