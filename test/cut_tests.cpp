#include <cmath>

#include "catch.hpp"
#include "core/context.h"
#include "synthesis/output.h"

namespace gca {
  
  TEST_CASE("Cut to GCODE") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("GCODE from one cut") {
      cut* s = mk_cut(point(0, 0, -1), point(0, 3, -1));
      vector<cut*> cuts;
      cuts.push_back(s);
      gprog* res = gcode_for_cuts(c, cuts);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G0(point(0, 0, 0)));
      correct->push_back(mk_G0(point(0, 0, 0)));
      correct->push_back(mk_G0(point(0, 0, -1)));
      correct->push_back(mk_G1(0, 3, -1));
      correct->push_back(mk_G0(point(0, 3, 0)));
      correct->push_back(mk_G0(point(0, 0, 0)));
      correct->push_back(mk_G0(point(0, 0, 0)));
      correct->push_back(mk_minstr(2));
      REQUIRE(*res == *correct);
    }

    SECTION("GCODE for adjacent cuts") {
      cut* s1 = mk_cut(point(0, 0, -1), point(0, 3, -1));
      cut* s2 = mk_cut(point(5, 3, -4), point(7, 2, -4));
      vector<cut*> cuts;
      cuts.push_back(s1);
      cuts.push_back(s2);
      gprog* res = gcode_for_cuts(c, cuts);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G0(point(0, 0, 0)));
      correct->push_back(mk_G0(point(0, 0, 0)));
      correct->push_back(mk_G0(point(0, 0, -1)));
      correct->push_back(mk_G1(0, 3, -1));
      correct->push_back(mk_G0(point(0, 3, 0)));
      correct->push_back(mk_G0(point(5, 3, 0)));
      correct->push_back(mk_G0(point(5, 3, -4)));
      correct->push_back(mk_G1(7, 2, -4));
      correct->push_back(mk_G0(point(7, 2, 0)));
      correct->push_back(mk_G0(point(0, 0, 0)));
      correct->push_back(mk_G0(point(0, 0, 0)));
      correct->push_back(mk_minstr(2));
      REQUIRE(*res == *correct);      
    }
  }

  TEST_CASE("Compute sink cut") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("X axis sink") {
      cut* s1 = mk_cut(point(0, 0, -1), point(1, 0, -1));
      cut* sink = sink_cut(c, s1, 1.0);
      cut* correct = mk_cut(point(-1, 0, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Y axis sink") {
      cut* s1 = mk_cut(point(0, 0, -1), point(0, 1, -1));
      cut* sink = sink_cut(c, s1, 1.0);
      cut* correct = mk_cut(point(0, -1, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Mixed axis sink q1") {
      cut* s1 = mk_cut(point(0, 0, -1), point(1, 1, -1));
      cut* sink = sink_cut(c, s1, 1.0);
      double v = sqrt(1.0/2.0);
      cut* correct = mk_cut(point(-v, -v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }
    
    SECTION("Mixed axis sink q2") {
      cut* s1 = mk_cut(point(0, 0, -1), point(-1, 1, -1));
      cut* sink = sink_cut(c, s1, 1.0);
      double v = sqrt(1.0/2.0);
      cut* correct = mk_cut(point(v, -v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Mixed axis sink q3") {
      cut* s1 = mk_cut(point(0, 0, -1), point(-1, -1, -1));
      cut* sink = sink_cut(c, s1, 1.0);
      double v = sqrt(1.0/2.0);
      cut* correct = mk_cut(point(v, v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Mixed axis sink q4") {
      cut* s1 = mk_cut(point(0, 0, -1), point(1, -1, -1));
      cut* sink = sink_cut(c, s1, 1.0);
      double v = sqrt(1.0/2.0);
      cut* correct = mk_cut(point(-v, v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }
  }
}
