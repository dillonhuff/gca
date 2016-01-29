#include <cmath>

#include "catch.hpp"
#include "core/context.h"
#include "synthesis/align_blade.h"
#include "synthesis/output.h"

namespace gca {
  
  TEST_CASE("Cut to GCODE") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("GCODE from one cut") {
      cut* s = mk_cut(point(0, 0, -1), point(0, 3, -1));
      vector<cut*> cuts;
      cuts.push_back(s);
      gprog* res = gcode_for_cuts(cuts);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G0(point(0, 0, 0)));
      correct->push_back(mk_G0(point(0, 0, 0)));
      correct->push_back(mk_G0(point(0, 0, -1)));
      correct->push_back(mk_G1(0, 3, -1));
      correct->push_back(mk_G0(point(0, 3, 0)));
      correct->push_back(mk_G0(point(0, 0, 0)));
      correct->push_back(mk_G0(point(0, 0, 0)));
      correct->push_back(mk_m2_instr());
      REQUIRE(*res == *correct);
    }

    SECTION("GCODE for adjacent cuts") {
      cut* s1 = mk_cut(point(0, 0, -1), point(0, 3, -1));
      cut* s2 = mk_cut(point(5, 3, -4), point(7, 2, -4));
      vector<cut*> cuts;
      cuts.push_back(s1);
      cuts.push_back(s2);
      gprog* res = gcode_for_cuts(cuts);
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
      correct->push_back(mk_m2_instr());
      REQUIRE(*res == *correct);      
    }
  }

  TEST_CASE("Compute sink cut") {
    
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("X axis sink") {
      cut* s1 = mk_cut(point(0, 0, -1), point(1, 0, -1));
      cut* sink = sink_cut(s1, 1.0);
      cut* correct = mk_cut(point(-1, 0, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Y axis sink") {
      cut* s1 = mk_cut(point(0, 0, -1), point(0, 1, -1));
      cut* sink = sink_cut(s1, 1.0);
      cut* correct = mk_cut(point(0, -1, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Mixed axis sink q1") {
      cut* s1 = mk_cut(point(0, 0, -1), point(1, 1, -1));
      cut* sink = sink_cut(s1, 1.0);
      double v = sqrt(1.0/2.0);
      cut* correct = mk_cut(point(-v, -v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }
    
    SECTION("Mixed axis sink q2") {
      cut* s1 = mk_cut(point(0, 0, -1), point(-1, 1, -1));
      cut* sink = sink_cut(s1, 1.0);
      double v = sqrt(1.0/2.0);
      cut* correct = mk_cut(point(v, -v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Mixed axis sink q3") {
      cut* s1 = mk_cut(point(0, 0, -1), point(-1, -1, -1));
      cut* sink = sink_cut(s1, 1.0);
      double v = sqrt(1.0/2.0);
      cut* correct = mk_cut(point(v, v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Mixed axis sink q4") {
      cut* s1 = mk_cut(point(0, 0, -1), point(1, -1, -1));
      cut* sink = sink_cut(s1, 1.0);
      double v = sqrt(1.0/2.0);
      cut* correct = mk_cut(point(-v, v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }
  }

  TEST_CASE("Align blade") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("X axis align") {
      point desired_dir = point(1, 0, 0);
      point current_dir = point(0, -1, 0);
      point desired_pos = point(1, 0, 0);
      double rad = 1.0;
      point c_pos;
      point center_off;
      align_coords(desired_dir,
		   desired_pos,
		   current_dir,
		   rad,
		   c_pos,
		   center_off);

      point correct_c_pos = point(0, -1, 0);
      point correct_center_off = point(0, 1, 0);
      REQUIRE(within_eps(correct_c_pos, c_pos));
      REQUIRE(within_eps(correct_center_off, center_off));
    }
    
  }
}
