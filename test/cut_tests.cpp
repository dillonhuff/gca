#include <cmath>

#include "catch.hpp"
#include "core/context.h"
#include "core/parser.h"
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

  TEST_CASE("Simple knife aligment code matches real CAM output") {
    arena_allocator a;
    set_system_allocator(&a);
    double safe_height = 0.35;
    double align_depth = 0.143;

    // SECTION("Case 1") {
    //   // G1 X15.791066 Y0.859332 Z0.075000
    //   // G1 X16.005220 Y0.588386 Z0.075000
    //   gprog* correct = read_file("/Users/dillon/CppWorkspace/gca/test/nc-files/align_test_1.nc");
    //   cout << "-- Correct " << endl;
    //   cout << *correct;
    //   gprog* p = mk_gprog();
    //   point sp(15.791066, 0.859332, 0.075000);
    //   point last_pos(16.005220, 0.588386, 0.075000);
    //   point last_orient = last_pos - sp;
    //   point next_pos(16.005220, 81.460030, 0.000000);
    //   point np(15.791067, 81.189087, 0.000000);
    //   point next_orient = np - next_pos;
    //   cout << "angle between orientations: " << angle_between(last_orient, next_orient) << endl;
    //   from_to_with_G0_drag_knife(safe_height,
    // 				 align_depth,
    // 				 p,
    // 				 last_pos,
    // 				 last_orient,
    // 				 next_pos,
    // 				 next_orient);
    //   cout << "-- Actual" << endl;
    //   cout << *p;
    //   REQUIRE(*p == *correct);
    // }

    SECTION("Case 2") {
      // Cut before
      // G1 X15.791066 Y0.859332 Z0.000000
      // G1 X16.005220 Y0.588386 Z0.000000

      // Cut after
      // G1 X20.299999 Y4.024209 Z0.075000
      // G1 X20.104019 Y4.272543 Z0.075000

      gprog* correct = read_file("/Users/dillon/CppWorkspace/gca/test/nc-files/align_test_2.nc");
      cout << "-- Correct " << endl;
      cout << *correct;
      gprog* p = mk_gprog();
      point sp(15.791066, 0.859332, 0.000000);
      point last_pos(16.005220, 0.588386, 0.000000);
      point last_orient = last_pos - sp;
      point next_pos(20.299999, 4.024209, 0.075000);
      point np(20.104019, 4.272543, 0.075000);
      point next_orient = np - next_pos;
      cout << "angle between orientations: " << angle_between(last_orient, next_orient) << endl;
      from_to_with_G0_drag_knife(safe_height,
				 align_depth,
				 p,
				 last_pos,
				 last_orient,
				 next_pos,
				 next_orient);
      cout << "-- Actual" << endl;
      cout << *p;
      REQUIRE(*p == *correct);
    }
  }
}
