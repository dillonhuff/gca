#include <cmath>

#include "catch.hpp"
#include "core/context.h"
#include "core/parser.h"
#include "synthesis/align_blade.h"
#include "synthesis/cut_to_gcode.h"
#include "synthesis/output.h"
#include "system/settings.h"

namespace gca {
  
  TEST_CASE("Cut to GCODE") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("GCODE from one cut") {
      linear_cut* s = linear_cut::make(point(0, 0, -1), point(0, 3, -1));
      vector<linear_cut*> cuts;
      cuts.push_back(s);
      gprog* res = gcode_for_cuts(cuts);
      gprog* correct = gprog::make();
      correct->push_back(g0_instr::make(point(0, 0, 0)));
      correct->push_back(g0_instr::make(point(0, 0, 0)));
      correct->push_back(g0_instr::make(point(0, 0, -1)));
      correct->push_back(g1_instr::make(0, 3, -1));
      correct->push_back(g0_instr::make(point(0, 3, 0)));
      correct->push_back(g0_instr::make(point(0, 0, 0)));
      correct->push_back(g0_instr::make(point(0, 0, 0)));
      correct->push_back(mk_m2_instr());
      REQUIRE(*res == *correct);
    }

    SECTION("GCODE for adjacent cuts") {
      linear_cut* s1 = linear_cut::make(point(0, 0, -1), point(0, 3, -1));
      linear_cut* s2 = linear_cut::make(point(5, 3, -4), point(7, 2, -4));
      vector<linear_cut*> cuts;
      cuts.push_back(s1);
      cuts.push_back(s2);
      gprog* res = gcode_for_cuts(cuts);
      gprog* correct = gprog::make();
      correct->push_back(g0_instr::make(point(0, 0, 0)));
      correct->push_back(g0_instr::make(point(0, 0, 0)));
      correct->push_back(g0_instr::make(point(0, 0, -1)));
      correct->push_back(g1_instr::make(0, 3, -1));
      correct->push_back(g0_instr::make(point(0, 3, 0)));
      correct->push_back(g0_instr::make(point(5, 3, 0)));
      correct->push_back(g0_instr::make(point(5, 3, -4)));
      correct->push_back(g1_instr::make(7, 2, -4));
      correct->push_back(g0_instr::make(point(7, 2, 0)));
      correct->push_back(g0_instr::make(point(0, 0, 0)));
      correct->push_back(g0_instr::make(point(0, 0, 0)));
      correct->push_back(mk_m2_instr());
      REQUIRE(*res == *correct);      
    }
  }

  TEST_CASE("Compute sink cut") {
    
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("X axis sink") {
      linear_cut* s1 = linear_cut::make(point(0, 0, -1), point(1, 0, -1));
      linear_cut* sink = sink_cut(s1, 1.0);
      linear_cut* correct = linear_cut::make(point(-1, 0, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Y axis sink") {
      linear_cut* s1 = linear_cut::make(point(0, 0, -1), point(0, 1, -1));
      linear_cut* sink = sink_cut(s1, 1.0);
      linear_cut* correct = linear_cut::make(point(0, -1, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Mixed axis sink q1") {
      linear_cut* s1 = linear_cut::make(point(0, 0, -1), point(1, 1, -1));
      linear_cut* sink = sink_cut(s1, 1.0);
      double v = sqrt(1.0/2.0);
      linear_cut* correct = linear_cut::make(point(-v, -v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }
    
    SECTION("Mixed axis sink q2") {
      linear_cut* s1 = linear_cut::make(point(0, 0, -1), point(-1, 1, -1));
      linear_cut* sink = sink_cut(s1, 1.0);
      double v = sqrt(1.0/2.0);
      linear_cut* correct = linear_cut::make(point(v, -v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Mixed axis sink q3") {
      linear_cut* s1 = linear_cut::make(point(0, 0, -1), point(-1, -1, -1));
      linear_cut* sink = sink_cut(s1, 1.0);
      double v = sqrt(1.0/2.0);
      linear_cut* correct = linear_cut::make(point(v, v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }

    SECTION("Mixed axis sink q4") {
      linear_cut* s1 = linear_cut::make(point(0, 0, -1), point(1, -1, -1));
      linear_cut* sink = sink_cut(s1, 1.0);
      double v = sqrt(1.0/2.0);
      linear_cut* correct = linear_cut::make(point(-v, v, 0), point(0, 0, -1));
      REQUIRE(*sink == *correct);
    }
  }

  TEST_CASE("Simple knife aligment code matches real CAM output") {
    arena_allocator a;
    set_system_allocator(&a);
    SECTION("Case 1") {
      // G1 X15.791066 Y0.859332 Z0.075000
      // G1 X16.005220 Y0.588386 Z0.075000
      double safe_height = 0.35;
      double align_depth = 0.143;

      string test_path = project_path + string("gca/test/nc-files/align_test_1.nc");
      gprog* correct = read_file(test_path);
      gprog* p = gprog::make();
      point sp(15.791066, 0.859332, 0.075000);
      point last_pos(16.005220, 0.588386, 0.075000);
      point last_orient = last_pos - sp;
      point next_pos(16.005220, 81.460030, 0.000000);
      point np(15.791067, 81.189087, 0.000000);
      point next_orient = np - next_pos;
      vector<cut*> cuts = from_to_with_G0_drag_knife(safe_height,
						     align_depth,
						     last_pos,
						     last_orient,
						     next_pos,
						     next_orient);
      for (unsigned i = 0; i < cuts.size(); i++) {
	append_cut(cuts[i], *p);
      }
      cout << "Actual: " << endl;
      cout << *p;
      cout << "Correct: " << endl;
      cout << *correct;
      REQUIRE(*p == *correct);
    }

    SECTION("Case 2") {
      // Cut before
      // G1 X15.791066 Y0.859332 Z0.000000
      // G1 X16.005220 Y0.588386 Z0.000000

      // Cut after
      // G1 X20.299999 Y4.024209 Z0.075000
      // G1 X20.104019 Y4.272543 Z0.075000

      double safe_height = 0.35;
      double align_depth = 0.143;


      string test_path = project_path + string("gca/test/nc-files/align_test_2.nc");
      gprog* correct = read_file(test_path);
      gprog* p = gprog::make();
      point sp(15.791066, 0.859332, 0.000000);
      point last_pos(16.005220, 0.588386, 0.000000);
      point last_orient = last_pos - sp;
      point next_pos(20.299999, 4.024209, 0.075000);
      point np(20.104019, 4.272543, 0.075000);
      point next_orient = np - next_pos;
      vector<cut*> cuts = from_to_with_G0_drag_knife(safe_height,
						     align_depth,
						     last_pos,
						     last_orient,
						     next_pos,
						     next_orient);
      for (unsigned i = 0; i < cuts.size(); i++) {
	append_cut(cuts[i], *p);
      }
      REQUIRE(*p == *correct);
    }

    SECTION("Case 3") {
      // Cut before
      // G1 X-1.732718 Y-32.033215 Z0.055000
      // G1 X-1.561738 Y-32.249390 Z0.055000

      // Cut after
      // G1 X-1.561737 Y-32.249390 Z0.000000
      // G1 X-1.872044 Y-31.853912 Z0.000000

      double safe_height = 0.31;
      double align_depth = 0.103;

      string test_path = project_path + string("gca/test/nc-files/align_test_3.nc");
      gprog* correct = read_file(test_path);
      gprog* p = gprog::make();
      point sp(-1.732718, -32.033215, 0.055000);
      point last_pos(-1.561738, -32.249390, 0.055000);
      point last_orient = last_pos - sp;
      point next_pos(-1.561737, -32.249390, 0.000000);
      point np(-1.872044, -31.853912, 0.000000);
      point next_orient = np - next_pos;
      vector<cut*> cuts = from_to_with_G0_drag_knife(safe_height,
						     align_depth,
						     last_pos,
						     last_orient,
						     next_pos,
						     next_orient);
      for (unsigned i = 0; i < cuts.size(); i++) {
	append_cut(cuts[i], *p);
      }
      REQUIRE(*p == *correct);      
    }
  }
}
