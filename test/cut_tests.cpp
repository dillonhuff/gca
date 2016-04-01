#include <cmath>

#include "catch.hpp"

#include "core/lexer.h"
#include "synthesis/align_blade.h"
#include "synthesis/cut_to_gcode.h"
#include "synthesis/output.h"
#include "synthesis/safe_move.h"
#include "system/settings.h"

namespace gca {

  TEST_CASE("Copying cuts") {
    arena_allocator a;
    set_system_allocator(&a);

    cut* s;
    cut* c;

    SECTION("Copy preserves info for safe move") {
      s = safe_move::make(point(1, 0, 0), point(1, 0, 0), DRILL);
      s->set_feedrate(lit::make(1.0));
      s->set_spindle_speed(lit::make(16000));
      c = s->copy();
      REQUIRE(*c == *s);
    }
    
    SECTION("Copy preserves info for linear cut") {
      s = linear_cut::make(point(1, 0, 0), point(1, 0, 0), DRAG_KNIFE);
      s->set_feedrate(lit::make(1.0));
      s->set_spindle_speed(lit::make(0));
      c = s->copy();
      REQUIRE(*c == *s);
    }

    SECTION("Copy preserves info for circular arc") {
      s = circular_arc::make(point(1, 0, 0), point(1, 1, 0), point(0, 0.5, 0), CLOCKWISE, YZ, DRILL);
      s->set_feedrate(lit::make(1.0));
      s->set_spindle_speed(lit::make(16000));
      c = s->copy();
      REQUIRE(*c == *s);
    }

    SECTION("Copy preserves info for hole punch") {
      s = hole_punch::make(point(1, 1, 1), 2.3, DRILL);
      s->set_feedrate(lit::make(1.0));
      s->set_spindle_speed(lit::make(16000));
      c = s->copy();
      REQUIRE(*c == *s);
    }
  }
  
  TEST_CASE("Simple knife aligment code matches real CAM output") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<block> blocks;
    vector<block> correct;
    
    SECTION("Case 1") {
      // G1 X15.791066 Y0.859332 Z0.075000
      // G1 X16.005220 Y0.588386 Z0.075000
      double safe_height = 0.35;
      double align_depth = 0.143;

      string test_path = project_path + string("gca/test/nc-files/align_test_1.nc");
      correct = lex_file(test_path);
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
	append_cut_block(cuts[i], blocks);
      }
      REQUIRE(blocks == correct);
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
      correct = lex_file(test_path);
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
	append_cut_block(cuts[i], blocks);
      }
      REQUIRE(blocks == correct);
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
      correct = lex_file(test_path);
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
	append_cut_block(cuts[i], blocks);
      }
      REQUIRE(blocks == correct);      
    }
  }
}
