#include "analysis/extract_cuts.h"
#include "catch.hpp"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/dxf_reader.h"
#include "backend/output.h"

namespace gca {

  TEST_CASE("G1 segmentation") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<vector<machine_state>> sections;

    vector<block> p;

    SECTION("No G1 segments") {
      p = lex_gprog("G90 M5");
      extract_cuts(p, sections);
      REQUIRE(sections.size() == 0);
    }

    SECTION("One G1 segments with 1 instruction") {
      p = lex_gprog("G90 \n G0 X1.0 Y0.0 Z0.0 \n G1 X2.0 Y3.0 Z0.0 M5");
      extract_cuts(p, sections);
      REQUIRE(sections.size() == 1);
    }

    SECTION("One hole punch") {
      cut_params params;
      params.safe_height = 0.35;
      params.material_depth = 0.09;
      params.cut_depth = 0.05;
      params.push_depth = 0.005;
      params.start_loc = point(0, 0, 0);
      params.start_orient = point(1, 0, 0);
      params.target_machine = CAMASTER;
      params.tools = DRILL_AND_DRAG_KNIFE;

      vector<cut*> lines;
      vector<hole_punch*> holes;
      holes.push_back(hole_punch::make(point(2, 2, 2), 0.125));
      vector<b_spline*> splines;
      shape_layout l(lines, holes, splines);
      
      p = shape_layout_to_gcode(l, params);

      extract_cuts(p, sections);

      REQUIRE(sections.size() == 1);
    }
    
  }

}
