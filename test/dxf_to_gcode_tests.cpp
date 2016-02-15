#include "analysis/extract_cuts.h"
#include "catch.hpp"
#include "checkers/forbidden_tool_checker.h"
#include "checkers/unsafe_spindle_checker.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/dxf_reader.h"
#include "synthesis/output.h"

namespace gca {

  unsigned num_cuts(const vector<cut_group>& cut_groups) {
    unsigned nc = 0;
    for (unsigned i = 0; i < cut_groups.size(); i++) {
      nc += cut_groups[i].size();
    }
    return nc;
  }

  TEST_CASE("Read rectangle file") {
    arena_allocator a;
    set_system_allocator(&a);
    
    string file_name = "/Users/dillon/CppWorkspace/gca/test/dxf-files/rect-2inx3in.DXF";
    shape_layout l = read_dxf(file_name.c_str());
    
    SECTION("All cuts parsed") {
      REQUIRE(l.lines.size() == 6);
    }

    SECTION("No cuts lost in cut group creation") {
      vector<cut_group> cgs;
      group_adjacent_cuts(l.lines, cgs, 30.0);
      unsigned nc = num_cuts(cgs);
      unsigned correct = l.lines.size();
      REQUIRE(nc == correct);
    }

    SECTION("Each cut is its own cut group") {
      vector<cut_group> cgs;
      group_adjacent_cuts(l.lines, cgs, 30.0);
      unsigned correct = l.lines.size();
      REQUIRE(cgs.size() == correct);
    }
  }


  TEST_CASE("Safety check files") {
    arena_allocator a;
    set_system_allocator(&a);

    cut_params params;
    params.safe_height = 0.35;
    params.material_depth = 0.09;
    params.cut_depth = 0.05;
    params.push_depth = 0.005;
    params.start_loc = point(0, 0, 0);
    params.start_orient = point(1, 0, 0);
    params.tools = ToolOptions::DRILL_AND_DRAG_KNIFE;
    
    string file_name = "/Users/dillon/CppWorkspace/gca/test/dxf-files/12-inch-spiral.DXF";

    vector<int> no_spindle_tools;
    no_spindle_tools.push_back(6);
      
    SECTION("spiral is safe") {
      gprog* p = dxf_to_gcode(file_name.c_str(), params);
      REQUIRE(check_for_unsafe_spindle_on(no_spindle_tools, 2, p) == 0);
    }
  }
  
  TEST_CASE("Existing shape layout") {
    arena_allocator a;
    set_system_allocator(&a);

    cut_params params;
    params.safe_height = 0.35;
    params.material_depth = 0.09;
    params.cut_depth = 0.05;
    params.push_depth = 0.005;
    params.start_loc = point(0, 0, 0);
    params.start_orient = point(1, 0, 0);
    
    vector<cut*> lines;
    vector<hole_punch*> holes;
    vector<b_spline*> splines;

    vector<int> permitted_tools;

    SECTION("No hole punches") {
      params.tools = ToolOptions::DRILL_AND_DRAG_KNIFE;
      lines.push_back(mk_linear_cut(point(0, 0, 0), point(1, 0, 0)));
      shape_layout l(lines, holes, splines);
      gprog* p = shape_layout_to_gcode(l, params);

      SECTION("No use of drill when there are no hole punches") {
	permitted_tools.push_back(6);
	REQUIRE(check_for_forbidden_tool_changes(permitted_tools, p) == 0);
      }

      SECTION("Drill produces code for linear cuts") {
	gprog* p = shape_layout_to_gcode(l, params);
	vector<cut_section> sections;
	extract_cuts(p, sections);
	cout << *p;
	REQUIRE(sections.size() == 2);
      }

      SECTION("Produces some code") {
	gprog c;
	gprog* r = append_footer(&c);
	REQUIRE(p->size() > r->size());
      }
    }

    SECTION("Lines and hole punches") {
      lines.push_back(mk_linear_cut(point(0, 0, 0), point(1, 0, 0)));
      holes.push_back(mk_hole_punch(2, 2, 2, 0.125));
      
      shape_layout l(lines, holes, splines);
      
      SECTION("With drill and drag knife") {
	params.tools = ToolOptions::DRILL_AND_DRAG_KNIFE;
	gprog* p = shape_layout_to_gcode(l, params);
	REQUIRE(check_for_forbidden_tool_changes(permitted_tools, p) == 2);
      }

      SECTION("With drill only") {
	params.tools = ToolOptions::DRILL_ONLY;
	gprog* p = shape_layout_to_gcode(l, params);
	REQUIRE(check_for_forbidden_tool_changes(permitted_tools, p) == 1);
      }

      SECTION("Drill produces code for linear cuts") {
	params.tools = ToolOptions::DRILL_ONLY;
	gprog* p = shape_layout_to_gcode(l, params);
	vector<cut_section> sections;
	extract_cuts(p, sections);
	cout << *p;
	REQUIRE(sections.size() == 2);
      }
    }
  }

  TEST_CASE("Drill 2 holes") {
    arena_allocator a;
    set_system_allocator(&a);

    cut_params params;
    params.safe_height = 0.35;
    params.material_depth = 0.09;
    params.cut_depth = 0.05;
    params.push_depth = 0.005;
    params.start_loc = point(0, 0, 0);
    params.start_orient = point(1, 0, 0);
    params.tools = ToolOptions::DRILL_AND_DRAG_KNIFE;

    vector<cut*> lines;
    vector<hole_punch*> holes;
    holes.push_back(mk_hole_punch(1, 1, 1, 0.125));
    holes.push_back(mk_hole_punch(2, 2, 2, 0.125));
    vector<b_spline*> splines;
    shape_layout l(lines, holes, splines);

    gprog* p = shape_layout_to_gcode(l, params);

    SECTION("Two cut groups in drill toolpath") {
      toolpath dt = drill_toolpath(l, params);
      REQUIRE(dt.cut_groups.size() == 2);
    }
  }

}
