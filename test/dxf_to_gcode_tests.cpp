#include "analysis/extract_cuts.h"
#include "catch.hpp"
#include "checkers/forbidden_tool_checker.h"
#include "checkers/unsafe_spindle_checker.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/dxf_reader.h"
#include "synthesis/output.h"
#include "system/settings.h"

namespace gca {

  TEST_CASE("Read rectangle file") {
    arena_allocator a;
    set_system_allocator(&a);

    string file_name = project_path + string("gca/test/dxf-files/rect-2inx3in.DXF");
    shape_layout l = read_dxf(file_name.c_str());
    
    SECTION("All cuts parsed") {
      REQUIRE(l.lines.size() == 6);
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
    params.target_machine = CAMASTER;
    params.tools = DRILL_AND_DRAG_KNIFE;
    
    string file_name =
      project_path + string("gca/test/dxf-files/12-inch-spiral.DXF");

    vector<int> no_spindle_tools;
    no_spindle_tools.push_back(6);
      
    SECTION("spiral is safe") {
      gprog* p = dxf_to_gcode(file_name.c_str(), params);
      REQUIRE(check_for_unsafe_spindle_on(no_spindle_tools, 2, p) == 0);
    }
  }
  
  TEST_CASE("Cut shape layout") {
    arena_allocator a;
    set_system_allocator(&a);

    cut_params params;
    params.safe_height = 0.35;
    params.material_depth = 0.09;
    params.cut_depth = 0.05;
    params.push_depth = 0.005;
    params.start_loc = point(0, 0, 0);
    params.start_orient = point(1, 0, 0);
    params.target_machine = CAMASTER;
    
    vector<cut*> lines;
    vector<hole_punch*> holes;
    vector<b_spline*> splines;

    vector<int> permitted_tools;

    SECTION("No hole punches") {
      params.tools = DRILL_AND_DRAG_KNIFE;
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
	REQUIRE(sections.size() == 2);
      }

      SECTION("Produces some code") {
	gprog c;
	gprog* r = append_footer(&c, params.target_machine);
	REQUIRE(p->size() > r->size());
      }
    }

    SECTION("3 separate lines, no hole punches") {
      lines.push_back(linear_cut::make(point(0, 0, 0), point(1, 0, 0)));
      lines.push_back(linear_cut::make(point(0, 2, 0), point(1, 6, 0)));
      lines.push_back(linear_cut::make(point(-3, 2, 0), point(0, 2, 0)));
      
      shape_layout l(lines, holes, splines);
      gprog* p = shape_layout_to_gcode(l, params);
      vector<cut_section> sections;
      extract_cuts(p, sections);
      REQUIRE(sections.size() == 6);
    }

    SECTION("Lines and hole punches") {
      lines.push_back(mk_linear_cut(point(0, 0, 0), point(1, 0, 0)));
      holes.push_back(mk_hole_punch(2, 2, 2, 0.125));
      
      shape_layout l(lines, holes, splines);
      
      SECTION("With drill and drag knife") {
	params.tools = DRILL_AND_DRAG_KNIFE;
	gprog* p = shape_layout_to_gcode(l, params);
	REQUIRE(check_for_forbidden_tool_changes(permitted_tools, p) == 2);
      }

      SECTION("With drill only") {
	params.tools = DRILL_ONLY;
	gprog* p = shape_layout_to_gcode(l, params);
	REQUIRE(check_for_forbidden_tool_changes(permitted_tools, p) == 1);
      }

      SECTION("Drill produces code for linear cuts") {
	params.tools = DRILL_ONLY;
	gprog* p = shape_layout_to_gcode(l, params);
	vector<cut_section> sections;
	extract_cuts(p, sections);
	REQUIRE(sections.size() == 2);
      }

      SECTION("Drag knife only produces code for linear cuts") {
	params.tools = DRAG_KNIFE_ONLY;
	gprog* p = shape_layout_to_gcode(l, params);
	vector<cut_section> sections;
	extract_cuts(p, sections);
	REQUIRE(sections.size() == 2);	
      }

      SECTION("DRAG_KNIFE_ONLY on means only the drag knife is used") {
	params.tools = DRAG_KNIFE_ONLY;
	gprog* p = shape_layout_to_gcode(l, params);
	REQUIRE(check_for_forbidden_tool_changes(permitted_tools, p) == 1);
      }
    }

    SECTION("One spline") {
      vector<point> control_points;
      control_points.push_back(point(1, 0, 0));
      control_points.push_back(point(2, 0, 0));
      
      vector<double> knots;
      knots.push_back(0.0);
      knots.push_back(0.0);
      knots.push_back(1.0);
      knots.push_back(1.0);

      int degree = 1;
      b_spline s(degree, control_points, knots);
      splines.push_back(&s);
      shape_layout l(lines, holes, splines);

      SECTION("Drill and drag knife produces code") {
	params.tools = DRILL_AND_DRAG_KNIFE;
	gprog* p = shape_layout_to_gcode(l, params);
	vector<cut_section> sections;
	extract_cuts(p, sections);
	REQUIRE(sections.size() == 2);
      }

      SECTION("Drill only produces code") {
	params.tools = DRILL_ONLY;
	gprog* p = shape_layout_to_gcode(l, params);
	vector<cut_section> sections;
	extract_cuts(p, sections);
	REQUIRE(sections.size() == 2);
      }
    }

    SECTION("Drill 2 holes") {
      params.tools = DRILL_AND_DRAG_KNIFE;
      holes.push_back(mk_hole_punch(1, 1, 1, 0.125));
      holes.push_back(mk_hole_punch(2, 2, 2, 0.125));
      vector<b_spline*> splines;
      shape_layout l(lines, holes, splines);

      gprog* p = shape_layout_to_gcode(l, params);

      toolpath dt = drill_toolpath(holes, params);
      REQUIRE(dt.cut_groups.size() == 2);
    }
  }

}
