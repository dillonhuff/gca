#include <sstream>

#include "analysis/extract_cuts.h"
#include "catch.hpp"
#include "checkers/bounds_checker.h"
#include "checkers/forbidden_tool_checker.h"
#include "checkers/unsafe_spindle_checker.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/shapes_to_toolpaths.h"
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

    shape_layout l = read_dxf(file_name.c_str());

    vector<block> p;
      
    SECTION("spiral is safe") {
      p = shape_layout_to_gcode(l, params);
      REQUIRE(check_for_unsafe_spindle_on(no_spindle_tools, 2, p) == 0);
    }
  }

  TEST_CASE("12 inch spiral splines are contiguous") {
    arena_allocator a;
    set_system_allocator(&a);

    cut_params params;
    params.default_feedrate = 30;
    params.set_default_feedrate = true;
    params.material_depth = 0.09;
    params.cut_depth = 0.05;
    params.safe_height = 0.25;
    params.machine_z_zero = -4.05;
    params.start_loc = point(1, 1, 0);
    params.start_orient = point(1, 0, 0);
    params.tools = DRAG_KNIFE_ONLY;
    params.target_machine = PROBOTIX_V90_MK2_VFD;

    string file_name =
      project_path + string("gca/test/dxf-files/12-inch-spiral.DXF");

    shape_layout lp = read_dxf(file_name.c_str());
    vector<cut*> lines;
    vector<hole_punch*> holes;
    shape_layout l(lines, holes, lp.splines);
    vector<block> p = shape_layout_to_gcode(l, params);

    vector<vector<machine_state>> sections;

    SECTION("2 paths for splines") {
      extract_cuts(p, sections);
      REQUIRE(sections.size() == 6);      
    }

    // SECTION("No standalone feedrate instructions, G53 moves, or toolchanges") {
    //   REQUIRE(count_if(p.begin(), p.end(), instr_is_forbidden_on_V90) == 0);
    // }

    // SECTION("All G1s have a feedrate") {
    //   REQUIRE(count_if(p.begin(), p.end(), g1_feedrate_omitted) == 0);
    // }
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

    vector<vector<machine_state>> sections;

    vector<block> p;
    
    SECTION("No hole punches") {
      params.tools = DRILL_AND_DRAG_KNIFE;
      lines.push_back(linear_cut::make(point(0, 0, 0), point(1, 0, 0)));
      shape_layout l(lines, holes, splines);
      p = shape_layout_to_gcode(l, params);

      SECTION("No use of drill when there are no hole punches") {
	permitted_tools.push_back(6);
	REQUIRE(check_for_forbidden_tool_changes(permitted_tools, p) == 0);
      }

      SECTION("Drill produces code for linear cuts") {
	p = shape_layout_to_gcode(l, params);
	extract_cuts(p, sections);
	REQUIRE(sections.size() == 2);
      }

      SECTION("Produces some code") {
	p = shape_layout_to_gcode(l, params);
	vector<block> blocks;
	append_footer_blocks(blocks, params.target_machine);
	REQUIRE(p.size() >= blocks.size());
      }
    }

    SECTION("3 separate lines, no hole punches") {
      lines.push_back(linear_cut::make(point(0, 0, 0), point(1, 0, 0)));
      lines.push_back(linear_cut::make(point(0, 2, 0), point(1, 6, 0)));
      lines.push_back(linear_cut::make(point(-3, 2, 0), point(0, 2, 0)));
      
      shape_layout l(lines, holes, splines);
      p = shape_layout_to_gcode(l, params);
      extract_cuts(p, sections);
      REQUIRE(sections.size() == 10);
    }

    SECTION("Lines and hole punches") {
      lines.push_back(linear_cut::make(point(0, 0, 0), point(1, 0, 0)));
      holes.push_back(hole_punch::make(point(2, 2, 2), 0.125));
      
      shape_layout l(lines, holes, splines);
      
      SECTION("With drill and drag knife") {
	params.tools = DRILL_AND_DRAG_KNIFE;
	p = shape_layout_to_gcode(l, params);
	REQUIRE(check_for_forbidden_tool_changes(permitted_tools, p) == 2);
      }

      SECTION("With drill only") {
	params.tools = DRILL_ONLY;
	p = shape_layout_to_gcode(l, params);
	REQUIRE(check_for_forbidden_tool_changes(permitted_tools, p) == 1);
      }

      SECTION("Drill produces code for linear cuts") {
	params.tools = DRILL_ONLY;
	p = shape_layout_to_gcode(l, params);
	extract_cuts(p, sections);
	REQUIRE(sections.size() == 3);
      }

      SECTION("Drag knife only produces code for linear cuts") {
	params.tools = DRAG_KNIFE_ONLY;
	p = shape_layout_to_gcode(l, params);
	extract_cuts(p, sections);
	REQUIRE(sections.size() == 2);	
      }

      SECTION("DRAG_KNIFE_ONLY on means only the drag knife is used") {
	params.tools = DRAG_KNIFE_ONLY;
	p = shape_layout_to_gcode(l, params);
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

      vector<block> p;

      SECTION("Drill and drag knife produces code") {
	params.tools = DRILL_AND_DRAG_KNIFE;
	p = shape_layout_to_gcode(l, params);
	extract_cuts(p, sections);
	REQUIRE(sections.size() == 2);
      }

      SECTION("Drill only produces code") {
	params.tools = DRILL_ONLY;
	p = shape_layout_to_gcode(l, params);
	extract_cuts(p, sections);
	REQUIRE(sections.size() == 2);
      }
    }

    SECTION("Drill 2 holes") {
      params.tools = DRILL_AND_DRAG_KNIFE;
      holes.push_back(hole_punch::make(point(1, 1, 1), 0.125));
      holes.push_back(hole_punch::make(point(2, 2, 2), 0.125));
      vector<b_spline*> splines;
      shape_layout l(lines, holes, splines);

      p = shape_layout_to_gcode(l, params);

      vector<cut*> dt = shape_cuts(l, params);
      REQUIRE(dt.size() == 2);
    }
  }

  TEST_CASE("Probotix, drill only gcode") {
    arena_allocator a;
    set_system_allocator(&a);

    double x_init = 5.5;
    double y_init = 6.5;
    
    cut_params params;
    params.safe_height = 0.35;
    params.material_depth = 0.011;
    params.cut_depth = 0.01;
    params.start_loc = point(x_init, y_init, 0.0);
    params.default_feedrate = 20;
    params.machine_z_is_inverted = true;
    params.machine_z_zero = -4.05;
    params.target_machine = PROBOTIX_V90_MK2_VFD;
    params.tools = DRILL_ONLY;

    vector<cut*> lines;
    lines.push_back(linear_cut::make(point(2, 3, 0), point(1, 3, 0)));
    vector<hole_punch*> holes;
    vector<b_spline*> splines;
    shape_layout l(lines, holes, splines);

    vector<block> p;

    SECTION("Inside safe machine bounds") {
      p = shape_layout_to_gcode(l, params);
      REQUIRE(check_bounds(p, GCA_ABSOLUTE,
			   1, 12,
			   1, 10,
			   -4.1, -0.05) == 0);
      
    }

    SECTION("Inside safe bounds with different machine_z_zero") {
      params.machine_z_zero = -1.0;
      p = shape_layout_to_gcode(l, params);
      REQUIRE(check_bounds(p, GCA_ABSOLUTE,
			   1, 12,
			   1, 10,
			   -2.1, -0.05) == 0);  
    }
  }

}
