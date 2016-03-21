#include "catch.hpp"
#include "core/arena_allocator.h"
#include "synthesis/linear_cut.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/shapes_to_toolpaths.h"

namespace gca {

  bool is_vertical_or_horizontal(const cut* c) {
    if (c->is_linear_cut()) {
      return (c->end.x == c->start.x) || (c->end.y == c->start.y);
    }
    return true;
  }

  TEST_CASE("Lines to toolpaths") {
    arena_allocator a;
    set_system_allocator(&a);

    cut_params params;

    vector<cut*> lines;
    vector<hole_punch*> holes;
    vector<b_spline*> splines;

    vector<cut*> cuts;

    SECTION("Drill and drag knife") {
      params.safe_height = 0.35;
      params.material_depth = 0.09;
      params.cut_depth = 0.05;
      params.push_depth = 0.005;
      params.start_loc = point(0, 0, 0);
      params.start_orient = point(1, 0, 0);
      params.target_machine = CAMASTER;
      params.tools = DRILL_AND_DRAG_KNIFE;
      
      SECTION("2 cuts") {
	lines.push_back(linear_cut::make(point(1, 0, 0), point(2, 0, 0)));
	shape_layout l(lines, holes, splines);
	cuts = shape_cuts(l, params);
	REQUIRE(cuts.size() == 2);
      }
      
      SECTION("4 cuts") {
	lines.push_back(linear_cut::make(point(1, 0, 0), point(2, 0, 0)));
	lines.push_back(linear_cut::make(point(5, 0, 0), point(6, 0, 0)));
	shape_layout l(lines, holes, splines);
	cuts = shape_cuts(l, params);
	REQUIRE(cuts.size() == 4);
      }

    }

    SECTION("Probotix") {
      params.safe_height = 0.35;
      params.material_depth = 0.011;
      params.cut_depth = 0.05;
      params.start_loc = point(0, 0, 0);
      params.start_orient = point(1, 0, 0);
      params.target_machine = PROBOTIX_V90_MK2_VFD;
      params.machine_z_is_inverted = true;
      params.machine_z_zero = -4.05;
      
      SECTION("DRILL_ONLY, 1 cut") {
	params.tools = DRILL_ONLY;
	lines.push_back(linear_cut::make(point(1, 0, 0), point(3, 4, 0)));
	shape_layout l(lines, holes, splines);
	cuts = shape_cuts(l, params);
	REQUIRE(cuts.size() == 1);
      }

      SECTION("DRAG_KNIFE_ONLY, cut square") {
	params.tools = DRAG_KNIFE_ONLY;
	double d = 2.0;
	point bottom_left_corner = point(9.4, 4.7, 0);
	point p0 = bottom_left_corner;
	point p1 = p0 + point(0, d, 0);
	point p2 = p0  + point(d, d, 0);
	point p3 = p0 + point(d, 0, 0);

	SECTION("Lines are in cut order") {
	  lines.push_back(linear_cut::make(p0, p1));
	  lines.push_back(linear_cut::make(p1, p2));
	  lines.push_back(linear_cut::make(p2, p3));
	  lines.push_back(linear_cut::make(p3, p0));	
	  shape_layout l(lines, holes, splines);
	  cuts = shape_cuts(l, params);

	  SECTION("Adjacent cuts get broken up") {
	    REQUIRE(cuts.size() == 8);
	  }

	  SECTION("All lines in the square are vertical or horizontal") {
	    REQUIRE(all_of(cuts.begin(), cuts.end(), is_vertical_or_horizontal));
	  }
	}

	SECTION("Lines are not in cut order") {
	  lines.push_back(linear_cut::make(p0, p1));
	  lines.push_back(linear_cut::make(p3, p0));
	  lines.push_back(linear_cut::make(p2, p3));
	  lines.push_back(linear_cut::make(p1, p2));
	  
	  shape_layout l(lines, holes, splines);
	  cuts = shape_cuts(l, params);

	  SECTION("Adjacent cuts get broken up") {
	    REQUIRE(cuts.size() == 8);
	  }

	  SECTION("All lines in the square are vertical or horizontal") {
	    REQUIRE(all_of(cuts.begin(), cuts.end(), is_vertical_or_horizontal));
	  }
	}
      }
    }

    SECTION("Probotix, multiple passes") {
      params.material_depth = 0.099;
      params.push_depth = 0.00;
      params.cut_depth = 0.05;
      params.safe_height = 0.45;
      params.machine_z_zero = -1.90425;
      params.start_loc = point(9, 8, 0);
      params.start_orient = point(0, -1, 0);
      params.tools = DRAG_KNIFE_ONLY;
      params.target_machine = PROBOTIX_V90_MK2_VFD;

      // Add outer rectangle
      double d = 2.0;
      point bottom_left_corner = point(9.4, 4.7, 0);
      point p0 = bottom_left_corner;
      point p1 = p0 + point(0, d, 0);
      point p2 = p0  + point(d, d, 0);
      point p3 = p0 + point(d, 0, 0);
      lines.push_back(linear_cut::make(p0, p1));
      lines.push_back(linear_cut::make(p1, p2));
      lines.push_back(linear_cut::make(p2, p3));
      lines.push_back(linear_cut::make(p3, p0));

      vector<hole_punch*> holes;
      vector<b_spline*> splines;
      shape_layout l(lines, holes, splines);
      cuts = shape_cuts(l, params);
      string s = shape_layout_to_gcode_string(l, params);
      cout << "Standard gcode: " << endl;
      cout << s << endl;

      SECTION("Adjacent cuts get broken up") {
	REQUIRE(cuts.size() == 12);
      }

      SECTION("All lines in the square are vertical or horizontal") {
	REQUIRE(all_of(cuts.begin(), cuts.end(), is_vertical_or_horizontal));
      }
    }
  }
}
