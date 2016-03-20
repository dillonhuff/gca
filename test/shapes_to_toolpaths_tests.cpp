#include "catch.hpp"
#include "core/arena_allocator.h"
#include "synthesis/linear_cut.h"
#include "synthesis/shapes_to_toolpaths.h"

namespace gca {

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
	lines.push_back(linear_cut::make(p0, p1));
	lines.push_back(linear_cut::make(p1, p2));
	lines.push_back(linear_cut::make(p2, p3));
	lines.push_back(linear_cut::make(p3, p0));	
	shape_layout l(lines, holes, splines);
	cuts = shape_cuts(l, params);
	REQUIRE(cuts.size() == 8);
      }

    }
  }
}
