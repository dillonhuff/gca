#include "catch.hpp"
#include "geometry/vtk_debug.h"
#include "synthesis/contour_planning.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Contouring") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Tapered extruded top and side cannot be contoured") {
      triangular_mesh m = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/TaperedExtrudedTopSide.stl", 0.001);

      auto decomp = compute_contour_surfaces(m);

      REQUIRE(!decomp);
    }

    SECTION("Arm joint") {
      triangular_mesh m = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Arm_Joint_Top.stl", 0.001);

      auto decomp = compute_contour_surfaces(m);

      REQUIRE(decomp);
      REQUIRE(within_eps(decomp->n, point(0, -1, 0), 0.001));
      REQUIRE(decomp->rest.size() == 0);

      SECTION("Outline 1 region connected to top and bottom") {
	vector<surface> rs =
	  regions_connected_to_both(decomp->outline, decomp->top, decomp->bottom);

	REQUIRE(rs.size() == 1);

	REQUIRE(rs.front().surface_area() < decomp->outline.surface_area());

	REQUIRE(rs.front().edges().size() > 0);
	for (auto e : rs.front().edges()) {
	  REQUIRE(rs.front().edge_face_neighbors(e).size() > 0);
	}

	REQUIRE(shared_edges(rs.front(), decomp->bottom).size() > 0);
      }
    }

    SECTION("Clipped Cylinder") {
      triangular_mesh m = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ClippedCylinder.stl", 0.001);

      auto decomp = compute_contour_surfaces(m);

      REQUIRE(decomp);
      REQUIRE(within_eps(decomp->n, point(0, 0, 1), 0.001));
      REQUIRE(decomp->rest.size() == 0);

      SECTION("Outline 1 region connected to top and bottom") {
	vector<surface> rs =
	  regions_connected_to_both(decomp->outline, decomp->top, decomp->bottom);

	REQUIRE(rs.size() == 1);

	REQUIRE(rs.front().surface_area() < decomp->outline.surface_area());

	REQUIRE(rs.front().edges().size() > 0);
	for (auto e : rs.front().edges()) {
	  REQUIRE(rs.front().edge_face_neighbors(e).size() > 0);
	}

	REQUIRE(shared_edges(rs.front(), decomp->bottom).size() > 0);
      }
    }

    SECTION("OnShape Part 1 1") {
      triangular_mesh m = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/Part Studio 1 - Part 1(1).stl", 0.001);

      auto decomp = compute_contour_surfaces(m);

      REQUIRE(decomp);
      REQUIRE(within_eps(decomp->n, point(0, -1, 0), 0.001));
      REQUIRE(decomp->rest.size() == 0);
    }
  }
}
