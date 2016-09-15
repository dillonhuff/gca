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

    SECTION("OnShape PSU Mount") {
      triangular_mesh m = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", 0.0001);

      auto decomp = compute_contour_surfaces(m);

      REQUIRE(decomp);
      REQUIRE(within_eps(decomp->n, point(0, 0, 1), 0.001));
      REQUIRE(decomp->rest.size() == 0);
    }

    SECTION("OnShape Part 1 1") {
      triangular_mesh m = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/Part Studio 1 - Part 1(1).stl", 0.001);

      auto decomp = compute_contour_surfaces(m);

      REQUIRE(decomp);
      REQUIRE(within_eps(decomp->n, point(0, -1, 0), 0.001));
      REQUIRE(decomp->rest.size() == 0);
    }

    SECTION("OnShape Part 1 2") {
      triangular_mesh m = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/Part Studio 1 - Part 1(2).stl", 0.001);

      auto decomp = compute_contour_surfaces(m);

      REQUIRE(decomp);
      REQUIRE(within_eps(decomp->n, point(0, 0, 1), 0.001));
      REQUIRE(decomp->rest.size() == 0);
    }

  }
}
