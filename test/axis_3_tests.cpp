#include "catch.hpp"
#include "synthesis/axis_3.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Pocketing") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("CylinderSquare") {
      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderSquare.stl").triangles;
      auto polygons = preprocess_triangles(triangles);
      auto pockets = make_pockets(polygons);

      SECTION("Two pockets") {
	REQUIRE(pockets.size() == 2);
      }

      cout << "pocket 0 start depth = " << pockets[0].start_depth << endl;
      cout << "pocket 0 end depth = " << pockets[0].end_depth << endl;
      cout << "pocket 1 start depth = " << pockets[1].start_depth << endl;
      cout << "pocket 1 end depth = " << pockets[1].end_depth << endl;

      SECTION("First pocket has one boundary") {
	REQUIRE(pockets.front().get_boundaries().size() == 1);
      }

      SECTION("First pocket ends at 0.200001") {
	REQUIRE(within_eps(pockets.front().end_depth, 0.200001) );
      }

      SECTION("First pocket has no holes") {
	REQUIRE(pockets.front().get_holes().size() == 0);
      }

      SECTION("Last pocket has 1 boundary") {
	REQUIRE(pockets.back().get_boundaries().size() == 1);
      }

      SECTION("Last pocket has one hole") {
	REQUIRE(pockets.back().get_holes().size() == 1);
      }
    }
  }

  TEST_CASE("Milling surfaces") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Rectangle cylinder") {
      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderSquare.stl").triangles;
      double tool_diameter = 0.05;
      auto mill_paths = mill_surface(triangles, tool_diameter);
      REQUIRE(mill_paths.size() > 0);
    }

    SECTION("Multiple cylinders") {
      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/MultipleCylinders.stl").triangles;
      double tool_diameter = 0.2;
      auto mill_lines = mill_surface_lines(triangles, tool_diameter);
      REQUIRE(mill_lines.size() > 0);
    }
  }

  TEST_CASE("Computing finishes") {
    arena_allocator a;
    set_system_allocator(&a);

    // SECTION("Chimney shape") {
    //   vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderChimneySlot.stl").triangles;
    //   vector<pocket_info_2P5D> finish_outlines = surface_finishes(triangles);
    //   REQUIRE(finish_outlines.size() == 1);
    // }
  }
}
