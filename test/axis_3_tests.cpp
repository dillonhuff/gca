#include "catch.hpp"
#include "synthesis/axis_3.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Triangle preprocessing") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("CylinderChimneySlot") {

      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderChimneySlot.stl").triangles;
      auto polygons = preprocess_triangles(triangles);
      stable_sort(begin(polygons), end(polygons),
		  [](const oriented_polygon& x,
		     const oriented_polygon& y)
		  { return x.height() < y.height(); });

      SECTION("One polygon per face") {//, and 1 more for the cylinder base") {
	REQUIRE(polygons.size() == 4);
      }

      SECTION("Top polygon has height 0.35") {
	double top_polygon_height = polygons.back().height();
	cout << "top height = " << top_polygon_height << endl;
	REQUIRE(within_eps(top_polygon_height, 0.35, 0.00001));
      }
      
    }
  }

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

      SECTION("First pocket has one boundary") {
	REQUIRE(pockets.front().get_boundaries().size() == 1);
      }

      SECTION("First pocket ends at 0.200001") {
	REQUIRE(within_eps(pockets.front().get_end_depth(), 0.200001) );
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

    SECTION("CylinderChimneySlot") {
      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderChimneySlot.stl").triangles;
      auto polygons = preprocess_triangles(triangles);
      auto pockets = make_pockets(polygons);

      SECTION("4 pockets") {
	REQUIRE(pockets.size() == 4);
      }
      
    }
  }

  TEST_CASE("Milling surfaces") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Rectangle cylinder") {
      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderSquare.stl").triangles;
      double tool_diameter = 0.05;
      double cut_depth = 0.25;
      auto mill_paths = mill_surface(triangles, tool_diameter, cut_depth);
      REQUIRE(mill_paths.size() > 0);
    }

    SECTION("Multiple cylinders") {
      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/MultipleCylinders.stl").triangles;
      double tool_diameter = 0.2;
      double cut_depth = 0.5;
      auto mill_lines = mill_surface_lines(triangles, tool_diameter, cut_depth);
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
