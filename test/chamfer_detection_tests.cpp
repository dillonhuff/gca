#include "catch.hpp"
#include "feature_recognition/chamfer_detection.h"
#include "geometry/vtk_debug.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/workpiece_clipping.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Chamfer detection") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Three chamfers") {

      triangular_mesh three_chamfer_mesh =
	parse_stl("./test/stl-files/ThreeChamfers.stl", 0.0001);

      point n(0, 0, 1);

      vector<double> chamfer_angles{45.0, 53.0};

      vector<vector<index_t>> chamfer_surfaces =
	chamfer_regions(three_chamfer_mesh, n, chamfer_angles);


      for (auto& surf : chamfer_surfaces) {
	vtk_debug_highlight_inds(surf, three_chamfer_mesh);
      }

      REQUIRE(chamfer_surfaces.size() == 3);
    }

    SECTION("One chamfer and one fillet") {

      triangular_mesh one_chamfer_mesh =
	parse_stl("./test/stl-files/OneChamferOneFillet.stl", 0.0001);

      point n(0, 0, 1);

      SECTION("With chamfer angle 45") {
	vector<double> chamfer_angles{45.0};
      
	vector<vector<index_t>> chamfer_surfaces =
	  chamfer_regions(one_chamfer_mesh, n, chamfer_angles);

	for (auto& surf : chamfer_surfaces) {
	  vtk_debug_highlight_inds(surf, one_chamfer_mesh);
	}

	REQUIRE(chamfer_surfaces.size() == 1);
      }

      SECTION("Without chamfer angle 45") {
	vector<double> chamfer_angles{57.0, 60.0, 90.0};
      
	vector<vector<index_t>> chamfer_surfaces =
	  chamfer_regions(one_chamfer_mesh, n, chamfer_angles);

	REQUIRE(chamfer_surfaces.size() == 0);
      }

    }

  }

}
