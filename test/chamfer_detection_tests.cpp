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

      tool t2(0.1, 3.0, 4, HSS, CHAMFER);
      t2.set_chamfer_included_angle(45.0*2.0);
    
      t2.set_cut_diameter(0.5);
      t2.set_cut_length(1.0);

      t2.set_shank_diameter(3.0 / 8.0);
      t2.set_shank_length(0.1);

      t2.set_holder_diameter(2.0);
      t2.set_holder_length(2.5);
      t2.set_tool_number(2);

      tool t3(0.1, 3.0, 4, HSS, CHAMFER);
      t3.set_chamfer_included_angle(36.0*2.0);
    
      t3.set_cut_diameter(0.5);
      t3.set_cut_length(1.0);

      t3.set_shank_diameter(3.0 / 8.0);
      t3.set_shank_length(0.1);

      t3.set_holder_diameter(2.0);
      t3.set_holder_length(2.5);
      t3.set_tool_number(3);

      vector<tool> tools{t2, t3};
      
      vector<chamfer> chamfer_surfaces =
	chamfer_regions(three_chamfer_mesh, n, tools);


      // for (auto& surf : chamfer_surfaces) {
      // 	vtk_debug_highlight_inds(surf.faces, three_chamfer_mesh);
      // }

      REQUIRE(chamfer_surfaces.size() == 3);
    }

    SECTION("One chamfer and one fillet") {

      triangular_mesh one_chamfer_mesh =
	parse_stl("./test/stl-files/OneChamferOneFillet.stl", 0.0001);

      point n(0, 0, 1);

      SECTION("With chamfer angle 45") {
	//	vector<double> chamfer_angles{45.0};

	tool t2(0.1, 3.0, 4, HSS, CHAMFER);
	t2.set_chamfer_included_angle(45.0*2.0);
    
	t2.set_cut_diameter(0.5);
	t2.set_cut_length(1.0);

	t2.set_shank_diameter(3.0 / 8.0);
	t2.set_shank_length(0.1);

	t2.set_holder_diameter(2.0);
	t2.set_holder_length(2.5);
	t2.set_tool_number(2);

	vector<tool> tools{t2};
      
	vector<chamfer> chamfer_surfaces =
	  chamfer_regions(one_chamfer_mesh, n, tools);

	// for (auto& surf : chamfer_surfaces) {
	//   vtk_debug_highlight_inds(surf.faces, one_chamfer_mesh);
	// }

	REQUIRE(chamfer_surfaces.size() == 1);
      }

      SECTION("Without chamfer angle 45") {

	vector<tool> tools{};
      
	vector<chamfer> chamfer_surfaces =
	  chamfer_regions(one_chamfer_mesh, n, tools);

	REQUIRE(chamfer_surfaces.size() == 0);
      }

    }

  }

}
