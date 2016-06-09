#include "catch.hpp"
#include "synthesis/fixture_analysis.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Complex rectangular part") {
    arena_allocator a;
    set_system_allocator(&a);
    vice test_vice = emco_vice(point(-0.8, -4.4, -3.3));
    workpiece workpiece_dims(3.5, 2.5, 2.3);
    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ComplexRectanglePart1.stl", 0.001);
    
    SECTION("surface allocation requires 4 arrangements") {
      auto outer_surfs = outer_surfaces(mesh);
      auto aligned_workpiece = align_workpiece(outer_surfs, workpiece_dims);
      classify_part_surfaces(outer_surfs, aligned_workpiece);
      vector<pair<stock_orientation, surface_list>> meshes =
	orientations_to_cut(mesh, outer_surfs);
      REQUIRE(meshes.size() == 4);
    }
  }
}
