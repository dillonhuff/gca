#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "feature_recognition/feature_decomposition.h"
#include "process_planning/feature_to_pocket.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/millability.h"
#include "synthesis/workpiece.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Pocketing") {
    arena_allocator a;
    set_system_allocator(&a);

    double workpiece_depth = 1.0;

    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.25);
    t1.set_cut_length(0.6);

    t1.set_shank_diameter(3.0 / 8.0);
    t1.set_shank_length(0.3);

    t1.set_holder_diameter(2.5);
    t1.set_holder_length(3.5);

    tool t2(0.1, 3.0, 4, HSS, FLAT_NOSE);
    t2.set_cut_diameter(0.1);
    t2.set_cut_length(0.6);

    t2.set_shank_diameter(3.0 / 8.0);
    t2.set_shank_length(0.3);

    t2.set_holder_diameter(2.5);
    t2.set_holder_length(3.5);
    
    vector<tool> tools{t1, t2};

    workpiece w(3.0, 3.0, 3.0, ALUMINUM);
    
    SECTION("Arm joint top") {
      point n(0, -1, 0);
      
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Arm_Joint_Top.stl", 0.001);

      auto surfs = outer_surfaces(mesh);
      triangular_mesh stock = align_workpiece(surfs, w);

      feature_decomposition* f = build_feature_decomposition(stock, mesh, n);
      tool_access_info tool_info = find_accessable_tools(f, tools);
      vector<pocket> pockets = feature_pockets(*f, n, tool_info);

      REQUIRE(pockets.size() == 8);
    }

    // TODO: Possible reintroduce this test when outline detection is improved
    // SECTION("CylinderChimneySlot") {
    //   auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderChimneySlot.stl", 0.001);
    //   feature_decomposition* f = build_feature_decomposition(mesh, point(0, 0, 1));
    //   vector<pocket> pockets = feature_pockets(*f);
      
    //   SECTION("4 pockets") {
    // 	REQUIRE(pockets.size() == 4);
    //   }
    // }

  }

}
