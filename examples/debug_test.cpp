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

    SECTION("CylinderSquare") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderSquare.stl", 0.001);

      feature_decomposition* f = build_feature_decomposition(mesh, point(0, 0, 1));
      tool_access_info tool_info = find_accessable_tools(f, tools);
      vector<pocket> pockets = feature_pockets(*f, tool_info);

      SECTION("Two pockets") {
	REQUIRE(pockets.size() == 2);
      }

      SECTION("One pocket ends at 0.200001") {
	REQUIRE(any_of(begin(pockets), end(pockets),
		       [](const pocket& p) {
			 return within_eps(p.get_end_depth(), 0.200001);
		       }));
      }

      SECTION("No pockets more than 0.75 inches tall") {
	REQUIRE(all_of(begin(pockets), end(pockets),
		       [](const pocket& p) {
			 return p.get_start_depth() - p.get_end_depth() < 0.75;
		       }));
      }

      SECTION("First pocket has no holes") {
      	REQUIRE(pockets.front().get_holes().size() == 0);
      }

      SECTION("Last pocket has one hole") {
      	REQUIRE(pockets.back().get_holes().size() == 1);
      }
      
    }

    SECTION("Arm joint top") {
      point n(0, -1, 0);
      
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Arm_Joint_Top.stl", 0.001);

      auto surfs = outer_surfaces(mesh);
      triangular_mesh stock = align_workpiece(surfs, w);

      feature_decomposition* f = build_feature_decomposition(stock, mesh, n);

      REQUIRE(collect_features(f).size() == 9);

      tool_access_info tool_info = find_accessable_tools(f, tools);
      vector<pocket> pockets = feature_pockets(*f, n, tool_info);

      REQUIRE(pockets.size() == collect_features(f).size());
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
