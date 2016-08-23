#include "catch.hpp"
#include "feature_recognition/feature_decomposition.h"
#include "process_planning/feature_to_pocket.h"
#include "synthesis/axis_3.h"
#include "synthesis/millability.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Pocketing") {
    arena_allocator a;
    set_system_allocator(&a);

    double workpiece_depth = 1.0;

    SECTION("CylinderSquare") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderSquare.stl", 0.001);
      feature_decomposition* f = build_feature_decomposition(mesh, point(0, 0, 1));
      vector<pocket> pockets = feature_pockets(*f);

      SECTION("Two pockets") {
	REQUIRE(pockets.size() == 2);
      }

      SECTION("One pocket ends at 0.200001") {
	REQUIRE(any_of(begin(pockets), end(pockets),
		       [](const pocket& p) {
			 return within_eps(p.get_end_depth(), 0.200001);
		       }));
      }

      SECTION("First pocket has no holes") {
      	REQUIRE(pockets.front().get_holes().size() == 0);
      }

      SECTION("Last pocket has one hole") {
      	REQUIRE(pockets.back().get_holes().size() == 1);
      }
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
