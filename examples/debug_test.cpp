#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "synthesis/contour_planning.h"
#include "system/parse_stl.h"
#include "utils/arena_allocator.h"

namespace gca {

  TEST_CASE("PSU Mount") {
    arena_allocator a;
    set_system_allocator(&a);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", 0.0001);

    point n(-1, 0, 0);

    feature_decomposition* f =
      build_feature_decomposition(mesh, n);

    vtk_debug_feature_tree(f);

    REQUIRE(f->num_levels() == 3);

    double current_min = 100000;
    auto replace_min = [&current_min](feature* f) {
      if (f != nullptr) {
	double bz = f->base_distance_along_normal();
	if (bz < current_min) {
	  current_min = bz;
	}
      }
    };
    traverse_bf(f, replace_min);

    double base_depth = min_distance_along(mesh.vertex_list(), n);

    cout << "current min = " << current_min << endl;
    cout << "base depth  = " << base_depth <<  endl;

    REQUIRE(within_eps(current_min, base_depth, 0.0001));
  }
  
}
