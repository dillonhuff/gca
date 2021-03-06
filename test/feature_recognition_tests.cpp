#include "catch.hpp"
#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "synthesis/contour_planning.h"
#include "synthesis/fixture_analysis.h"
#include "system/parse_stl.h"
#include "utils/arena_allocator.h"

namespace gca {

  TEST_CASE("Part Studio 4 Part 1") {
    arena_allocator a;
    set_system_allocator(&a);

    auto mesh =
      parse_and_scale_stl("test/stl-files/onshape_parts/Part Studio 4 - Part 1.stl", 0.25, 0.001);

    point n(0, 0, 1);

    workpiece wp(1.5, 1.5, 2.5, ALUMINUM);

    vector<surface> stable_surfaces = outer_surfaces(mesh);
    triangular_mesh stock = align_workpiece(stable_surfaces, wp);
    
    feature_decomposition* f =
      build_feature_decomposition(stock, mesh, n);

    REQUIRE(f->num_levels() == 7);

    REQUIRE(f->num_features() == 7);

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

    REQUIRE(within_eps(current_min, base_depth));
  }
  

  TEST_CASE("Arm joint top") {
    arena_allocator a;
    set_system_allocator(&a);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Arm_Joint_Top.stl", 0.001);

    point n(0, -1, 0);
    
    feature_decomposition* f =
      build_feature_decomposition(mesh, n);

    REQUIRE(f->num_levels() == 4);

    REQUIRE(f->num_features() == 8);

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

    REQUIRE(within_eps(current_min, base_depth));

    int num_through_features = 0;
    for (auto f : collect_features(f)) {
      if (f->is_through()) {
	num_through_features++;
      }
    }
    REQUIRE(num_through_features == 6);
  }

  TEST_CASE("Rectangle with circular notch") {
    arena_allocator a;
    set_system_allocator(&a);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/RectangleWithCircularNotch.stl", 0.001);

    point n(0, -1, 0);

    surface_levels ls = initial_surface_levels({mesh}, n);

    REQUIRE(ls.size() == 1);
    REQUIRE(ls.front().size() == 2);

    feature_decomposition* f =
      build_feature_decomposition(mesh, n);

    REQUIRE(f->num_levels() == 2);

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

    REQUIRE(!within_eps(current_min, base_depth));
    
  }

  // NOTE: Removed since this will not be a tested case in the final system
  // TEST_CASE("Magnetic latch top") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca//test/stl-files/onshape_parts/Magnetic Latch Top - Part 1.stl", 0.001);

  //   point n(0, 0, -1);

  //   feature_decomposition* f =
  //     build_feature_decomposition(mesh, n);

  //   //vtk_debug_feature_tree(f);

  //   REQUIRE(f->num_levels() == 6);

  //   double current_min = 100000;
  //   auto replace_min = [&current_min](feature* f) {
  //     if (f != nullptr) {
  // 	double bz = f->base_distance_along_normal();
  // 	if (bz < current_min) {
  // 	  current_min = bz;
  // 	}
  //     }
  //   };
  //   traverse_bf(f, replace_min);

  //   double base_depth = min_distance_along(mesh.vertex_list(), n);

  //   cout << "current min = " << current_min << endl;
  //   cout << "base depth  = " << base_depth <<  endl;

  //   REQUIRE(within_eps(current_min, base_depth, 0.0001));
  // }

  TEST_CASE("PSU Mount") {
    arena_allocator a;
    set_system_allocator(&a);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", 0.0001);

    point n(-1, 0, 0);

    feature_decomposition* f =
      build_feature_decomposition(mesh, n);

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
