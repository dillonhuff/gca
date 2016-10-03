#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "geometry/mesh_operations.h"
#include "geometry/vtk_debug.h"
#include "synthesis/fixture_analysis.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Single Shot Tray") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = emco_vice(point(1.3, -4.4, 3.3));
    std::vector<plate_height> plates{0.1, 0.3};
    fixtures fixes(test_vice, plates);

    workpiece workpiece_dims(2.0, 2.0, 3.98, ALUMINUM);

    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.25);
    t1.set_cut_length(0.6);

    t1.set_shank_diameter(3.0 / 8.0);
    t1.set_shank_length(0.3);

    t1.set_holder_diameter(2.5);
    t1.set_holder_length(3.5);
    
    tool t2(0.5, 3.0, 4, HSS, FLAT_NOSE);
    t2.set_cut_diameter(0.5);
    t2.set_cut_length(0.3);

    t2.set_shank_diameter(0.5);
    t2.set_shank_length(0.5);

    t2.set_holder_diameter(2.5);
    t2.set_holder_length(3.5);

    tool t3{0.2334, 3.94, 4, HSS, FLAT_NOSE};
    t3.set_cut_diameter(0.12);
    t3.set_cut_length(1.2);

    t3.set_shank_diameter(0.5);
    t3.set_shank_length(0.05);

    t3.set_holder_diameter(2.5);
    t3.set_holder_length(3.5);
    
    vector<tool> tools{t1, t2, t3};

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/Part Studio 1 - SSTray.stl", 0.001);

    SECTION("Feature decomposition in (0, 1, 0)") {
      vector<surface> stable_surfaces = outer_surfaces(mesh);
      triangular_mesh wp_mesh = align_workpiece(stable_surfaces,
						workpiece_dims);

      auto fd = build_feature_decomposition(wp_mesh, mesh, point(0, 1, 0));

      // for (auto f : collect_features(fd)) {
      // 	vtk_debug_feature(*f);
      // }

      // vtk_debug_feature_decomposition(fd);

      REQUIRE(fd);
    }
    
    SECTION("Align workpiece around part") {
      vector<surface> stable_surfaces = outer_surfaces(mesh);

      // vtk_debug_highlight_inds(stable_surfaces);
      
      triangular_mesh wp_mesh = align_workpiece(stable_surfaces,
						workpiece_dims);

      vector<triangular_mesh> to_sub{wp_mesh};
      vector<triangular_mesh> res = boolean_difference(mesh, to_sub);

      vector<triangular_mesh> to_sub_next{mesh};
      vector<triangular_mesh> res_sub = boolean_difference(wp_mesh, to_sub_next);

      // for (auto m : res_sub) {
      // 	vtk_debug_mesh(m);
      // }
      
      REQUIRE(res.size() == 0);
    }

    SECTION("Feature decomposition in (0, 0, 1)") {
      vector<surface> stable_surfaces = outer_surfaces(mesh);
      triangular_mesh wp_mesh = align_workpiece(stable_surfaces,
						workpiece_dims);

      auto fd = build_feature_decomposition(wp_mesh, mesh, point(0, 0, 1));

      REQUIRE(fd);
    }

    SECTION("Make fixture plan") {

      fixture_plan p = make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

      REQUIRE(p.fixtures().size() == 2);
    }
  }

}
