#define CATCH_CONFIG_MAIN

#include "analysis/gcode_to_cuts.h"
#include "catch.hpp"
#include "geometry/triangular_mesh.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/workpiece_clipping.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  void sanity_check_toolpaths(const fabrication_plan& plan) {
    for (auto step : plan.steps()) {
      REQUIRE(step.toolpaths().size() > 0);
    }
  }

  TEST_CASE("Box with hole") {
    arena_allocator a;
    set_system_allocator(&a);

    // TODO: Make emco_vice again
    vice test_vice = large_jaw_vice(5, point(-0.8, -4.4, -3.3));
    std::vector<plate_height> parallel_plates{0.1, 0.3};
    fixtures fixes(test_vice, parallel_plates);

    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.25);
    t1.set_cut_length(0.6);

    t1.set_shank_diameter(3.0 / 8.0);
    t1.set_shank_length(0.3);

    t1.set_holder_diameter(2.5);
    t1.set_holder_length(3.5);
    
    tool t2(0.5, 3.0, 4, HSS, FLAT_NOSE);
    t2.set_cut_diameter(0.5);
    t2.set_cut_length(1.3);

    t2.set_shank_diameter(0.5);
    t2.set_shank_length(0.5);

    t2.set_holder_diameter(2.5);
    t2.set_holder_length(3.5);

    vector<tool> tools{t1, t2};
    workpiece workpiece_dims(3.0, 3.0, 3.0, ACETAL); //1.5, 1.2, 1.5, ACETAL);

    SECTION("Box with hole has 2 clippings") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithTopHole.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 2);
    }

    SECTION("Box with two holes has a contour and 1 pocketing") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWith2Holes.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 3);
    }

    // TODO: When part axis selection can break ties, add this part back. For now
    // it seems like the surface area heuristic is good enough
    // SECTION("Box with protrusion") {
    //   // TODO: Restore old dims
    //   workpiece workpiece_dims(3.0, 3.0, 3.0, ALUMINUM); //1.5, 1.2, 2.0, ALUMINUM);
    //   auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithProtrusion.stl", 0.001);
    //   auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);
    //   REQUIRE(result_programs.size() == 2);
    // }

    SECTION("Box with thru hole") {
      workpiece workpiece_dims(1.51, 1.51, 2.0, ACETAL);
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithThruHole.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 2);
    }
    
  }

}
