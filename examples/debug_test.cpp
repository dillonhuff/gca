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

  TEST_CASE("Part with hole unreachable from the top") {
    arena_allocator a;
    set_system_allocator(&a);

    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.25);
    t1.set_cut_length(0.6);

    t1.set_shank_diameter(3.0 / 8.0);
    t1.set_shank_length(0.3);

    t1.set_holder_diameter(2.5);
    t1.set_holder_length(3.5);
    
    tool t2(0.1, 3.0, 4, HSS, FLAT_NOSE);
    t2.set_cut_diameter(0.1);
    t2.set_cut_length(1.0);

    t2.set_shank_diameter(0.5);
    t2.set_shank_length(0.4);

    t2.set_holder_diameter(2.5);
    t2.set_holder_length(3.5);

    std::vector<tool> tools{t1, t2};

    vice test_vice = large_jaw_vice(5, point(-0.8, -4.4, -3.3));
    std::vector<plate_height> parallel_plates{0.1, 0.3, 0.5};
    fixtures fixes(test_vice, parallel_plates);

    workpiece workpiece_dims(3.0, 3.0, 3.0, ACETAL);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/Part Studio 1 - Part 1.stl", 0.0001);

    auto result_plan = make_fabrication_plan(mesh, fixes, tools, {workpiece_dims});

    SECTION("Produces only workpiece clipping programs") {
      REQUIRE(result_plan.steps().size() == 2);
    }

    SECTION("Workpiece clipping programs actually contain code") {
      sanity_check_toolpaths(result_plan);
    }
    
  }

}
