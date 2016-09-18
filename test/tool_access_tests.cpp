#include "catch.hpp"
#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "process_planning/feature_selection.h"
#include "process_planning/tool_access.h"
#include "synthesis/fixture_analysis.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Nested Thru holes") {
    arena_allocator a;
    set_system_allocator(&a);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Box1InchWithNestedThruHoles.stl", 0.001);

    point n(0, 0, 1);

    auto f = build_feature_decomposition(mesh, n);

    REQUIRE(f->num_levels() == 4);

    tool t1(0.30, 3.0, 2, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.3);
    t1.set_cut_length(0.4);

    t1.set_shank_diameter(0.2);
    t1.set_shank_length(0.2);

    t1.set_holder_diameter(2.0);
    t1.set_holder_length(2.5);

    tool t2(0.14, 3.15, 2, HSS, FLAT_NOSE);
    t2.set_cut_diameter(0.14);
    t2.set_cut_length(0.3);

    t2.set_shank_diameter(0.5);
    t2.set_shank_length(0.1);

    t2.set_holder_diameter(2.0);
    t2.set_holder_length(2.5);
    
    vector<tool> tools{t1, t2};

    tool_access_info tool_info = find_accessable_tools(f, tools);

    REQUIRE(tool_info.size() == f->num_features());

    auto top = f->child(0);

    REQUIRE(tool_info[top->feature()].size() == 2);

    auto outer_hole = top->child(0);

    REQUIRE(tool_info[outer_hole->feature()].size() == 1);

    auto inner_hole = outer_hole->child(0);

    REQUIRE(tool_info[inner_hole->feature()].size() == 0);
  }

  TEST_CASE("Select features for Part 1 (1)") {
    arena_allocator a;
    set_system_allocator(&a);

    tool t3{0.2334, 3.94, 4, HSS, FLAT_NOSE};
    t3.set_cut_diameter(0.2334);
    t3.set_cut_length(1.2);

    t3.set_shank_diameter(0.5);
    t3.set_shank_length(0.05);

    t3.set_holder_diameter(2.5);
    t3.set_holder_length(3.5);

    
    vector<tool> tools{t3};
      
    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/Part Studio 1 - Part 1(1).stl", 0.0001);

    auto surfs = outer_surfaces(mesh);
    workpiece w(2.1, 2.1, 2.1, ALUMINUM);
    triangular_mesh stock = align_workpiece(surfs, w);
    
    point n(1, 0, 0);

    feature_decomposition* f = build_feature_decomposition(stock, mesh, n);
    tool_access_info inf = find_accessable_tools(f, tools);

    vtk_debug_feature_decomposition(f);

    unsigned num_unreachable_features = 0;
    for (auto feature_and_tools : inf) {
      if (feature_and_tools.second.size() == 0) {
	num_unreachable_features++;
      }
    }

    REQUIRE(num_unreachable_features == 1);
  }

}
