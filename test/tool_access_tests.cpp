#include "catch.hpp"
#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "process_planning/feature_selection.h"
#include "process_planning/tool_access.h"
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
    vector<tool> tools{t1, t2};

    tool_access_info tool_info = find_accessable_tools(f, tools);

    REQUIRE(tool_info.size() == f->num_features());
  }

}
