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

    
  }

}
