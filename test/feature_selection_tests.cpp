#include "catch.hpp"
#include "process_planning/feature_selection.h"
#include "feature_recognition/feature_decomposition.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("PSU Mount Feature Selection") {
    arena_allocator a;
    set_system_allocator(&a);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", 0.0001);

    auto ft = build_feature_decomposition(mesh, point(0, 0, 1));
    auto bt = build_feature_decomposition(mesh, point(0, 0, -1));

    vector<feature_decomposition*> pruned =
      select_top_and_bottom_features(ft, bt);

    REQUIRE(pruned.size() == 2);

    REQUIRE(collect_features(pruned[1]).size() == 3);
    REQUIRE(collect_features(pruned[1]).size() == 6);
  }

}
