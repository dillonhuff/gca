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

    REQUIRE(collect_features(ft).size() == 7);
    
    auto bt = build_feature_decomposition(mesh, point(0, 0, -1));

    REQUIRE(collect_features(bt).size() == 5);

    containment_map base_cont = cont_map(bt, ft);
    unsigned num_base_features_contained = num_contained_features(base_cont);

    REQUIRE(num_base_features_contained == 3);

    containment_map top_cont = cont_map(ft, bt);
    unsigned num_top_features_contained = num_contained_features(top_cont);

    REQUIRE(num_top_features_contained == 3);
    
    vector<feature_decomposition*> pruned =
      select_top_and_bottom_features(ft, bt);

    REQUIRE(pruned.size() == 2);

    REQUIRE(collect_features(pruned[1]).size() == 3);
    REQUIRE(collect_features(pruned[1]).size() == 6);
  }

}
