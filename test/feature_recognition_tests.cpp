#include "catch.hpp"
#include "feature_recognition/feature_decomposition.h"
#include "synthesis/contour_planning.h"
#include "system/parse_stl.h"
#include "utils/arena_allocator.h"

namespace gca {

  TEST_CASE("Arm joint top") {
    arena_allocator a;
    set_system_allocator(&a);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Arm_Joint_Top.stl", 0.001);

    feature_decomposition* f =
      build_feature_decomposition(mesh, point(0, -1, 0));

    REQUIRE(f->num_levels() == 4);
  }
}
