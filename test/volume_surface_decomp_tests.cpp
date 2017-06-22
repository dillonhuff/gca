#include "catch.hpp"

#include "feature_recognition/visual_debug.h"
#include "geometry/mesh_operations.h"
#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_debug.h"
#include "synthesis/visual_debug.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Subtracting from ") {
    auto mesh =
      parse_stl("./test/stl-files/onshape_parts/100-009 - Part 1.stl", 0.0001);
    REQUIRE(false);
  }

}
