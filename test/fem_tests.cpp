#include "catch.hpp"

namespace gca {

  TEST_CASE("PSU mount test") {
    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", 0.0001);
    vice v = emco_vice();

    auto surfs = outer_surfaces(mesh);
    vector<clamp_orientation> orients = all_viable_orientations(surfs, v);

    
  }
}
