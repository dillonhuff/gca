#include "catch.hpp"

#include "feature_recognition/fillet_detection.h"
#include "geometry/vtk_debug.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("100-009 - Part 1.stl") {
    auto mesh =
      parse_stl("./test/stl-files/onshape_parts/100-009 - Part 1.stl", 0.0001);


    vector<vector<surface> > fillets =
      detect_fillets(mesh);

    visualize_surface_decomp(fillets);

    REQUIRE(fillets.size() == 2);
  }

}
