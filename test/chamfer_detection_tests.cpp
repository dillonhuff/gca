#include "catch.hpp"
#include "feature_recognition/chamfer_detection.h"
#include "geometry/vtk_debug.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/workpiece_clipping.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Three chamfers") {

    triangular_mesh three_chamfer_mesh =
      parse_stl("./test/stl-files/ThreeChamfers.stl", 0.0001);

    point n(0, 0, 1);
    vector<vector<index_t>> chamfer_surfaces =
      chamfer_regions(three_chamfer_mesh, n);


    for (auto& surf : chamfer_surfaces) {
      vtk_debug_highlight_inds(surf, three_chamfer_mesh);
    }

    REQUIRE(chamfer_surfaces.size() == 3);
    
  }

}
