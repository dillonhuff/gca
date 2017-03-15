#include "catch.hpp"

#include "geometry/vtk_debug.h"
#include "process_planning/surface_planning.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Parsing that weird failing print object") {
    triangular_mesh m =
      parse_stl("./test/stl-files/onshape_parts/caliperbedlevelingi3v2_fixed - Part 1.stl", 0.0001);

    auto sfc = build_surface_milling_constraints(m);

    for (auto& r : sfc.hard_corner_groups()) {
      vtk_debug_highlight_inds(r);
    }
    
  }

}
