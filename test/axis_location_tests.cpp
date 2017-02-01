#include "catch.hpp"
#include "process_planning/axis_location.h"
#include "synthesis/fixture_analysis.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("CircleWithFilletAndSide needs only 1 freeform surface direction") {
    arena_allocator a;
    set_system_allocator(&a);

    workpiece wp(1.75, 1.75, 2.5, ALUMINUM);
    fabrication_inputs inputs = current_fab_inputs(wp);

    triangular_mesh part =
      parse_stl("./test/stl-files/CircleWithFilletAndSide.stl", 0.0001);

    vector<surface> stable_surfaces = outer_surfaces(part);
    triangular_mesh stock = align_workpiece(stable_surfaces, wp);

    vector<direction_info> norms =
      select_cut_directions(stock, part, inputs.f, inputs.tools);
    
    unsigned num_freeform_dirs = 0;
    for (auto n : norms) {
      if (n.search_for_freeform_features) {
	num_freeform_dirs++;
      }
    }

    REQUIRE(num_freeform_dirs == 1);
  }

}
