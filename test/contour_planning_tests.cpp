#include "catch.hpp"
#include "synthesis/contour_planning.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Arm joint can be cut in one contour along (0, 1, 0)") {
    arena_allocator a;
    set_system_allocator(&a);

    triangular_mesh m = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Arm_Joint_Top.stl", 0.001);

    auto decomp = contour_surface_decomposition_in_dir(m, point(0, 1, 0));

    REQUIRE(decomp);
    for (auto s : decomp->rest) {
      cout << "Surface size = " << s.index_list().size() << endl;
      for (auto i : s.index_list()) {
	cout << "--- Orientation " << i << " = " << s.face_orientation(i) << endl;
      }
    }
    REQUIRE(decomp->rest.size() == 0);
  }
}
