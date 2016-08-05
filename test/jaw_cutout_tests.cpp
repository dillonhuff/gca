#include "catch.hpp"

#include "synthesis/jaw_cutout.h"

namespace gca {

  TEST_CASE("Clipped cylinder jaw cutout") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = large_jaw_vice(4.5, point(-0.8, -4.4, -3.3));
    std::vector<plate_height> parallel_plates{0.5, 0.7};
    fixtures fixes(test_vice, parallel_plates);
    
    tool t1(0.1, 3.0, 4, HSS, FLAT_NOSE);
    vector<tool> tools{t1};
    workpiece workpiece_dims(3.0, 1.9, 3.0, ACETAL);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ClippedCylinder.stl", 0.001);

    auto decomp = contour_surface_decomposition_in(part_mesh, point(0, 0, 1));
    REQUIRE(decomp);
    
    soft_jaws jaws = make_soft_jaws(*decomp, v);

    REQUIRE();
  }
}
