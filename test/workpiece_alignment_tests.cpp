#include "catch.hpp"
#include "synthesis/fixture_analysis.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Displacement only alignment") {
    arena_allocator a;
    set_system_allocator(&a);

    workpiece w(2, 2, 2, ALUMINUM);

    auto m = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Pill.stl", 0.001);

    triangular_mesh mesh =
      m.apply_to_vertices([](const point p)
			  { return point(p.x + 20, p.y + 20, p.z + 20); });

    auto surfs = outer_surfaces(mesh);
    triangular_mesh aligned = align_workpiece(surfs, w);
    box b = aligned.bounding_box();
    for (auto i : mesh.vertex_indexes()) {
      point p = mesh.vertex(i);
      REQUIRE(b.contains(p));
    }
  }
}
