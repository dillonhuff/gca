#include "catch.hpp"
#include "geometry/triangular_mesh.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Get triangles in to and out of mesh") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Box mesh in and out") {
      auto triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/SlicedCone.stl").triangles;
      triangular_mesh m = make_mesh(triangles);
    }
  }
}
