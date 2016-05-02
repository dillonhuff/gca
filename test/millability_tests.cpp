#include "catch.hpp"
#include "synthesis/millability.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Box millability from the top") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Cube0p5.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      vector<index_t> millable = millable_faces(point(1, 0, 0), mesh);
      REQUIRE(millable.size() == 6);
    }
  }

}
