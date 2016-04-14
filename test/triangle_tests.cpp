#include "catch.hpp"
#include "geometry/polygon.h"
#include "geometry/triangle.h"
#include "system/algorithm.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Identify millable surfaces") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Box") {
      auto triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Box1x1x1.stl").triangles;
      auto faces = millable_surfaces(triangles);
      REQUIRE(faces.size() == 1);
    }

    SECTION("Multiple cylinders") {
      auto triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/MultipleCylinders.stl").triangles;
      auto faces = millable_surfaces(triangles);
      REQUIRE(faces.size() == 3);
    }
  }
  
}
