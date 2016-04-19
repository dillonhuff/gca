#include "catch.hpp"
#include "synthesis/axis_3.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Milling surfaces") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Rectangle cylinder") {
      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderSquare.stl").triangles;
      double tool_diameter = 0.125;
      auto mill_paths = mill_surface(triangles, tool_diameter);
      REQUIRE(mill_paths.size() > 0);
    }

    SECTION("Multiple cylinders") {
      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/MultipleCylinders.stl").triangles;
      double tool_diameter = 0.000125;
      auto mill_lines = mill_surface_lines(triangles, tool_diameter);
      REQUIRE(mill_lines.size() > 0);
    }
  }
}
