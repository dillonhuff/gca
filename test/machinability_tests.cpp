#include "catch.hpp"

#include "process_planning/surface_planning.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Machinability") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Magnetic latch top is not millable") {
      auto mesh = parse_stl("./test/stl-files/onshape_parts/Magnetic Latch Top - Part 1.stl", 0.0001);

      surface_milling_constraints smc =
	build_surface_milling_constraints(mesh);

      REQUIRE(smc.has_unmillable_inside_corner());
    }
  }

}
