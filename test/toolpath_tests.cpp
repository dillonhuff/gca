#include "catch.hpp"
#include "system/arena_allocator.h"

#include "synthesis/linear_cut.h"
#include "synthesis/toolpath.h"

namespace gca {

  TEST_CASE("Cuts to toolpaths") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<cut*> cuts;
    

    SECTION("No cuts") {
    }
  }
}
