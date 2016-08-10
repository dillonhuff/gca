#include "catch.hpp"
#include "geometry/rigid_arrangement.h"
#include "system/parse_stl.h"
#include "utils/arena_allocator.h"

namespace gca {

  TEST_CASE("Labeled mesh construction") {
    arena_allocator al;
    set_system_allocator(&al);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ClippedPill.stl", 0.001);

    auto surfs = outer_surfaces(mesh);

    REQUIRE(surfs.size() > 0);

    labeled_mesh a(mesh);
    a.insert_label("surf", surfs.back().index_list());

    REQUIRE(a.labeled_surface("surf").index_list().size() == surfs.back().index_list().size());

    labeled_mesh b(a);
    REQUIRE(b.labeled_surface("surf").index_list().size() == surfs.back().index_list().size());
    
  }

  TEST_CASE("Basic labels") {
    arena_allocator al;
    set_system_allocator(&al);
    
    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ClippedPill.stl", 0.001);

    auto surfs = outer_surfaces(mesh);

    REQUIRE(surfs.size() > 0);

    rigid_arrangement a;
    a.insert("mesh", mesh);
    a.insert_label("mesh", "surf", surfs.back().index_list());

    SECTION("Find labeled surface in original mesh") {
      REQUIRE(a.labeled_surface("mesh", "surf").index_list().size() == surfs.back().index_list().size());
    }

    SECTION("Find labeled surface in inserted mesh") {

      rigid_arrangement b;
      b.insert("next_mesh", a.labeled_mesh("mesh"));

      REQUIRE(b.labeled_surface("next_mesh", "surf").index_list().size() == surfs.back().index_list().size());
    }

    SECTION("Find labeled surface in copied arrangement") {

      rigid_arrangement b(a);

      REQUIRE(b.labeled_surface("mesh", "surf").index_list().size() == surfs.back().index_list().size());
    }

  }
}
