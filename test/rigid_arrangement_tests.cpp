#include "catch.hpp"
#include "geometry/rigid_arrangement.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Basic labels") {
    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ClippedPill.stl", 0.001);

    auto surfs = outer_surfaces(mesh);

    REQUIRE(surfs.size() > 0);

    rigid_arrangement a;
    a.insert("mesh", mesh);
    a.insert_label("mesh", "surf", surfs.back().index_list());

    REQUIRE(a.labeled_surface("mesh", "surf").index_list().size() == surfs.back().index_list().size());

    rigid_arrangement b;
    b.insert("next_mesh", a.labeled_mesh("mesh"));

    REQUIRE(b.labeled_surface("mesh", "surf").index_list().size() == surfs.back().index_list().size());
  }
}
