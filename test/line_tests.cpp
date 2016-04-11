#include "catch.hpp"
#include "geometry/line.h"

namespace gca {

  TEST_CASE("Same segment several times") {
    point bl(0, 0, 0);
    point ul(0, 1, 0);
    point br(1, 0, 0);
    point ur(1, 1, 0);

    vector<line> lines{line(ur, bl), line(ur, bl)};
    REQUIRE(count_in(line(ur, bl), lines) == 2);
  }
}
