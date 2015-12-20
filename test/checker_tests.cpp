#include "catch.hpp"

#include "src/context.h"
#include "src/bounds_checker.h"

namespace gca {

  TEST_CASE("Checkers") {
    context c;

    SECTION("Program bounds checker true") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(12.5, -10.3, 0.0));
      p->push_back(c.mk_minstr(2));
      bounds_checker b(0, 30, -20, -10, -5.0, 2.0);
      REQUIRE(b.check(cout, p));
    }

    SECTION("Program bounds checker false") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G1(12.5, -10.3, 0.0));
      p->push_back(c.mk_minstr(2));
      bounds_checker b(0, 9, -20, -10, 0.0, 2.0);
      REQUIRE(!b.check(cout, p));
    }
    
  }
}
