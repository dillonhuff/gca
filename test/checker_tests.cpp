#include "bounds_checker.h"
#include "catch.hpp"
#include "context.h"
#include "g0_move_checker.h"
#include "multi_checker.h"

namespace gca {

  TEST_CASE("Checkers") {
    context c;

    SECTION("Program bounds checker true") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(12.5, -10.3, 0.0));
      p->push_back(c.mk_minstr(2));
      bounds_checker b(0, 30, -20, -10, -5.0, 2.0);
      REQUIRE(b.check(cout, p) == 0);
    }

    SECTION("Program bounds checker false") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G1(12.5, -10.3, 0.0));
      p->push_back(c.mk_minstr(2));
      bounds_checker b(0, 9, -20, -10, 0.0, 2.0);
      REQUIRE(b.check(cout, p) == 1);
    }

    SECTION("g0_move_checker no mistake") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(2.0, 2.0, 0.0));
      g0_move_checker c;
      REQUIRE(c.check(cout, p) == 0);
    }

    SECTION("g0_move_checker mistake") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(2.0, 2.0, 1.0));
      g0_move_checker c;
      REQUIRE(c.check(cout, p) == 1);
    }

    SECTION("g0_move_checker several instructions no mistake") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G1(2.0, 2.0, 1.0));
      p->push_back(c.mk_minstr(2));
      g0_move_checker c;
      REQUIRE(c.check(cout, p) == 0);
    }

    SECTION("g0_move_checker several instructions relative mistake") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(0.0, 2.0, 1.0, GCA_RELATIVE));
      p->push_back(c.mk_minstr(2));
      g0_move_checker c;
      REQUIRE(c.check(cout, p) == 1);
    }

    SECTION("Multi checker several instructions 1 error") {
      g0_move_checker f;
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(3.0, 0.0, 1.0));
      p->push_back(c.mk_G1(3.0, -15.0, 3.0));
      p->push_back(c.mk_minstr(2));
      REQUIRE(f.check(cout, p) == 1);
    }

    SECTION("Multi checker 2 mistakes absolute coords") {
      bounds_checker b(0, 9, -20, -10, 0.0, 2.0);
      g0_move_checker f;
      vector<checker*> checkers;
      checkers.push_back(&b);
      checkers.push_back(&f);
      multi_checker mc(checkers);
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(3.0, -11.0, 1.0));
      p->push_back(c.mk_G1(3.0, -15.0, 3.0));
      p->push_back(c.mk_minstr(2));
      REQUIRE(mc.check(cout, p) == 2);
    }
    
  }
}
