#include "abs_to_rel.h"
#include "catch.hpp"
#include "context.h"
#include "feed_changer.h"
#include "g0_filter.h"

namespace gca {
  
  TEST_CASE("Feed upgrader") {
    context c;
    gprog* p = c.mk_gprog();
    double initial_feedrate = 1.0;
    p->push_back(c.mk_G1(1.0, 1.0, 1.0, initial_feedrate));
    double new_feedrate = 4.0;
    feed_changer f(initial_feedrate, new_feedrate);
    gprog* n = f.apply(c, p);
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G1(1.0, 1.0, 1.0, new_feedrate));
    REQUIRE(*n == *correct);
  }

  TEST_CASE("No irrelevant G0 moves") {
    context c;
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G0(1.0, 1.0, 1.0));
    g0_filter f;
    gprog* r = f.apply(c, p);
    REQUIRE(*r == *p);
  }

  TEST_CASE("g0_filter no G0 moves") {
    context c;
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G1(1.0, 2.0, 2.0));
    g0_filter f;
    gprog* r = f.apply(c, p);
    REQUIRE(*r == *p);
  }

  TEST_CASE("g0_filter dont remove G1") {
    context c;
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G0(1.0, 2.0, 2.0));
    p->push_back(c.mk_G1(1.0, 2.0, 2.0));
    g0_filter f;
    gprog* r = f.apply(c, p);
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G0(1.0, 2.0, 2.0));
    correct->push_back(c.mk_G1(1.0, 2.0, 2.0));
    REQUIRE(*r == *correct);
  }

  TEST_CASE("g0_filter several pointless moves") {
    context c;
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G0(1.0, 2.0, 2.0));
    p->push_back(c.mk_G0(1.0, 2.0, 2.0));
    p->push_back(c.mk_G0(1.0, 2.0, 2.0));
    g0_filter f;
    gprog* r = f.apply(c, p);
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G0(1.0, 2.0, 2.0));
    REQUIRE(*r == *correct);
  }

  TEST_CASE("g0_filter several pointless moves with G1, M2") {
    context c;
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G0(1.0, 2.0, -2.0));
    p->push_back(c.mk_G0(1.0, 2.0, 0.0));
    p->push_back(c.mk_G0(1.0, 2.0, -2.00000001));
    p->push_back(c.mk_G1(1.0, 2.0, 2.0));
    p->push_back(c.mk_minstr(2));
    g0_filter f;
    gprog* r = f.apply(c, p);
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G0(1.0, 2.0, -2.00000001));
    correct->push_back(c.mk_G1(1.0, 2.0, 2.0));
    correct->push_back(c.mk_minstr(2));
    cout << "-- Correct" << endl;
    cout << *correct;
    cout << "-- Actual" << endl;
    cout << *r;
    REQUIRE(*r == *correct);
  }

  TEST_CASE("abs -> rel conversion") {
    context c;
    gprog* p = c.mk_gprog();
    gprog* r = c.mk_gprog();
    gprog* correct = c.mk_gprog();
    abs_to_rel f;

    SECTION("abs -> rel 1 instruction is the same") {
      p->push_back(c.mk_G0(1.0, 1.0, 1.0));
      correct->push_back(c.mk_G0(1.0, 1.0, 1.0, GCA_RELATIVE));
      r = f.apply(c, p);
      cout << "-- correct" << endl;
      cout << *correct;
      cout << "-- r " << endl;
      cout << *r;
      REQUIRE(*r == *correct);
    }

    SECTION("abs -> rel 2 instructions") {
      
    }
    
  }
  
}
