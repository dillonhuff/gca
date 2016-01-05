#include "abs_to_rel.h"
#include "catch.hpp"
#include "context.h"
#include "feed_changer.h"
#include "g0_filter.h"
#include "parser.h"
#include "rel_to_abs.h"
#include "tiler.h"

namespace gca {

  TEST_CASE("Feed changer with G0") {
    context c;
    gprog* p = parse_gprog(c, "G91 G0 X1.5 G1 F2 X2.0 Y3.0 Z5.5");
    double initial_feedrate = 2.0;
    double new_feedrate = 5.0;
    feed_changer f(c, initial_feedrate, new_feedrate);
    gprog* n = f.apply(p);
    gprog* correct = parse_gprog(c, "G91 G0 X1.5 G1 F5 X2.0 Y3.0 Z5.5");
    REQUIRE(*n == *correct);
  }
  
  TEST_CASE("Feed changer") {
    context c;
    gprog* p = c.mk_gprog();
    double initial_feedrate = 1.0;
    p->push_back(c.mk_G1(1.0, 1.0, 1.0, initial_feedrate));
    double new_feedrate = 4.0;
    feed_changer f(c, initial_feedrate, new_feedrate);
    gprog* n = f.apply(p);
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G1(1.0, 1.0, 1.0, new_feedrate));
    REQUIRE(*n == *correct);
  }

  TEST_CASE("Feed changer relative coordinates") {
    context c;
    gprog* p = c.mk_gprog();
    double initial_feedrate = 1.0;
    p->push_back(c.mk_G1(1.0, 1.0, 1.0, initial_feedrate, GCA_RELATIVE));
    double new_feedrate = 4.0;
    feed_changer f(c, initial_feedrate, new_feedrate);
    gprog* n = f.apply(p);
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G1(1.0, 1.0, 1.0, new_feedrate, GCA_RELATIVE));
    REQUIRE(*n == *correct);
  }
  
  TEST_CASE("No irrelevant G0 moves") {
    context c;
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G0(point(1.0, 1.0, 1.0)));
    g0_filter f(c);
    gprog* r = f.apply(c, p);
    REQUIRE(*r == *p);
  }

  TEST_CASE("g0_filter no G0 moves") {
    context c;
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G1(1.0, 2.0, 2.0));
    g0_filter f(c);
    gprog* r = f.apply(c, p);
    REQUIRE(*r == *p);
  }

  TEST_CASE("g0_filter dont remove G1") {
    context c;
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G0(point(1.0, 2.0, 2.0)));
    p->push_back(c.mk_G1(1.0, 2.0, 2.0));
    g0_filter f(c);
    gprog* r = f.apply(c, p);
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G0(point(1.0, 2.0, 2.0)));
    correct->push_back(c.mk_G1(1.0, 2.0, 2.0));
    REQUIRE(*r == *correct);
  }

  TEST_CASE("g0_filter several pointless moves") {
    context c;
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G0(1.0, 2.0, 2.0));
    p->push_back(c.mk_G0(1.0, 2.0, 2.0));
    p->push_back(c.mk_G0(1.0, 2.0, 2.0));
    g0_filter f(c);
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
    g0_filter f(c);
    gprog* r = f.apply(c, p);
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G0(1.0, 2.0, -2.00000001));
    correct->push_back(c.mk_G1(1.0, 2.0, 2.0));
    correct->push_back(c.mk_minstr(2));
    REQUIRE(*r == *correct);
  }

  TEST_CASE("g0_filter starting on G1") {
    context c;
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G1(1, 1, 1));
    p->push_back(c.mk_G0(1, 1, 1));
    g0_filter f(c);
    gprog* r = f.apply(c, p);
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G1(1, 1, 1));
    REQUIRE(*r == *correct);
  }

  TEST_CASE("g0_filter starting on G1 multiple instructions") {
    context c;
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G1(1, 1, 0));
    p->push_back(c.mk_G1(1, 1, 1));
    p->push_back(c.mk_G0(1, 2, 0));
    p->push_back(c.mk_G0(1, 1, 1));
    p->push_back(c.mk_G1(2, 3, 3));
    g0_filter f(c);
    gprog* r = f.apply(c, p);
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G1(1, 1, 0));
    correct->push_back(c.mk_G1(1, 1, 1));
    correct->push_back(c.mk_G1(2, 3, 3));
    REQUIRE(*r == *correct);
  }
  
  TEST_CASE("abs -> rel conversion") {
    context c;
    gprog* p = c.mk_gprog();
    gprog* r = c.mk_gprog();
    gprog* correct = c.mk_gprog();
    abs_to_rel f(c);

    SECTION("abs -> rel 1 instruction is the same") {
      p->push_back(c.mk_G0(1.0, 1.0, 1.0));
      correct->push_back(c.mk_G0(1.0, 1.0, 1.0, GCA_RELATIVE));
      r = f.apply(c, p);
      REQUIRE(*r == *correct);
    }

    SECTION("abs -> rel 1 m instruction is the same") {
      p->push_back(c.mk_minstr(2));
      correct->push_back(c.mk_minstr(2));
      r = f.apply(c, p);
      REQUIRE(*r == *correct);
    }
    
    SECTION("abs -> rel 2 instructions") {
      p->push_back(c.mk_G1(1.0, 0, 0));
      p->push_back(c.mk_G0(2.0, 3.5, 8));
      correct->push_back(c.mk_G1(1.0, 0, 0, 1.0, GCA_RELATIVE));
      correct->push_back(c.mk_G0(1.0, 3.5, 8, GCA_RELATIVE));
      r = f.apply(c, p);
      REQUIRE(*r == *correct);
    }
    
  }

  TEST_CASE("rel -> abs conversion") {
    context c;
    gprog* p = c.mk_gprog();
    gprog* r;
    gprog* correct = c.mk_gprog();
    rel_to_abs f;

    SECTION("One M2 instruction is the same") {
      p->push_back(c.mk_minstr(2));
      correct->push_back(c.mk_minstr(2));
      r = f.apply(c, p);
      REQUIRE(*r == *correct);
    }

    SECTION("One G0 instruction is the same") {
      p->push_back(c.mk_G0(point(1.0, 2.0, 3.0), GCA_RELATIVE));
      correct->push_back(c.mk_G0(point(1.0, 2.0, 3.0), GCA_ABSOLUTE));
      r = f.apply(c, p);
      REQUIRE(*r == *correct);
    }

    SECTION("Two instructions instructions") {
      p->push_back(c.mk_G0(point(1.0, 2.0, 3.0), GCA_RELATIVE));
      p->push_back(c.mk_G1(-2.0, 2.0, -10.0, 2.5, GCA_RELATIVE));
      correct->push_back(c.mk_G0(point(1.0, 2.0, 3.0), GCA_ABSOLUTE));
      correct->push_back(c.mk_G1(-1.0, 4.0, -7.0, 2.5, GCA_ABSOLUTE));
      r = f.apply(c, p);
      REQUIRE(*r == *correct);
    }    
  }

  TEST_CASE("Tiler") {
    context c;
    gprog* p = c.mk_gprog();
    gprog* r;
    gprog* correct = c.mk_gprog();

    SECTION("No op program") {
      tiler t(2, point(1, 0, 0), point(1, 0, 0));
      p->push_back(c.mk_minstr(2));
      correct->push_back(c.mk_G91());
      correct->push_back(c.mk_minstr(2));
      r = t.apply(c, p);
      REQUIRE(*r == *correct);
    }

    SECTION("No op program") {
      tiler t(2, point(1, 0, 0), point(1, 0, 0));
      p->push_back(c.mk_minstr(30));
      correct->push_back(c.mk_G91());
      correct->push_back(c.mk_minstr(2));
      r = t.apply(c, p);
      REQUIRE(*r == *correct);
    }
    
    SECTION("Tile lines") {
      double depth = -5;
      point start = point(0, 0, 0);
      point shift = point(2, 0, 0);
      tiler t(3, start, shift);
      p->push_back(c.mk_G1(0, 0, depth));
      p->push_back(c.mk_G1(1, 0, depth));
      p->push_back(c.mk_G1(1, -1, depth));
      p->push_back(c.mk_minstr(2));
      point e1 = point(1, -1, depth);
      point s1 = point(2, 0, depth);
      point d1 = s1 - e1;
      
      correct->push_back(c.mk_G91());

      correct->push_back(c.mk_G1(0, 0, depth, 1.0, GCA_RELATIVE));
      correct->push_back(c.mk_G1(1, 0, 0, 1.0, GCA_RELATIVE));
      correct->push_back(c.mk_G1(0, -1, 0, 1.0, GCA_RELATIVE));

      correct->push_back(c.mk_G0(0, 0, -depth, GCA_RELATIVE));
      correct->push_back(c.mk_G0(d1.x, d1.y, 0, GCA_RELATIVE));

      correct->push_back(c.mk_G1(0, 0, depth, 1.0, GCA_RELATIVE));
      correct->push_back(c.mk_G1(1, 0, 0, 1.0, GCA_RELATIVE));
      correct->push_back(c.mk_G1(0, -1, 0, 1.0, GCA_RELATIVE));

      correct->push_back(c.mk_G0(0, 0, -depth, GCA_RELATIVE));
      correct->push_back(c.mk_G0(d1.x, d1.y, 0, GCA_RELATIVE));

      correct->push_back(c.mk_G1(0, 0, depth, 1.0, GCA_RELATIVE));
      correct->push_back(c.mk_G1(1, 0, 0, 1.0, GCA_RELATIVE));
      correct->push_back(c.mk_G1(0, -1, 0, 1.0, GCA_RELATIVE));
      
      correct->push_back(c.mk_minstr(2));
      
      r = t.apply(c, p);
      REQUIRE(*r == *correct);
    }
  }
  
}
