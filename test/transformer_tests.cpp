#include "catch.hpp"
#include "core/context.h"
#include "core/parser.h"
#include "transformers/abs_to_rel.h"
#include "transformers/feed_changer.h"
#include "transformers/g0_filter.h"
#include "transformers/rel_to_abs.h"
#include "transformers/tiler.h"

namespace gca {

  TEST_CASE("Feed changer with G0") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = parse_gprog(c, "G91 G0 X1.5 G1 F2 X2.0 Y3.0 Z5.5");
    value* initial_feedrate = c.mk_lit(2.0);
    value* new_feedrate = c.mk_lit(5.0);
    gprog* correct = parse_gprog(c, "G91 G0 X1.5 G1 F5 X2.0 Y3.0 Z5.5");
    REQUIRE(*change_feeds(c, p, initial_feedrate, new_feedrate) == *correct);
  }
  
  TEST_CASE("Feed changer") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = c.mk_gprog();
    value* initial_feedrate = c.mk_lit(1.0);
    p->push_back(c.mk_G1(1.0, 1.0, 1.0, initial_feedrate));
    value* new_feedrate = c.mk_lit(4.0);
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G1(1.0, 1.0, 1.0, new_feedrate));
    REQUIRE(*change_feeds(c, p, initial_feedrate, new_feedrate) == *correct);
  }

  TEST_CASE("Feed changer relative coordinates") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = c.mk_gprog();
    value* initial_feedrate = c.mk_lit(1.0);
    p->push_back(c.mk_G1(1.0, 1.0, 1.0, initial_feedrate));
    value* new_feedrate = c.mk_lit(4.0);
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G1(1.0, 1.0, 1.0, new_feedrate));
    REQUIRE(*change_feeds(c, p, initial_feedrate, new_feedrate) == *correct);
  }

  TEST_CASE("Feed changer with variables") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);
    gprog* p = parse_gprog(c, "G1 F15 X1.0 Y1.0 Z2.0");
    lit* init_f = c.mk_lit(15.0);
    var* new_f = c.mk_var(1);
    value* default_val = c.mk_lit(13);
    gprog* correct = parse_gprog(c, "#1=13 G1 F#1 X1.0 Y1.0 Z2.0");
    REQUIRE(*generalize_feeds(c, p, default_val, init_f, new_f) == *correct);
  }
  
  TEST_CASE("No irrelevant G0 moves") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G0(point(1.0, 1.0, 1.0)));
    REQUIRE(*filter_G0_moves(c, p) == *p);
  }

  TEST_CASE("g0_filter no G0 moves") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G1(1.0, 2.0, 2.0));
    REQUIRE(*filter_G0_moves(c, p) == *p);
  }

  TEST_CASE("g0_filter dont remove G1") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G0(point(1.0, 2.0, 2.0)));
    p->push_back(c.mk_G1(1.0, 2.0, 2.0));
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G0(point(1.0, 2.0, 2.0)));
    correct->push_back(c.mk_G1(1.0, 2.0, 2.0));
    REQUIRE(*filter_G0_moves(c, p) == *correct);
  }

  TEST_CASE("g0_filter several pointless moves") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G0(1.0, 2.0, 2.0));
    p->push_back(c.mk_G0(1.0, 2.0, 2.0));
    p->push_back(c.mk_G0(1.0, 2.0, 2.0));
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G0(1.0, 2.0, 2.0));
    REQUIRE(*filter_G0_moves(c, p) == *correct);
  }

  TEST_CASE("g0_filter several pointless moves with G1, M2") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G0(1.0, 2.0, -2.0));
    p->push_back(c.mk_G0(1.0, 2.0, 0.0));
    p->push_back(c.mk_G0(1.0, 2.0, -2.00000001));
    p->push_back(c.mk_G1(1.0, 2.0, 2.0));
    p->push_back(c.mk_minstr(2));
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G0(1.0, 2.0, -2.00000001));
    correct->push_back(c.mk_G1(1.0, 2.0, 2.0));
    correct->push_back(c.mk_minstr(2));
    REQUIRE(*filter_G0_moves(c, p) == *correct);
  }

  TEST_CASE("g0_filter starting on G1") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G1(1, 1, 1));
    p->push_back(c.mk_G0(1, 1, 1));
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G1(1, 1, 1));
    REQUIRE(*filter_G0_moves(c, p) == *correct);
  }

  TEST_CASE("g0_filter starting on G1 multiple instructions") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = c.mk_gprog();
    p->push_back(c.mk_G1(1, 1, 0));
    p->push_back(c.mk_G1(1, 1, 1));
    p->push_back(c.mk_G0(1, 2, 0));
    p->push_back(c.mk_G0(1, 1, 1));
    p->push_back(c.mk_G1(2, 3, 3));
    gprog* correct = c.mk_gprog();
    correct->push_back(c.mk_G1(1, 1, 0));
    correct->push_back(c.mk_G1(1, 1, 1));
    correct->push_back(c.mk_G1(2, 3, 3));
    REQUIRE(*filter_G0_moves(c, p) == *correct);
  }
  
  TEST_CASE("abs -> rel conversion") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = c.mk_gprog();
    gprog* r = c.mk_gprog();
    gprog* correct = c.mk_gprog();
    abs_to_rel f(c, GCA_ABSOLUTE);

    SECTION("abs -> rel 1 instruction is the same") {
      p->push_back(c.mk_G0(1.0, 1.0, 1.0));
      correct->push_back(c.mk_G0(1.0, 1.0, 1.0));
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
      correct->push_back(c.mk_G1(1.0, 0, 0, 1.0));
      correct->push_back(c.mk_G0(1.0, 3.5, 8));
      r = f.apply(c, p);
      REQUIRE(*r == *correct);
    }
    
  }

  TEST_CASE("rel -> abs conversion") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = c.mk_gprog();
    gprog* r;
    gprog* correct = c.mk_gprog();
    rel_to_abs f(c, GCA_ABSOLUTE);

    SECTION("One M2 instruction is the same") {
      p->push_back(c.mk_G91());
      p->push_back(c.mk_minstr(2));
      correct->push_back(c.mk_minstr(2));
      r = f.apply(c, p);
      REQUIRE(*r == *correct);
    }

    SECTION("One G0 instruction is the same") {
      p->push_back(c.mk_G91());
      p->push_back(c.mk_G0(point(1.0, 2.0, 3.0)));
      correct->push_back(c.mk_G0(point(1.0, 2.0, 3.0)));
      r = f.apply(c, p);
      REQUIRE(*r == *correct);
    }

    SECTION("Two instructions instructions") {
      p->push_back(c.mk_G91());
      p->push_back(c.mk_G0(point(1.0, 2.0, 3.0)));
      p->push_back(c.mk_G1(-2.0, 2.0, -10.0, 2.5));
      correct->push_back(c.mk_G0(point(1.0, 2.0, 3.0)));
      correct->push_back(c.mk_G1(-1.0, 4.0, -7.0, 2.5));
      r = f.apply(c, p);
      REQUIRE(*r == *correct);
    }    
  }

  TEST_CASE("Tiler") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = c.mk_gprog();
    gprog* r;
    gprog* correct = c.mk_gprog();

    SECTION("No op program") {
      tiler t(2, point(1, 0, 0), point(1, 0, 0));
      p->push_back(c.mk_minstr(2));
      correct->push_back(c.mk_G91());
      correct->push_back(c.mk_minstr(2));
      r = t.apply(c, p, GCA_ABSOLUTE);
      REQUIRE(*r == *correct);
    }

    SECTION("No move instructions") {
      tiler t(2, point(1, 0, 0), point(1, 0, 0));
      p->push_back(c.mk_minstr(30));
      correct->push_back(c.mk_G91());
      correct->push_back(c.mk_minstr(2));
      r = t.apply(c, p, GCA_ABSOLUTE);
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

      correct->push_back(c.mk_G1(0, 0, depth, 1.0));
      correct->push_back(c.mk_G1(1, 0, 0, 1.0));
      correct->push_back(c.mk_G1(0, -1, 0, 1.0));

      correct->push_back(c.mk_G0(0, 0, -depth));
      correct->push_back(c.mk_G0(d1.x, d1.y, 0));

      correct->push_back(c.mk_G1(0, 0, depth, 1.0));
      correct->push_back(c.mk_G1(1, 0, 0, 1.0));
      correct->push_back(c.mk_G1(0, -1, 0, 1.0));

      correct->push_back(c.mk_G0(0, 0, -depth));
      correct->push_back(c.mk_G0(d1.x, d1.y, 0));

      correct->push_back(c.mk_G1(0, 0, depth, 1.0));
      correct->push_back(c.mk_G1(1, 0, 0, 1.0));
      correct->push_back(c.mk_G1(0, -1, 0, 1.0));
      
      correct->push_back(c.mk_minstr(2));
      
      r = t.apply(c, p, GCA_ABSOLUTE);
      REQUIRE(*r == *correct);
    }
  }

  TEST_CASE("Coordinate generalizer") {
    context c;
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("One move") {
      gprog* p = parse_gprog(c, "G1 X0.0 Y-1.2 Z2.3");
      gprog* correct = parse_gprog(c, "#1=2.0 G1 X0.0 Y-1.2 Z#1");
    }
  }
  
}
