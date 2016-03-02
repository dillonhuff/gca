#include "catch.hpp"
#include "core/parser.h"
#include "transformers/abs_to_rel.h"
#include "transformers/feed_changer.h"
#include "transformers/g0_filter.h"
#include "transformers/rel_to_abs.h"
#include "transformers/scale_xyz.h"
#include "transformers/shift_xyz.h"
#include "transformers/tiler.h"

namespace gca {

  TEST_CASE("Transformer tests") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Feed changer with G0") {
      gprog* p = parse_gprog("G91 G0 X1.5 G1 F2 X2.0 Y3.0 Z5.5");
      value* initial_feedrate = lit::make(2.0);
      value* new_feedrate = lit::make(5.0);
      gprog* correct = parse_gprog("G91 G0 X1.5 G1 F5 X2.0 Y3.0 Z5.5");
      REQUIRE(*change_feeds(p, initial_feedrate, new_feedrate) == *correct);
    }
  
    SECTION("Feed changer") {
      gprog* p = gprog::make();
      value* initial_feedrate = lit::make(1.0);
      p->push_back(g1_instr::make(1.0, 1.0, 1.0, initial_feedrate));
      value* new_feedrate = lit::make(4.0);
      gprog* correct = gprog::make();
      correct->push_back(g1_instr::make(1.0, 1.0, 1.0, new_feedrate));
      REQUIRE(*change_feeds(p, initial_feedrate, new_feedrate) == *correct);
    }

    SECTION("Feed changer relative coordinates") {
      gprog* p = gprog::make();
      value* initial_feedrate = lit::make(1.0);
      p->push_back(g1_instr::make(1.0, 1.0, 1.0, initial_feedrate));
      value* new_feedrate = lit::make(4.0);
      gprog* correct = gprog::make();
      correct->push_back(g1_instr::make(1.0, 1.0, 1.0, new_feedrate));
      REQUIRE(*change_feeds(p, initial_feedrate, new_feedrate) == *correct);
    }

    SECTION("Feed changer with variables") {
      gprog* p = parse_gprog("G1 F15 X1.0 Y1.0 Z2.0");
      lit* init_f = lit::make(15.0);
      var* new_f = var::make(1);
      value* default_val = lit::make(13);
      gprog* correct = parse_gprog("#1=13 G1 F#1 X1.0 Y1.0 Z2.0");
      REQUIRE(*generalize_feeds(p, default_val, init_f, new_f) == *correct);
    }
  
    SECTION("No irrelevant G0 moves") {
      gprog* p = gprog::make();
      p->push_back(g0_instr::make(point(1.0, 1.0, 1.0)));
      REQUIRE(*filter_G0_moves(p) == *p);
    }

    SECTION("g0_filter no G0 moves") {
      gprog* p = gprog::make();
      p->push_back(g1_instr::make(1.0, 2.0, 2.0));
      REQUIRE(*filter_G0_moves(p) == *p);
    }

    SECTION("g0_filter dont remove G1") {
      gprog* p = gprog::make();
      p->push_back(g0_instr::make(point(1.0, 2.0, 2.0)));
      p->push_back(g1_instr::make(1.0, 2.0, 2.0));
      gprog* correct = gprog::make();
      correct->push_back(g0_instr::make(point(1.0, 2.0, 2.0)));
      correct->push_back(g1_instr::make(1.0, 2.0, 2.0));
      REQUIRE(*filter_G0_moves(p) == *correct);
    }

    SECTION("g0_filter several pointless moves") {
      gprog* p = gprog::make();
      p->push_back(g0_instr::make(1.0, 2.0, 2.0));
      p->push_back(g0_instr::make(1.0, 2.0, 2.0));
      p->push_back(g0_instr::make(1.0, 2.0, 2.0));
      gprog* correct = gprog::make();
      correct->push_back(g0_instr::make(1.0, 2.0, 2.0));
      REQUIRE(*filter_G0_moves(p) == *correct);
    }

    SECTION("g0_filter several pointless moves with G1, M2") {
      gprog* p = gprog::make();
      p->push_back(g0_instr::make(1.0, 2.0, -2.0));
      p->push_back(g0_instr::make(1.0, 2.0, 0.0));
      p->push_back(g0_instr::make(1.0, 2.0, -2.00000001));
      p->push_back(g1_instr::make(1.0, 2.0, 2.0));
      p->push_back(m2_instr::make());
      gprog* correct = gprog::make();
      correct->push_back(g0_instr::make(1.0, 2.0, -2.00000001));
      correct->push_back(g1_instr::make(1.0, 2.0, 2.0));
      correct->push_back(m2_instr::make());
      REQUIRE(*filter_G0_moves(p) == *correct);
    }

    SECTION("g0_filter starting on G1") {
      gprog* p = gprog::make();
      p->push_back(g1_instr::make(1, 1, 1));
      p->push_back(g0_instr::make(1, 1, 1));
      gprog* correct = gprog::make();
      correct->push_back(g1_instr::make(1, 1, 1));
      REQUIRE(*filter_G0_moves(p) == *correct);
    }

    SECTION("g0_filter starting on G1 multiple instructions") {
      gprog* p = gprog::make();
      p->push_back(g1_instr::make(1, 1, 0));
      p->push_back(g1_instr::make(1, 1, 1));
      p->push_back(g0_instr::make(1, 2, 0));
      p->push_back(g0_instr::make(1, 1, 1));
      p->push_back(g1_instr::make(2, 3, 3));
      gprog* correct = gprog::make();
      correct->push_back(g1_instr::make(1, 1, 0));
      correct->push_back(g1_instr::make(1, 1, 1));
      correct->push_back(g1_instr::make(2, 3, 3));
      REQUIRE(*filter_G0_moves(p) == *correct);
    }
  }
  
  TEST_CASE("abs -> rel conversion") {
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = gprog::make();
    gprog* r = gprog::make();
    gprog* correct = gprog::make();
    abs_to_rel f(GCA_ABSOLUTE);

    SECTION("abs -> rel 1 instruction is the same") {
      p->push_back(g0_instr::make(1.0, 1.0, 1.0));
      correct->push_back(g0_instr::make(1.0, 1.0, 1.0));
      r = f.apply(p);
      REQUIRE(*r == *correct);
    }

    SECTION("abs -> rel 1 m instruction is the same") {
      p->push_back(m2_instr::make());
      correct->push_back(m2_instr::make());
      r = f.apply(p);
      REQUIRE(*r == *correct);
    }
    
    SECTION("abs -> rel 2 instructions") {
      p->push_back(g1_instr::make(1.0, 0, 0));
      p->push_back(g0_instr::make(2.0, 3.5, 8));
      correct->push_back(g1_instr::make(1.0, 0, 0, 1.0));
      correct->push_back(g0_instr::make(1.0, 3.5, 8));
      r = f.apply(p);
      REQUIRE(*r == *correct);
    }
    
  }

  TEST_CASE("rel -> abs conversion") {
    
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = gprog::make();
    gprog* r;
    gprog* correct = gprog::make();
    rel_to_abs f(GCA_ABSOLUTE);

    SECTION("One M2 instruction is the same") {
      p->push_back(g91_instr::make());
      p->push_back(m2_instr::make());
      correct->push_back(m2_instr::make());
      r = f.apply(p);
      REQUIRE(*r == *correct);
    }

    SECTION("One G0 instruction is the same") {
      p->push_back(g91_instr::make());
      p->push_back(g0_instr::make(point(1.0, 2.0, 3.0)));
      correct->push_back(g0_instr::make(point(1.0, 2.0, 3.0)));
      r = f.apply(p);
      REQUIRE(*r == *correct);
    }

    SECTION("Two instructions instructions") {
      p->push_back(g91_instr::make());
      p->push_back(g0_instr::make(point(1.0, 2.0, 3.0)));
      p->push_back(g1_instr::make(-2.0, 2.0, -10.0, 2.5));
      correct->push_back(g0_instr::make(point(1.0, 2.0, 3.0)));
      correct->push_back(g1_instr::make(-1.0, 4.0, -7.0, 2.5));
      r = f.apply(p);
      REQUIRE(*r == *correct);
    }    
  }

  TEST_CASE("Tiler") {
    
    arena_allocator a;
    set_system_allocator(&a);    
    gprog* p = gprog::make();
    gprog* r;
    gprog* correct = gprog::make();

    SECTION("No op program") {
      tiler t(2, point(1, 0, 0), point(1, 0, 0));
      p->push_back(m2_instr::make());
      correct->push_back(g91_instr::make());
      correct->push_back(m2_instr::make());
      r = t.apply(p, GCA_ABSOLUTE);
      REQUIRE(*r == *correct);
    }

    SECTION("No move instructions") {
      tiler t(2, point(1, 0, 0), point(1, 0, 0));
      p->push_back(m30_instr::make());
      correct->push_back(g91_instr::make());
      correct->push_back(m2_instr::make());
      r = t.apply(p, GCA_ABSOLUTE);
      REQUIRE(*r == *correct);
    }
    
    SECTION("Tile lines") {
      double depth = -5;
      point start = point(0, 0, 0);
      point shift = point(2, 0, 0);
      tiler t(3, start, shift);
      p->push_back(g1_instr::make(0, 0, depth));
      p->push_back(g1_instr::make(1, 0, depth));
      p->push_back(g1_instr::make(1, -1, depth));
      p->push_back(m2_instr::make());
      point e1 = point(1, -1, depth);
      point s1 = point(2, 0, depth);
      point d1 = s1 - e1;
      
      correct->push_back(g91_instr::make());

      correct->push_back(g1_instr::make(0, 0, depth, 1.0));
      correct->push_back(g1_instr::make(1, 0, 0, 1.0));
      correct->push_back(g1_instr::make(0, -1, 0, 1.0));

      correct->push_back(g0_instr::make(0, 0, -depth));
      correct->push_back(g0_instr::make(d1.x, d1.y, 0));

      correct->push_back(g1_instr::make(0, 0, depth, 1.0));
      correct->push_back(g1_instr::make(1, 0, 0, 1.0));
      correct->push_back(g1_instr::make(0, -1, 0, 1.0));

      correct->push_back(g0_instr::make(0, 0, -depth));
      correct->push_back(g0_instr::make(d1.x, d1.y, 0));

      correct->push_back(g1_instr::make(0, 0, depth, 1.0));
      correct->push_back(g1_instr::make(1, 0, 0, 1.0));
      correct->push_back(g1_instr::make(0, -1, 0, 1.0));
      
      correct->push_back(m2_instr::make());
      
      r = t.apply(p, GCA_ABSOLUTE);
      REQUIRE(*r == *correct);
    }
  }

  TEST_CASE("Coordinate generalizer") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("One move") {
      gprog* p = parse_gprog("G1 X0.0 Y-1.2 Z2.3");
      gprog* correct = parse_gprog("#1=2.0 G1 X0.0 Y-1.2 Z#1");
    }
  }

  TEST_CASE("Scale xyz") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("No moves, PROBOTIX") {
      gprog* p = parse_gprog("M2");
      gprog* r = scale_xyz(2.0, *p);
      gprog* correct = parse_gprog("M2");
      REQUIRE(*r == *correct);
    }

    SECTION("G0 and G1, PROBOTIX") {
      gprog* p = parse_gprog("G90 M5 S0 G0 X2.0 Y-1.0 Z0.0 G1 F9 X1.0 Y2.0 Z3.0 M2");
      gprog* r = scale_xyz(3.0, *p);
      gprog* correct = parse_gprog("G90 M5 S0 G0 X6.0 Y-3.0 Z0.0 G1 F9 X3.0 Y6.0 Z9.0 M2");
      REQUIRE(*r == *correct);
    }
  }

  TEST_CASE("Shift xyz") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("One instruction") {
      gprog* p = parse_gprog("G90 M5 S0 G1 X1.0 Y-1.0 Z2.0 M2");
      gprog* r = shift_xyz(1.0, 3.0, 1.5, *p);
      gprog* correct = parse_gprog("G90 M5 S0 G1 X2.0 Y2.0 Z3.5 M2");
      REQUIRE(*r == *correct);
    }
  }
}
