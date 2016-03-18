#include "catch.hpp"
#include "core/parser.h"
#include "core/lexer.h"
#include "transformers/feed_changer.h"
#include "transformers/g0_filter.h"

namespace gca {

  TEST_CASE("Transformer tests") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<block> p;
    vector<block> correct;
    value* initial_feedrate;
    value* new_feedrate;

    SECTION("Feed changer with G0") {
      p = lex_gprog("G91 G0 X1.5 \n G1 F2 X2.0 Y3.0 Z5.5");
      initial_feedrate = lit::make(2.0);
      new_feedrate = lit::make(5.0);
      correct = lex_gprog("G91 G0 X1.5 \n G1 F5 X2.0 Y3.0 Z5.5");
      REQUIRE(change_feeds(p, initial_feedrate, new_feedrate) == correct);
    }

    SECTION("Feed changer") {
      p = lex_gprog("G1 X1.0 Y1.0 Z1.0 F1.0");
      initial_feedrate = lit::make(1.0);
      new_feedrate = lit::make(4.0);
      correct = lex_gprog("G1 X1.0 Y1.0 Z1.0 F4.0");
      REQUIRE(change_feeds(p, initial_feedrate, new_feedrate) == correct);
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
}
