#include "analysis/position_table.h"
#include "catch.hpp"
#include "core/lexer.h"

namespace gca {

  TEST_CASE("Compute position table") {
    arena_allocator a;
    set_system_allocator(&a);
    
    vector<block> p;
    vector<machine_state> s;
    position_table t;
    position_table c;

    SECTION("Empty program") {
      t = program_position_table(s);
      REQUIRE(t == c);
    }

    SECTION("One instruction with no moves") {
      p = lex_gprog("O1001");
      s = all_program_states(p);
      t = program_position_table(s);
      add_unk_row(c);
      REQUIRE(t == c);
    }

    SECTION("One instruction with no moves NEQ") {
      p = lex_gprog("O1001");
      s = all_program_states(p);
      t = program_position_table(s);
      REQUIRE(t != c);
    }

    SECTION("G0 sets all coordinates") {
      p = lex_gprog("O1001 \n G90 G54 \n G0 X1.0 Y2.0 Z3.0 F3.0");
      s = all_program_states(p);
      t = program_position_table(s);
      add_unk_row(c);
      add_unk_row(c);
      update_table(G54_COORD_SYSTEM, position(1, 2, 3), c);
      REQUIRE(t == c);
    }
  }
}
