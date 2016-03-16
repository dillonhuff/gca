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
  }
}
