#include "analysis/machine_state.h"
#include "catch.hpp"
#include "core/arena_allocator.h"
#include "core/lexer.h"

namespace gca {

  TEST_CASE("One block updates from default") {
    arena_allocator a;
    set_system_allocator(&a);

    machine_state s;
    machine_state r;
    machine_state c;

    SECTION("Update with empty block") {
      block b;
      r = next_machine_state(b, s);
      REQUIRE(c == r);
    }

    SECTION("Update feedrate") {
      block b = lex_gprog("F12").front();
      r = next_machine_state(b, s);
      c.feedrate = lit::make(12);
      REQUIRE(c == r);
    }

    SECTION("Update feedrate NEQ") {
      block b = lex_gprog("F12").front();
      r = next_machine_state(b, s);
      c.feedrate = lit::make(10);
      REQUIRE(c != r);
    }
  }
}
