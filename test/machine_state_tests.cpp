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

    SECTION("Spindle speed") {
      block b = lex_gprog("S2000").front();
      r = next_machine_state(b, s);
      c.spindle_speed = lit::make(2000);
      REQUIRE(c == r);
    }

    SECTION("Active move is G0") {
      block b = lex_gprog("G0").front();
      r = next_machine_state(b, s);
      c.active_move_type = FAST_MOVE;
      REQUIRE(c == r);
    }

    SECTION("Active move is G1") {
      block b = lex_gprog("G1").front();
      r = next_machine_state(b, s);
      c.active_move_type = FAST_MOVE;
      REQUIRE(c != r);
    }

    SECTION("Active move is G2") {
      block b = lex_gprog("G2").front();
      r = next_machine_state(b, s);
      c.active_move_type = CLOCKWISE_CIRCULAR_MOVE;
      REQUIRE(c == r);
    }

    SECTION("Active move is G3") {
      block b = lex_gprog("G3").front();
      r = next_machine_state(b, s);
      c.active_move_type = COUNTERCLOCKWISE_CIRCULAR_MOVE;
      REQUIRE(c == r);
    }

    SECTION("Absolute distance mode") {
      block b = lex_gprog("G90").front();
      r = next_machine_state(b, s);
      c.active_distance_mode = ABSOLUTE_DISTANCE_MODE;
      REQUIRE(c == r);
    }

    SECTION("Relative distance mode") {
      block b = lex_gprog("G91").front();
      r = next_machine_state(b, s);
      c.active_distance_mode = RELATIVE_DISTANCE_MODE;
      REQUIRE(c == r);
    }
    
  }
}
