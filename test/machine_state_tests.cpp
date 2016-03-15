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

    SECTION("G54 is active coord system") {
      block b = lex_gprog("G54").front();
      r = next_machine_state(b, s);
      c.active_coord_system = G54_COORD_SYSTEM;
      REQUIRE(c == r);
    }

    SECTION("G54 is active coord system NEQ") {
      block b = lex_gprog("G54").front();
      r = next_machine_state(b, s);
      REQUIRE(c != r);
    }

    SECTION("Tool height comp negative NEQ") {
      block b = lex_gprog("G43 H7").front();
      r = next_machine_state(b, s);
      REQUIRE(c != r);
    }

    SECTION("Tool height comp negative") {
      block b = lex_gprog("G43 H7").front();
      r = next_machine_state(b, s);
      c.tool_height_comp = TOOL_HEIGHT_COMP_NEGATIVE;
      c.tool_height_value = ilit::make(7);
      REQUIRE(c == r);
    }
    
    SECTION("Tool radius comp off") {
      block b = lex_gprog("G40").front();
      r = next_machine_state(b, s);
      REQUIRE(c != r);
    }

    SECTION("Select XY plane") {
      block b = lex_gprog("G17").front();
      r = next_machine_state(b, s);
      c.active_plane = ZX_PLANE;
      REQUIRE(c != r);
    }

    SECTION("Move home") {
      block b = lex_gprog("G28 X0.0 Y2.0 Z3.0").front();
      r = next_machine_state(b, s);
      c.active_non_modal_setting = MOVE_HOME_THROUGH_POINT;
      REQUIRE(c != r);
    }

    SECTION("Spindle on") {
      block b = lex_gprog("M5").front();
      r = next_machine_state(b, s);
      c.spindle_setting = SPINDLE_CLOCKWISE;
      REQUIRE(c != r);
    }

    SECTION("Spindle counterclockwise") {
      vector<block> p = lex_gprog("S3000 M4\n G41 H3");
      machine_state t = next_machine_state(p[0], s);
      r = next_machine_state(p[1], t);
      c.spindle_setting = SPINDLE_COUNTERCLOCKWISE;
      c.spindle_speed = lit::make(3000);
      c.tool_radius_comp = TOOL_RADIUS_COMP_LEFT;
      c.tool_radius_value = ilit::make(3);
      REQUIRE(c == r);
    }
    
    SECTION("Active tool tests") {
      block b = lex_gprog("T8 M6").front();
      r = next_machine_state(b, s);
      c.last_referenced_tool = ilit::make(8);
      c.active_tool = ilit::make(8);
      REQUIRE(c == r);
    }

    SECTION("Flood coolant on") {
      block b = lex_gprog("M8").front();
      r = next_machine_state(b, s);
      c.coolant_setting = COOLANT_OFF;
      REQUIRE(c != r);
    }

    SECTION("Circle parameters NEQ") {
      block b = lex_gprog("I1.0 J2.0 K3.0").front();
      r = next_machine_state(b, s);
      REQUIRE(c != r);
    }

    SECTION("Tool radius comp negative NEQ") {
      block b = lex_gprog("G41 H7").front();
      r = next_machine_state(b, s);
      REQUIRE(c != r);
    }

    SECTION("Tool radius comp negative") {
      block b = lex_gprog("G41 H2").front();
      r = next_machine_state(b, s);
      c.tool_radius_comp = TOOL_RADIUS_COMP_LEFT;
      c.tool_radius_value = ilit::make(2);
      REQUIRE(c == r);
    }
  }

  TEST_CASE("Multi block example") {
    arena_allocator a;
    set_system_allocator(&a);

    machine_state r;
    machine_state c;

    SECTION("Spindle manipulation") {
      vector<block> p = lex_gprog("M3 \n S3000 M4\n G41 H3");
      vector<machine_state> ms = all_program_states(p);
      r = ms.back();
      c.spindle_setting = SPINDLE_COUNTERCLOCKWISE;
      c.spindle_speed = lit::make(3000);
      c.tool_radius_comp = TOOL_RADIUS_COMP_LEFT;
      c.tool_radius_value = ilit::make(3);
      REQUIRE(c == r);
    }
    
  }
}
