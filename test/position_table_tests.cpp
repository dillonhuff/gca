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
      p = lex_gprog("O1001 \n G90 G54 \n G0 X1.0 Y2.0 Z3.0");
      s = all_program_states(p);
      t = program_position_table(s);
      add_unk_row(c);
      add_unk_row(c);
      add_unk_row(c);
      update_table(G54_COORD_SYSTEM, position(1, 2, 3), c);
      REQUIRE(t == c);
    }

    SECTION("G0 sets partial coordinates") {
      p = lex_gprog("G90 G54 \n G0 X1.0 Y2.0");
      s = all_program_states(p);
      t = program_position_table(s);
      add_unk_row(c);
      add_unk_row(c);
      update_table(G54_COORD_SYSTEM,
		   position(lit::make(1), lit::make(2), omitted::make()),
		   c);
      REQUIRE(t == c);
    }

    SECTION("G0 sets partial coordinates, then G1") {
      p = lex_gprog("G90 G54 \n G0 X1.0 Y2.0 \n G1 Z-2.0 F3.0 ");
      s = all_program_states(p);
      t = program_position_table(s);
      add_unk_row(c);
      add_unk_row(c);
      update_table(G54_COORD_SYSTEM,
		   position(lit::make(1), lit::make(2), omitted::make()),
		   c);
      update_table(G54_COORD_SYSTEM, position(1, 2, -2), c);
      REQUIRE(t == c);
    }

    SECTION("G0 sets partial coordinates, that are preserved across blocks") {
      p = lex_gprog("G90 G54 \n G0 X1.0 Y2.0 \n G91");
      s = all_program_states(p);
      t = program_position_table(s);
      add_unk_row(c);
      add_unk_row(c);
      update_table(G54_COORD_SYSTEM,
		   position(lit::make(1), lit::make(2), omitted::make()),
		   c);
      update_table(G54_COORD_SYSTEM,
		   position(lit::make(1), lit::make(2), omitted::make()),
		   c);
      REQUIRE(t == c);
    }
    
    SECTION("G0 sets partial coordinates, then relative G1") {
      p = lex_gprog("G90 G54 \n G0 X1.0 Y2.0 \n G91 \n G1 X2.0 Y1.5 Z2.0");
      s = all_program_states(p);
      t = program_position_table(s);
      add_unk_row(c);
      add_unk_row(c);
      update_table(G54_COORD_SYSTEM,
		   position(lit::make(1), lit::make(2), omitted::make()),
		   c);
      update_table(G54_COORD_SYSTEM,
		   position(lit::make(1), lit::make(2), omitted::make()),
		   c);
      update_table(G54_COORD_SYSTEM,
		   position(lit::make(3.0), lit::make(3.5), omitted::make()),
		   c);
      REQUIRE(t == c);
    }

    SECTION("Tool change wipes out all positions") {
      p = lex_gprog("G90 G54 \n G0 X1.0 Y2.0 \n T2 M6 \n G1 X2.0 Y1.5 Z2.0");
      s = all_program_states(p);
      t = program_position_table(s);
      add_unk_row(c);
      add_unk_row(c);
      update_table(G54_COORD_SYSTEM,
		   position(lit::make(1), lit::make(2), omitted::make()),
		   c);
      add_unk_row(c);
      update_table(G54_COORD_SYSTEM, position(2, 1.5, 2), c);
      REQUIRE(t == c);
    }

    SECTION("G28 returns to machine home") {
      p = lex_gprog("G90 G54 \n G28 X-.1");
      s = all_program_states(p);
      t = program_position_table(s);
      add_unk_row(c);
      add_unk_row(c);
      update_table(MACHINE_COORD_SYSTEM, position(0.0, 0.0, 0.0), c);
      REQUIRE(t == c);
    }

    SECTION("G28 returns to machine home, then moves in G54") {
      p = lex_gprog("G90 G54 \n G28 X-.1 \n G0 X1.0 Y2.0 Z3.0");
      s = all_program_states(p);
      t = program_position_table(s);
      add_unk_row(c);
      add_unk_row(c);
      update_table(MACHINE_COORD_SYSTEM, position(0.0, 0.0, 0.0), c);
      update_table(G54_COORD_SYSTEM, position(1.0, 2.0, 3.0), c);
      REQUIRE(t == c);
    }
    
    SECTION("Setup then several partial moves") {
      p = lex_gprog("G90 G1 X0 Y0 Z0 \n G91 G1 X8 \n G0 X7");
      s = all_program_states(p);
      t = program_position_table(s);
      add_unk_row(c);
      update_table(UNKNOWN_COORD_SYSTEM, position(0.0, 0.0, 0.0), c);
      update_table(UNKNOWN_COORD_SYSTEM, position(8.0, 0.0, 0.0), c);
      update_table(UNKNOWN_COORD_SYSTEM, position(15.0, 0.0, 0.0), c);
      REQUIRE(t == c);
    }
    
  }
}
