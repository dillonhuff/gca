#include "catch.hpp"
#include "checkers/bounds_checker.h"
#include "checkers/extra_instruction_checker.h"
#include "checkers/forbidden_tool_checker.h"
#include "checkers/g0_move_checker.h"
#include "checkers/unsafe_spindle_checker.h"
#include "core/context.h"
#include "core/parser.h"

namespace gca {

  TEST_CASE("Checkers") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Extra instruction checker one warning") {
      gprog* p = parse_gprog("G91 G91");
      REQUIRE((check_for_extra_instructions(p, GCA_ABSOLUTE) == 1));
    }

    SECTION("Program bounds checker true") {
      gprog* p = mk_gprog();
      p->push_back(g0_instr::make(12.5, -10.3, 0.0));
      p->push_back(mk_m2_instr());
      REQUIRE((check_bounds(p, GCA_ABSOLUTE, 0, 30, -20, -10, -5.0, 2.0) == 0));
    }

    SECTION("Program bounds checker false") {
      gprog* p = mk_gprog();
      p->push_back(mk_G1(12.5, -10.3, 0.0));
      p->push_back(mk_m2_instr());
      REQUIRE(check_bounds(p, GCA_ABSOLUTE, 0, 9, -20, -10, 0.0, 2.0)  == 1);
    }

    SECTION("Program bounds checker true relative") {
      gprog* p = parse_gprog("G91 G1 X8 G0 X7");
      REQUIRE(check_bounds(p, GCA_ABSOLUTE, 0, 9, -20, 10, 0.0, 2.0) == 1);
    }

    SECTION("g0 move checker no g0s") {
      gprog* p = parse_gprog("G1 X19");
      REQUIRE(check_for_diagonal_G0_moves(p, GCA_ABSOLUTE) == 0);
    }
    
    SECTION("g0_move_checker no mistake") {
      gprog* p = mk_gprog();
      p->push_back(g0_instr::make(2.0, 2.0, 0.0));
      REQUIRE(check_for_diagonal_G0_moves(p, GCA_ABSOLUTE) == 0);
    }

    SECTION("g0_move_checker mistake") {
      gprog* p = mk_gprog();
      p->push_back(g0_instr::make(2.0, 2.0, 1.0));
      REQUIRE(check_for_diagonal_G0_moves(p, GCA_ABSOLUTE) == 1);
    }

    SECTION("g0_move_checker several instructions no mistake") {
      gprog* p = mk_gprog();
      p->push_back(mk_G1(2.0, 2.0, 1.0));
      p->push_back(mk_m2_instr());
      REQUIRE(check_for_diagonal_G0_moves(p, GCA_ABSOLUTE) == 0);
    }

    SECTION("g0_move_checker several instructions one mistake") {
      gprog* p = parse_gprog("G1 X2.0 Y2.0 Z1.0 G0 X0.0 Y0.0 Z2.0 M2");
      REQUIRE(check_for_diagonal_G0_moves(p, GCA_ABSOLUTE) == 1);
    }
    
    SECTION("g0_move_checker several instructions relative mistake") {
      gprog* p = mk_gprog();
      p->push_back(g0_instr::make(0.0, 2.0, 1.0));
      p->push_back(mk_m2_instr());
      REQUIRE(check_for_diagonal_G0_moves(p, GCA_ABSOLUTE) == 1);
    }
  }

  TEST_CASE("Bad tool change checker") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("No bad tool change") {
      gprog* p = parse_gprog("T6 G1 X1.0 Y2.0 Z1.0");
      vector<int> permitted_tools;
      permitted_tools.push_back(6);
      REQUIRE(check_for_forbidden_tool_changes(permitted_tools, p) == 0);
    }

    SECTION("Bad tool change") {
      gprog* p = parse_gprog("T2 G1 X1.0 Y2.0 Z1.0");
      vector<int> permitted_tools;
      permitted_tools.push_back(6);
      REQUIRE(check_for_forbidden_tool_changes(permitted_tools, p) == 1);
    }
  }

  TEST_CASE("Unsafe spindle on checker") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("No bad spindle use") {
      gprog* p = parse_gprog("M5 T6 T2 S0 M3");
      vector<int> no_spindle_tools;
      no_spindle_tools.push_back(6);
      REQUIRE(check_for_unsafe_spindle_on(no_spindle_tools, 2, p) == 0);
    }

    SECTION("Bad spindle use") {
      gprog* p = parse_gprog("M5 T6 S1200 M3");
      vector<int> no_spindle_tools;
      no_spindle_tools.push_back(6);
      REQUIRE(check_for_unsafe_spindle_on(no_spindle_tools, 2, p) == 1);
    }
  }
}
