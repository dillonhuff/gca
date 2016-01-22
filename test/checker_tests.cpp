#include "catch.hpp"
#include "checkers/bounds_checker.h"
#include "checkers/extra_instruction_checker.h"
#include "checkers/g0_move_checker.h"
#include "core/context.h"
#include "core/parser.h"

namespace gca {

  TEST_CASE("Checkers") {
    context c;

    SECTION("Extra instruction checker one warning") {
      gprog* p = parse_gprog(c, "G91 G91");
      REQUIRE(check_for_extra_instructions(p, GCA_ABSOLUTE) == 1);
    }

    SECTION("Program bounds checker true") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(12.5, -10.3, 0.0));
      p->push_back(c.mk_minstr(2));
      REQUIRE(check_bounds(p, GCA_ABSOLUTE, 0, 30, -20, -10, -5.0, 2.0) == 0);
    }

    SECTION("Program bounds checker false") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G1(12.5, -10.3, 0.0));
      p->push_back(c.mk_minstr(2));
      REQUIRE(check_bounds(p, GCA_ABSOLUTE, 0, 9, -20, -10, 0.0, 2.0)  == 1);
    }

    SECTION("Program bounds checker true relative") {
      gprog* p = parse_gprog(c, "G91 G1 X8 G0 X7");
      REQUIRE(check_bounds(p, GCA_ABSOLUTE, 0, 9, -20, 10, 0.0, 2.0) == 1);
    }

    SECTION("g0 move checker no g0s") {
      gprog* p = parse_gprog(c, "G1 X19");
      REQUIRE(check_for_diagonal_G0_moves(p, GCA_ABSOLUTE) == 0);
    }
    
    SECTION("g0_move_checker no mistake") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(2.0, 2.0, 0.0));
      REQUIRE(check_for_diagonal_G0_moves(p, GCA_ABSOLUTE) == 0);
    }

    SECTION("g0_move_checker mistake") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(2.0, 2.0, 1.0));
      REQUIRE(check_for_diagonal_G0_moves(p, GCA_ABSOLUTE) == 1);
    }

    SECTION("g0_move_checker several instructions no mistake") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G1(2.0, 2.0, 1.0));
      p->push_back(c.mk_minstr(2));
      REQUIRE(check_for_diagonal_G0_moves(p, GCA_ABSOLUTE) == 0);
    }

    SECTION("g0_move_checker several instructions one mistake") {
      gprog* p = parse_gprog(c, "G1 X2.0 Y2.0 Z1.0 G0 X0.0 Y0.0 Z2.0 M2");
      REQUIRE(check_for_diagonal_G0_moves(p, GCA_ABSOLUTE) == 1);
    }
    
    SECTION("g0_move_checker several instructions relative mistake") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(0.0, 2.0, 1.0));
      p->push_back(c.mk_minstr(2));
      REQUIRE(check_for_diagonal_G0_moves(p, GCA_ABSOLUTE) == 1);
    }
    
  }
}
