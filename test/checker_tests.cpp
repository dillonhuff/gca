#include "bounds_checker.h"
#include "catch.hpp"
#include "context.h"
#include "extra_instruction_checker.h"
#include "g0_move_checker.h"
#include "parser.h"

namespace gca {

  TEST_CASE("Checkers") {
    context c;

    SECTION("Extra instruction checker one warning") {
      gprog* p = parse_gprog(c, "G91 G91");
      extra_instruction_checker checker(GCA_ABSOLUTE);
      checker.exec(p);
      warning_state* s = static_cast<warning_state*>(checker.get_state(GCA_WARNING_STATE));
      int num_warnings = s->num_warnings();
      REQUIRE(num_warnings == 1);
    }

    SECTION("Program bounds checker true") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(12.5, -10.3, 0.0));
      p->push_back(c.mk_minstr(2));
      bounds_checker b(GCA_ABSOLUTE, 0, 30, -20, -10, -5.0, 2.0);
      b.exec(p);
      warning_state* s = static_cast<warning_state*>(b.get_state(GCA_WARNING_STATE));
      int num_warnings = s->num_warnings();
      REQUIRE(num_warnings == 0);
    }

    SECTION("Program bounds checker false") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G1(12.5, -10.3, 0.0));
      p->push_back(c.mk_minstr(2));
      bounds_checker b(GCA_ABSOLUTE, 0, 9, -20, -10, 0.0, 2.0);
      b.exec(p);
      warning_state* s = static_cast<warning_state*>(b.get_state(GCA_WARNING_STATE));
      int num_warnings = s->num_warnings();
      REQUIRE(num_warnings == 1);
    }

    SECTION("Program bounds checker true relative") {
      gprog* p = parse_gprog(c, "G91 G1 X8 G0 X7");
      bounds_checker b(GCA_ABSOLUTE, 0, 9, -20, 10, 0.0, 2.0);
      b.exec(p);
      warning_state* s = static_cast<warning_state*>(b.get_state(GCA_WARNING_STATE));
      int num_warnings = s->num_warnings();
      REQUIRE(num_warnings == 1);
    }
    
    SECTION("g0_move_checker no mistake") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(2.0, 2.0, 0.0));
      g0_move_checker checker(GCA_ABSOLUTE);
      checker.exec(p);
      warning_state* s = static_cast<warning_state*>(checker.get_state(GCA_WARNING_STATE));
      int num_warnings = s->num_warnings();
      REQUIRE(num_warnings == 0);
    }

    SECTION("g0_move_checker mistake") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(2.0, 2.0, 1.0));
      g0_move_checker checker(GCA_ABSOLUTE);
      checker.exec(p);
      warning_state* s = static_cast<warning_state*>(checker.get_state(GCA_WARNING_STATE));
      int num_warnings = s->num_warnings();
      REQUIRE(num_warnings == 1);
    }

    SECTION("g0_move_checker several instructions no mistake") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G1(2.0, 2.0, 1.0));
      p->push_back(c.mk_minstr(2));
      g0_move_checker checker(GCA_ABSOLUTE);
      checker.exec(p);
      warning_state* s = static_cast<warning_state*>(checker.get_state(GCA_WARNING_STATE));
      int num_warnings = s->num_warnings();
      REQUIRE(num_warnings == 0);
    }

    SECTION("g0_move_checker several instructions one mistake") {
      gprog* p = parse_gprog(c, "G1 X2.0 Y2.0 Z1.0 G0 X0.0 Y0.0 Z2.0 M2");
      g0_move_checker checker(GCA_ABSOLUTE);
      checker.exec(p);
      warning_state* s = static_cast<warning_state*>(checker.get_state(GCA_WARNING_STATE));
      int num_warnings = s->num_warnings();
      REQUIRE(num_warnings == 1);
    }
    
    SECTION("g0_move_checker several instructions relative mistake") {
      gprog* p = c.mk_gprog();
      p->push_back(c.mk_G0(0.0, 2.0, 1.0, GCA_RELATIVE));
      p->push_back(c.mk_minstr(2));
      g0_move_checker checker(GCA_ABSOLUTE);
      checker.exec(p);
      warning_state* s = static_cast<warning_state*>(checker.get_state(GCA_WARNING_STATE));
      int num_warnings = s->num_warnings();
      REQUIRE(num_warnings == 1);
    }
    
  }
}
