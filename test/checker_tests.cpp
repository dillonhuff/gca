#include "catch.hpp"
#include "checkers/bounds_checker.h"
#include "checkers/forbidden_tool_checker.h"
#include "checkers/unsafe_spindle_checker.h"
#include "core/lexer.h"
#include "core/parser.h"

namespace gca {

  TEST_CASE("Bounds checker") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<block> p;

    SECTION("Program bounds checker true") {
      p = lex_gprog("G0 X12.5 Y-10.3 Z0.0 \n M2");
      REQUIRE((check_bounds(p, GCA_ABSOLUTE, 0, 30, -20, -10, -5.0, 2.0) == 0));
    }

    SECTION("Program bounds checker false") {
      p = lex_gprog("G1 X12.5 Y-10.3 Z0.0");
      REQUIRE(check_bounds(p, GCA_ABSOLUTE, 0, 9, -20, -10, 0.0, 2.0)  == 1);
    }

    SECTION("Program bounds checker true relative") {
      p = lex_gprog("G90 G1 X0 Y0 Z0 \n G91 G1 X8 \n G0 X7");
      REQUIRE(check_bounds(p, GCA_ABSOLUTE, 0, 9, -20, 10, 0.0, 2.0) == 1);
    }
  }

  TEST_CASE("Bad tool change checker") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<block> p;

    SECTION("No bad tool change") {
      p = lex_gprog("T6 G1 X1.0 Y2.0 Z1.0");
      vector<int> permitted_tools;
      permitted_tools.push_back(6);
      REQUIRE(check_for_forbidden_tool_changes(permitted_tools, p) == 0);
    }

    SECTION("Bad tool change") {
      p = lex_gprog("T2 G1 X1.0 Y2.0 Z1.0");
      vector<int> permitted_tools;
      permitted_tools.push_back(6);
      REQUIRE(check_for_forbidden_tool_changes(permitted_tools, p) == 1);
    }
  }

  TEST_CASE("Unsafe spindle on checker") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<block> p;

    SECTION("No bad spindle use") {
      p = lex_gprog("M5 T6 \n T2 S0 M3");
      vector<int> no_spindle_tools;
      no_spindle_tools.push_back(6);
      REQUIRE(check_for_unsafe_spindle_on(no_spindle_tools, 2, p) == 0);
    }

    SECTION("Bad spindle use") {
      p = lex_gprog("M5 T6 \n S1200 M3");
      vector<int> no_spindle_tools;
      no_spindle_tools.push_back(6);
      REQUIRE(check_for_unsafe_spindle_on(no_spindle_tools, 2, p) == 1);
    }
  }
}
