#define CATCH_CONFIG_MAIN

#include <string>

#include "src/gprog.h"
#include "src/parser.h"
#include "test/catch.hpp"

namespace gca {

  TEST_CASE("Parse GCODE") {
    context c;

    SECTION("Parse empty string") {
      string s = "";
      gprog* p = parse_gprog(c, s);
      REQUIRE(p != NULL);
    }

    SECTION("Parse M2 line") {
      string s = "M2";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_minstr(2));
      REQUIRE(p->size() == 1);
      REQUIRE(*p == *correct);
    }

    SECTION("Parse M30 line") {
      string s = "M30";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_minstr(30));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G00 line") {
      string s = "G00 X30.0 Y12 Z-1.5";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G0(30.0, 12.0, -1.5));
      REQUIRE(*p == *correct);
    }
    
  }

}
