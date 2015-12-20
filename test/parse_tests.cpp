#define CATCH_CONFIG_MAIN

#include <string>

#include "catch.hpp"
#include "gprog.h"
#include "parser.h"

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

    SECTION("Parse G00 line, no spaces") {
      string s = "G00X12.0Y8.0Z-4.5";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G0(12.0, 8.0, -4.5));
      REQUIRE(*p == *correct);
    }    
    
    SECTION("Parse G00 line, all 12.0") {
      string s = "G00 X12.0 Y12.0 Z12.0";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G0(12.0, 12.0, 12.0));
      REQUIRE(*p == *correct);
    }
    
   SECTION("Parse G00 line") {
      string s = "G00 X30.0 Y12 Z-1.5";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G0(30.0, 12.0, -1.5));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G1 line") {
      string s = "G1 X32.0 Y-6.0 Z-1.5";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G1(32.0, -6.0, -1.5));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G1 line with defaults") {
      string s = "G1 Z-1.5";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G1(0.0, 0.0, -1.5));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G1 line with front feedrate") {
      string s = "G1 F4 X0.0 Y0.0 Z0.0";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G1(0.0, 0.0, 0.0, 4.0));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G1 line with back feedrate") {
      string s = "G1 X0.0 Y0.0 Z0.0 F4";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G1(0.0, 0.0, 0.0, 4.0));
      REQUIRE(*p == *correct);
    }
    
    SECTION("Parse Multi-line GCODE") {
      string s = "G1 Z-1.5\nG0X12.5\nM2";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G1(0.0, 0.0, -1.5));
      correct->push_back(c.mk_G0(12.5, 0.0, 0));
      correct->push_back(c.mk_minstr(2));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse Multi-line GCODE with comments") {
      string s = "G1 Z-1.5 (comment 1)\nG0X12.5 (Comment \n number 2) (s) \n( f)M2 \n(Comment G1 X0.0)";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G1(0.0, 0.0, -1.5));
      correct->push_back(c.mk_G0(12.5, 0.0, 0));
      correct->push_back(c.mk_minstr(2));
      REQUIRE(*p == *correct);
    }

    SECTION("Read and parse file") {
      string fn = "/Users/dillon/CppWorkspace/gca/test/test_1.txt";
      gprog* p = read_file(c, fn);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G1(0.0, 0.0, -1.5));
      correct->push_back(c.mk_G0(12.5, 1.5, 0));
      correct->push_back(c.mk_G1(18.0, 1.5, -0.25));
      correct->push_back(c.mk_minstr(2));
      cout << "-- Correct" << endl;
      cout << *correct;
      cout << "-- Actual" << endl;
      cout << *p;
      REQUIRE(*p == *correct);
    }    
  }

}
