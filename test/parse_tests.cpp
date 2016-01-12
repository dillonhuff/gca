#define CATCH_CONFIG_MAIN

#include <string>

#include "catch.hpp"
#include "core/gprog.h"
#include "core/parser.h"

namespace gca {

  TEST_CASE("Parse GCODE with absolute coordinates") {
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
      correct->push_back(c.mk_G0(point(12.0, 8.0, -4.5)));
      REQUIRE(*p == *correct);
    }    
    
    SECTION("Parse G00 line, all 12.0") {
      string s = "G00 X12.0 Y12.0 Z12.0";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G0(point(12.0, 12.0, 12.0)));
      REQUIRE(*p == *correct);
    }
    
   SECTION("Parse G00 line") {
      string s = "G00 X30.0 Y12 Z-1.5";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G0(point(30.0, 12.0, -1.5)));
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
      string s = "G0 X2.75 Y8.0 Z0.0 G1 Z-1.5";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G0(point(2.75, 8.0, 0.0)));
      correct->push_back(c.mk_G1(2.75, 8.0, -1.5));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G0 line with defaults") {
      string s = "G1 X2.75 Y8.0 Z0.0 G0 Z-1.5";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G1(2.75, 8.0, 0.0));
      correct->push_back(c.mk_G0(point(2.75, 8.0, -1.5)));
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
      string s = "G1 X1.0 Y0.0 Z-1.5\nG0X12.5\nM2";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G1(1.0, 0.0, -1.5));
      correct->push_back(c.mk_G0(point(12.5, 0.0, -1.5)));
      correct->push_back(c.mk_minstr(2));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse Multi-line GCODE with comments") {
      string s = "G1 X0.0 Y2.75 Z-1.5 (comment 1)\nG0X12.5 (Comment \n number 2) (s) \n( f)M2 \n(Comment G1 X0.0)";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G1(0.0, 2.75, -1.5));
      correct->push_back(c.mk_G0(point(12.5, 2.75, -1.5)));
      correct->push_back(c.mk_minstr(2));
      REQUIRE(*p == *correct);
    }

    SECTION("Read and parse file") {
      string fn = "/Users/dillon/CppWorkspace/gca/test/test_1.txt";
      gprog* p = read_file(c, fn);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G1(0.0, 0.0, -1.5));
      correct->push_back(c.mk_G0(point(12.5, 1.5, 0)));
      correct->push_back(c.mk_G1(18.0, 1.5, -0.25));
      correct->push_back(c.mk_minstr(2));
      REQUIRE(*p == *correct);
    }

    SECTION("Read and parse real CNC file") {
      string fn = "/Users/dillon/CppWorkspace/gca/test/drill1.tap";
      gprog* p = read_file(c, fn);
      cout << "Real CNC program" << endl;
      cout << *p;
      REQUIRE(p->size() == 42);
    }    
    
  }

  TEST_CASE("Parse GCODE with relative coordinates") {
    context c;
    
    SECTION("Parse G91") {
      string s = "G91\n";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G91());
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G91 and change to relative") {
      string s = "G91 G0 X1.0 Y0.0 Z0.0\n";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G91());
      correct->push_back(c.mk_G0(1.0, 0.0, 0.0, GCA_RELATIVE));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse multi line relative code") {
      string s = "G91\nG0 X1.0 Y1.0 Z1.0\nG0 Y0.5\nM2";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G91());
      correct->push_back(c.mk_G0(1.0, 1.0, 1.0, GCA_RELATIVE));
      correct->push_back(c.mk_G0(0.0, 0.5, 0.0, GCA_RELATIVE));
      correct->push_back(c.mk_minstr(2));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G53") {
      string s = "G53 X3.0 Y2.0 Z1.0";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G53(point(3, 2, 1)));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G53 with default position") {
      string s = "G90 G53 Z1.0";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_G90());
      correct->push_back(c.mk_G53(point(0, 0, 1)));
      REQUIRE(*p == *correct);
    }
    
  }

  TEST_CASE("Parse GCODE with variables") {
    context c;

    SECTION("Parse assign") {
      string s = "#1=14";
      gprog* p = parse_gprog(c, s);
      gprog* correct = c.mk_gprog();
      correct->push_back(c.mk_assign(c.mk_var(1), c.mk_lit(14)));
      REQUIRE(*p == *correct);
    }
  }

}
