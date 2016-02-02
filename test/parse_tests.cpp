#define CATCH_CONFIG_MAIN

#include <string>

#include "catch.hpp"
#include "core/gprog.h"
#include "core/parser.h"

namespace gca {

  TEST_CASE("Parse GCODE with absolute coordinates") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Parse empty string") {
      string s = "";
      gprog* p = parse_gprog(s);
      REQUIRE(p != NULL);
    }

    SECTION("Parse M2 line") {
      string s = "M2";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(new (allocate<m2_instr>()) m2_instr());
      REQUIRE(p->size() == 1);
      REQUIRE(*p == *correct);
    }

    SECTION("Parse M30 line") {
      string s = "M30";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_m30_instr());
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G00 line, no spaces") {
      string s = "G00X12.0Y8.0Z-4.5";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G0(point(12.0, 8.0, -4.5)));
      REQUIRE(*p == *correct);
    }    
    
    SECTION("Parse G00 line, all 12.0") {
      string s = "G00 X12.0 Y12.0 Z12.0";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G0(point(12.0, 12.0, 12.0)));
      REQUIRE(*p == *correct);
    }
    
   SECTION("Parse G00 line") {
      string s = "G00 X30.0 Y12 Z-1.5";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G0(point(30.0, 12.0, -1.5)));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G1 line") {
      string s = "G1 X32.0 Y-6.0 Z-1.5";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G1(mk_lit(32.0), mk_lit(-6.0), mk_lit(-1.5), mk_omitted()));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G1 line with defaults") {
      string s = "G0 X2.75 Y8.0 Z0.0 G1 Z-1.5";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G0(mk_lit(2.75), mk_lit(8), mk_lit(0)));
      correct->push_back(mk_G1(mk_omitted(), mk_omitted(), mk_lit(-1.5), mk_omitted()));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G0 line with defaults") {
      string s = "G1 X2.75 Y8.0 Z0.0 G0 Z-1.5";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G1(mk_lit(2.75), mk_lit(8.0), mk_lit(0.0), mk_omitted()));
      correct->push_back(mk_G0(mk_omitted(), mk_omitted(), mk_lit(-1.5)));
      REQUIRE(*p == *correct);
    }
    
    SECTION("Parse G1 line with front feedrate") {
      string s = "G1 F4 X0.0 Y0.0 Z0.0";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G1(0.0, 0.0, 0.0, 4.0));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G1 line with back feedrate") {
      string s = "G1 X0.0 Y0.0 Z0.0 F4";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G1(0.0, 0.0, 0.0, 4.0));
      REQUIRE(*p == *correct);
    }
    
    SECTION("Parse Multi-line GCODE") {
      string s = "G1 X1.0 Y0.0 Z-1.5\nG0X12.5\nM2";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G1(mk_lit(1.0), mk_lit(0.0), mk_lit(-1.5), mk_omitted()));
      correct->push_back(mk_G0(mk_lit(12.5), mk_omitted(), mk_omitted()));
      correct->push_back(new (allocate<m2_instr>()) m2_instr());
      REQUIRE(*p == *correct);
    }

    SECTION("Parse Multi-line GCODE with comments") {
      string s = "G1 X0.0 Y2.75 Z-1.5 (comment 1)\nG0X12.5 (Comment \n number 2) (s) \n( f)M2 \n(Comment G1 X0.0)";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G1(mk_lit(0.0), mk_lit(2.75), mk_lit(-1.5), mk_omitted()));
      correct->push_back(mk_comment('(', ')', "comment 1"));
      correct->push_back(mk_G0(mk_lit(12.5), mk_omitted(), mk_omitted()));
      correct->push_back(mk_comment('(', ')', "Comment \n number 2"));
      correct->push_back(mk_comment('(', ')', "s"));
      correct->push_back(mk_comment('(', ')', " f"));
      correct->push_back(new (allocate<m2_instr>()) m2_instr());
      correct->push_back(mk_comment('(', ')', "Comment G1 X0.0"));
      REQUIRE(*p == *correct);
    }

    SECTION("Read and parse file") {
      string fn = "/Users/dillon/CppWorkspace/gca/test/test_1.txt";
      gprog* p = read_file(fn);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G1(mk_lit(0.0), mk_lit(0.0), mk_lit(-1.5), mk_omitted()));
      correct->push_back(mk_G0(point(12.5, 1.5, 0)));
      correct->push_back(mk_G1(mk_lit(18.0), mk_lit(1.5), mk_lit(-0.25), mk_omitted()));
      correct->push_back(new (allocate<m2_instr>()) m2_instr());
      REQUIRE(*p == *correct);
    }

    SECTION("Read and parse real CNC file") {
      string fn = "/Users/dillon/CppWorkspace/gca/test/drill1.tap";
      gprog* p = read_file(fn);
      REQUIRE(p->size() == 61);
    }
    
  }

  TEST_CASE("Parse GCODE with relative coordinates") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Parse G91") {
      string s = "G91\n";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G91());
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G91 and change to relative") {
      string s = "G91 G0 X1.0 Y0.0 Z0.0\n";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G91());
      correct->push_back(mk_G0(1.0, 0.0, 0.0));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse multi line relative code") {
      string s = "G91\nG0 X1.0 Y1.0 Z1.0\nG0 Y0.5\nM2";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G91());
      correct->push_back(mk_G0(1.0, 1.0, 1.0));
      correct->push_back(mk_G0(mk_omitted(), mk_lit(0.5), mk_omitted()));
      correct->push_back(new (allocate<m2_instr>()) m2_instr());
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G53") {
      string s = "G53 X3.0 Y2.0 Z1.0";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G53(mk_lit(3), mk_lit(2), mk_lit(1)));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse G53 with default position") {
      string s = "G90 G53 Z1.0";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G90());
      correct->push_back(mk_G53(mk_omitted(), mk_omitted(), mk_lit(1)));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse F instruction") {
      gprog* p = parse_gprog("F15 XY");
      gprog* correct = mk_gprog();
      correct->push_back(mk_f_instr(15, "XY"));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse F instruction not all") {
      gprog* p = parse_gprog("F15 XYZ");
      gprog* incorrect = mk_gprog();
      incorrect->push_back(mk_f_instr(15, "XY"));
      REQUIRE(*p != *incorrect);
    }
    
  }

  TEST_CASE("Parse GCODE with variables") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Parse assign") {
      string s = "#1=14";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_assign(mk_var(1), mk_lit(14)));
      REQUIRE(*p == *correct);
    }

    SECTION("Parse variable coordinate") {
      string s = "G0 X#1025 Y2.5 Z3.0";
      gprog* p = parse_gprog(s);
      gprog* correct = mk_gprog();
      correct->push_back(mk_G0(mk_var(1025), mk_lit(2.5), mk_lit(3)));
      REQUIRE(*p == *correct);
    }
  }

  TEST_CASE("Parsing G2 and G3") {
    arena_allocator a;
    set_system_allocator(&a);


    SECTION("G2 IJ") {
      gprog* p = parse_gprog("G02 X1.0 Y2.0 Z3.0 I-2.0 J0.15");
      gprog* correct = mk_gprog();
      correct->push_back(mk_G2(mk_lit(1), mk_lit(2), mk_lit(3),
				 mk_lit(-2.0), mk_lit(0.15), mk_omitted(),
				 mk_omitted()));
      REQUIRE(*p == *correct);
    }

    SECTION("G3 IK") {
      gprog* p = parse_gprog("G3 X1.0 Y2.0 Z3.0 I-2.0 K0.15");
      gprog* correct = mk_gprog();
      correct->push_back(mk_G3(mk_lit(1), mk_lit(2), mk_lit(3),
				 mk_lit(-2.0), mk_omitted(), mk_lit(0.15),
				 mk_omitted()));
      REQUIRE(*p == *correct);
    }
    
  }

}
