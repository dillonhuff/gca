#define CATCH_CONFIG_MAIN

#include <string>

#include "catch.hpp"
#include "core/gprog.h"
#include "core/parser.h"
#include "system/settings.h"

namespace gca {

  // TEST_CASE("Parse GCODE with absolute coordinates") {
  //   arena_allocator a;
  //   set_system_allocator(&a);
    
  //   SECTION("Parse G1 line with defaults") {
  //     string s = "G0 X2.75 Y8.0 Z0.0 G1 Z-1.5";
  //     gprog* p = parse_gprog(s);
  //     gprog* correct = gprog::make();
  //     correct->push_back(g0_instr::make(lit::make(2.75), lit::make(8), lit::make(0)));
  //     correct->push_back(g1_instr::make(omitted::make(), omitted::make(), lit::make(-1.5), omitted::make()));
  //     REQUIRE((*p == *correct));
  //   }

  //   SECTION("Parse G0 line with defaults") {
  //     string s = "G1 X2.75 Y8.0 Z0.0 G0 Z-1.5";
  //     gprog* p = parse_gprog(s);
  //     gprog* correct = gprog::make();
  //     correct->push_back(g1_instr::make(lit::make(2.75), lit::make(8.0), lit::make(0.0), omitted::make()));
  //     correct->push_back(g0_instr::make(omitted::make(), omitted::make(), lit::make(-1.5)));
  //     REQUIRE((*p == *correct));
  //   }
    
  //   SECTION("Parse Multi-line GCODE") {
  //     string s = "G1 X1.0 Y0.0 Z-1.5\nG0X12.5\nM2";
  //     gprog* p = parse_gprog(s);
  //     gprog* correct = gprog::make();
  //     correct->push_back(g1_instr::make(lit::make(1.0), lit::make(0.0), lit::make(-1.5), omitted::make()));
  //     correct->push_back(g0_instr::make(lit::make(12.5), omitted::make(), omitted::make()));
  //     correct->push_back(new (allocate<m2_instr>()) m2_instr());
  //     REQUIRE((*p == *correct));
  //   }

  //   SECTION("Parse Multi-line GCODE with comments") {
  //     string s = "G1 X0.0 Y2.75 Z-1.5 (comment 1)\nG0X12.5 (Comment number 2) (s) \n( f)M2 \n(Comment G1 X0.0)";
  //     gprog* p = parse_gprog(s);
  //     gprog* correct = gprog::make();
  //     correct->push_back(g1_instr::make(lit::make(0.0), lit::make(2.75), lit::make(-1.5), omitted::make()));
  //     correct->push_back(comment::make('(', ')', "(comment 1)"));
  //     correct->push_back(g0_instr::make(lit::make(12.5), omitted::make(), omitted::make()));
  //     correct->push_back(comment::make('(', ')', "(Comment number 2)"));
  //     correct->push_back(comment::make('(', ')', "(s)"));
  //     correct->push_back(comment::make('(', ')', "( f)"));
  //     correct->push_back(new (allocate<m2_instr>()) m2_instr());
  //     correct->push_back(comment::make('(', ')', "(Comment G1 X0.0)"));
  //     REQUIRE((*p == *correct));
  //   }

  //   SECTION("Read and parse file") {
  //     string fn = project_path + string("gca/test/test_1.txt");
  //     gprog* p = read_file(fn);
  //     gprog* correct = gprog::make();
  //     correct->push_back(g1_instr::make(lit::make(0.0), lit::make(0.0), lit::make(-1.5), omitted::make()));
  //     correct->push_back(g0_instr::make(point(12.5, 1.5, 0)));
  //     correct->push_back(g1_instr::make(lit::make(18.0), lit::make(1.5), lit::make(-0.25), omitted::make()));
  //     correct->push_back(new (allocate<m2_instr>()) m2_instr());
  //     REQUIRE((*p == *correct));
  //   }

  //   SECTION("Read and parse real CNC file") {
  //     string fn = project_path + string("gca/test/drill1.tap");
  //     gprog* p = read_file(fn);
  //     REQUIRE((p->size() == 61));
  //   }
  // }

  // TEST_CASE("Parse GCODE with relative coordinates") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   SECTION("Parse G91") {
  //     string s = "G91\n";
  //     gprog* p = parse_gprog(s);
  //     gprog* correct = gprog::make();
  //     correct->push_back(g91_instr::make());
  //     REQUIRE((*p == *correct));
  //   }

  //   SECTION("Parse G91 and change to relative") {
  //     string s = "G91 G0 X1.0 Y0.0 Z0.0\n";
  //     gprog* p = parse_gprog(s);
  //     gprog* correct = gprog::make();
  //     correct->push_back(g91_instr::make());
  //     correct->push_back(g0_instr::make(1.0, 0.0, 0.0));
  //     REQUIRE(((*p) == (*correct)));
  //   }

  //   SECTION("Parse G53 with default position") {
  //     string s = "G90 G53 Z1.0";
  //     gprog* p = parse_gprog(s);
  //     gprog* correct = gprog::make();
  //     correct->push_back(g90_instr::make());
  //     correct->push_back(g53_instr::make(omitted::make(), omitted::make(), lit::make(1)));
  //     REQUIRE(((*p) == (*correct)));
  //   }

  //   SECTION("Parse F instruction") {
  //     gprog* p = parse_gprog("F15 XY");
  //     gprog* correct = gprog::make();
  //     correct->push_back(f_instr::make(15, "XY"));
  //     REQUIRE(((*p) == (*correct)));
  //   }

  //   SECTION("Parse F instruction not all") {
  //     gprog* p = parse_gprog("F15 XYZ");
  //     gprog* incorrect = gprog::make();
  //     incorrect->push_back(f_instr::make(15, "XY"));
  //     REQUIRE(((*p) != (*incorrect)));
  //   }
    
  // }

  // TEST_CASE("Parse GCODE with variables") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   SECTION("Parse assign") {
  //     string s = "#1=14";
  //     gprog* p = parse_gprog(s);
  //     gprog* correct = gprog::make();
  //     correct->push_back(assign_instr::make(var::make(1), lit::make(14)));
  //     REQUIRE(((*p) == (*correct)));
  //   }

  //   SECTION("Parse variable coordinate") {
  //     string s = "G0 X#1025 Y2.5 Z3.0";
  //     gprog* p = parse_gprog(s);
  //     gprog* correct = gprog::make();
  //     correct->push_back(g0_instr::make(var::make(1025), lit::make(2.5), lit::make(3)));
  //     REQUIRE(((*p) == (*correct)));
  //   }
  // }

  // TEST_CASE("Parsing G2 and G3") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   SECTION("G2 IJ") {
  //     gprog* p = parse_gprog("G02 X1.0 Y2.0 Z3.0 I-2.0 J0.15");
  //     gprog* correct = gprog::make();
  //     correct->push_back(g2_instr::make(lit::make(1), lit::make(2), lit::make(3),
  // 				 lit::make(-2.0), lit::make(0.15), omitted::make(),
  // 				 omitted::make()));
  //     REQUIRE(((*p) == (*correct)));
  //   }

  //   SECTION("G3 IK") {
  //     gprog* p = parse_gprog("G3 X1.0 Y2.0 Z3.0 I-2.0 K0.15");
  //     gprog* correct = gprog::make();
  //     correct->push_back(g3_instr::make(lit::make(1), lit::make(2), lit::make(3),
  // 				 lit::make(-2.0), omitted::make(), lit::make(0.15),
  // 				 omitted::make()));
  //     REQUIRE(((*p) == (*correct)));
  //   }
  // }

  // TEST_CASE("Miscellaneous control instructions") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   gprog correct;

  //   SECTION("G21 G64 P.1 S6000 M4") {
  //     gprog* p = parse_gprog("G21 G64 P.1 S6000 M4");
  //     correct.push_back(g21_instr::make());
  //     correct.push_back(g64_instr::make(lit::make(0.1)));
  //     correct.push_back(s_instr::make(6000));
  //     correct.push_back(m4_instr::make());
  //     REQUIRE(correct == *p);
  //   }

  //   SECTION("Plane selection instructions") {
  //     gprog* p = parse_gprog("G17 G18 G19");
  //     correct.push_back(g17_instr::make());
  //     correct.push_back(g18_instr::make());
  //     correct.push_back(g19_instr::make());
  //     REQUIRE(correct == *p);
  //   }

  //   SECTION("Coolant instructions: M7 M8 M9") {
  //     gprog* p = parse_gprog("M7 M8 M9");
  //     correct.push_back(m7_instr::make());
  //     correct.push_back(m8_instr::make());
  //     correct.push_back(m9_instr::make());
  //     REQUIRE(correct == *p);
  //   }

  //   SECTION("G43 H02") {
  //     gprog* p = parse_gprog("G43 H02");
  //     correct.push_back(g43_instr::make(lit::make(2)));
  //     REQUIRE(correct == *p);
  //   }

  //   SECTION("%\n \n % ") {
  //     gprog* p = parse_gprog("%\n \n % ");
  //     REQUIRE(correct == *p);
  //   }

  //   SECTION("O1001") {
  //     gprog* p = parse_gprog("O1001");
  //     correct.push_back(o_instr::make(1001));
  //     REQUIRE(correct == *p);
  //   }
    
  //   SECTION("N102") {
  //     gprog* p = parse_gprog("N102");
  //     correct.push_back(n_instr::make(102));
  //     REQUIRE(correct == *p);
  //   }

  //   SECTION("G54 G17 G40 G49 G80 G90") {
  //     gprog* p = parse_gprog("G54 G17 G40 G49 G80 G90");
  //     correct.push_back(g54_instr::make());
  //     correct.push_back(g17_instr::make());
  //     correct.push_back(g40_instr::make());
  //     correct.push_back(g49_instr::make());
  //     correct.push_back(g80_instr::make());
  //     correct.push_back(g90_instr::make());
  //     REQUIRE(correct == *p);
  //   }

  //   SECTION("G28") {
  //     gprog* p = parse_gprog("G28 Z0.");
  //     correct.push_back(g28_instr::make(omitted::make(),
  // 					omitted::make(),
  // 					lit::make(0)));
  //     REQUIRE(correct == *p);
  //   }

  //   SECTION("M6 and M97") {
  //     gprog* p = parse_gprog("M6 M97 P136381 M97 P120 L34");
  //     correct.push_back(m6_instr::make());
  //     correct.push_back(m97_instr::make(lit::make(136381), omitted::make()));
  //     correct.push_back(m97_instr::make(lit::make(120), lit::make(34)));
  //     REQUIRE(correct == *p);
  //   }

  //   SECTION("G81") {
  //     gprog* p = parse_gprog("G81 G98 X2.1469 Y2.234 Z-.628 R.1 F12.");
  //     correct.push_back(g81_instr::make(false,
  // 					lit::make(2.1469),
  // 					lit::make(2.234),
  // 					lit::make(-0.628),
  // 					omitted::make(),
  // 					lit::make(0.1),
  // 					omitted::make(),
  // 					lit::make(12.0)));
  //     REQUIRE(correct == *p);
  //   }

  //   SECTION("G85") {
  //     gprog* p = parse_gprog("G85 G98 X2.1469 Y2.234 Z-.628 R.1 F12.");
  //     correct.push_back(g85_instr::make(false,
  // 					lit::make(2.1469),
  // 					lit::make(2.234),
  // 					lit::make(-0.628),
  // 					omitted::make(),
  // 					lit::make(0.1),
  // 					omitted::make(),
  // 					lit::make(12.0)));
  //     REQUIRE(correct == *p);
  //   }
    
  //   SECTION("M99") {
  //     gprog* p = parse_gprog("M99");
  //     correct.push_back(m99_instr::make());
  //     REQUIRE(correct == *p);
  //   }

  //   SECTION("G41 and 42") {
  //     gprog* p = parse_gprog("G41 D52 G42 D17");
  //     correct.push_back(g41_instr::make(lit::make(52)));
  //     correct.push_back(g42_instr::make(lit::make(17)));
  //     REQUIRE(correct == *p);
  //   }

  //   SECTION("M0 and M1") {
  //     gprog* p = parse_gprog("M01 M00");
  //     correct.push_back(m1_instr::make());
  //     correct.push_back(m0_instr::make());
  //     REQUIRE(correct == *p);
  //   }
  // }

}
