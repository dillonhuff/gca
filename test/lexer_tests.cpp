#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "gcode/lexer.h"

namespace gca {

  TEST_CASE("Lexer tests") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<block> p;
    vector<block> correct;
    string s;

    SECTION("Lex empty string") {
      s = "";
      p = lex_gprog(s);
      REQUIRE(p.size() == 0);
    }

    SECTION("Lex empty lines") {
      s = "\n \n%\n";
      p = lex_gprog(s);
      REQUIRE(p.size() == 0);
    }

    SECTION("Parse M2 line") {
      p = lex_gprog("M2");
      block b;
      b.push_back(token('M', 2));
      correct.push_back(b);
      REQUIRE(p == correct);
    }

    SECTION("Parse M30 line") {
      p = lex_gprog("M30");
      block b;
      b.push_back(token('M', 30));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Parse G00 line, no spaces") {
      p = lex_gprog("G00X12.0Y8.0Z-4.5");
      block b;
      b.push_back(token('G', 0));
      b.push_back(token('X', 12.0));
      b.push_back(token('Y', 8.0));
      b.push_back(token('Z', -4.5));
      correct.push_back(b);
      REQUIRE((p == correct));
    }    

    SECTION("Parse G00 line, all 12.0") {
      p = lex_gprog("G00 X12.0 Y12.0 Z12.0");
      block b;
      b.push_back(token('G', 0));
      b.push_back(token('X', 12.0));
      b.push_back(token('Y', 12.0));
      b.push_back(token('Z', 12.0));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Parse G00 line") {
      p = lex_gprog("G00 X30.0 Y12 Z-1.5");
      block b;
      b.push_back(token('G', 0));
      b.push_back(token('X', 30.0));
      b.push_back(token('Y', 12.0));
      b.push_back(token('Z', -1.5));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Parse G1 line") {
      p = lex_gprog("G1 X32.0 Y-6.0 Z-1.5");
      block b;
      b.push_back(token('G', 1));
      b.push_back(token('X', 32.0));
      b.push_back(token('Y', -6.0));
      b.push_back(token('Z', -1.5));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Parse G1 line with feedrate") {
      p = lex_gprog("G1 F4 X32.0 Y-6.0 Z-1.5");
      block b;
      b.push_back(token('G', 1));
      b.push_back(token('F', 4.0));
      b.push_back(token('X', 32.0));
      b.push_back(token('Y', -6.0));
      b.push_back(token('Z', -1.5));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Parse G1 line with feedrate at back") {
      p = lex_gprog("G1 X32.0 Y-6.0 Z-1.5 F4.5");
      block b;
      b.push_back(token('G', 1));
      b.push_back(token('X', 32.0));
      b.push_back(token('Y', -6.0));
      b.push_back(token('Z', -1.5));
      b.push_back(token('F', 4.5));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Multi line code with relative coordinates") {
      p = lex_gprog("G91\nG0 X1.0 Y1.0 Z1.0\nG0 Y0.5\nM2");
      block b1;
      b1.push_back(token('G', 91));
      block b2;
      b2.push_back(token('G', 0));
      b2.push_back(token('X', 1.0));
      b2.push_back(token('Y', 1.0));
      b2.push_back(token('Z', 1.0));
      block b3;
      b3.push_back(token('G', 0));
      b3.push_back(token('Y', 0.5));
      block b4;
      b4.push_back(token('M', 2));
      correct.push_back(b1);
      correct.push_back(b2);
      correct.push_back(b3);
      correct.push_back(b4);
      REQUIRE((p == correct));
    }

    SECTION("G53") {
      p = lex_gprog("G53 X3.0 Y2.0 Z1.0");
      block b;
      b.push_back(token('G', 53));
      b.push_back(token('X', 3.0));
      b.push_back(token('Y', 2.0));
      b.push_back(token('Z', 1.0));
      correct.push_back(b);
      REQUIRE(p == correct);
    }

    SECTION("2 G0s in a row") {
      p = lex_gprog("G0 X1 Y1 Z4\n X3 Y4 Z2");
      block b1;
      b1.push_back(token('G', 0));
      b1.push_back(token('X', 1.0));
      b1.push_back(token('Y', 1.0));
      b1.push_back(token('Z', 4.0));
      block b2;
      b2.push_back(token('X', 3.0));
      b2.push_back(token('Y', 4.0));
      b2.push_back(token('Z', 2.0));
      correct.push_back(b1);
      correct.push_back(b2);
      REQUIRE(p == correct);
    }

    SECTION("G2 with feedrate") {
      p = lex_gprog("G2 F12.5 X3 Y4.5 Z1 I1.0 J1.75");
      block b;
      b.push_back(token('G', 2));
      b.push_back(token('F', 12.5));
      b.push_back(token('X', 3.0));
      b.push_back(token('Y', 4.5));
      b.push_back(token('Z', 1.0));
      b.push_back(token('I', 1.0));
      b.push_back(token('J', 1.75));
      correct.push_back(b);
      REQUIRE(p == correct);
    }

    SECTION("2 G1s in a row with feedrate") {
      p = lex_gprog("G1 X2 Y2 Z2 \n F5.0 Z3");
      block b1;
      b1.push_back(token('G', 1));
      b1.push_back(token('X', 2.0));
      b1.push_back(token('Y', 2.0));
      b1.push_back(token('Z', 2.0));
      block b2;
      b2.push_back(token('F', 5.0));
      b2.push_back(token('Z', 3.0));
      correct.push_back(b1);
      correct.push_back(b2);
      REQUIRE(p == correct);
    }

    SECTION("Nested parens comment") {
      p = lex_gprog("(())");
      block b;
      b.push_back(token("(())"));
      correct.push_back(b);
      REQUIRE(p == correct);
    }

    SECTION("G83") {
      p = lex_gprog("G83 G99 X3.1587 Y4.2467 Z-.15 R.1 Q.0547 F3.");
      block b;
      b.push_back(token('G', 83));
      b.push_back(token('G', 99));
      b.push_back(token('X', 3.1587));
      b.push_back(token('Y', 4.2467));
      b.push_back(token('Z', -0.15));
      b.push_back(token('R', 0.1));
      b.push_back(token('Q', 0.0547));
      b.push_back(token('F', 3.0));
      correct.push_back(b);
      REQUIRE(p == correct);
    }

    SECTION("No spaces") {
      p = lex_gprog("N31S6000M3");
      block b;
      b.push_back(token('N', 31));
      b.push_back(token('S', 6000.0));
      b.push_back(token('M', 3));
      correct.push_back(b);
      REQUIRE(p == correct);
    }

    SECTION("No spaces final .") {
      p = lex_gprog("N23111G91G28Z0.M19");
      block b;
      b.push_back(token('N', 23111));
      b.push_back(token('G', 91));
      b.push_back(token('G', 28));
      b.push_back(token('Z', 0.0));
      b.push_back(token('M', 19));
      correct.push_back(b);
      REQUIRE(p == correct);
    }

    SECTION("O instruction") {
      p = lex_gprog("O1001");
      block b;
      b.push_back(token('O', 1001));
      correct.push_back(b);
      REQUIRE(p == correct);
    }

    SECTION("Example originally from gcode_to_cuts") {
      vector<block> p = lex_gprog("G90 S2000 M3 \n G0 X0 Y0 Z0 \n G1 X1 Y1 Z1 \n S1000 G1 X2 Y2 Z2");
      REQUIRE(p.size() == 4);
    }
  }
}
