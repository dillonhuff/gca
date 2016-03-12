#include "catch.hpp"
#include "core/lexer.h"

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
      cout << p << endl;
      REQUIRE(p.size() == 0);
    }

    SECTION("Parse M2 line") {
      p = lex_gprog("M2");
      block b;
      b.push_back(icode::make('M', 2));
      correct.push_back(b);
      REQUIRE(p == correct);
    }

    SECTION("Parse M30 line") {
      p = lex_gprog("M30");
      block b;
      b.push_back(icode::make('M', 30));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Parse G00 line, no spaces") {
      p = lex_gprog("G00X12.0Y8.0Z-4.5");
      block b;
      b.push_back(icode::make('G', 0));
      b.push_back(icode::make('X', 12.0));
      b.push_back(icode::make('Y', 8.0));
      b.push_back(icode::make('Z', -4.5));
      correct.push_back(b);
      REQUIRE((p == correct));
    }    

    SECTION("Parse G00 line, all 12.0") {
      p = lex_gprog("G00 X12.0 Y12.0 Z12.0");
      block b;
      b.push_back(icode::make('G', 0));
      b.push_back(icode::make('X', 12.0));
      b.push_back(icode::make('Y', 12.0));
      b.push_back(icode::make('Z', 12.0));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Parse G00 line") {
      p = lex_gprog("G00 X30.0 Y12 Z-1.5");
      block b;
      b.push_back(icode::make('G', 0));
      b.push_back(icode::make('X', 30.0));
      b.push_back(icode::make('Y', 12.0));
      b.push_back(icode::make('Z', -1.5));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Parse G1 line") {
      p = lex_gprog("G1 X32.0 Y-6.0 Z-1.5");
      block b;
      b.push_back(icode::make('G', 1));
      b.push_back(icode::make('X', 32.0));
      b.push_back(icode::make('Y', -6.0));
      b.push_back(icode::make('Z', -1.5));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Parse G1 line with feedrate") {
      p = lex_gprog("G1 F4 X32.0 Y-6.0 Z-1.5");
      block b;
      b.push_back(icode::make('G', 1));
      b.push_back(icode::make('F', 4.0));
      b.push_back(icode::make('X', 32.0));
      b.push_back(icode::make('Y', -6.0));
      b.push_back(icode::make('Z', -1.5));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Parse G1 line with feedrate at back") {
      p = lex_gprog("G1 X32.0 Y-6.0 Z-1.5 F4.5");
      block b;
      b.push_back(icode::make('G', 1));
      b.push_back(icode::make('X', 32.0));
      b.push_back(icode::make('Y', -6.0));
      b.push_back(icode::make('Z', -1.5));
      b.push_back(icode::make('F', 4.5));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Multi line code with relative coordinates") {
      p = lex_gprog("G91\nG0 X1.0 Y1.0 Z1.0\nG0 Y0.5\nM2");
      block b1;
      b1.push_back(icode::make('G', 91));
      block b2;
      b2.push_back(icode::make('G', 0));
      b2.push_back(icode::make('X', 1.0));
      b2.push_back(icode::make('Y', 1.0));
      b2.push_back(icode::make('Z', 1.0));
      block b3;
      b3.push_back(icode::make('G', 0));
      b3.push_back(icode::make('Y', 0.5));
      block b4;
      b4.push_back(icode::make('M', 2));
      correct.push_back(b1);
      correct.push_back(b2);
      correct.push_back(b3);
      correct.push_back(b4);
      REQUIRE((p == correct));
    }

    SECTION("G53") {
      p = lex_gprog("G53 X3.0 Y2.0 Z1.0");
      block b;
      b.push_back(icode::make('G', 53));
      b.push_back(icode::make('X', 3.0));
      b.push_back(icode::make('Y', 2.0));
      b.push_back(icode::make('Z', 1.0));
      correct.push_back(b);
      REQUIRE(p == correct);
    }

    SECTION("2 G0s in a row") {
      p = lex_gprog("G0 X1 Y1 Z4\n X3 Y4 Z2");
      block b1;
      b1.push_back(icode::make('G', 0));
      b1.push_back(icode::make('X', 1.0));
      b1.push_back(icode::make('Y', 1.0));
      b1.push_back(icode::make('Z', 4.0));
      block b2;
      b2.push_back(icode::make('X', 3.0));
      b2.push_back(icode::make('Y', 4.0));
      b2.push_back(icode::make('Z', 2.0));
      correct.push_back(b1);
      correct.push_back(b2);
      REQUIRE(p == correct);
    }

    SECTION("G2 with feedrate") {
      p = lex_gprog("G2 F12.5 X3 Y4.5 Z1 I1.0 J1.75");
      block b;
      b.push_back(icode::make('G', 2));
      b.push_back(icode::make('F', 12.5));
      b.push_back(icode::make('X', 3.0));
      b.push_back(icode::make('Y', 4.5));
      b.push_back(icode::make('Z', 1.0));
      b.push_back(icode::make('I', 1.0));
      b.push_back(icode::make('J', 1.75));
      correct.push_back(b);
      REQUIRE(p == correct);
    }

    SECTION("2 G1s in a row with feedrate") {
      p = lex_gprog("G1 X2 Y2 Z2 \n F5.0 Z3");
      block b1;
      b1.push_back(icode::make('G', 1));
      b1.push_back(icode::make('X', 2.0));
      b1.push_back(icode::make('Y', 2.0));
      b1.push_back(icode::make('Z', 2.0));
      block b2;
      b2.push_back(icode::make('F', 5.0));
      b2.push_back(icode::make('Z', 3.0));
      correct.push_back(b1);
      correct.push_back(b2);
      REQUIRE(p == correct);
    }

    SECTION("Nested parens comment") {
      p = lex_gprog("(())");
      block b;
      b.push_back(cmt::make("(())"));
      correct.push_back(b);
      REQUIRE(p == correct);
    }

    SECTION("G83") {
      p = lex_gprog("G83 G99 X3.1587 Y4.2467 Z-.15 R.1 Q.0547 F3.");
      block b;
      b.push_back(icode::make('G', 83));
      b.push_back(icode::make('G', 99));
      b.push_back(icode::make('X', 3.1587));
      b.push_back(icode::make('Y', 4.2467));
      b.push_back(icode::make('Z', -0.15));
      b.push_back(icode::make('R', 0.1));
      b.push_back(icode::make('Q', 0.0547));
      b.push_back(icode::make('F', 3.0));
      correct.push_back(b);
      REQUIRE(p == correct);
    }
  }
}
