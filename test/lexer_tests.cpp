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
      b.push_back(icode::make('M', *ilit::make(2)));
      correct.push_back(b);
      REQUIRE(p == correct);
    }

    SECTION("Parse M30 line") {
      p = lex_gprog("M30");
      block b;
      b.push_back(icode::make('M', *ilit::make(30)));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Parse G00 line, no spaces") {
      p = lex_gprog("G00X12.0Y8.0Z-4.5");
      block b;
      b.push_back(icode::make('G', *ilit::make(0)));
      b.push_back(icode::make('X', *lit::make(12.0)));
      b.push_back(icode::make('Y', *lit::make(8.0)));
      b.push_back(icode::make('Z', *lit::make(-4.5)));
      correct.push_back(b);
      REQUIRE((p == correct));
    }    

    SECTION("Parse G00 line, all 12.0") {
      p = lex_gprog("G00 X12.0 Y12.0 Z12.0");
      block b;
      b.push_back(icode::make('G', *ilit::make(0)));
      b.push_back(icode::make('X', *lit::make(12.0)));
      b.push_back(icode::make('Y', *lit::make(12.0)));
      b.push_back(icode::make('Z', *lit::make(12.0)));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Parse G00 line") {
      p = lex_gprog("G00 X30.0 Y12 Z-1.5");
      block b;
      b.push_back(icode::make('G', *ilit::make(0)));
      b.push_back(icode::make('X', *lit::make(30.0)));
      b.push_back(icode::make('Y', *lit::make(12.0)));
      b.push_back(icode::make('Z', *lit::make(-1.5)));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Parse G1 line") {
      p = lex_gprog("G1 X32.0 Y-6.0 Z-1.5");
      block b;
      b.push_back(icode::make('G', *ilit::make(1)));
      b.push_back(icode::make('X', *lit::make(32.0)));
      b.push_back(icode::make('Y', *lit::make(-6.0)));
      b.push_back(icode::make('Z', *lit::make(-1.5)));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Parse G1 line with feedrate") {
      p = lex_gprog("G1 F4 X32.0 Y-6.0 Z-1.5");
      block b;
      b.push_back(icode::make('G', *ilit::make(1)));
      b.push_back(icode::make('F', *lit::make(4)));
      b.push_back(icode::make('X', *lit::make(32.0)));
      b.push_back(icode::make('Y', *lit::make(-6.0)));
      b.push_back(icode::make('Z', *lit::make(-1.5)));
      correct.push_back(b);
      REQUIRE((p == correct));
    }


    SECTION("Parse G1 line with feedrate at back") {
      p = lex_gprog("G1 X32.0 Y-6.0 Z-1.5 F4.5");
      block b;
      b.push_back(icode::make('G', *ilit::make(1)));
      b.push_back(icode::make('X', *lit::make(32.0)));
      b.push_back(icode::make('Y', *lit::make(-6.0)));
      b.push_back(icode::make('Z', *lit::make(-1.5)));
      b.push_back(icode::make('F', *lit::make(4.5)));
      correct.push_back(b);
      REQUIRE((p == correct));
    }

    SECTION("Multi line code with relative coordinates") {
      p = lex_gprog("G91\nG0 X1.0 Y1.0 Z1.0\nG0 Y0.5\nM2");
      block b1;
      b1.push_back(icode::make('G', *ilit::make(91)));
      block b2;
      b2.push_back(icode::make('G', *ilit::make(0)));
      b2.push_back(icode::make('X', *lit::make(1.0)));
      b2.push_back(icode::make('Y', *lit::make(1.0)));
      b2.push_back(icode::make('Z', *lit::make(1.0)));
      block b3;
      b3.push_back(icode::make('G', *ilit::make(0)));
      b3.push_back(icode::make('Y', *lit::make(0.5)));
      block b4;
      b4.push_back(icode::make('M', *ilit::make(2)));
      correct.push_back(b1);
      correct.push_back(b2);
      correct.push_back(b3);
      correct.push_back(b4);
    }
  }
}
