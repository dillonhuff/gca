#include "analysis/unfold.h"
#include "catch.hpp"
#include "gcode/lexer.h"

namespace gca {

  TEST_CASE("Unfold gcodes") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<block> p;
    vector<block> res;
    vector<block> correct;

    SECTION("Empty program") {
      p = lex_gprog("");
      res = unfold_gprog(p);
      REQUIRE(res == correct);
    }

    SECTION("Without control flow the program is the same") {
      p = lex_gprog("O101 \n G1 X2\r\n X1\n M30");
      res = unfold_gprog(p);
      REQUIRE(res == p);
    }

    SECTION("One subroutine call") {
      p = lex_gprog("G0 X1.0 \n M97 P3 \n G0 Y2.0 \n M30 \n N3 \n G2 X3.0 \n M99");
      res = unfold_gprog(p);
      block b1;
      b1.push_back(token('G', 0));
      b1.push_back(token('X', 1.0));
      block b2;
      b2.push_back(token('N', 3));
      block b3;
      b3.push_back(token('G', 2));
      b3.push_back(token('X', 3.0));
      block b4;
      b4.push_back(token('G', 0));
      b4.push_back(token('Y', 2.0));
      block b5;
      b5.push_back(token('M', 30));
      correct.push_back(b1);
      correct.push_back(b2);
      correct.push_back(b3);
      correct.push_back(b4);
      correct.push_back(b5);
      REQUIRE(res == correct);
    }
  }
}
