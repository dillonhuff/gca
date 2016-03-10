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
      b.push_back(icode::make('M', *lit::make(2)));
      correct.push_back(b);
      REQUIRE(p == correct);
    }
  }
}
