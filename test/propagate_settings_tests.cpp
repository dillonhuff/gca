#include "analysis/propagate_settings.h"
#include "catch.hpp"
#include "core/lexer.h"

namespace gca {

  TEST_CASE("Propagate settings") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<block> p;
    vector<block> res;
    vector<block> correct;

    SECTION("Empty program") {
      p = lex_gprog("");
      res = propagate_settings(p);
      REQUIRE(res == correct);
    }

    SECTION("G0") {
      p = lex_gprog("G90 G41\n G0 X1 \n G40 G91");
      res = propagate_settings(p);
      block b1;
      b1.push_back(icode::make('G', 90));
      b1.push_back(icode::make('G', 41));
      block b2;
      b2.push_back(icode::make('G', 90));      
      b2.push_back(icode::make('G', 41));
      b2.push_back(icode::make('G', 0));
      b2.push_back(icode::make('X', 1.0));
      block b3;
      b3.push_back(icode::make('G', 0));
      b3.push_back(icode::make('G', 40));
      b3.push_back(icode::make('G', 91));
      correct.push_back(b1);
      correct.push_back(b2);
      correct.push_back(b3);
      REQUIRE(res == correct);
    }
  }
}
