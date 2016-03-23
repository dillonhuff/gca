#include "catch.hpp"
#include "core/lexer.h"
#include "transformers/feed_changer.h"

namespace gca {

  TEST_CASE("Transformer tests") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<block> p;
    vector<block> correct;
    value* initial_feedrate;
    value* new_feedrate;

    SECTION("Feed changer with G0") {
      p = lex_gprog("G91 G0 X1.5 \n G1 F2 X2.0 Y3.0 Z5.5");
      initial_feedrate = lit::make(2.0);
      new_feedrate = lit::make(5.0);
      correct = lex_gprog("G91 G0 X1.5 \n G1 F5 X2.0 Y3.0 Z5.5");
      REQUIRE(change_feeds(p, initial_feedrate, new_feedrate) == correct);
    }

    SECTION("Feed changer") {
      p = lex_gprog("G1 X1.0 Y1.0 Z1.0 F1.0");
      initial_feedrate = lit::make(1.0);
      new_feedrate = lit::make(4.0);
      correct = lex_gprog("G1 X1.0 Y1.0 Z1.0 F4.0");
      REQUIRE(change_feeds(p, initial_feedrate, new_feedrate) == correct);
    }
  }
}
