#include "analysis/extract_cuts.h"
#include "catch.hpp"
#include "core/context.h"
#include "core/parser.h"

namespace gca {
  
  TEST_CASE("G1 segmentation") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("No G1 segments") {
      gprog* p = parse_gprog("G90 M5");
      vector<cut_section> sections;
      extract_cuts(p, sections);
      REQUIRE(sections.size() == 0);
    }

    SECTION("One G1 segments with 1 instruction") {
      gprog* p = parse_gprog("G90 G0 X1.0 Y0.0 Z0.0 G1 X2.0 Y3.0 Z0.0 M5");
      vector<cut_section> sections;
      extract_cuts(p, sections);
      REQUIRE(sections.size() == 1);
    }
    
  }

}
