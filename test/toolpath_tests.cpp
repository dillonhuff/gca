#include "catch.hpp"
#include "core/arena_allocator.h"
#include "core/context.h"
#include "synthesis/toolpath.h"

namespace gca {

  unsigned num_cuts(const vector<cut_group>& cut_groups) {
    unsigned nc = 0;
    for (unsigned i = 0; i < cut_groups.size(); i++) {
      nc += cut_groups[i].size();
    }
    return nc;
  }


  TEST_CASE("Several non-adjacent lines") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<cut*> cuts;
    cuts.push_back(mk_linear_cut(point(0, 0, 0), point(1, 0, 0)));
    cuts.push_back(mk_linear_cut(point(1, 0, 0), point(1, 1, 0)));
    
    vector<cut_group> cgs;

    SECTION("No cuts lost in cut group creation") {
      vector<cut_group> cgs;
      group_adjacent_cuts(cuts, cgs, 30.0);
      unsigned nc = num_cuts(cgs);
      unsigned correct = cuts.size();
      REQUIRE(nc == correct);
    }

    SECTION("Each cut is its own cut group") {
      vector<cut_group> cgs;
      group_adjacent_cuts(cuts, cgs, 30.0);
      unsigned correct = cuts.size();
      REQUIRE(cgs.size() == correct);
    }
  }
}
