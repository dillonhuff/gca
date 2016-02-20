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


  TEST_CASE("Cut groups for lines") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<cut*> cuts;
    vector<cut_group> cgs;

    SECTION("One cut") {
      cuts.push_back(mk_linear_cut(point(0, 0, 0), point(1, 0, 0)));
      group_adjacent_cuts(cuts, cgs, 30.0);
      REQUIRE(cgs.size() == 1);
    }
    
    SECTION("Several non-continuous lines") {
      cuts.push_back(mk_linear_cut(point(0, 0, 0), point(1, 0, 0)));
      cuts.push_back(mk_linear_cut(point(1, 0, 0), point(1, 1, 0)));
      cuts.push_back(mk_linear_cut(point(2, 0, 0), point(3, 2, 0)));

      SECTION("No cuts lost in cut group creation") {
	group_adjacent_cuts(cuts, cgs, 30.0);
	unsigned nc = num_cuts(cgs);
	unsigned correct = cuts.size();
	REQUIRE(nc == correct);
      }

      SECTION("Each cut is its own cut group") {
	group_adjacent_cuts(cuts, cgs, 30.0);
	unsigned correct = cuts.size();
	REQUIRE(cgs.size() == correct);
      }
    }

    SECTION("One cut group with multiple cuts") {
      cuts.push_back(mk_linear_cut(point(0, 0, 0), point(1, 0, 0)));
      cuts.push_back(mk_linear_cut(point(1, 0, 0), point(2, 0, 0)));

      group_adjacent_cuts(cuts, cgs, 30.0);
      REQUIRE(cgs.size() == 1);
    }

    SECTION("2 distinct adjacent groups") {
      cuts.push_back(mk_linear_cut(point(0, 0, 0), point(1, 0, 0)));
      cuts.push_back(mk_linear_cut(point(1, 0, 0), point(2, 0.05, 0)));
      cuts.push_back(mk_linear_cut(point(2, 0, 0), point(3, 2, 0)));
      
      group_adjacent_cuts(cuts, cgs, 30.0);

      SECTION("No cuts lost in cut group creation") {
	unsigned nc = num_cuts(cgs);
	unsigned correct = cuts.size();
	REQUIRE(nc == correct);
      }

      SECTION("Split into 2 groups") {
	REQUIRE(cgs.size() == 2);
      }
    }
  }
}
