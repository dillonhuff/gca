#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "synthesis/fixture_analysis.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("More parallel plates") {
    arena_allocator a;
    set_system_allocator(&a);

    // Change back to emco_vice
    vice test_vice = large_jaw_vice(5, point(-0.8, -4.4, -3.3));
    std::vector<plate_height> parallel_plates{0.5, 0.7};
    fixtures fixes(test_vice, parallel_plates);

    tool t1(0.1, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.1);
    t1.set_cut_length(0.4);

    t1.set_shank_diameter(3.0 / 8.0);
    t1.set_shank_length(0.1);

    t1.set_holder_diameter(2.0);
    t1.set_holder_length(2.5);
    
    vector<tool> tools{t1};

    workpiece workpiece_dims(3.0, 1.9, 3.0, ACETAL);

    SECTION("onshape part 1 (1)") {
      tool t3{0.2334, 3.94, 4, HSS, FLAT_NOSE};
      t3.set_cut_diameter(0.2334);
      t3.set_cut_length(1.2);

      t3.set_shank_diameter(0.5);
      t3.set_shank_length(0.05);

      t3.set_holder_diameter(2.5);
      t3.set_holder_length(3.5);

      vector<tool> tools{t1, t3};
      
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/Part Studio 1 - Part 1(1).stl", 0.0001);

      fixture_plan p = make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

      REQUIRE(p.fixtures().size() == 2);

      for (auto p : p.fixtures()[1].pockets) {
	cout << p.pocket_type() << endl;
      }

      REQUIRE(p.fixtures()[1].pockets.size() == 1);
      REQUIRE(p.fixtures()[0].pockets.size() == 3);
    }
  }

}
