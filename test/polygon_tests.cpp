#include "catch.hpp"
#include "geometry/polygon.h"
#include "geometry/triangle.h"
#include "system/algorithm.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Triangles to oriented polygons") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Empty triangle vector") {
      vector<triangle> ts;
      auto res = merge_triangles(ts);
      REQUIRE(res.size() == 0);
    }

    SECTION("One square") {
      point n(0, 0, 1);
      point bl(0, 0, 0);
      point ul(0, 1, 0);
      point br(1, 0, 0);
      point ur(1, 1, 0);

      vector<triangle> ts{triangle(n, bl, ul, ur), triangle(n, bl, br, ur)};
      auto merged = merge_triangles(ts);
      REQUIRE(merged.size() == 1);
    }

    SECTION("Real stl file") {
      auto triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ktoolcor.stl").triangles;
      delete_if(triangles, [](const triangle& t) { return !is_upward_facing(t, 1e-2); });
      greedy_adjacent_chains(triangles.begin(), triangles.end(),
			     [](const triangle& x,
				const triangle& y)
			     { return same_orientation(x, y, 0.01); });
      vector<vector<triangle>> constant_orientation_groups;
      split_by(triangles, constant_orientation_groups,
	       [](const triangle& x,
		  const triangle& y)
	       { return same_orientation(x, y, 0.01); });
      vector<oriented_polygon> polys;
      for (auto g : constant_orientation_groups) {
	assert(g.size() > 0);
	auto merged_polys = merge_triangles(g);
	polys.insert(polys.end(), merged_polys.begin(), merged_polys.end());
      }
      REQUIRE(polys.size() > constant_orientation_groups.size());
    }
  }
}
