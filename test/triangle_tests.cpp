#include "catch.hpp"
#include "geometry/polygon.h"
#include "geometry/triangle.h"
#include "system/algorithm.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Merge triangles, CylinderChimneySlot") {
    vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderChimneySlot.stl").triangles;
    delete_if(triangles,
	      [](const triangle t)
	      { return !is_upward_facing(t, 0.05); });
    auto polygons = mesh_bounds(triangles);
    stable_sort(begin(polygons), end(polygons),
		[](const oriented_polygon& x,
		   const oriented_polygon& y)
		{ return x.height() < y.height(); });

    SECTION("Top polygon has height 0.35") {
      double top_polygon_height = polygons.back().height();
      REQUIRE(within_eps(top_polygon_height, 0.35, 0.00001));
    }
  }

  TEST_CASE("Identify millable surfaces") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Box") {
      auto triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Box1x1x1.stl").triangles;
      auto faces = millable_surfaces(triangles);
      REQUIRE(faces.size() == 1);
    }

    SECTION("Multiple cylinders, 2 levels") {
      auto triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/MultipleCylinders.stl").triangles;
      auto faces = millable_surfaces(triangles);
      REQUIRE(faces.size() == 2);
    }
    
    SECTION("Multiple cylinders, 3 levels") {
      auto triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/MultipleCylinders2.stl").triangles;
      auto faces = millable_surfaces(triangles);
      REQUIRE(faces.size() == 3);
    }
  }

  TEST_CASE("Segment intersection with triangles") {
    auto triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/SlicedCone.stl").triangles;
    // cout << "# triangles: " << triangles.size() << endl;
    // for (auto t : triangles) {
    //   cout << t << endl;
    // }

    SECTION("Line at 10.0") {
      point s(0.332573, 0.612317, 10.0);
      point e(0.357573, 0.112317, 10.0);
      line l(s, e);
      REQUIRE(!intersects_triangles(l, triangles));
    }

    SECTION("Line at 0.1") {
      point s(0.332573, 0.612317, 0.1);
      point e(0.357573, 0.112317, 0.1);
      line l(s, e);
      REQUIRE(intersects_triangles(l, triangles));
    }
  }
  
}
