#include "catch.hpp"
#include "synthesis/fixture_analysis.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Parallel plates") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = emco_vice(point(-0.8, -4.4, -3.3));
    std::vector<plate_height> parallel_plates{0.5, 0.7};
    fixtures fixes(test_vice, parallel_plates);

    tool t1(0.1, 3.0, 4, HSS, FLAT_NOSE);
    vector<tool> tools{t1};
    workpiece workpiece_dims(3.0, 1.9, 3.0, ACETAL);

    SECTION("Clipped pill") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ClippedPill.stl", 0.001);

      // fixture_plan p = make_fixture_plan(mesh, fixes, tools, workpiece_dims);

      // REQUIRE(p.fixtures().size() == 2);
    }

    SECTION("Round with thru holes") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/RoundEdges2Holes.stl", 0.001);

      cout << "Making round w/ thru holes plan" << endl;

      fixture_plan p = make_fixture_plan(mesh, fixes, tools, workpiece_dims);

      REQUIRE(p.fixtures().size() == 2);
    }

    SECTION("Cylinder") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ShortCylinder.stl", 0.001);

      fixture_plan p = make_fixture_plan(mesh, fixes, tools, workpiece_dims);

      REQUIRE(p.fixtures().size() == 3);
    }

    // TODO: Reintroduce this test
    SECTION("Clipped Cylinder") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ClippedCylinder.stl", 0.001);

      vice test_vice = large_jaw_vice(4.5, point(-0.8, -4.4, -3.3));
      std::vector<plate_height> parallel_plates{0.5, 0.7};
      fixtures fixes(test_vice, parallel_plates);

      fixture_plan p = make_fixture_plan(mesh, fixes, tools, workpiece_dims);

      REQUIRE(p.fixtures().size() == 3);
    }
    
  }

  TEST_CASE("Tapered top and several slanted verticals") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = large_jaw_vice(4.0, point(1.2, -4.4, 3.3));
    fixtures fixes(test_vice);
    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    vector<tool> tools{t1};
    workpiece workpiece_dims(3.5, 3.0, 3.98, ALUMINUM);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/TaperedTopSeveralVerticals.stl", 0.001);

    auto outer_surfs = outer_surfaces(mesh);

    SECTION("14 outer surfaces") {
      REQUIRE(outer_surfs.size() == 14);
    }

    SECTION("9 setups") {
      fixture_plan p = make_fixture_plan(mesh, fixes, tools, workpiece_dims);
      REQUIRE(p.fixtures().size() == 9);
    }
  }

  TEST_CASE("Tapered extrude top and side") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = large_jaw_vice(3.5, point(-0.8, -4.4, -3.3));
    fixtures fixes(test_vice);
    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    vector<tool> tools{t1};
    workpiece workpiece_dims(2.5, 1.8, 2.3, BRASS);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/TaperedExtrudedTopSide.stl", 0.001);

    auto outer_surfs = outer_surfaces(mesh);

    SECTION("6 outer surfaces") {
      REQUIRE(outer_surfs.size() == 6);
    }

    SECTION("8 setups") {
      fixture_plan p = make_fixture_plan(mesh, fixes, tools, workpiece_dims);

      REQUIRE(p.fixtures().size() == 8);

      // No use of base plates
      for (auto fixture : p.fixtures()) {
  	REQUIRE(!(fixture.fix.v.has_parallel_plate()));
      }
    }
  }

  TEST_CASE("Shape with outermost surfaces that are not part of any stable orientation") {
    arena_allocator a;
    set_system_allocator(&a);
    vice test_vice = large_jaw_vice(4.0, point(1.8, 4.2, 3.3));
    fixtures fixes(test_vice);
    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    vector<tool> tools{t1};
    workpiece workpiece_dims(3.0, 3.0, 3.0, ACETAL);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/LittleHouse.stl", 0.001);

    auto outer_surfs = outer_surfaces(mesh);
    
    SECTION("10 outer surfaces") {
      REQUIRE(outer_surfs.size() == 10);
    }

    SECTION("7 setups") {
      fixture_plan p = make_fixture_plan(mesh, fixes, tools, workpiece_dims);

      REQUIRE(p.fixtures().size() == 7);

      // No use of base plates
      for (auto fixture : p.fixtures()) {
	REQUIRE(!(fixture.fix.v.has_parallel_plate()));
      }
    }
  }

  TEST_CASE("Tapered extrude top") {
    arena_allocator a;
    set_system_allocator(&a);
    vice test_vice = large_jaw_vice(3.0, point(-0.8, -4.4, -3.3));
    fixtures fixes(test_vice);
    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    tool t2(0.5, 3.0, 4, HSS, FLAT_NOSE);
    vector<tool> tools{t1, t2};
    workpiece workpiece_dims(2.5, 1.9, 2.3, ACETAL);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/TaperedExtrudeTop.stl", 0.001);

    auto outer_surfs = outer_surfaces(mesh);

    SECTION("6 outer surfaces") {
      REQUIRE(outer_surfs.size() == 6);
    }
    
    SECTION("7 setups, no duplicates") {
      fixture_plan p = make_fixture_plan(mesh, fixes, tools, workpiece_dims);

      REQUIRE(p.fixtures().size() == 7);

      // No use of base plates
      for (auto fixture : p.fixtures()) {
  	REQUIRE(!(fixture.fix.v.has_parallel_plate()));
      }
    }
  }

  TEST_CASE("Complex rectangular part") {
    arena_allocator a;
    set_system_allocator(&a);
    vice test_vice = large_jaw_vice(3.0, point(-0.8, -4.4, -3.3));
    fixtures fixes(test_vice);
    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    tool t2(0.5, 3.0, 4, HSS, FLAT_NOSE);
    vector<tool> tools{t1, t2};
    workpiece workpiece_dims(2.5, 1.9, 2.3, ALUMINUM);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ComplexRectanglePart1.stl", 0.001);
    
    SECTION("10 setups") {
      fixture_plan p = make_fixture_plan(mesh, fixes, tools, workpiece_dims);

      REQUIRE(p.fixtures().size() == 10);

      // No use of base plates
      for (auto fixture : p.fixtures()) {
	REQUIRE(!(fixture.fix.v.has_parallel_plate()));
      }
    }
  }

  TEST_CASE("Box with thru hole") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = emco_vice(point(1.3, -4.4, 3.3));
    std::vector<plate_height> plates{0.1, 0.3};
    fixtures fixes(test_vice, plates);
    workpiece workpiece_dims(2.0, 2.0, 3.98, ALUMINUM);
    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    tool t2(0.5, 3.0, 4, HSS, FLAT_NOSE);
    vector<tool> tools{t1, t2};

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithThruHole.stl", 0.001);

    fixture_plan p = make_fixture_plan(mesh, fixes, tools, workpiece_dims);

    REQUIRE(p.fixtures().size() == 2);
  }

}
