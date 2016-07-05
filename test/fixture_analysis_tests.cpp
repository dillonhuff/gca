#include "catch.hpp"
#include "synthesis/fixture_analysis.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Clipped pill") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = emco_vice(point(1.3, -4.4, 3.3));
    fixtures fixes(test_vice);
    workpiece workpiece_dims(3.81, 3.2, 3.98, ACETAL);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ClippedPill.stl", 0.001);

    auto outer_surfs = outer_surfaces(mesh);
    auto aligned_workpiece = align_workpiece(outer_surfs, workpiece_dims);
    classify_part_surfaces(outer_surfs, aligned_workpiece);
    auto surfs_to_cut = surfaces_to_cut(mesh, outer_surfs);

    SECTION("6 surfaces that are always stable") {
      unsigned num_stable = 0;
      for (auto s : outer_surfs) {
	if (s.is_SA()) {
	  num_stable++;
	}
      }
      REQUIRE(num_stable == 6);
    }


    SECTION("4 surfaces to cut") {
      REQUIRE(surfs_to_cut.size() == 4);
    }

    SECTION("2 setups") {
      vector<fixture> all_orients =
	all_stable_fixtures(outer_surfs, fixes);

      surface_map orients =
	pick_orientations(mesh, surfs_to_cut, all_orients);

      REQUIRE(orients.size() == 2);

      // No use of base plates
      for (auto fixture : orients) {
	REQUIRE(!(all_orients[fixture.first].v.has_protective_base_plate()));
      }
    }
  }

  TEST_CASE("Tapered top and several slanted verticals") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = emco_vice(point(1.2, -4.4, 3.3));
    fixtures fixes(test_vice);
    workpiece workpiece_dims(4.0, 4.0, 3.98, ALUMINUM);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/TaperedTopSeveralVerticals.stl", 0.001);

    auto outer_surfs = outer_surfaces(mesh);
    auto aligned_workpiece = align_workpiece(outer_surfs, workpiece_dims);
    classify_part_surfaces(outer_surfs, aligned_workpiece);
    auto surfs_to_cut = surfaces_to_cut(mesh, outer_surfs);

    SECTION("14 outer surfaces") {
      REQUIRE(outer_surfs.size() == 14);
    }

    SECTION("8 surfaces to cut") {
      REQUIRE(surfs_to_cut.size() == 8);
    }

    SECTION("3 setups") {
      vector<fixture> all_orients =
	all_stable_fixtures(outer_surfs, fixes);

      surface_map orients =
	pick_orientations(mesh, surfs_to_cut, all_orients);

      REQUIRE(orients.size() == 3);
    }
  }
  
  TEST_CASE("Tapered extrude top and side") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = emco_vice(point(-0.8, -4.4, -3.3));
    fixtures fixes(test_vice);
    workpiece workpiece_dims(3.5, 2.5, 2.3, BRASS);
    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/TaperedExtrudedTopSide.stl", 0.001);

    auto outer_surfs = outer_surfaces(mesh);
    auto aligned_workpiece = align_workpiece(outer_surfs, workpiece_dims);
    classify_part_surfaces(outer_surfs, aligned_workpiece);
    auto surfs_to_cut = surfaces_to_cut(mesh, outer_surfs);

    SECTION("6 outer surfaces") {
      REQUIRE(outer_surfs.size() == 6);
    }

    SECTION("2 setups") {
      vector<fixture> all_orients =
	all_stable_fixtures(outer_surfs, fixes);

      surface_map orients =
	pick_orientations(mesh, surfs_to_cut, all_orients);

      REQUIRE(orients.size() == 2);

      // No use of base plates
      for (auto fixture : orients) {
	REQUIRE(!(all_orients[fixture.first].v.has_protective_base_plate()));
      }
    }

    SECTION("each orientation has 1 connected component") {
      vector<fixture> all_orients =
	all_stable_fixtures(outer_surfs, fixes);

      surface_map orients =
	pick_orientations(mesh, surfs_to_cut, all_orients);
      
      for (auto p : orients) {
	auto surface_inds = p.second;
	auto connected_comps =
	  connected_components_by(surface_inds,
				  [surfs_to_cut](const unsigned i, const unsigned j)
				  {
				    return surfaces_share_edge(i, j, surfs_to_cut);
				  });
	REQUIRE(connected_comps.size() == 1);
      }
    }
  }

  TEST_CASE("Shape with outermost surfaces that are not part of any stable orientation") {
    arena_allocator a;
    set_system_allocator(&a);
    vice test_vice = emco_vice(point(1.8, 4.2, 3.3));
    fixtures fixes(test_vice);
    workpiece workpiece_dims(2.0, 2.0, 2.0, ACETAL);
    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/LittleHouse.stl", 0.001);

    auto outer_surfs = outer_surfaces(mesh);
    auto aligned_workpiece = align_workpiece(outer_surfs, workpiece_dims);
    classify_part_surfaces(outer_surfs, aligned_workpiece);
    auto surfs_to_cut = surfaces_to_cut(mesh, outer_surfs);

    SECTION("4 surfaces to cut") {
      REQUIRE(surfs_to_cut.size() == 4);
    }
    
    SECTION("10 outer surfaces") {
      REQUIRE(outer_surfs.size() == 10);
    }

    SECTION("6 surfaces that are always stable") {
      unsigned num_stable = 0;
      for (auto s : outer_surfs) {
	if (s.is_SA()) {
	  num_stable++;
	}
      }
      REQUIRE(num_stable == 6);
    }

    SECTION("1 setup") {
      vector<fixture> all_orients =
	all_stable_fixtures(outer_surfs, fixes);

      surface_map orients =
	pick_orientations(mesh, surfs_to_cut, all_orients);

      REQUIRE(orients.size() == 1);

      // No use of base plates
      for (auto fixture : orients) {
	REQUIRE(!(all_orients[fixture.first].v.has_protective_base_plate()));
      }
    }

    SECTION("each orientation has 1 connected component") {
      vector<fixture> all_orients =
	all_stable_fixtures(outer_surfs, fixes);

      surface_map orients =
	pick_orientations(mesh, surfs_to_cut, all_orients);
      
      for (auto p : orients) {
	auto surface_inds = p.second;
	auto connected_comps =
	  connected_components_by(surface_inds,
				  [surfs_to_cut](const unsigned i, const unsigned j)
				  {
				    return surfaces_share_edge(i, j, surfs_to_cut);
				  });
	REQUIRE(connected_comps.size() == 1);
      }
    }
  }
  
  TEST_CASE("Tapered extrude top") {
    arena_allocator a;
    set_system_allocator(&a);
    vice test_vice = emco_vice(point(-0.8, -4.4, -3.3));
    fixtures fixes(test_vice);
    workpiece workpiece_dims(3.5, 2.5, 2.3, ACETAL);
    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/TaperedExtrudeTop.stl", 0.001);

    auto outer_surfs = outer_surfaces(mesh);
    auto aligned_workpiece = align_workpiece(outer_surfs, workpiece_dims);
    classify_part_surfaces(outer_surfs, aligned_workpiece);

    SECTION("6 outer surfaces") {
      REQUIRE(outer_surfs.size() == 6);
    }
    
    SECTION("1 setup, no duplicates, each orientation has 1 connected component") {

      auto surfs_to_cut = surfaces_to_cut(mesh, outer_surfs);

      // All faces that need to be cut are at least somewhat upward facing
      REQUIRE(all_of(begin(surfs_to_cut), end(surfs_to_cut),
		     [](const surface& s) {
		       return s.face_orientation(s.index_list().front()).z > 0.2; }));
      
      vector<fixture> all_orients =
	all_stable_fixtures(outer_surfs, fixes);
      
      surface_map orients =
	pick_orientations(mesh, surfs_to_cut, all_orients);

      // TEST: 1 setup
      REQUIRE(orients.size() == 1);

      // No use of base plates
      for (auto fixture : orients) {
	REQUIRE(!(all_orients[fixture.first].v.has_protective_base_plate()));
      }


      // NO DUPLICATION
      for (unsigned i = 0; i < orients.size(); i++) {
	for (unsigned j = i + 1; j < orients.size(); j++) {
	  if (orients.find(i) != end(orients) &&
	      orients.find(j) != end(orients) &&
	      i != j) {
	    vector<unsigned> ig = orients[i];
	    vector<unsigned> jg = orients[j];
	    REQUIRE(intersection(ig, jg).size() == 0);
	  }
	}
      }
    }
  }

  TEST_CASE("Complex rectangular part") {
    arena_allocator a;
    set_system_allocator(&a);
    vice test_vice = emco_vice(point(-0.8, -4.4, -3.3));
    fixtures fixes(test_vice);
    workpiece workpiece_dims(3.5, 2.5, 2.3, ALUMINUM);
    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ComplexRectanglePart1.stl", 0.001);
    
    SECTION("4 setups, no duplicates, each orientation has 1 connected component") {
      auto outer_surfs = outer_surfaces(mesh);
      auto aligned_workpiece = align_workpiece(outer_surfs, workpiece_dims);
      classify_part_surfaces(outer_surfs, aligned_workpiece);

      auto surfs_to_cut = surfaces_to_cut(mesh, outer_surfs);
      vector<fixture> all_orients =
	all_stable_fixtures(outer_surfs, fixes);
      
      surface_map orients =
	pick_orientations(mesh, surfs_to_cut, all_orients);

      // TEST: 4 setups
      REQUIRE(orients.size() == 4);

      // No use of base plates
      for (auto fixture : orients) {
	REQUIRE(!(all_orients[fixture.first].v.has_protective_base_plate()));
      }


      // NO DUPLICATION
      for (unsigned i = 0; i < orients.size(); i++) {
	for (unsigned j = i + 1; j < orients.size(); j++) {
	  if (orients.find(i) != end(orients) &&
	      orients.find(j) != end(orients) &&
	      i != j) {
	    vector<unsigned> ig = orients[i];
	    vector<unsigned> jg = orients[j];
	    REQUIRE(intersection(ig, jg).size() == 0);
	  }
	}
      }
      
      // TEST: Each has 1 connected component
      for (auto p : orients) {
	auto surface_inds = p.second;
	auto connected_comps =
	  connected_components_by(surface_inds,
				  [surfs_to_cut](const unsigned i, const unsigned j)
				  {
				    return surfaces_share_edge(i, j, surfs_to_cut);
				  });
	REQUIRE(connected_comps.size() == 1);
      }
    }
  }

  TEST_CASE("Box with thru hole") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = emco_vice(point(1.3, -4.4, 3.3));
    std::vector<plate_height> plates{0.1, 0.3};
    fixtures fixes(test_vice, plates);
    workpiece workpiece_dims(3.81, 3.2, 3.98, ALUMINUM);
    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    tool t2(0.5, 3.0, 4, HSS, FLAT_NOSE);
    vector<tool> tools{t1, t2};

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithThruHole.stl", 0.001);

    fixture_plan p = make_fixture_plan(mesh, fixes, tools, workpiece_dims);

    // TODO: Eventually with optimizations this will be 6
    REQUIRE(p.fixtures().size() == 7);
  }

}
