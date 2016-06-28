#include "analysis/gcode_to_cuts.h"
#include "catch.hpp"
#include "geometry/triangular_mesh.h"
#include "synthesis/axis_3.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Mesh to gcode") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = emco_vice(point(-0.8, -4.4, -3.3));
    std::vector<plate_height> plates{0.1, 0.3};
    fixtures fixes(test_vice, plates);

    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    vector<tool> tools{t1};
    workpiece workpiece_dims(1.5, 1.2, 1.5, ACETAL);

    SECTION("Simple box") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Cube0p5.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);

      SECTION("Produces only workpiece clipping programs") {
	REQUIRE(result_programs.size() == 6);
      }

      SECTION("Workpiece clipping programs actually contain code") {
	for (auto r : result_programs) {
	  REQUIRE(r.blocks.size() > 0);
	}
      }
    }

    SECTION("Box with hole has 6 clippings and one pocketing") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithTopHole.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 7);
    }

    SECTION("Box with two holes has 6 clippings and two pocketings") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWith2Holes.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 8);
    }

    SECTION("Box with protrusion") {
      workpiece workpiece_dims(1.5, 1.2, 2.0, ALUMINUM);
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithProtrusion.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 7);
    }

    SECTION("Box with thru hole") {
      workpiece workpiece_dims(1.51, 1.51, 2.0, ACETAL);
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithThruHole.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 7);
    }
    
    // TODO: Reintroduce this test
    // SECTION("Mesh box plinth has 6 clippings and one pocketing") {
    //   auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/MeshBoxPlinth.stl").triangles;
    //   auto mesh = make_mesh(box_triangles, 0.001);
    //   auto result_programs = mesh_to_gcode(mesh, test_vice, tools, workpiece_dims);
    //   REQUIRE(result_programs.size() == 7);
    // }

  }

    TEST_CASE("Mesh to gcode with parallel plates") {
      arena_allocator a;
      set_system_allocator(&a);

      vice test_vice = emco_vice(point(-0.8, -4.4, -3.3));
      std::vector<plate_height> base_plates{};
      std::vector<plate_height> parallel_plates{0.5};
      fixtures fixes(test_vice, base_plates, parallel_plates);

      tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
      vector<tool> tools{t1};
      workpiece workpiece_dims(4.0, 3.0, 4.0, ACETAL);

      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ComplexRectanglePart1.stl", 0.001);

      auto programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);
      REQUIRE(programs.size() == 6);

    }

  TEST_CASE("Complex part pocket ordering") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = emco_vice(point(1.3, -4.4, 3.3));
    fixtures fixes(test_vice);
    workpiece workpiece_dims(3.81, 3.2, 3.98, BRASS);
    tool t1(0.25, 3.0, 4, CARBIDE, FLAT_NOSE);
    vector<tool> tools{t1};

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ComplexRectanglePart1.stl", 0.001);

    auto outer_surfs = outer_surfaces(mesh);
    auto aligned_workpiece = align_workpiece(outer_surfs, workpiece_dims);
    classify_part_surfaces(outer_surfs, aligned_workpiece);
    auto surfs_to_cut = surfaces_to_cut(mesh, outer_surfs);

    fixture_plan plan =
      make_fixture_plan(mesh, outer_surfs, fixes, tools, workpiece_dims);

    for (auto orient_surfaces_pair : plan.fixtures()) {
      auto m = oriented_part_mesh(orient_surfaces_pair.first.orient,
				  orient_surfaces_pair.second,
				  orient_surfaces_pair.first.v);
      double workpiece_height = aligned_workpiece.sides[2].len();
      vector<pocket> pockets =
	make_surface_pockets(m.second, m.first, workpiece_height);
      double last_depth = max_distance_along(m.first.vertex_list(), point(0, 0, 1));
      for (auto p : pockets) {
	REQUIRE(p.get_end_depth() <= last_depth);
	last_depth = p.get_end_depth();
      }
    }
  }

  TEST_CASE("Outer surfaces") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Simple box") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Cube0p5.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      auto surfaces = outer_surfaces(mesh);

      

      SECTION("Simple box has 6 outer surfaces") {
	REQUIRE(surfaces.size() == 6);
      }

      SECTION("All simple box surfaces are part of a SA faces") {
	vector<index_t> fis = mesh.face_indexes();
	workpiece workpiece_dims(1.5, 1.2, 1.5, ALUMINUM);
	auto workpiece_mesh = align_workpiece(surfaces, workpiece_dims);
	classify_part_surfaces(surfaces, workpiece_mesh);
	remove_SA_surfaces(surfaces, fis);
	REQUIRE(fis.size() == 0);
      }
    }

    SECTION("Box with 1 hole") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithTopHole.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      auto surfaces = outer_surfaces(mesh);
      SECTION("A box with 1 hole has 6 outer surfaces") {
	REQUIRE(surfaces.size() == 6);
      }
    }

    SECTION("Box with 2 holes") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWith2Holes.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      auto surfaces = outer_surfaces(mesh);

      SECTION("A box with 2 holes has 6 outer surfaces") {
	REQUIRE(surfaces.size() == 6);
      }
    }
  }

  bool all_z_coords_above(const std::vector<block>& blocks, double z) {
    vector<vector<cut*>> cuts;
    auto r = gcode_to_cuts(blocks, cuts);
    assert(r == GCODE_TO_CUTS_SUCCESS);
    for (auto cb : cuts) {
      for (auto c : cb) {
	if (c->get_start().z <= z || c->get_end().z <= z) {
	  cout << "Cut above " << z << " is " << endl << *c << endl;
	  return false;
	}
      }
    }
    return true;
  }


  bool all_safe_moves_above(const std::vector<block>& blocks, double z) {
    vector<vector<cut*>> cuts;
    auto r = gcode_to_cuts(blocks, cuts);
    assert(r == GCODE_TO_CUTS_SUCCESS);
    for (auto cb : cuts) {
      for (auto c : cb) {
	if (c->is_safe_move() && (c->get_end().z <= z)) {
	  cout << "Cut above " << z << " is " << endl << *c << endl;
	  return false;
	}
      }
    }
    return true;
  }

  bool no_gouging_within(const std::vector<block>& blocks,
			 const triangular_mesh& m,
			 const double tolerance) {
    vector<vector<cut*>> cuts;
    auto r = gcode_to_cuts(blocks, cuts);
    assert(r == GCODE_TO_CUTS_SUCCESS);
    for (auto cb : cuts) {
      for (auto c : cb) {
	for (double t = 0.0; t < 1.0; t += 0.2) {
	  point v = c->value_at(t);
	  // Reverse the y negation in emco_f1_code
	  maybe<double> mv = m.z_at(v.x, -v.y);
	  double tool_head_z = v.z - 3.15;
	  double mesh_z = mv.t - tolerance;
	  if (mv.just && tool_head_z < mesh_z) {
	    cout << "Cut is \n" << *c << endl;
	    cout << "Mesh z = " << mv.t << endl;
	    cout << "Tool head position = " << v << endl;
	    cout << "tool head z - mesh z = " << tool_head_z - mesh_z << endl;
	    return false;
	  }
	}
      }
    }
    return true;
  }
  
  TEST_CASE("Toolpath bounds") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = emco_vice(point(-1.8, -0.4, 3.3));
    std::vector<plate_height> plates{0.15, 0.03};
    fixtures fixes(test_vice, plates);

    tool t1(0.35, 3.0, 4, HSS, FLAT_NOSE);
    tool t2(0.14, 3.15, 2, HSS, FLAT_NOSE);
    vector<tool> tools{t1, t2};
    workpiece workpiece_dims(1.7, 2.1, 1.65, ACETAL);

    SECTION("Box with 2 holes") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWith2Holes.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);

      SECTION("Never cut below vice") {
    	for (auto program : result_programs) {
    	  REQUIRE(all_z_coords_above(program.blocks, test_vice.base_z()));
    	}
      }
    }

    SECTION("Complex rectangular part 1") {
      vice test_vice = current_setup();
      std::vector<plate_height> plates{0.1, 0.3};
      fixtures fixes(test_vice, plates);

      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ComplexRectanglePart1.stl", 0.001);
      auto part_ss = outer_surfaces(mesh);
      auto aligned_workpiece = align_workpiece(part_ss, workpiece_dims);
      classify_part_surfaces(part_ss, aligned_workpiece);
      vector<pair<triangular_mesh, surface_list>> meshes =
	part_arrangements(mesh, part_ss, test_vice);
      vector<gcode_program> programs;
      cut_secured_meshes(meshes, programs, tools);

      SECTION("One program per mesh") {
	REQUIRE(meshes.size() == programs.size());
      }
      
      SECTION("Toolpaths don't gouge the mesh") {
	for (unsigned i = 0; i < meshes.size(); i++) {
	  auto program = programs[i];
	  auto mesh = meshes[i].first;
	  REQUIRE(no_gouging_within(program.blocks, mesh, 0.05));
	}
      }
    }
  }
}
