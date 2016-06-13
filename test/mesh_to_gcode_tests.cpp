#include "analysis/gcode_to_cuts.h"
#include "catch.hpp"
#include "geometry/triangular_mesh.h"
#include "synthesis/mesh_to_gcode.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Mesh to gcode") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = emco_vice(point(-0.8, -4.4, -3.3));
    tool t1(0.25, 3.0, FLAT_NOSE);
    vector<tool> tools{t1};
    workpiece workpiece_dims(1.5, 1.2, 1.5);

    SECTION("Simple box") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Cube0p5.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, test_vice, tools, workpiece_dims);

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
      auto result_programs = mesh_to_gcode(mesh, test_vice, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 7);
    }

    SECTION("Box with two holes has 6 clippings and two pocketings") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWith2Holes.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, test_vice, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 8);
    }

    SECTION("Box with protrusion") {
      workpiece workpiece_dims(1.5, 1.2, 2.0);
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithProtrusion.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, test_vice, tools, workpiece_dims);
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
	workpiece workpiece_dims(1.5, 1.2, 1.5);
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
  
  TEST_CASE("Toolpath bounds") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = emco_vice(point(-1.8, -0.4, 3.3));
    tool t1(0.35, 3.0, FLAT_NOSE);
    vector<tool> tools{t1};
    workpiece workpiece_dims(1.7, 2.1, 1.65);

    SECTION("Box with 2 holes") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWith2Holes.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, test_vice, tools, workpiece_dims);

      SECTION("Never cut below vice") {
    	for (auto program : result_programs) {
    	  REQUIRE(all_z_coords_above(program.blocks, test_vice.base_z()));
    	}
      }

      SECTION("G0 moves are above the workpiece") {
	workpiece workpiece_dims(2.0, 2.0, 2.0);
    	double workpiece_height = test_vice.base_z() + workpiece_dims.sides[2].len();
    	for (auto program : result_programs) {
    	  REQUIRE(all_safe_moves_above(program.blocks, workpiece_height));
    	}
      }
    }
  }
}
