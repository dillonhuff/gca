#include "catch.hpp"
#include "geometry/triangular_mesh.h"
#include "synthesis/mesh_to_gcode.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Mesh to gcode") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice(1.5, 1.5, 0.75, Y_AXIS);
    tool t1(0.3, FLAT_NOSE);
    vector<tool> tools{t1};
    workpiece_dimensions workpiece_dims(1.5, 1.2, 1.5);
    
    SECTION("Simple box produces only workpiece tightening programs") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Cube0p5.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      auto result_programs = mesh_to_gcode(mesh, test_vice, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 6);
    }

    SECTION("Box with hole has 6 clippings and one pocketing") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithTopHole.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      auto result_programs = mesh_to_gcode(mesh, test_vice, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 7);
    }

    SECTION("Box with two holes has 6 clippings and two pocketings") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWith2Holes.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      auto result_programs = mesh_to_gcode(mesh, test_vice, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 8);
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
	workpiece_dimensions workpiece_dims(1.5, 1.2, 1.5);
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

      for (auto s : surfaces) {
	cout << s.face_orientation(s.front()) << endl;
      }

      SECTION("A box with 1 hole has 6 outer surfaces") {
	REQUIRE(surfaces.size() == 6);
      }
    }

    SECTION("Box with 2 holes") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWith2Holes.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      auto surfaces = outer_surfaces(mesh);

      for (auto s : surfaces) {
	cout << s.face_orientation(s.front()) << endl;
      }

      SECTION("A box with 2 holes has 6 outer surfaces") {
	REQUIRE(surfaces.size() == 6);
      }
    }
  }
}