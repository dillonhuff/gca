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

    SECTION("Box with hole has 6 clippings and two pocketings") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWith2Holes.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      auto result_programs = mesh_to_gcode(mesh, test_vice, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 8);
    }

  }

  TEST_CASE("Stable surfaces") {
    arena_allocator a;
    set_system_allocator(&a);

    auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Cube0p5.stl").triangles;
    auto mesh = make_mesh(box_triangles, 0.001);
    auto stable_surfaces = part_stable_surfaces(mesh);

    SECTION("Simple box produces only workpiece tightening programs") {
      REQUIRE(stable_surfaces.size() == 6);
    }

    SECTION("All simple box surfaces are part of a stable face") {
      vector<index_t> fis = mesh.face_indexes();
      remove_sa_surfaces(stable_surfaces,
			 fis);
      REQUIRE(fis.size() == 0);
    }
  }
}
