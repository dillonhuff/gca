#include "catch.hpp"
#include "synthesis/millability.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Millability") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Box millability from the top") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Cube0p5.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      vector<index_t> millable = millable_faces(point(0, 0, 1), mesh);
      REQUIRE(millable.size() == 10);
    }

    SECTION("Box with hole millability from the top") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithTopHole.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      vector<index_t> millable = millable_faces(point(0, 0, 1), mesh);
      REQUIRE(millable.size() == box_triangles.size() - 2);
    }

    SECTION("Box with hole millability from the bottom") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithTopHole.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      vector<index_t> millable = millable_faces(point(0, 0, -1), mesh);
      REQUIRE(millable.size() == 10);
    }

    SECTION("Mesh box plinth") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/MeshBoxPlinth.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      vector<index_t> millable = millable_faces(point(0, 0, 1), mesh);
      REQUIRE(millable.size() == (box_triangles.size() - 2));
    }
  }

}
