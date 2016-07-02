#include "catch.hpp"
#include "synthesis/millability.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Millability") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Box millability") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Cube0p5.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);

      SECTION("From the x front") {
      	vector<index_t> millable = millable_faces(point(1, 0, 0), mesh);
      	REQUIRE(millable.size() == 10);
      }

      SECTION("From the x back") {
      	vector<index_t> millable = millable_faces(point(-1, 0, 0), mesh);
      	REQUIRE(millable.size() == 10);
      }

      SECTION("From the y front") {
      	vector<index_t> millable = millable_faces(point(0, 1, 0), mesh);
      	REQUIRE(millable.size() == 10);
      }

      SECTION("From the y back") {
      	vector<index_t> millable = millable_faces(point(0, -1, 0), mesh);
      	REQUIRE(millable.size() == 10);
      }
      
      SECTION("From the top") {
      	vector<index_t> millable = millable_faces(point(0, 0, 1), mesh);
      	REQUIRE(millable.size() == 10);
      }

      SECTION("From the bottom") {
      	vector<index_t> millable = millable_faces(point(0, 0, -1), mesh);
      	REQUIRE(millable.size() == 10);
      }

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
      point p(0, 0, -1);
      vector<index_t> millable = millable_faces(p, mesh);
      REQUIRE(millable.size() == 10);
    }

    // TODO: Reintroduce this test
    // SECTION("Mesh box plinth") {
    //   auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/MeshBoxPlinth.stl").triangles;
    //   auto mesh = make_mesh(box_triangles, 0.001);
    //   vector<index_t> millable = millable_faces(point(0, 0, 1), mesh);
    //   REQUIRE(millable.size() == (box_triangles.size() - 2));
    // }
  }

}
