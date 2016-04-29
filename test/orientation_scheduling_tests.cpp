#include "catch.hpp"
#include "geometry/triangular_mesh.h"
#include "synthesis/orientation_scheduling.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

using namespace gca;

TEST_CASE("Box orientation scheduling") {
  arena_allocator a;
  set_system_allocator(&a);

  auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Cube0p5.stl").triangles;
  auto mesh = make_mesh(box_triangles, 0.001);

  SECTION("Box faces") {
    auto face_orients = part_face_orientations(mesh);
    for (auto f : face_orients) {
      cout << f << endl;
    }
    REQUIRE(face_orients.size() == 6);
  }

  SECTION("Box scheduling") {
    REQUIRE(workpiece_orientations(mesh).size() == 6);
  }
}
