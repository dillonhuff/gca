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
  }
}
