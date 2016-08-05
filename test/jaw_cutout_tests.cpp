#include "catch.hpp"
#include "geometry/vtk_debug.h"
#include "synthesis/jaw_cutout.h"
#include "synthesis/workpiece_clipping.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Clipped cylinder jaw cutout") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = large_jaw_vice(4.5, point(-0.8, -4.4, -3.3));
    std::vector<plate_height> parallel_plates{0.5, 0.7};
    fixtures fixes(test_vice, parallel_plates);
    
    tool t1(0.1, 3.0, 4, HSS, FLAT_NOSE);
    vector<tool> tools{t1};
    workpiece workpiece_dims(3.0, 1.9, 3.0, ACETAL);

    auto part_mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ClippedCylinder.stl", 0.001);

    auto decomp = contour_surface_decomposition_in_dir(part_mesh, point(0, 0, 1));

    REQUIRE(decomp);
    
    soft_jaws jaws = make_soft_jaws(*decomp, test_vice, -1*(decomp->n));
    triangular_mesh* axis_jaw = jaws.a_jaw;
    triangular_mesh* neg_axis_jaw = jaws.an_jaw;
    
    REQUIRE(axis_jaw->is_connected());
    REQUIRE(neg_axis_jaw->is_connected());

    vector<surface> axis_surfs = surfaces_to_cut(*axis_jaw);
    vector<surface> neg_axis_surfs = surfaces_to_cut(*neg_axis_jaw);

    REQUIRE(axis_surfs.size() == 8);
    REQUIRE(neg_axis_surfs.size() == 8);

    clipping_plan p =
      workpiece_clipping_programs(workpiece_dims, *axis_jaw, tools, fixes);

    for (auto s : p.surfaces_left_to_cut()) {
      cout << "--- normal = " << s.face_orientation(s.front()) << endl;
    }
    
    vtk_debug_highlight_inds(p.surfaces_left_to_cut());

    REQUIRE(p.stable_surfaces().size() == 6);
    REQUIRE(p.surfaces_left_to_cut().size() == 2);
  }
}
