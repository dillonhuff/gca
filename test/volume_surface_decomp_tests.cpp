#include "catch.hpp"

#include "feature_recognition/visual_debug.h"
#include "geometry/mesh_operations.h"
#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_debug.h"
#include "synthesis/visual_debug.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Subtracting from ") {
    auto mesh =
      parse_stl("./test/stl-files/onshape_parts/100-009 - Part 1.stl", 0.0001);

    box bb = mesh.bounding_box();
    double eps = 0.000001;
    bb.x_min += eps;
    bb.y_min += eps;
    bb.z_min += eps;

    bb.x_max -= eps;
    bb.y_max -= eps;
    bb.z_max -= eps;

    triangular_mesh stock = make_mesh(box_triangles(bb), 0.001);

    //vtk_debug_meshes({mesh, stock});

    Nef_polyhedron mesh_nef = trimesh_to_nef_polyhedron(mesh);
    Nef_polyhedron stock_nef = trimesh_to_nef_polyhedron(stock);
    auto negative_space = stock_nef - mesh_nef;

    cout << "about to convert to trimeshes" << endl;
    auto neg_meshes = nef_polyhedron_to_trimeshes(negative_space);

    vtk_debug_meshes(neg_meshes);
  }

}
