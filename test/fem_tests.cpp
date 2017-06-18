#include "catch.hpp"

#include "geometry/vtk_debug.h"
#include "synthesis/clamp_orientation.h"
#include "system/parse_stl.h"

namespace gca {

  vector<index_t>
  coplanar_triangles(const plane p, const triangular_mesh& mesh) {
    vector<index_t> inds;
    for (auto i : mesh.face_indexes()) {
      triangle t = mesh.face_triangle(i);
      if (angle_eps(p.normal(), normal(t), 0.0, 0.1)) {
	point diff = p.pt() - t.v1;
	if (angle_eps(diff, p.normal(), 90.0, 0.1)) {
	  inds.push_back(i);
	}
      }
    }
    return inds;
  }

  TEST_CASE("PSU mount test") {
    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", 0.0001);
    vice v = emco_vice(point(0, 0, 0));

    auto surfs = outer_surfaces(mesh);
    vector<clamp_orientation> orients = all_viable_clamp_orientations(surfs, v);

    REQUIRE(orients.size() > 0);

    clamp_orientation test_orient = orients[0];
    
    cout << "top orient 1 = " << test_orient.top_normal() << endl;

    plane left_pl = test_orient.left_plane();
    plane right_pl = test_orient.right_plane();

    auto coplanar_tris = coplanar_triangles(left_pl, mesh);
    vtk_debug_highlight_inds(coplanar_tris, mesh);

    coplanar_tris = coplanar_triangles(right_pl, mesh);
    vtk_debug_highlight_inds(coplanar_tris, mesh);
  }
}
