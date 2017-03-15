#include "catch.hpp"

#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "process_planning/surface_planning.h"
#include "system/parse_stl.h"
#include "synthesis/clamp_orientation.h"

namespace gca {

  TEST_CASE("Parsing that weird failing print object") {
    triangular_mesh m =
      //parse_stl("./test/stl-files/onshape_parts/caliperbedlevelingi3v2_fixed - Part 1.stl", 0.0001);
      // parse_stl("./test/stl-files/onshape_parts/SHUTTLEMODULE - SHUTTLEBODY.stl", 0.0001);
      parse_stl("./test/stl-files/onshape_parts/CTT-CM - Part 1.stl", 0.0001);

    auto sfc = build_surface_milling_constraints(m);

    vector<vector<surface> > corner_groups =
      sfc.hard_corner_groups();

    Nef_polyhedron mesh_nef = trimesh_to_nef_polyhedron(m);
    for (auto& r : corner_groups) {
      for (auto& s : r ) {
	plane p = surface_plane(s);
	vtk_debug(m, p);

	auto clipped_nef = clip_nef(mesh_nef, p);
	auto clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef);
	vtk_debug_meshes(clipped_meshes);


	clipped_nef = clip_nef(mesh_nef, p.flip());
	clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef);
	vtk_debug_meshes(clipped_meshes);
	
	
      }
    }

  }

}
