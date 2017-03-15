#include "catch.hpp"

#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "process_planning/surface_planning.h"
#include "system/parse_stl.h"
#include "synthesis/clamp_orientation.h"

namespace gca {

  void search_part_space(const triangular_mesh& m) {
    auto sfc = build_surface_milling_constraints(m);

    vector<vector<surface> > corner_groups =
      sfc.hard_corner_groups();

    if (corner_groups.size() == 0) {
      cout << "No hard corner groups left" << endl;
      vtk_debug_mesh(m);
      return;
    }

    Nef_polyhedron mesh_nef = trimesh_to_nef_polyhedron(m);
    for (auto& r : corner_groups) {
      for (auto& s : r ) {
	plane p = surface_plane(s);
	vtk_debug(m, p);

	auto clipped_nef = clip_nef(mesh_nef, p.slide(0.0001));
	auto clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef);
	vtk_debug_meshes(clipped_meshes);

	for (auto& m : clipped_meshes) {
	  search_part_space(m);
	}

	clipped_nef = clip_nef(mesh_nef, p.flip().slide(0.0001));
	clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef);
	vtk_debug_meshes(clipped_meshes);

	for (auto& m : clipped_meshes) {
	  search_part_space(m);
	}
      }
    }


  }

  TEST_CASE("Parsing that weird failing print object") {
    triangular_mesh m =
      //parse_stl("./test/stl-files/onshape_parts/caliperbedlevelingi3v2_fixed - Part 1.stl", 0.0001);
      // parse_stl("./test/stl-files/onshape_parts/SHUTTLEMODULE - SHUTTLEBODY.stl", 0.0001);
      // parse_stl("./test/stl-files/onshape_parts/CTT-CM - Part 1.stl", 0.0001);
      parse_stl("./test/stl-files/onshape_parts/artusitestp1 - Part 1.stl", 0.0001);

    search_part_space(m);

  }

}
