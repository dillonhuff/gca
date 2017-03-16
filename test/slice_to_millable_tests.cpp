#include "catch.hpp"

#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "process_planning/surface_planning.h"
#include "system/parse_stl.h"
#include "synthesis/clamp_orientation.h"

namespace gca {

  struct part_search_result {
    Nef_polyhedron part_nef;
  };

  bool is_rectilinear(const triangular_mesh& m,
		      const std::vector<std::vector<surface> >& corner_groups) {
    
  }

  std::vector<part_search_result>
  search_part_space(const Nef_polyhedron& part_nef) {
    vector<triangular_mesh> ms = nef_polyhedron_to_trimeshes(part_nef);

    if (ms.size() != 1) {
      return {};
    }

    auto m = ms.front();

    auto sfc = build_surface_milling_constraints(m);
    vector<vector<surface> > corner_groups =
      sfc.hard_corner_groups();

    cout << "# of hard corner groups = " << corner_groups.size() << endl;
    //vtk_debug_mesh(m);

    if (is_rectilinear(m, corner_groups)) {
      cout << "Rectilinear!" << endl;
      return {{part_nef}};
    }

    if (corner_groups.size() == 0) {
      cout << "No hard corner groups left" << endl;
      //vtk_debug_mesh(m);
      return {{part_nef}};
    }

    for (auto& r : corner_groups) {
      //vtk_debug_highlight_inds(r);

      for (auto& s : r) {
	plane p = surface_plane(s);
	vtk_debug(m, p);

	auto clipped_nef = clip_nef(part_nef, p.slide(0.0001));
	//auto clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef);
	//vtk_debug_meshes(clipped_meshes);

	auto res = search_part_space(clipped_nef);

	clipped_nef = clip_nef(part_nef, p.flip().slide(0.0001));
	//clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef);
	//vtk_debug_meshes(clipped_meshes);

	res = search_part_space(clipped_nef);
      }
    }

    return {};
  }

  TEST_CASE("Parsing that weird failing print object") {
    triangular_mesh m =
      //parse_stl("./test/stl-files/onshape_parts/caliperbedlevelingi3v2_fixed - Part 1.stl", 0.0001);
      // parse_stl("./test/stl-files/onshape_parts/SHUTTLEMODULE - SHUTTLEBODY.stl", 0.0001);
      //parse_stl("./test/stl-files/onshape_parts/CTT-CM - Part 1.stl", 0.0001);
      //parse_stl("./test/stl-files/onshape_parts/artusitestp1 - Part 1.stl", 0.0001);
      //parse_stl("test/stl-files/onshape_parts/Rear Slot - Rear Slot.stl", 0.0001);

      parse_stl("test/stl-files/onshape_parts/SmallReverseCameraMount - Part 1.stl", 0.0001);

    search_part_space(trimesh_to_nef_polyhedron(m));

  }

}
