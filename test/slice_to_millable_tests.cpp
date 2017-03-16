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

  bool is_centralized(const std::vector<surface>& corner_group) {
    int num_surfaces_with_ortho_multiple_ortho_connections = 0;

    for (unsigned i = 0; i < corner_group.size(); i++) {
      int num_ortho_connections = 0;
      const surface& l = corner_group[i];
      for (unsigned j = 0; j < corner_group.size(); j++) {
	if (i != j) {
	  const surface& r = corner_group[j];
	  if (share_orthogonal_valley_edge(l, r)) {
	    num_ortho_connections++;
	  }
	}
      }

      if (num_ortho_connections > 1) {
	num_surfaces_with_ortho_multiple_ortho_connections++;
      }
    }

    return num_surfaces_with_ortho_multiple_ortho_connections <= 1;
  }

  bool all_corner_groups_millable(const std::vector<std::vector<surface> >& corner_groups) {
    for (auto& cg : corner_groups) {
      if (cg.size() > 2) {
	if (!is_centralized(cg)) {
	  return false;
	}
      }
    }
    return true;
  }

  bool is_rectilinear(const triangular_mesh& m,
		      const std::vector<std::vector<surface> >& corner_groups) {
    // vector<surface> sfs = outer_surfaces(m);

    // DBG_ASSERT(sfs.size() > 0);

    // vector<plane> stock_planes = set_right_handed(max_area_basis(sfs));


    if (all_corner_groups_millable(corner_groups)) {
      return true;
    }

    return false;
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

    int num_planes = 0;
    for (auto& r : corner_groups) {
      //vtk_debug_highlight_inds(r);

      if (!is_centralized(r)) {
	num_planes += r.size();
      }
    }

    cout << "Number of possible clipping planes = " << num_planes << endl;

    DBG_ASSERT(false);

    for (auto& r : corner_groups) {
      //vtk_debug_highlight_inds(r);

      if (!is_centralized(r)) {
	for (auto& s : r) {
	  plane p = surface_plane(s);
	  //vtk_debug(m, p);

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
    }

    return {};
  }

  // parse_stl("./test/stl-files/onshape_parts/SHUTTLEMODULE - SHUTTLEBODY.stl", 0.0001);  

  TEST_CASE("Parsing that weird failing print object") {
    triangular_mesh m =
      //parse_stl("./test/stl-files/onshape_parts/caliperbedlevelingi3v2_fixed - Part 1.stl", 0.0001);

      //parse_stl("./test/stl-files/onshape_parts/CTT-CM - Part 1.stl", 0.0001);
      //parse_stl("./test/stl-files/onshape_parts/artusitestp1 - Part 1.stl", 0.0001);
      //parse_stl("test/stl-files/onshape_parts/Rear Slot - Rear Slot.stl", 0.0001);

      parse_stl("test/stl-files/onshape_parts/SmallReverseCameraMount - Part 1.stl", 0.0001);

    auto res = search_part_space(trimesh_to_nef_polyhedron(m));

    cout << "Size of result = " << res.size() << endl;

  }

}
