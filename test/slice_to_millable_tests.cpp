#include "catch.hpp"

#include <vtkProperty.h>

#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "process_planning/surface_planning.h"
#include "system/parse_stl.h"
#include "synthesis/clamp_orientation.h"
#include "synthesis/millability.h"
#include "synthesis/visual_debug.h"

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

  std::vector<shared_edge>
  edges_to_fillet(const std::vector<surface>& cg,
		  const triangular_mesh& m,
		  const point dir) {
    vector<shared_edge> edges;

    for (auto& s : cg) {
      for (auto& other_s : cg ) {
	if (!s.contained_by(other_s)) {
	  auto new_edges = all_shared_edges(s.index_list(), other_s.index_list(), m);
	  delete_if(new_edges,
		    [m](const shared_edge e) {
		      return !is_valley_edge(e, m) || !angle_eps(e, m, 90, 0.5);
		    });

	  delete_if(new_edges,
		    [m, dir](const shared_edge e) {
		      point start = m.vertex(e.e.l);
		      point end = m.vertex(e.e.r);
		      point diff = end - start;
		      return !angle_eps(diff, dir, 180.0, 1.0) &&
			!angle_eps(diff, dir, 0.0, 1.0);
		    });

	  concat(edges, new_edges);
	}
      }
    }

    return edges;
    
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

  std::vector<surface> find_access_surfaces(const std::vector<surface>& cg) {
    vector<point> norms;
    for (auto& s : cg) {
      norms.push_back(normal(s));
    }

    return find_access_surfaces(cg, norms);
  }
  
  void vtk_debug_shared_edges(const std::vector<shared_edge>& edges,
			      const triangular_mesh& m) {
    vector<polyline> lines;
    for (auto s : edges) {
      point p1 = m.vertex(s.e.l);
      point p2 = m.vertex(s.e.r);
      lines.push_back(polyline({p1, p2}));
    }

    auto lines_pd = polydata_for_polylines(lines);
    color_polydata(lines_pd, 0, 255, 0);
    auto lines_act = polydata_actor(lines_pd);
    lines_act->GetProperty()->SetLineWidth(10);

    auto mesh_pd = polydata_for_trimesh(m);
    color_polydata(mesh_pd, 255, 0, 0);
    auto mesh_act = polydata_actor(mesh_pd);
    mesh_act->GetProperty()->SetOpacity(0.4);

    visualize_actors({lines_act, mesh_act});
  }

  bool all_surfaces_are_millable_from(const point dir,
				      const std::vector<surface>& sfs) {
    if (sfs.size() == 0) { return true; }

    const triangular_mesh& m = sfs.front().get_parent_mesh();

    vector<index_t> all_inds;
    for (auto& s : sfs) {
      concat(all_inds, s.index_list());
    }
    all_inds = sort_unique(all_inds);

    auto millable_inds = millable_faces(dir, m);

    return intersection(millable_inds, all_inds).size() == all_inds.size();
  }
				      

  bool
  solveable_by_filleting(const triangular_mesh& m,
			 const std::vector<std::vector<surface> >& corner_groups) {
    vector<surface> sfs = outer_surfaces(m);
    DBG_ASSERT(sfs.size() > 0);
    vector<plane> stock_planes = set_right_handed(max_area_basis(sfs));
    vector<point> dirs;
    for (auto& p : stock_planes) {
      dirs.push_back(p.normal());
      dirs.push_back(-1*p.normal());
    }

    for (auto cg : corner_groups) {

      for (auto access_dir : dirs) {

	if (!is_centralized(cg)) {

	  if (all_surfaces_are_millable_from(access_dir, cg)) {
	    vector<shared_edge> edges = edges_to_fillet(cg, m, access_dir);
	    vtk_debug_shared_edges(edges, m);
	  }
	}

      }
    }

    return true;
  }
  
  bool is_rectilinear(const triangular_mesh& m,
		      const std::vector<std::vector<surface> >& corner_groups) {


    if (all_corner_groups_millable(corner_groups)) {
      return true;
    }

    return false;
  }

  bool simplified_corners(const std::vector<std::vector<surface> >& corner_groups,
			  const std::vector<triangular_mesh>& pos_meshes) {
    int groups_after_cut = 0;

    for (auto& m : pos_meshes) {
      auto sfc = build_surface_milling_constraints(m);
      vector<vector<surface> > mcgs =
	sfc.hard_corner_groups();

      for (auto& cg : mcgs) {
	if (!is_centralized(cg)) {
	  groups_after_cut++;
	}
      }

    }

    int groups_before = 0;
    for (auto& cg : corner_groups) {
      if (!is_centralized(cg)) {
	groups_before++;
      }
    }

    cout << "Decentralized groups before cut = " << groups_before << endl;
    cout << "Decentralized groups after  cut = " << groups_after_cut << endl;

    return groups_after_cut < groups_before;
  }


  int count_planes(const std::vector<std::vector<surface> >& corner_groups) {
    int num_planes = 0;
    for (auto& r : corner_groups) {
      //vtk_debug_highlight_inds(r);

      if (!is_centralized(r)) {
	num_planes += r.size();
      }
    }

    return num_planes;
  }

  std::vector<std::vector<part_search_result> >
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

    // if (solveable_by_filleting(m, corner_groups)) {
    //   cout << "Solveable by filleting" << endl;
    //   return {{part_nef}};
    // }

    if (is_rectilinear(m, corner_groups)) {
      cout << "Rectilinear!" << endl;
      return { {{part_nef}} };
    }

    if (corner_groups.size() == 0) {
      cout << "No hard corner groups left" << endl;
      //vtk_debug_mesh(m);
      return { {{part_nef}} };
    }

    int num_planes = count_planes(corner_groups); //0;
    cout << "Number of possible clipping planes = " << num_planes << endl;

    for (auto& r : corner_groups) {
      //vtk_debug_highlight_inds(r);

      if (!is_centralized(r)) {
	for (auto& s : r) {
	  plane p = surface_plane(s);
	  vtk_debug(m, p);

	  auto clipped_nef_pos = clip_nef(part_nef, p.slide(0.0001));
	  auto clipped_nef_neg = clip_nef(part_nef, p.flip().slide(0.0001));

	  auto clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef_pos);
	  vtk_debug_meshes(clipped_meshes);

	  clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef_neg);
	  vtk_debug_meshes(clipped_meshes);
	  
	  vector<triangular_mesh> pos_meshes =
	    nef_polyhedron_to_trimeshes(clipped_nef_pos);
	  concat(pos_meshes, nef_polyhedron_to_trimeshes(clipped_nef_neg));

	  if (simplified_corners(corner_groups, pos_meshes)) {

	    cout << "Simlified corners! continuing" << endl;
	  
	    auto res_pos = search_part_space(clipped_nef_pos);
	    auto res_neg = search_part_space(clipped_nef_neg);
	  }

	}
      }
    }

    return {};
  }

  TEST_CASE("Parsing that weird failing print object") {
    triangular_mesh m =
      parse_stl("./test/stl-files/onshape_parts/caliperbedlevelingi3v2_fixed - Part 1.stl", 0.0001);
      //parse_stl("./test/stl-files/onshape_parts/CTT-CM - Part 1.stl", 0.0001);
      //parse_stl("./test/stl-files/onshape_parts/artusitestp1 - Part 1.stl", 0.0001);
      //parse_stl("test/stl-files/onshape_parts/Rear Slot - Rear Slot.stl", 0.0001);

      //parse_stl("test/stl-files/onshape_parts/SmallReverseCameraMount - Part 1.stl", 0.0001);

    auto res = search_part_space(trimesh_to_nef_polyhedron(m));

    cout << "Size of result = " << res.size() << endl;

  }

}
