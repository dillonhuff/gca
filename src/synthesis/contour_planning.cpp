#include "synthesis/contour_planning.h"
#include "synthesis/millability.h"

#include "geometry/vtk_debug.h"

namespace gca {

  boost::optional<surface>
  contour_outline(const std::vector<index_t>& inds,
		  const triangular_mesh& part_mesh,
		  const point n) {
    vector<index_t> millable_faces =
      select(inds,
      	     [part_mesh, n](const index_t i)
      	     { return within_eps(angle_between(part_mesh.face_orientation(i), n), 90, 2.0); });

    vector<index_t> outer_faces =
      merge_surfaces(outer_surfaces(part_mesh)).index_list();

    vector<index_t> outer_vertical_faces =
      intersection(outer_faces, millable_faces);

    //vtk_debug_highlight_inds(outer_vertical_faces, part_mesh);

    auto regions =
      connect_regions(outer_vertical_faces, part_mesh);

    if (regions.size() == 1) {
      return surface(&part_mesh, regions.front());
    }

    return boost::none;
  }

  boost::optional<contour_surface_decomposition>
  contour_surface_decomposition_in_dir(const triangular_mesh& part_mesh,
				       const point n) {

    std::vector<index_t> inds = part_mesh.face_indexes();
    boost::optional<surface> bottom = mesh_top_surface(part_mesh, -1*n);
    
    if (bottom) {
      cout << "Has bottom" << endl;
      subtract(inds, bottom->index_list());
      boost::optional<surface> top = mesh_top_surface(part_mesh, n);

      if (top) {
	cout << "Has top" << endl;

	subtract(inds, top->index_list());
	
	boost::optional<surface> outline =
	  contour_outline(inds, part_mesh, n);
	if (outline) {
	  cout << "Has outline" << endl;
	  subtract(inds, outline->index_list());

	  // vtk_debug_highlight_inds(outline->index_list(),
	  // 			   outline->get_parent_mesh());

	  vector<surface> surfs_to_cut = surfaces_to_cut(inds, part_mesh);
	  
	  vector<surface> from_n = surfaces_visible_from(surfs_to_cut, n);
	  if (from_n.size() > 0) {
	    subtract(inds, merge_surfaces(from_n).index_list());
	  }

	  vector<surface> from_minus_n = surfaces_visible_from(surfs_to_cut, -1*n);
	  if (from_minus_n.size() > 0) {
	    subtract(inds, merge_surfaces(from_minus_n).index_list());
	  }

	  surfs_to_cut = surfaces_to_cut(inds, part_mesh);

	  return contour_surface_decomposition{n, *outline, *top, *bottom, from_n, from_minus_n, surfs_to_cut};
	  
	} else {
	  cout << "No outline" << endl;
	}


      } else {
	cout << "No top" << endl;
      }
    } else {
      cout << "No bottom" << endl;
    }

    return boost::none;
  }

  // TODO: Actually implement with surface sorting
  std::vector<point>
  possible_contour_normals(const triangular_mesh& part_mesh) {
    return {{0, 0, 1}, {0, 1, 0}};
  }

  std::vector<gca::edge>
  boundary_edges(const surface& s) {
    std::vector<gca::edge> bound_edges;
    for (auto e : s.get_parent_mesh().edges()) {
      auto l_face_neighbors = s.get_parent_mesh().vertex_face_neighbors(e.l);
      auto r_face_neighbors = s.get_parent_mesh().vertex_face_neighbors(e.r);
      auto face_neighbors = intersection(l_face_neighbors, r_face_neighbors);
      bool contains_some_neighbors = false;
      bool contains_all_neighbors = true;
      for (auto facet : face_neighbors) {
	if (s.contains(facet)) {
	  contains_some_neighbors = true;
	} else {
	  contains_all_neighbors = false;
	}
      }
      if (contains_some_neighbors && !contains_all_neighbors) {
	bound_edges.push_back(e);
      }
    }
    return bound_edges;
  }

  std::vector<std::vector<gca::edge>>
  connected_boundary_edge_sets(const surface& s) {
    std::vector<gca::edge> bound_edges = boundary_edges(s);
    auto edge_group_inds =
      connected_components_by(bound_edges,
			      [](const gca::edge x, const gca::edge y)
			      { return share_endpoint(x, y); });
    std::vector<std::vector<gca::edge>> edge_groups;
    for (auto inds : edge_group_inds) {
      vector<gca::edge> es;
      for (auto i : inds) {
	es.push_back(bound_edges[i]);
      }
      edge_groups.push_back(es);
    }
    return edge_groups;
  }

  // TODO: Optimize this hideously slow comparison
  bool outline_shares_edges(const contour_surface_decomposition& surfs) {
    const surface& outline = surfs.outline;
    const surface& bottom = surfs.bottom;
    auto outline_sets = connected_boundary_edge_sets(outline);
    assert(outline_sets.size() > 0);

    auto bottom_sets = connected_boundary_edge_sets(bottom);
    assert(bottom_sets.size() > 0);

    for (auto s : outline_sets) {
      for (auto e : bottom_sets) {
	if (intersection(s, e).size() == e.size() && (s.size() == e.size())) {
	  return true;
	}
      }
    }
    return false;
  }

  boost::optional<contour_surface_decomposition>
  compute_contour_surfaces(const triangular_mesh& part_mesh) {

    vector<point> candidate_contour_normals =
      possible_contour_normals(part_mesh);

    for (auto n : candidate_contour_normals) {
      cout << "Checking for contour in " << n << endl;
      boost::optional<contour_surface_decomposition> surfs =
	contour_surface_decomposition_in_dir(part_mesh, n);
      if (surfs) {
	if (outline_shares_edges(*surfs)) {
	  return surfs;
	} else {
	  contour_surface_decomposition decomposition{-1*surfs->n, surfs->outline, surfs->bottom, surfs->top, surfs->visible_from_minus_n, surfs->visible_from_n, surfs->rest};
	  if (outline_shares_edges(decomposition)) {
	    return decomposition;
	  }
	}
      }
    }

    return boost::none;
  }

  std::vector<surface>
  regions_connected_to_both(const surface& to_check,
			    const surface& top,
			    const surface& bottom) {
    vector<surface> subsurfs =
      constant_orientation_subsurfaces(to_check);
    delete_if(subsurfs,
	      [top, bottom](const surface& s)
	      { return !(surfaces_share_edge(s, top) && surfaces_share_edge(s, bottom)); });

    auto merged =
      connected_components_by(subsurfs,
			      [](const surface& x, const surface& y)
			      { return surfaces_share_edge(x, y); });
    return merge_surface_groups(subsurfs, merged);
  }

}
