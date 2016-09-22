#include "geometry/offset.h"
#include "geometry/vtk_debug.h"
#include "process_planning/axis_location.h"
#include "synthesis/contour_planning.h"
#include "synthesis/millability.h"

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

	  return contour_surface_decomposition{n, *outline, *top, *bottom, surfs_to_cut};
	  
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
    DBG_ASSERT(outline_sets.size() > 0);

    auto bottom_sets = connected_boundary_edge_sets(bottom);
    DBG_ASSERT(bottom_sets.size() > 0);

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

    point n = part_axis(part_mesh);

    cout << "Checking for contour in " << n << endl;

    boost::optional<contour_surface_decomposition> surfs =
      contour_surface_decomposition_in_dir(part_mesh, n);

    if (surfs) {
      cout << "Found contour in " << n << endl;
    }

    return surfs;
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

  point pick_jaw_cutout_axis(const contour_surface_decomposition& surfs) {
    vector<surface> viable_regions =
      regions_connected_to_both(surfs.outline, surfs.top, surfs.bottom);

    // TODO: Later sort multiple regions
    DBG_ASSERT(viable_regions.size() == 1);

    surface r = viable_regions.front();

    std::vector<gca::edge> edges = shared_edges(r, surfs.bottom);

    DBG_ASSERT(edges.size() > 0);

    // TODO: Check edge lengths
    auto middle_ind = static_cast<unsigned>(ceil((edges.size() + 1) / 2.0) - 1);

    DBG_ASSERT(0 <= middle_ind && middle_ind < edges.size());

    gca::edge e = edges[middle_ind];
    auto tris = r.edge_face_neighbors(e);
    if (!(tris.size() == 1)) {
      cout << "# of tris = " << tris.size() << endl;
      DBG_ASSERT(false);
    }
    return r.face_orientation(tris[0]);
  }


  boost::optional<oriented_polygon>
  simple_outline(const triangular_mesh& out,
		 const point n) {
    auto bound = contour_outline(out.face_indexes(), out, n);
    if (bound) {
    } else {
      return boost::none;
    }
    vector<oriented_polygon> outlines =
      mesh_bounds((*bound).index_list(), (*bound).get_parent_mesh());

    if (outlines.size() != 2) {
      DBG_ASSERT(false);
    }

    oriented_polygon outl =
      min_e(outlines, [n](const oriented_polygon& p)
	    { return min_distance_along(p.vertices(), n); });

    return outl;
  }
  
}
