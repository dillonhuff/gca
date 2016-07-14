#include "geometry/surface.h"

namespace gca {

  bool surfaces_share_edge(const surface& l,
			   const surface& r) {
    auto ind1 = l.index_list();
    auto ind2 = r.index_list();
    return share_edge(ind1, ind2, l.get_parent_mesh());
  }
  
  bool surfaces_share_edge(const unsigned i,
			   const unsigned j,
			   const std::vector<surface>& surfaces) {
    auto ind1 = surfaces[i].index_list();
    auto ind2 = surfaces[j].index_list();
    return share_edge(ind1, ind2, surfaces[i].get_parent_mesh());
  }

  std::vector<index_t> surface_vertexes(const surface& s) {
    vector<index_t> inds;
    for (auto i : s.index_list()) {
      triangle_t t = s.get_parent_mesh().triangle_vertices(i);
      inds.push_back(t.v[0]);
      inds.push_back(t.v[1]);
      inds.push_back(t.v[2]);
    }
    sort(begin(inds), end(inds));
    return inds;
  }

  bool orthogonal_flat_surfaces(const surface* l, const surface* r) {
    point l_orient = l->face_orientation(l->front());
    point r_orient = r->face_orientation(r->front());
    double theta = angle_between(l_orient, r_orient);
    return within_eps(theta, 90, 0.1);
  }

  bool parallel_flat_surfaces(const surface* l, const surface* r) {
    point l_orient = l->face_orientation(l->front());
    point r_orient = r->face_orientation(r->front());
    double theta = angle_between(l_orient, r_orient);
    return within_eps(theta, 180, 0.1);
  }

  std::vector<surface> outer_surfaces(const triangular_mesh& part) {
    auto const_orient_face_indices = const_orientation_regions(part);
    vector<surface> surfaces;
    for (auto f : const_orient_face_indices) {
      assert(f.size() > 0);
      if (is_outer_surface(f, part)) {
	surfaces.push_back(surface(&part, f));
      }
    }
    return surfaces;
  }

  surface merge_surfaces(const std::vector<surface>& surfaces) {
    assert(surfaces.size() > 0);
    vector<index_t> inds;
    for (auto s : surfaces) {
      concat(inds, s.index_list());
    }
    return surface(&(surfaces.front().get_parent_mesh()), inds);
  }

  void remove_contained_surfaces(const std::vector<surface>& stable_surfaces,
				 std::vector<surface>& surfaces_to_cut) {
    vector<index_t> stable_surface_inds;
    for (auto s : stable_surfaces) {
      concat(stable_surface_inds, s.index_list());
    }
    sort(begin(stable_surface_inds), end(stable_surface_inds));

    delete_if(surfaces_to_cut,
	      [&stable_surface_inds](const surface& s)
	      { return s.contained_by_sorted(stable_surface_inds); });
  }

  // void remove_clipped_surfaces(const std::vector<surface>& stable_surfaces,
  // 			       std::vector<surface>& surfaces_to_cut) {
  //   vector<surface> sa_surfs;
  //   for (auto s : stable_surfaces) {
  //     if (s.is_SA()) {
  // 	sa_surfs.push_back(s);
  //     }
  //   }
  //   remove_contained_surfaces(sa_surfs, surfaces_to_cut);
  // }

  boost::optional<surface>
  part_outline_surface(std::vector<surface>* surfaces_to_cut,
		       const point n) {
    vector<surface> vertical_surfs =
      select(*surfaces_to_cut,
    	     [n](const surface& s)
    	     { return s.orthogonal_to(n, 0.01); });

    vector<vector<unsigned>> merge_groups =
      connected_components_by(vertical_surfs, [](const surface& l, const surface& r)
    			      { return surfaces_share_edge(l, r); });

    if (merge_groups.size() == 1) {
      return merge_surfaces(vertical_surfs);
    }

    return boost::none;
  }

  boost::optional<surface>
  part_outline_surface(const triangular_mesh& m,
		       const point n) {
    vector<surface> surfs = surfaces_to_cut(m);
    return part_outline_surface(&surfs, n);
  }

  // TODO: Need to add normal vectors, how to match this with
  // the code in make_fixture_plan?
  boost::optional<oriented_polygon>
  part_outline(std::vector<surface>* surfaces_to_cut) {
    point n(0, 0, 1);

    auto m = part_outline_surface(surfaces_to_cut, n);
    if (m) {
      vector<oriented_polygon> outlines =
	mesh_bounds((*m).index_list(), (*m).get_parent_mesh());
      if (outlines.size() == 2) {
	vector<surface> vertical_surfs{*m};
	remove_contained_surfaces(vertical_surfs, *surfaces_to_cut);
	return outlines.front();
      }
    }

    return boost::none;
  }

  std::vector<surface> surfaces_to_cut(const triangular_mesh& part) {
    double normal_degrees_delta = 30.0;
    auto inds = part.face_indexes();

    
    vector<vector<index_t>> delta_regions =
      normal_delta_regions(inds, part, normal_degrees_delta);
    vector<surface> surfaces;
    for (auto r : delta_regions) {
      surfaces.push_back(surface(&part, r));
    }
    return surfaces;
  }

}
