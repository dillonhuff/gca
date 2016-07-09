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

  bool any_SA_surface_contains(index_t i,
  			       const std::vector<surface>& surfaces) {
    for (auto surface : surfaces) {
      if (surface.is_SA() && surface.contains(i)) { return true; }
    }
    return false;
  }

  void remove_SA_surfaces(const std::vector<surface>& surfaces,
  			  std::vector<index_t>& indices) {
    delete_if(indices,
  	      [&surfaces](index_t i)
  	      { return any_SA_surface_contains(i, surfaces); });
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

}
