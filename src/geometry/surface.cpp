#include "geometry/surface.h"

namespace gca {

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

}
