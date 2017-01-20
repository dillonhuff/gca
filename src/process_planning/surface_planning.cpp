#include "process_planning/surface_planning.h"

#include "geometry/triangular_mesh_utils.h"

namespace gca {

std::vector<std::vector<surface> >
surface_milling_constraints::hard_corner_groups() const {
return scs;
  }

  surface_milling_constraints
  build_surface_milling_constraints(const triangular_mesh& part) {

    auto regions = const_orientation_regions(part);
    vector<surface> const_surfs = inds_to_surfaces(regions, part);
    vector<vector<surface> > surf_complexes =
      connected_components_by_elems(const_surfs,
				    [](const surface& l, const surface& r) {
				      return share_orthogonal_valley_edge(l, r);
				    });

    delete_if(surf_complexes, [](const vector<surface>& surf_complex) {
	return surf_complex.size() < 2;
      });
    
    return surface_milling_constraints(surf_complexes);
  }
  
}
