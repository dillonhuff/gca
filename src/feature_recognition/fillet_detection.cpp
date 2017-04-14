#include "feature_recognition/fillet_detection.h"

#include "geometry/vtk_debug.h"
#include "geometry/triangular_mesh_utils.h"

namespace gca {

  std::vector<std::vector<surface> >
  detect_fillets(const triangular_mesh& part) {
    auto regions = const_orientation_regions(part);
    vector<surface> const_surfs = inds_to_surfaces(regions, part);

    auto group_surfs =
      [](const surface& l, const surface& r) {
      vector<shared_edge> shared =
      all_shared_edges(l.index_list(),
		       r.index_list(),
		       l.get_parent_mesh());

      if (shared.size() == 0) { return false; }

      for (auto s : shared) {
	if (!is_valley_edge(s, l.get_parent_mesh()) &&
	    angle_eps(s, l.get_parent_mesh(), 90.0, 10.0)) {
	  return false;
	}
      }
				    

      return (l.surface_area() < 5*r.surface_area()) &&
      (r.surface_area() < 5*l.surface_area());
    };

    vector<vector<surface> > similar_size =
      connected_components_by_elems(const_surfs, group_surfs);


    delete_if(similar_size, [](const vector<surface>& s) {
	return s.size() == 1;
      });

    //visualize_surface_decomp(similar_size);
    // for (auto& sg : similar_size) {
    //   cout << "NORMALS" << endl;
    //   for (auto s : sg) {
    // 	cout << normal(s) << endl;
    //   }
    //   vtk_debug_highlight_inds(sg);
    // }

    return similar_size;
  }

}
