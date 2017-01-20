#include "geometry/extrusion.h"
#include "geometry/surface.h"
#include "geometry/triangular_mesh_utils.h"
#include "process_planning/mandatory_volumes.h"
#include "synthesis/millability.h"

namespace gca {

  triangular_mesh
  extrude_mandatory_volume(const surface& s,
			   const std::vector<surface>& surf_complex,
			   const point n) {
    cout << "# of surfaces = " << surf_complex.size() << endl;

    // surface min_surf = min_e(surf_complex,
    // 			     [n](const surface& sf) { return min_in_dir(sf, n); });
    surface max_surf = max_e(surf_complex,
			     [n](const surface& sf) { return max_in_dir(sf, n); });

    double dist = max_in_dir(max_surf, n) - min_in_dir(s, n);

    cout << "Min distance along " << n << " = " << min_in_dir(s, n) << endl;
    cout << "Max distance along " << n << " = " << max_in_dir(max_surf, n) << endl;
    cout << "Distance to extrude = " << dist << endl;

    auto vol = extrude_surface_negative(s.index_list(), s.get_parent_mesh(), n, dist);
    //vtk_debug_mesh(vol);

    return vol;
  }

  std::vector<mandatory_volume>
  build_mandatory_volumes(const std::vector<surface>& surf_complex) {
    if (surf_complex.size() == 0) { return {}; }

    std::vector<index_t> face_inds;
    for (auto& s : surf_complex) {
      concat(face_inds, s.index_list());
    }

    const triangular_mesh& m = surf_complex.front().get_parent_mesh();

    vector<mandatory_volume> vols;
    for (auto& s : surf_complex) {
      point s_n = normal(s);
      cout << "Trying normal = " << s_n << endl;

      vector<index_t> vert_or_horiz =
	select(face_inds, [s_n, m](const index_t& i) {
	    return all_parallel_to({i}, m, s_n, 1.0) ||
	    all_orthogonal_to({i}, m, s_n, 1.0);
	  });

      if (vert_or_horiz.size() == face_inds.size()) {
      
	auto millable_faces = prismatic_millable_faces(s_n, m);
	if (intersection(millable_faces, face_inds).size() == face_inds.size()) {
	  cout << "Viable direction = " << s_n << endl;

	  vols.push_back(mandatory_volume{extrude_mandatory_volume(s, surf_complex, s_n), s_n});
	}
      }
    }
    return vols;
  }

  std::vector<std::vector<mandatory_volume> >
  mandatory_volumes(const triangular_mesh& part) {
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

    vector<vector<mandatory_volume> > volumes;
    for (auto& sc : surf_complexes) {
      //vtk_debug_highlight_inds(sc);
      volumes.push_back(build_mandatory_volumes(sc));
    }

    return volumes;
  }
  
}
