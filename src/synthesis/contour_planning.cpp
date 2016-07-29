#include "synthesis/contour_planning.h"
#include "synthesis/millability.h"

namespace gca {

  boost::optional<contour_surface_decomposition>
  contour_surface_decomposition_in_dir(const triangular_mesh& part_mesh,
				       const point n) {
    vector<surface> surfs_to_cut = surfaces_to_cut(part_mesh);

    assert(surfs_to_cut.size() > 0);

    boost::optional<surface> bottom = mesh_top_surface(part_mesh, -1*n);
    
    if (bottom) {
      cout << "Has bottom" << endl;
      boost::optional<surface> top = mesh_top_surface(part_mesh, n);

      if (top) {
	cout << "Has top" << endl;
	std::vector<surface> vertical_surfs =
	  connected_vertical_surfaces(part_mesh, n);
	boost::optional<surface> outline =
	  part_outline_surface(&vertical_surfs, n);

	if (outline) {
	  cout << "Has outline" << endl;

	  // TODO: Refine this analysis to include surface grouping
	  // TODO: Remove massive overkill surface culling
	  vector<surface> sfs = {*outline, *top, *bottom};
	  remove_contained_surfaces({*outline, *top, *bottom}, surfs_to_cut);
	  vector<surface> from_n = surfaces_visible_from(surfs_to_cut, n);
	  concat(sfs, from_n);
	  remove_contained_surfaces(sfs, surfs_to_cut);
	  vector<surface> from_minus_n = surfaces_visible_from(surfs_to_cut, -1*n);
	  concat(sfs, from_minus_n);
	  remove_contained_surfaces(sfs, surfs_to_cut);

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

  boost::optional<contour_surface_decomposition>
  compute_contour_surfaces(const triangular_mesh& part_mesh) {

    vector<point> candidate_contour_normals =
      possible_contour_normals(part_mesh);

    point n(0, 0, 1);
    
    for (auto n : candidate_contour_normals) {
      boost::optional<contour_surface_decomposition> surfs =
	contour_surface_decomposition_in_dir(part_mesh, n);
      if (surfs) { return surfs; }
    }

    return boost::none;
  }
  
}
