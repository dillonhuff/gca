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

  boost::optional<contour_surface_decomposition>
  compute_contour_surfaces(const triangular_mesh& part_mesh) {

    vector<point> candidate_contour_normals =
      possible_contour_normals(part_mesh);

    for (auto n : candidate_contour_normals) {
      boost::optional<contour_surface_decomposition> surfs =
	contour_surface_decomposition_in_dir(part_mesh, n);
      if (surfs) { return surfs; }
    }

    return boost::none;
  }
  
}
